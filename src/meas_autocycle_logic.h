#pragma once

#include <limits.h>
#include <stdint.h>

namespace logic::meas_autocycle {

constexpr uint32_t kMarginTicks = 10UL * 60UL * 100000UL;
constexpr uint32_t kLimitTicks = UINT32_MAX - kMarginTicks;
constexpr uint32_t kUploadRetryMs = 30000UL;
constexpr uint32_t kRestartRetryMs = 5000UL;

struct State {
  bool pending = false;
  bool waitingUpload = false;
  uint32_t lastAttemptMs = 0;
};

struct StepResult {
  bool limitReached = false;
  bool stopRequested = false;
  bool stopFailed = false;
  bool waitingForUpload = false;
  bool uploadAttempted = false;
  bool uploadCompleted = false;
  bool restartAttempted = false;
  bool restartStarted = false;
};

inline void clear(State& state) {
  state.pending = false;
  state.waitingUpload = false;
  state.lastAttemptMs = 0;
}

inline bool shouldWaitForUpload(bool cloudEnabled, bool uploadOnStop,
                                bool linkOk, bool internetOk) {
  return cloudEnabled && uploadOnStop && linkOk && internetOk;
}

inline bool hasReachedLimit(bool haveStartTimestamp, uint64_t elapsedTicks) {
  return haveStartTimestamp && elapsedTicks >= static_cast<uint64_t>(kLimitTicks);
}

inline StepResult processLimitReached(State& state, uint32_t nowMs,
                                      bool haveStartTimestamp,
                                      uint64_t elapsedTicks,
                                      bool waitForUpload,
                                      bool stopOk,
                                      bool uploadedImmediately,
                                      bool restartSucceeded) {
  StepResult result;
  if (state.pending || !hasReachedLimit(haveStartTimestamp, elapsedTicks)) {
    return result;
  }

  result.limitReached = true;
  result.stopRequested = true;
  if (!stopOk) {
    result.stopFailed = true;
    return result;
  }

  state.pending = true;
  state.lastAttemptMs = nowMs;

  if (waitForUpload && !uploadedImmediately) {
    state.waitingUpload = true;
    result.waitingForUpload = true;
    return result;
  }

  state.waitingUpload = false;
  result.restartAttempted = true;
  if (restartSucceeded) {
    result.restartStarted = true;
    clear(state);
  }
  return result;
}

inline StepResult processPending(State& state, uint32_t nowMs,
                                 bool stillWaitForUpload,
                                 bool uploadSucceeded,
                                 bool restartSucceeded) {
  StepResult result;
  if (!state.pending) return result;

  if (state.waitingUpload) {
    if (!stillWaitForUpload) {
      state.waitingUpload = false;
      state.lastAttemptMs = nowMs;
      return result;
    }

    if (static_cast<uint32_t>(nowMs - state.lastAttemptMs) < kUploadRetryMs) {
      return result;
    }

    state.lastAttemptMs = nowMs;
    result.uploadAttempted = true;
    if (!uploadSucceeded) {
      return result;
    }

    state.waitingUpload = false;
    result.uploadCompleted = true;
    return result;
  }

  if (static_cast<uint32_t>(nowMs - state.lastAttemptMs) < kRestartRetryMs) {
    return result;
  }

  state.lastAttemptMs = nowMs;
  result.restartAttempted = true;
  if (restartSucceeded) {
    result.restartStarted = true;
    clear(state);
  }
  return result;
}

}  // namespace logic::meas_autocycle

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace logic::storage_fault {

constexpr uint8_t kDegradedThreshold = 1u;
constexpr uint8_t kFaultThreshold = 4u;
constexpr uint32_t kFaultWindowMs = 15000UL;

enum class State : uint8_t {
  HEALTHY = 0,
  DEGRADED = 1,
  FAULTED = 2,
  RECOVERING = 3,
};

constexpr size_t kLastErrorBytes = 64U;
constexpr size_t kLastPathBytes = 128U;

struct Status {
  State state = State::HEALTHY;
  uint8_t faultScore = 0;
  uint32_t faultCount = 0;
  uint32_t firstFaultMs = 0;
  uint32_t lastFaultMs = 0;
  uint32_t lastSuccessMs = 0;
  bool recoveryRebootAttempted = false;
  bool uploadBlocked = false;
  char lastError[kLastErrorBytes] = {0};
  char lastPath[kLastPathBytes] = {0};
};

inline void copyStringCapped(const char* src, char* dest, size_t destSize) {
  if (!dest || destSize == 0) return;
  dest[0] = '\0';
  if (!src) return;
  size_t i = 0;
  for (; i + 1 < destSize && src[i] != '\0'; ++i) dest[i] = src[i];
  dest[i] = '\0';
}

inline void clear(Status& status) {
  status = Status{};
}

inline bool isHealthy(const Status& status) {
  return status.state == State::HEALTHY;
}

inline bool isDegraded(const Status& status) {
  return status.state == State::DEGRADED;
}

inline bool isFaulted(const Status& status) {
  return status.state == State::FAULTED;
}

inline bool isRecovering(const Status& status) {
  return status.state == State::RECOVERING;
}

inline const char* stateName(State state) {
  switch (state) {
    case State::HEALTHY: return "HEALTHY";
    case State::DEGRADED: return "DEGRADED";
    case State::FAULTED: return "FAULTED";
    case State::RECOVERING: return "RECOVERING";
  }
  return "HEALTHY";
}

inline void setRecovering(Status& status, bool attempted) {
  status.state = State::RECOVERING;
  status.recoveryRebootAttempted = attempted;
}

inline void noteFailure(Status& status, uint32_t nowMs, const char* path,
                        const char* errorText, uint8_t severity = 1u) {
  if (severity == 0u) severity = 1u;
  if (severity > kFaultThreshold) severity = kFaultThreshold;

  if (status.lastFaultMs == 0 || (nowMs - status.lastFaultMs) > kFaultWindowMs) {
    status.faultScore = 0;
    status.firstFaultMs = nowMs;
  } else if (status.firstFaultMs == 0) {
    status.firstFaultMs = nowMs;
  }

  uint16_t nextScore = static_cast<uint16_t>(status.faultScore) + severity;
  status.faultScore = nextScore > 255u ? 255u : static_cast<uint8_t>(nextScore);
  status.lastFaultMs = nowMs;
  ++status.faultCount;
  copyStringCapped(errorText, status.lastError, sizeof(status.lastError));
  copyStringCapped(path, status.lastPath, sizeof(status.lastPath));

  if (status.faultScore >= kFaultThreshold) status.state = State::FAULTED;
  else if (status.faultScore >= kDegradedThreshold) status.state = State::DEGRADED;
  else status.state = State::HEALTHY;
}

inline void noteSuccess(Status& status, uint32_t nowMs) {
  status.lastSuccessMs = nowMs;
  if (status.state == State::FAULTED) return;

  if (status.faultScore > 0) --status.faultScore;
  if (status.faultScore == 0) {
    status.state = State::HEALTHY;
    status.firstFaultMs = 0;
    status.lastFaultMs = 0;
    status.lastError[0] = '\0';
    status.lastPath[0] = '\0';
    status.uploadBlocked = false;
  } else {
    status.state = State::DEGRADED;
  }
}

inline void markFaulted(Status& status, uint32_t nowMs, const char* path,
                        const char* errorText) {
  status.state = State::FAULTED;
  status.faultScore = kFaultThreshold;
  status.lastFaultMs = nowMs;
  if (status.firstFaultMs == 0) status.firstFaultMs = nowMs;
  ++status.faultCount;
  copyStringCapped(errorText, status.lastError, sizeof(status.lastError));
  copyStringCapped(path, status.lastPath, sizeof(status.lastPath));
}

inline void markRecoveryAttempted(Status& status, bool attempted) {
  status.recoveryRebootAttempted = attempted;
}

inline void setUploadBlocked(Status& status, bool blocked) {
  status.uploadBlocked = blocked;
}

}  // namespace logic::storage_fault

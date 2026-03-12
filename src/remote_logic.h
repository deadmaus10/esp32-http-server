#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "logic_utils.h"

namespace logic::remote {

constexpr uint32_t kPollIntervalMs = 2000UL;
constexpr uint32_t kPollPortalIntervalMs = 30000UL;
constexpr uint32_t kStartStopCooldownMs = 1500UL;
constexpr uint32_t kRebootCooldownMs = 60000UL;
constexpr uint32_t kPollMaxBackoffMs = 30000UL;

enum class Action {
  MeasureStartStop,
  Reboot,
};

enum class RejectionReason {
  None,
  Cooldown,
};

inline int parseRateFromParams(const char* params) {
  if (!params) return -1;
  const char* key = strstr(params, "rate=");
  if (!key) return -1;

  key += 5;
  char buf[12];
  size_t pos = 0;
  while (key[pos] != '\0' && key[pos] != '&' && pos + 1 < sizeof(buf)) {
    buf[pos] = key[pos];
    ++pos;
  }
  buf[pos] = '\0';

  int value = atoi(buf);
  return logic::isValidSps(value) ? value : -1;
}

inline RejectionReason actionRejection(Action action, uint32_t nowMs,
                                       uint32_t lastStartStopMs,
                                       uint32_t lastRebootMs) {
  if (action == Action::Reboot) {
    return (nowMs - lastRebootMs) < kRebootCooldownMs
             ? RejectionReason::Cooldown
             : RejectionReason::None;
  }

  return (nowMs - lastStartStopMs) < kStartStopCooldownMs
           ? RejectionReason::Cooldown
           : RejectionReason::None;
}

inline bool actionAllowed(Action action, uint32_t nowMs,
                          uint32_t lastStartStopMs,
                          uint32_t lastRebootMs) {
  return actionRejection(action, nowMs, lastStartStopMs, lastRebootMs) ==
         RejectionReason::None;
}

inline uint32_t nextPollIntervalAfterFailure(uint8_t failStreak) {
  uint32_t backoff = kPollIntervalMs;
  for (uint8_t i = 0; i < failStreak; ++i) {
    if (backoff >= kPollMaxBackoffMs / 2U) {
      backoff = kPollMaxBackoffMs;
      break;
    }
    backoff *= 2U;
  }
  if (backoff > kPollMaxBackoffMs) backoff = kPollMaxBackoffMs;
  return backoff;
}

inline uint32_t effectivePollInterval(uint32_t currentIntervalMs,
                                      bool portalClientConnected) {
  if (portalClientConnected && currentIntervalMs < kPollPortalIntervalMs) {
    return kPollPortalIntervalMs;
  }
  return currentIntervalMs;
}

inline bool isDuplicateCommandId(const char* incomingId,
                                 const char* lastCommandId) {
  return incomingId && lastCommandId &&
         strcmp(incomingId, lastCommandId) == 0;
}

}  // namespace logic::remote

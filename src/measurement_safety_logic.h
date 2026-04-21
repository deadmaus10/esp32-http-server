#pragma once

#include <stdint.h>

namespace logic::measurement_safety {

constexpr uint32_t kDeferredNetworkIntervalMs = 60000UL;

inline uint32_t deferredIntervalMs(bool measurementActive,
                                   uint32_t baseIntervalMs,
                                   uint32_t deferredMinIntervalMs = kDeferredNetworkIntervalMs) {
  if (!measurementActive) return baseIntervalMs;
  return (baseIntervalMs < deferredMinIntervalMs) ? deferredMinIntervalMs : baseIntervalMs;
}

inline uint32_t autoCycleTerminalFileIndex(uint64_t limitTicks, uint32_t fileSpanTicks) {
  if (limitTicks == 0 || fileSpanTicks == 0) return 0U;
  return static_cast<uint32_t>((limitTicks - 1ULL) / static_cast<uint64_t>(fileSpanTicks));
}

inline uint32_t clampFileIndexForAutoCycle(uint32_t rawFileIndex,
                                           uint64_t limitTicks,
                                           uint32_t fileSpanTicks) {
  const uint32_t terminal = autoCycleTerminalFileIndex(limitTicks, fileSpanTicks);
  return (rawFileIndex > terminal) ? terminal : rawFileIndex;
}

}  // namespace logic::measurement_safety

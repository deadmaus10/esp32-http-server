#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace logic::upload_retry {

constexpr uint32_t kManifestMagic = 0x55505259UL;  // "UPRY"
constexpr uint16_t kManifestVersion = 1;
constexpr uint32_t kPendingMagic = 0x41555253UL;   // "AURS"
constexpr uint16_t kPendingVersion = 1;
constexpr uint32_t kMaxTrackedParts = 8192U;
constexpr size_t kBitmapBytes = (kMaxTrackedParts + 7U) / 8U;
constexpr size_t kSessionDirBytes = 128U;
constexpr size_t kLastErrorBytes = 96U;

enum PendingFlags : uint16_t {
  kPendingWaitingUpload = 1u << 0,
  kPendingRestart = 1u << 1,
};

struct Manifest {
  uint32_t magic = kManifestMagic;
  uint16_t version = kManifestVersion;
  uint16_t reserved = 0;
  uint32_t finalFileIndex = 0;
  uint32_t attemptCount = 0;
  int32_t lastFailedIndex = -1;
  int32_t lastHttpCode = -1;
  char sessionDir[kSessionDirBytes] = {0};
  char lastError[kLastErrorBytes] = {0};
  uint8_t uploaded[kBitmapBytes] = {0};
};

struct PendingState {
  uint32_t magic = kPendingMagic;
  uint16_t version = kPendingVersion;
  uint16_t flags = 0;
  uint32_t finalFileIndex = 0;
  char sessionDir[kSessionDirBytes] = {0};
};

inline void copyStringCapped(const char* src, char* dest, size_t destSize) {
  if (!dest || destSize == 0) return;
  dest[0] = '\0';
  if (!src) return;
  size_t i = 0;
  for (; i + 1 < destSize && src[i] != '\0'; ++i) dest[i] = src[i];
  dest[i] = '\0';
}

inline bool isTrackedIndex(uint32_t idx) {
  return idx < kMaxTrackedParts;
}

inline bool isValidSessionDir(const char* sessionDir) {
  return sessionDir && sessionDir[0] != '\0';
}

inline void initManifest(Manifest& manifest, const char* sessionDir,
                         uint32_t finalFileIndex) {
  manifest = Manifest{};
  manifest.finalFileIndex = finalFileIndex;
  manifest.lastFailedIndex = -1;
  manifest.lastHttpCode = -1;
  copyStringCapped(sessionDir, manifest.sessionDir, sizeof(manifest.sessionDir));
}

inline void initPendingState(PendingState& state, const char* sessionDir,
                             uint32_t finalFileIndex,
                             bool waitingUpload,
                             bool pendingRestart) {
  state = PendingState{};
  state.finalFileIndex = finalFileIndex;
  state.flags = 0;
  if (waitingUpload) state.flags |= kPendingWaitingUpload;
  if (pendingRestart) state.flags |= kPendingRestart;
  copyStringCapped(sessionDir, state.sessionDir, sizeof(state.sessionDir));
}

inline bool validateManifest(const Manifest& manifest) {
  return manifest.magic == kManifestMagic &&
         manifest.version == kManifestVersion &&
         isValidSessionDir(manifest.sessionDir) &&
         isTrackedIndex(manifest.finalFileIndex);
}

inline bool validatePendingState(const PendingState& state) {
  return state.magic == kPendingMagic &&
         state.version == kPendingVersion &&
         isValidSessionDir(state.sessionDir) &&
         isTrackedIndex(state.finalFileIndex);
}

inline bool sessionMatches(const Manifest& manifest, const char* sessionDir,
                           uint32_t finalFileIndex) {
  return validateManifest(manifest) &&
         strcmp(manifest.sessionDir, sessionDir ? sessionDir : "") == 0 &&
         manifest.finalFileIndex == finalFileIndex;
}

inline bool pendingMatches(const PendingState& state, const char* sessionDir,
                           uint32_t finalFileIndex) {
  return validatePendingState(state) &&
         strcmp(state.sessionDir, sessionDir ? sessionDir : "") == 0 &&
         state.finalFileIndex == finalFileIndex;
}

inline size_t bitmapOffset(uint32_t idx) {
  return static_cast<size_t>(idx / 8U);
}

inline uint8_t bitmapMask(uint32_t idx) {
  return static_cast<uint8_t>(1u << (idx % 8U));
}

inline bool isUploaded(const Manifest& manifest, uint32_t idx) {
  if (!validateManifest(manifest) || idx > manifest.finalFileIndex || !isTrackedIndex(idx)) {
    return false;
  }
  return (manifest.uploaded[bitmapOffset(idx)] & bitmapMask(idx)) != 0;
}

inline bool markUploaded(Manifest& manifest, uint32_t idx) {
  if (!validateManifest(manifest) || idx > manifest.finalFileIndex || !isTrackedIndex(idx)) {
    return false;
  }
  manifest.uploaded[bitmapOffset(idx)] |= bitmapMask(idx);
  return true;
}

inline void beginPass(Manifest& manifest) {
  if (!validateManifest(manifest)) return;
  ++manifest.attemptCount;
}

inline void markFailure(Manifest& manifest, uint32_t idx, int httpCode,
                        const char* errorText) {
  if (!validateManifest(manifest)) return;
  manifest.lastFailedIndex = isTrackedIndex(idx) ? static_cast<int32_t>(idx) : -1;
  manifest.lastHttpCode = httpCode;
  copyStringCapped(errorText, manifest.lastError, sizeof(manifest.lastError));
}

inline uint32_t uploadedCount(const Manifest& manifest) {
  if (!validateManifest(manifest)) return 0;
  uint32_t total = 0;
  for (uint32_t idx = 0; idx <= manifest.finalFileIndex; ++idx) {
    if (isUploaded(manifest, idx)) ++total;
  }
  return total;
}

inline uint32_t remainingCount(const Manifest& manifest) {
  if (!validateManifest(manifest)) return 0;
  return (manifest.finalFileIndex + 1U) - uploadedCount(manifest);
}

inline bool isComplete(const Manifest& manifest) {
  return validateManifest(manifest) && remainingCount(manifest) == 0U;
}

inline bool pendingWaitingUpload(const PendingState& state) {
  return validatePendingState(state) &&
         (state.flags & kPendingWaitingUpload) != 0;
}

inline bool pendingRestart(const PendingState& state) {
  return validatePendingState(state) &&
         (state.flags & kPendingRestart) != 0;
}

}  // namespace logic::upload_retry

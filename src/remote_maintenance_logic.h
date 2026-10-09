#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>
namespace logic::maintenance {
inline uint32_t checksum(const void* data, size_t length) {
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  uint32_t hash = 2166136261U;
  for (size_t i = 0; i < length; ++i) hash = (hash ^ bytes[i]) * 16777619U;
  return hash;
}
inline bool allowed(bool measuring, bool taskRunning, bool autoPending,
                    bool waitingUpload, bool ota, bool rebootPending) {
  return !(measuring || taskRunning || autoPending || waitingUpload || ota || rebootPending);
}
inline bool sha256Hex(const char* value) {
  if (!value || strlen(value) != 64) return false;
  for (size_t i = 0; i < 64; ++i) {
    const char c = value[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
  }
  return true;
}
inline bool firmwarePath(const char* path) {
  if (!path || strncmp(path, "/firmware/", 10) != 0 || strlen(path) <= 10) return false;
  // Only fixed files under the production firmware directory; no URL parsing ambiguity.
  for (const char* p = path + 10; *p; ++p) {
    const char c = *p;
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.')) return false;
  }
  return strstr(path, "..") == nullptr;
}
inline bool contentLength(const char* value, uint32_t maximum, uint32_t& out) {
  if (!value || !*value) return false;
  uint32_t n = 0;
  for (const char* p = value; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    uint32_t digit = static_cast<uint32_t>(*p - '0');
    if (n > maximum / 10 || (n == maximum / 10 && digit > maximum % 10)) return false;
    n = n * 10 + digit;
  }
  if (!n) return false;
  out = n;
  return true;
}
}

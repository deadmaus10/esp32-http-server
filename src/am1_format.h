#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace logic::am1 {

constexpr size_t kNumSensors = 4;

struct Layout {
  uint16_t version = 0;
  uint8_t channels = 0;
  size_t headerSize = 0;
  size_t frameSize = 0;
  uint32_t timeScaleUs = 0;
};

#pragma pack(push, 1)
struct BaseHeader {
  char magic[4];
  uint16_t ver;
  uint16_t reserved;
  uint32_t start_epoch;
  uint32_t time_scale_us;
};

struct HeaderV1 {
  BaseHeader base;
  uint16_t sps0;
  uint16_t sps1;
  uint8_t gain0_code;
  uint8_t gain1_code;
  float sh0;
  float sh1;
  float fs0;
  float fs1;
  float off0;
  float off1;
};

struct HeaderV2 {
  BaseHeader base;
  uint16_t sps[kNumSensors];
  uint8_t gain_code[kNumSensors];
  float sh[kNumSensors];
  float fs[kNumSensors];
  float off[kNumSensors];
};

struct FrameV1 {
  uint32_t t_10us;
  int16_t raw0;
  int16_t raw1;
};

struct FrameV2 {
  uint32_t t_10us;
  int16_t raw[kNumSensors];
};
#pragma pack(pop)

static_assert(sizeof(BaseHeader) == 16, "AM1 base header size drifted");
static_assert(sizeof(HeaderV1) == 46, "AM1 V1 header size drifted");
static_assert(sizeof(HeaderV2) == 76, "AM1 V2 header size drifted");
static_assert(sizeof(FrameV1) == 8, "AM1 V1 frame size drifted");
static_assert(sizeof(FrameV2) == 12, "AM1 V2 frame size drifted");

inline bool inspectLayout(const uint8_t* data, size_t len, Layout& out) {
  if (!data || len < sizeof(BaseHeader)) return false;

  const BaseHeader* base = reinterpret_cast<const BaseHeader*>(data);
  if (memcmp(base->magic, "AM01", 4) != 0) return false;

  out.version = base->ver;
  out.channels = (base->ver <= 1U) ? 2U : static_cast<uint8_t>(kNumSensors);
  out.headerSize = (base->ver <= 1U) ? sizeof(HeaderV1) : sizeof(HeaderV2);
  out.frameSize = (base->ver <= 1U) ? sizeof(FrameV1) : sizeof(FrameV2);
  out.timeScaleUs = base->time_scale_us;

  return len >= out.headerSize;
}

}  // namespace logic::am1

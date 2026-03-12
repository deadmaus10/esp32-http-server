#include <string.h>

#include <unity.h>

#include "am1_format.h"

namespace am1 = logic::am1;

void test_am1_layout_sizes_match_firmware_format() {
  TEST_ASSERT_EQUAL_UINT32(16U, static_cast<uint32_t>(sizeof(am1::BaseHeader)));
  TEST_ASSERT_EQUAL_UINT32(46U, static_cast<uint32_t>(sizeof(am1::HeaderV1)));
  TEST_ASSERT_EQUAL_UINT32(76U, static_cast<uint32_t>(sizeof(am1::HeaderV2)));
  TEST_ASSERT_EQUAL_UINT32(8U, static_cast<uint32_t>(sizeof(am1::FrameV1)));
  TEST_ASSERT_EQUAL_UINT32(12U, static_cast<uint32_t>(sizeof(am1::FrameV2)));
}

void test_am1_detects_v1_layout() {
  am1::HeaderV1 header{};
  memcpy(header.base.magic, "AM01", 4);
  header.base.ver = 1;
  header.base.time_scale_us = 10;

  am1::Layout layout{};
  const bool ok = am1::inspectLayout(
    reinterpret_cast<const uint8_t*>(&header),
    sizeof(header),
    layout
  );

  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_UINT16(1U, layout.version);
  TEST_ASSERT_EQUAL_UINT8(2U, layout.channels);
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(sizeof(am1::HeaderV1)),
                           static_cast<uint32_t>(layout.headerSize));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(sizeof(am1::FrameV1)),
                           static_cast<uint32_t>(layout.frameSize));
  TEST_ASSERT_EQUAL_UINT32(10U, layout.timeScaleUs);
}

void test_am1_detects_v2_layout() {
  am1::HeaderV2 header{};
  memcpy(header.base.magic, "AM01", 4);
  header.base.ver = 2;
  header.base.time_scale_us = 10;

  am1::Layout layout{};
  const bool ok = am1::inspectLayout(
    reinterpret_cast<const uint8_t*>(&header),
    sizeof(header),
    layout
  );

  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_EQUAL_UINT16(2U, layout.version);
  TEST_ASSERT_EQUAL_UINT8(4U, layout.channels);
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(sizeof(am1::HeaderV2)),
                           static_cast<uint32_t>(layout.headerSize));
  TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(sizeof(am1::FrameV2)),
                           static_cast<uint32_t>(layout.frameSize));
}

void test_am1_rejects_invalid_or_truncated_headers() {
  am1::HeaderV2 header{};
  memcpy(header.base.magic, "BAD!", 4);
  header.base.ver = 2;

  am1::Layout layout{};
  TEST_ASSERT_FALSE(am1::inspectLayout(
    reinterpret_cast<const uint8_t*>(&header),
    sizeof(header),
    layout
  ));

  memcpy(header.base.magic, "AM01", 4);
  TEST_ASSERT_FALSE(am1::inspectLayout(
    reinterpret_cast<const uint8_t*>(&header),
    sizeof(am1::BaseHeader) - 1U,
    layout
  ));
  TEST_ASSERT_FALSE(am1::inspectLayout(
    reinterpret_cast<const uint8_t*>(&header),
    sizeof(am1::HeaderV2) - 1U,
    layout
  ));
}

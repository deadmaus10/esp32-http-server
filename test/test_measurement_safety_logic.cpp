#include <unity.h>

#include "measurement_safety_logic.h"

namespace measurement_safety = logic::measurement_safety;

void test_measurement_safety_clamps_network_intervals_while_active() {
  TEST_ASSERT_EQUAL_UINT32(
    60000UL,
    measurement_safety::deferredIntervalMs(true, 2000UL)
  );
  TEST_ASSERT_EQUAL_UINT32(
    60000UL,
    measurement_safety::deferredIntervalMs(true, 5000UL)
  );
}

void test_measurement_safety_keeps_long_intervals_unchanged() {
  TEST_ASSERT_EQUAL_UINT32(
    120000UL,
    measurement_safety::deferredIntervalMs(true, 120000UL)
  );
  TEST_ASSERT_EQUAL_UINT32(
    2000UL,
    measurement_safety::deferredIntervalMs(false, 2000UL)
  );
}

void test_measurement_safety_terminal_file_index_matches_4h_session() {
  constexpr uint64_t limitTicks = 4ULL * 60ULL * 60ULL * 100000ULL;
  constexpr uint32_t fileSpanTicks = 1800UL * 100000UL;
  TEST_ASSERT_EQUAL_UINT32(
    7U,
    measurement_safety::autoCycleTerminalFileIndex(limitTicks, fileSpanTicks)
  );
}

void test_measurement_safety_clamps_trailing_file_index_to_terminal_part() {
  constexpr uint64_t limitTicks = 4ULL * 60ULL * 60ULL * 100000ULL;
  constexpr uint32_t fileSpanTicks = 1800UL * 100000UL;
  TEST_ASSERT_EQUAL_UINT32(
    7U,
    measurement_safety::clampFileIndexForAutoCycle(8U, limitTicks, fileSpanTicks)
  );
  TEST_ASSERT_EQUAL_UINT32(
    7U,
    measurement_safety::clampFileIndexForAutoCycle(7U, limitTicks, fileSpanTicks)
  );
}

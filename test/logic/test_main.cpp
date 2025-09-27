#include <unity.h>

#include "logic_utils.h"

using logic::AlarmThresholds;

namespace {
constexpr AlarmThresholds kThresholds{3.0f, 3.5f, 22.0f, 21.5f};
}

void test_ads_lsb_mv_lookup() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.1875f, logic::adsLSB_mV(GAIN_TWOTHIRDS));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.1250f, logic::adsLSB_mV(GAIN_ONE));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0625f, logic::adsLSB_mV(GAIN_TWO));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.03125f, logic::adsLSB_mV(GAIN_FOUR));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.015625f, logic::adsLSB_mV(GAIN_EIGHT));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0078125f, logic::adsLSB_mV(GAIN_SIXTEEN));
}

void test_map_current_pct_to_mm_clamps_and_scales() {
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, logic::mapCurrentPctToMm(-10.0f, 40.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 20.0f, logic::mapCurrentPctToMm(50.0f, 40.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 60.0f, logic::mapCurrentPctToMm(150.0f, 40.0f, 20.0f));
}

void test_alarm_hysteresis_transitions() {
  // Normal -> under/over
  TEST_ASSERT_EQUAL_UINT8(ALARM_UNDER, logic::evalWithHyst(ALARM_NORMAL, 2.9f, kThresholds));
  TEST_ASSERT_EQUAL_UINT8(ALARM_OVER, logic::evalWithHyst(ALARM_NORMAL, 23.0f, kThresholds));
  TEST_ASSERT_EQUAL_UINT8(ALARM_NORMAL, logic::evalWithHyst(ALARM_NORMAL, 12.0f, kThresholds));

  // Under -> normal when above clear, or -> over when jumping high
  TEST_ASSERT_EQUAL_UINT8(ALARM_NORMAL, logic::evalWithHyst(ALARM_UNDER, 3.6f, kThresholds));
  TEST_ASSERT_EQUAL_UINT8(ALARM_OVER, logic::evalWithHyst(ALARM_UNDER, 23.0f, kThresholds));
  TEST_ASSERT_EQUAL_UINT8(ALARM_UNDER, logic::evalWithHyst(ALARM_UNDER, 3.2f, kThresholds));

  // Over -> normal or -> under when dropping far
  TEST_ASSERT_EQUAL_UINT8(ALARM_NORMAL, logic::evalWithHyst(ALARM_OVER, 21.0f, kThresholds));
  TEST_ASSERT_EQUAL_UINT8(ALARM_UNDER, logic::evalWithHyst(ALARM_OVER, 2.5f, kThresholds));
  TEST_ASSERT_EQUAL_UINT8(ALARM_OVER, logic::evalWithHyst(ALARM_OVER, 22.5f, kThresholds));
}

void test_gain_code_round_trip() {
  for (uint8_t code = 0; code <= 5; ++code) {
    adsGain_t gain = logic::codeToGain(code);
    TEST_ASSERT_EQUAL_UINT8(code, logic::gainToCode(gain));
  }
}

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_ads_lsb_mv_lookup);
  RUN_TEST(test_map_current_pct_to_mm_clamps_and_scales);
  RUN_TEST(test_alarm_hysteresis_transitions);
  RUN_TEST(test_gain_code_round_trip);
  return UNITY_END();
}


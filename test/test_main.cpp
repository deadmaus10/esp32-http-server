#include <unity.h>

void test_ads_lsb_mv_lookup();
void test_map_current_pct_to_mm_clamps_and_scales();
void test_alarm_hysteresis_transitions();
void test_gain_code_round_trip();

void test_meas_autocycle_ignores_prelimit_ticks();
void test_meas_autocycle_waits_for_upload_when_needed();
void test_meas_autocycle_retries_upload_then_restarts();
void test_meas_autocycle_skips_upload_wait_when_network_drops();
void test_meas_autocycle_immediate_restart_without_upload_wait();
void test_meas_autocycle_stop_failure_keeps_state_idle();

void test_remote_rate_parser_accepts_valid_rates();
void test_remote_rate_parser_rejects_invalid_values();
void test_remote_cooldowns_match_firmware_thresholds();
void test_remote_backoff_doubles_until_cap();
void test_remote_poll_interval_slows_with_portal_clients();
void test_remote_duplicate_id_detection();

void test_am1_layout_sizes_match_firmware_format();
void test_am1_detects_v1_layout();
void test_am1_detects_v2_layout();
void test_am1_rejects_invalid_or_truncated_headers();

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_ads_lsb_mv_lookup);
  RUN_TEST(test_map_current_pct_to_mm_clamps_and_scales);
  RUN_TEST(test_alarm_hysteresis_transitions);
  RUN_TEST(test_gain_code_round_trip);

  RUN_TEST(test_meas_autocycle_ignores_prelimit_ticks);
  RUN_TEST(test_meas_autocycle_waits_for_upload_when_needed);
  RUN_TEST(test_meas_autocycle_retries_upload_then_restarts);
  RUN_TEST(test_meas_autocycle_skips_upload_wait_when_network_drops);
  RUN_TEST(test_meas_autocycle_immediate_restart_without_upload_wait);
  RUN_TEST(test_meas_autocycle_stop_failure_keeps_state_idle);

  RUN_TEST(test_remote_rate_parser_accepts_valid_rates);
  RUN_TEST(test_remote_rate_parser_rejects_invalid_values);
  RUN_TEST(test_remote_cooldowns_match_firmware_thresholds);
  RUN_TEST(test_remote_backoff_doubles_until_cap);
  RUN_TEST(test_remote_poll_interval_slows_with_portal_clients);
  RUN_TEST(test_remote_duplicate_id_detection);

  RUN_TEST(test_am1_layout_sizes_match_firmware_format);
  RUN_TEST(test_am1_detects_v1_layout);
  RUN_TEST(test_am1_detects_v2_layout);
  RUN_TEST(test_am1_rejects_invalid_or_truncated_headers);

  return UNITY_END();
}

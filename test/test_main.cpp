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
void test_upload_retry_manifest_tracks_partial_success();
void test_upload_retry_manifest_completes_after_remaining_file_uploads();
void test_upload_retry_manifest_rejects_invalid_or_mismatched_state();
void test_upload_retry_pending_state_round_trips_flags();
void test_upload_retry_classifies_session_level_errors();
void test_upload_retry_backoff_caps_at_max_delay();
void test_storage_fault_transitions_from_healthy_to_degraded_to_faulted();
void test_storage_fault_success_recovers_degraded_state();
void test_storage_fault_window_resets_old_failures();
void test_storage_fault_recovery_marker_and_upload_block_flags();
void test_measurement_safety_clamps_network_intervals_while_active();
void test_measurement_safety_keeps_long_intervals_unchanged();
void test_measurement_safety_terminal_file_index_matches_4h_session();
void test_measurement_safety_clamps_trailing_file_index_to_terminal_part();

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
  RUN_TEST(test_upload_retry_manifest_tracks_partial_success);
  RUN_TEST(test_upload_retry_manifest_completes_after_remaining_file_uploads);
  RUN_TEST(test_upload_retry_manifest_rejects_invalid_or_mismatched_state);
  RUN_TEST(test_upload_retry_pending_state_round_trips_flags);
  RUN_TEST(test_upload_retry_classifies_session_level_errors);
  RUN_TEST(test_upload_retry_backoff_caps_at_max_delay);
  RUN_TEST(test_storage_fault_transitions_from_healthy_to_degraded_to_faulted);
  RUN_TEST(test_storage_fault_success_recovers_degraded_state);
  RUN_TEST(test_storage_fault_window_resets_old_failures);
  RUN_TEST(test_storage_fault_recovery_marker_and_upload_block_flags);
  RUN_TEST(test_measurement_safety_clamps_network_intervals_while_active);
  RUN_TEST(test_measurement_safety_keeps_long_intervals_unchanged);
  RUN_TEST(test_measurement_safety_terminal_file_index_matches_4h_session);
  RUN_TEST(test_measurement_safety_clamps_trailing_file_index_to_terminal_part);

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

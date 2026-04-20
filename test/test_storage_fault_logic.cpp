#include <unity.h>

#include "storage_fault_logic.h"

namespace storage_fault = logic::storage_fault;

void test_storage_fault_transitions_from_healthy_to_degraded_to_faulted() {
  storage_fault::Status status{};

  storage_fault::noteFailure(status, 1000UL, "/meas/part_0001.am1", "write_fail");
  TEST_ASSERT_TRUE(storage_fault::isDegraded(status));
  TEST_ASSERT_EQUAL_UINT8(1u, status.faultScore);

  storage_fault::noteFailure(status, 2000UL, "/meas/part_0001.am1", "write_fail");
  storage_fault::noteFailure(status, 3000UL, "/meas/part_0001.am1", "write_fail");
  storage_fault::noteFailure(status, 4000UL, "/meas/part_0001.am1", "write_fail");

  TEST_ASSERT_TRUE(storage_fault::isFaulted(status));
  TEST_ASSERT_EQUAL_UINT8(storage_fault::kFaultThreshold, status.faultScore);
  TEST_ASSERT_EQUAL_STRING("/meas/part_0001.am1", status.lastPath);
  TEST_ASSERT_EQUAL_STRING("write_fail", status.lastError);
}

void test_storage_fault_success_recovers_degraded_state() {
  storage_fault::Status status{};

  storage_fault::noteFailure(status, 1000UL, "/logs/log0.log", "open_fail");
  storage_fault::noteFailure(status, 2000UL, "/logs/log0.log", "open_fail");
  TEST_ASSERT_TRUE(storage_fault::isDegraded(status));

  storage_fault::noteSuccess(status, 3000UL);
  TEST_ASSERT_TRUE(storage_fault::isDegraded(status));
  storage_fault::noteSuccess(status, 4000UL);

  TEST_ASSERT_TRUE(storage_fault::isHealthy(status));
  TEST_ASSERT_EQUAL_UINT8(0u, status.faultScore);
  TEST_ASSERT_EQUAL_STRING("", status.lastError);
  TEST_ASSERT_EQUAL_STRING("", status.lastPath);
}

void test_storage_fault_window_resets_old_failures() {
  storage_fault::Status status{};

  storage_fault::noteFailure(status, 1000UL, "/meas/a.am1", "write_fail");
  storage_fault::noteFailure(status, 1000UL + storage_fault::kFaultWindowMs + 5UL,
                             "/meas/a.am1", "write_fail");

  TEST_ASSERT_TRUE(storage_fault::isDegraded(status));
  TEST_ASSERT_EQUAL_UINT8(1u, status.faultScore);
}

void test_storage_fault_recovery_marker_and_upload_block_flags() {
  storage_fault::Status status{};

  storage_fault::setRecovering(status, true);
  storage_fault::setUploadBlocked(status, true);

  TEST_ASSERT_TRUE(storage_fault::isRecovering(status));
  TEST_ASSERT_TRUE(status.recoveryRebootAttempted);
  TEST_ASSERT_TRUE(status.uploadBlocked);

  storage_fault::clear(status);
  TEST_ASSERT_TRUE(storage_fault::isHealthy(status));
  TEST_ASSERT_FALSE(status.recoveryRebootAttempted);
  TEST_ASSERT_FALSE(status.uploadBlocked);
}

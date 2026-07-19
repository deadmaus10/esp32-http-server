#include <unity.h>

#include "meas_autocycle_logic.h"

namespace autocycle = logic::meas_autocycle;

static constexpr uint32_t kTestLimitTicks =
  autocycle::effectiveLimitTicks(autocycle::kTargetLimitTicks);

void test_meas_autocycle_ignores_prelimit_ticks() {
  autocycle::State state{};
  const auto step = autocycle::processLimitReached(
    state,
    1000U,
    true,
    static_cast<uint64_t>(kTestLimitTicks) - 1ULL,
    kTestLimitTicks,
    true,
    true,
    false,
    false
  );

  TEST_ASSERT_FALSE(step.limitReached);
  TEST_ASSERT_FALSE(step.stopRequested);
  TEST_ASSERT_FALSE(state.pending);
}

void test_meas_autocycle_waits_for_upload_when_needed() {
  autocycle::State state{};
  const bool waitUpload = autocycle::shouldWaitForUpload(true, true, true, true);
  const auto step = autocycle::processLimitReached(
    state,
    5000U,
    true,
    kTestLimitTicks,
    kTestLimitTicks,
    waitUpload,
    true,
    false,
    false
  );

  TEST_ASSERT_TRUE(step.limitReached);
  TEST_ASSERT_TRUE(step.stopRequested);
  TEST_ASSERT_TRUE(step.waitingForUpload);
  TEST_ASSERT_TRUE(state.pending);
  TEST_ASSERT_TRUE(state.waitingUpload);
  TEST_ASSERT_EQUAL_UINT32(5000U, state.lastAttemptMs);
}

void test_meas_autocycle_retries_upload_then_restarts() {
  autocycle::State state{};
  autocycle::processLimitReached(
    state,
    1000U,
    true,
    kTestLimitTicks,
    kTestLimitTicks,
    true,
    true,
    false,
    false
  );

  auto step = autocycle::processPending(state, 20000U, true, false, false);
  TEST_ASSERT_FALSE(step.uploadAttempted);
  TEST_ASSERT_TRUE(state.waitingUpload);

  step = autocycle::processPending(state, 31000U, true, false, false);
  TEST_ASSERT_TRUE(step.uploadAttempted);
  TEST_ASSERT_FALSE(step.uploadCompleted);
  TEST_ASSERT_TRUE(state.waitingUpload);

  step = autocycle::processPending(state, 62000U, true, true, false);
  TEST_ASSERT_TRUE(step.uploadAttempted);
  TEST_ASSERT_TRUE(step.uploadCompleted);
  TEST_ASSERT_FALSE(state.waitingUpload);
  TEST_ASSERT_TRUE(state.pending);

  step = autocycle::processPending(state, 66000U, true, false, true);
  TEST_ASSERT_FALSE(step.restartAttempted);
  TEST_ASSERT_TRUE(state.pending);

  step = autocycle::processPending(state, 67000U, true, false, true);
  TEST_ASSERT_TRUE(step.restartAttempted);
  TEST_ASSERT_TRUE(step.restartStarted);
  TEST_ASSERT_FALSE(state.pending);
  TEST_ASSERT_FALSE(state.waitingUpload);
}

void test_meas_autocycle_skips_upload_wait_when_network_drops() {
  autocycle::State state{};
  autocycle::processLimitReached(
    state,
    1000U,
    true,
    kTestLimitTicks,
    kTestLimitTicks,
    true,
    true,
    false,
    false
  );

  auto step = autocycle::processPending(state, 5000U, false, false, false);
  TEST_ASSERT_FALSE(step.uploadAttempted);
  TEST_ASSERT_FALSE(state.waitingUpload);
  TEST_ASSERT_TRUE(state.pending);

  step = autocycle::processPending(state, 9000U, false, false, true);
  TEST_ASSERT_FALSE(step.restartAttempted);
  TEST_ASSERT_TRUE(state.pending);

  step = autocycle::processPending(state, 10000U, false, false, true);
  TEST_ASSERT_TRUE(step.restartAttempted);
  TEST_ASSERT_TRUE(step.restartStarted);
  TEST_ASSERT_FALSE(state.pending);
}

void test_meas_autocycle_immediate_restart_without_upload_wait() {
  autocycle::State state{};
  const bool waitUpload = autocycle::shouldWaitForUpload(false, true, true, true);
  const auto step = autocycle::processLimitReached(
    state,
    7000U,
    true,
    kTestLimitTicks,
    kTestLimitTicks,
    waitUpload,
    true,
    false,
    true
  );

  TEST_ASSERT_TRUE(step.limitReached);
  TEST_ASSERT_TRUE(step.stopRequested);
  TEST_ASSERT_TRUE(step.restartAttempted);
  TEST_ASSERT_TRUE(step.restartStarted);
  TEST_ASSERT_FALSE(state.pending);
}

void test_meas_autocycle_stop_failure_keeps_state_idle() {
  autocycle::State state{};
  const auto step = autocycle::processLimitReached(
    state,
    8000U,
    true,
    kTestLimitTicks,
    kTestLimitTicks,
    true,
    false,
    false,
    false
  );

  TEST_ASSERT_TRUE(step.limitReached);
  TEST_ASSERT_TRUE(step.stopRequested);
  TEST_ASSERT_TRUE(step.stopFailed);
  TEST_ASSERT_FALSE(state.pending);
}

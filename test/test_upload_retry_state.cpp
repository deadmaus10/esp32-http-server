#include <unity.h>

#include "upload_retry_state.h"

namespace upload_retry = logic::upload_retry;

void test_upload_retry_manifest_tracks_partial_success() {
  upload_retry::Manifest manifest{};
  upload_retry::initManifest(manifest, "/meas/sess_a", 2U);

  TEST_ASSERT_TRUE(upload_retry::validateManifest(manifest));
  TEST_ASSERT_EQUAL_UINT32(3U, upload_retry::remainingCount(manifest));

  upload_retry::beginPass(manifest);
  upload_retry::markUploaded(manifest, 0U);
  upload_retry::markUploaded(manifest, 2U);
  upload_retry::markFailure(manifest, 1U, 503, "server_busy");

  TEST_ASSERT_EQUAL_UINT32(1U, manifest.attemptCount);
  TEST_ASSERT_TRUE(upload_retry::isUploaded(manifest, 0U));
  TEST_ASSERT_FALSE(upload_retry::isUploaded(manifest, 1U));
  TEST_ASSERT_TRUE(upload_retry::isUploaded(manifest, 2U));
  TEST_ASSERT_EQUAL_INT32(1, manifest.lastFailedIndex);
  TEST_ASSERT_EQUAL_INT32(503, manifest.lastHttpCode);
  TEST_ASSERT_EQUAL_UINT32(1U, upload_retry::remainingCount(manifest));
  TEST_ASSERT_FALSE(upload_retry::isComplete(manifest));
}

void test_upload_retry_manifest_completes_after_remaining_file_uploads() {
  upload_retry::Manifest manifest{};
  upload_retry::initManifest(manifest, "/meas/sess_b", 3U);

  upload_retry::markUploaded(manifest, 0U);
  upload_retry::markUploaded(manifest, 1U);
  upload_retry::markUploaded(manifest, 2U);
  TEST_ASSERT_FALSE(upload_retry::isComplete(manifest));

  upload_retry::markUploaded(manifest, 3U);
  TEST_ASSERT_TRUE(upload_retry::isComplete(manifest));
  TEST_ASSERT_EQUAL_UINT32(0U, upload_retry::remainingCount(manifest));
}

void test_upload_retry_manifest_rejects_invalid_or_mismatched_state() {
  upload_retry::Manifest manifest{};
  upload_retry::initManifest(manifest, "/meas/sess_c", 4U);

  TEST_ASSERT_TRUE(upload_retry::sessionMatches(manifest, "/meas/sess_c", 4U));
  TEST_ASSERT_FALSE(upload_retry::sessionMatches(manifest, "/meas/sess_c", 5U));
  TEST_ASSERT_FALSE(upload_retry::sessionMatches(manifest, "/meas/sess_d", 4U));

  manifest.magic = 0;
  TEST_ASSERT_FALSE(upload_retry::validateManifest(manifest));
  TEST_ASSERT_FALSE(upload_retry::markUploaded(manifest, 0U));
}

void test_upload_retry_pending_state_round_trips_flags() {
  upload_retry::PendingState state{};
  upload_retry::initPendingState(state, "/meas/sess_d", 7U, true, true);

  TEST_ASSERT_TRUE(upload_retry::validatePendingState(state));
  TEST_ASSERT_TRUE(upload_retry::pendingMatches(state, "/meas/sess_d", 7U));
  TEST_ASSERT_TRUE(upload_retry::pendingWaitingUpload(state));
  TEST_ASSERT_TRUE(upload_retry::pendingRestart(state));
}

void test_upload_retry_classifies_session_level_errors() {
  TEST_ASSERT_TRUE(upload_retry::isSessionLevelUploadError(
    "connect_fail host=playground.martinfuri.hu port=443"));
  TEST_ASSERT_TRUE(upload_retry::isSessionLevelUploadError("offline"));
  TEST_ASSERT_TRUE(upload_retry::isSessionLevelUploadError("no status"));

  TEST_ASSERT_FALSE(upload_retry::isSessionLevelUploadError("server_busy"));
  TEST_ASSERT_FALSE(upload_retry::isSessionLevelUploadError("missing_file"));
  TEST_ASSERT_FALSE(upload_retry::isSessionLevelUploadError(""));
  TEST_ASSERT_FALSE(upload_retry::isSessionLevelUploadError(nullptr));
}

void test_upload_retry_backoff_caps_at_max_delay() {
  TEST_ASSERT_EQUAL_UINT32(30000U, upload_retry::retryDelayMsForAttempt(0U));
  TEST_ASSERT_EQUAL_UINT32(30000U, upload_retry::retryDelayMsForAttempt(1U));
  TEST_ASSERT_EQUAL_UINT32(60000U, upload_retry::retryDelayMsForAttempt(2U));
  TEST_ASSERT_EQUAL_UINT32(120000U, upload_retry::retryDelayMsForAttempt(3U));
  TEST_ASSERT_EQUAL_UINT32(240000U, upload_retry::retryDelayMsForAttempt(4U));
  TEST_ASSERT_EQUAL_UINT32(300000U, upload_retry::retryDelayMsForAttempt(5U));
  TEST_ASSERT_EQUAL_UINT32(300000U, upload_retry::retryDelayMsForAttempt(100U));
}

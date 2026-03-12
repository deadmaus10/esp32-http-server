#include <unity.h>

#include "remote_logic.h"

namespace remote = logic::remote;

void test_remote_rate_parser_accepts_valid_rates() {
  TEST_ASSERT_EQUAL_INT(920, remote::parseRateFromParams("rate=920"));
  TEST_ASSERT_EQUAL_INT(128, remote::parseRateFromParams("mode=x&rate=128&foo=1"));
  TEST_ASSERT_EQUAL_INT(3300, remote::parseRateFromParams("a=1&rate=3300"));
}

void test_remote_rate_parser_rejects_invalid_values() {
  TEST_ASSERT_EQUAL_INT(-1, remote::parseRateFromParams(nullptr));
  TEST_ASSERT_EQUAL_INT(-1, remote::parseRateFromParams("foo=1"));
  TEST_ASSERT_EQUAL_INT(-1, remote::parseRateFromParams("rate=123"));
  TEST_ASSERT_EQUAL_INT(-1, remote::parseRateFromParams("rate=abc"));
}

void test_remote_cooldowns_match_firmware_thresholds() {
  TEST_ASSERT_FALSE(remote::actionAllowed(remote::Action::MeasureStartStop, 2000U, 600U, 0U));
  TEST_ASSERT_TRUE(remote::actionAllowed(remote::Action::MeasureStartStop, 2100U, 600U, 0U));

  TEST_ASSERT_FALSE(remote::actionAllowed(remote::Action::Reboot, 61000U, 0U, 2000U));
  TEST_ASSERT_TRUE(remote::actionAllowed(remote::Action::Reboot, 62000U, 0U, 2000U));
}

void test_remote_backoff_doubles_until_cap() {
  TEST_ASSERT_EQUAL_UINT32(4000U, remote::nextPollIntervalAfterFailure(1U));
  TEST_ASSERT_EQUAL_UINT32(8000U, remote::nextPollIntervalAfterFailure(2U));
  TEST_ASSERT_EQUAL_UINT32(16000U, remote::nextPollIntervalAfterFailure(3U));
  TEST_ASSERT_EQUAL_UINT32(30000U, remote::nextPollIntervalAfterFailure(4U));
  TEST_ASSERT_EQUAL_UINT32(30000U, remote::nextPollIntervalAfterFailure(10U));
}

void test_remote_poll_interval_slows_with_portal_clients() {
  TEST_ASSERT_EQUAL_UINT32(30000U, remote::effectivePollInterval(2000U, true));
  TEST_ASSERT_EQUAL_UINT32(30000U, remote::effectivePollInterval(30000U, true));
  TEST_ASSERT_EQUAL_UINT32(8000U, remote::effectivePollInterval(8000U, false));
}

void test_remote_duplicate_id_detection() {
  TEST_ASSERT_TRUE(remote::isDuplicateCommandId("cmd-42", "cmd-42"));
  TEST_ASSERT_FALSE(remote::isDuplicateCommandId("cmd-42", "cmd-43"));
  TEST_ASSERT_FALSE(remote::isDuplicateCommandId(nullptr, "cmd-42"));
}

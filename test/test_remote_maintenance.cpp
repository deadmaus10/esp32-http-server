#include <unity.h>
#include "../src/remote_maintenance_logic.h"
using namespace logic::maintenance;
void test_remote_maintenance_requires_complete_standby() {
  TEST_ASSERT_TRUE(allowed(false, false, false, false, false, false));
  TEST_ASSERT_FALSE(allowed(true, false, false, false, false, false));
  TEST_ASSERT_FALSE(allowed(false, true, false, false, false, false));
  TEST_ASSERT_FALSE(allowed(false, false, true, false, false, false));
  TEST_ASSERT_FALSE(allowed(false, false, false, true, false, false));
  TEST_ASSERT_FALSE(allowed(false, false, false, false, true, false));
  TEST_ASSERT_FALSE(allowed(false, false, false, false, false, true));
}
void test_remote_maintenance_validates_firmware_path_and_digest() {
  TEST_ASSERT_TRUE(firmwarePath("/firmware/albasqueeze-2026.bin"));
  TEST_ASSERT_FALSE(firmwarePath("https://other.example/fw.bin"));
  TEST_ASSERT_FALSE(firmwarePath("/firmware/../config.php"));
  TEST_ASSERT_FALSE(firmwarePath("/firmware/a%2f.bin"));
  TEST_ASSERT_FALSE(firmwarePath("/firmware/a.bin\r\nHost: other"));
  TEST_ASSERT_FALSE(firmwarePath("/firmware/a.bin?redirect=1"));
  TEST_ASSERT_FALSE(firmwarePath("/firmware/"));
  TEST_ASSERT_TRUE(sha256Hex("0123456789abcdef0123456789ABCDEF0123456789abcdef0123456789abcdef"));
  TEST_ASSERT_FALSE(sha256Hex("0123456789abcdef"));
  TEST_ASSERT_FALSE(sha256Hex("z123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"));
}
void test_remote_maintenance_bounds_download_size() {
  uint32_t length = 99;
  TEST_ASSERT_TRUE(contentLength("1310720", 1310720, length));
  TEST_ASSERT_EQUAL_UINT32(1310720, length);
  TEST_ASSERT_FALSE(contentLength("1310721", 1310720, length));
  TEST_ASSERT_FALSE(contentLength("42949672960", 1310720, length));
  TEST_ASSERT_FALSE(contentLength("-1", 1310720, length));
  TEST_ASSERT_FALSE(contentLength("123junk", 1310720, length));
  TEST_ASSERT_FALSE(contentLength("", 1310720, length));
  TEST_ASSERT_FALSE(contentLength("0", 1310720, length));
}

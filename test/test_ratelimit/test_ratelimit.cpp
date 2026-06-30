// Host-native unit tests for the pure cross-trigger cooldown. pio test -e native

#include <unity.h>
#include "ratelimit.h"

using namespace ratelimit;

void test_first_event_always_allowed(void) {
  TEST_ASSERT_TRUE(allow(/*fired*/ false, 0, 0, 30000));
  TEST_ASSERT_TRUE(allow(false, 999999, 1000, 30000)); // lastMs irrelevant first time
}

void test_cooldown_zero_disables(void) {
  TEST_ASSERT_TRUE(allow(true, 1000, 1001, 0)); // 1ms after, but cd=0 -> allow
}

void test_blocked_within_cooldown(void) {
  TEST_ASSERT_FALSE(allow(true, 1000, 1000 + 29999, 30000)); // 1ms short
}

void test_allowed_at_and_after_cooldown(void) {
  TEST_ASSERT_TRUE(allow(true, 1000, 1000 + 30000, 30000)); // exactly elapsed
  TEST_ASSERT_TRUE(allow(true, 1000, 1000 + 50000, 30000));
}

void test_wraparound_safe(void) {
  // last just before the millis() wrap; now just after -> elapsed 0x20 ms only
  uint32_t last = 0xFFFFFFF0u, now = 0x0000000Fu; // elapsed = 0x1F = 31ms
  TEST_ASSERT_FALSE(allow(true, last, now, 30000)); // 31ms < 30s -> blocked
  TEST_ASSERT_TRUE(allow(true, last, now, 30));     // 31ms >= 30ms -> allowed
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_first_event_always_allowed);
  RUN_TEST(test_cooldown_zero_disables);
  RUN_TEST(test_blocked_within_cooldown);
  RUN_TEST(test_allowed_at_and_after_cooldown);
  RUN_TEST(test_wraparound_safe);
  return UNITY_END();
}

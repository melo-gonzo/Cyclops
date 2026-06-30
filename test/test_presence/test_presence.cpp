// Host-native unit tests for the pure presence tracker. pio test -e native

#include <unity.h>
#include "presence.h"

using namespace presence;

static const uint32_t W = 30000; // window

void test_touch_counts_distinct(void) {
  Entry t[4] = {};
  touch(t, 4, 0x0A000001, 1000, W);
  touch(t, 4, 0x0A000002, 1000, W);
  TEST_ASSERT_EQUAL_INT(2, activeCount(t, 4, 1000, W));
}

void test_touch_dedups_same_ip(void) {
  Entry t[4] = {};
  touch(t, 4, 0x0A000001, 1000, W);
  touch(t, 4, 0x0A000001, 2000, W); // same client again
  TEST_ASSERT_EQUAL_INT(1, activeCount(t, 4, 2000, W));
}

void test_expired_entries_drop_from_count(void) {
  Entry t[4] = {};
  touch(t, 4, 0x0A000001, 1000, W);
  touch(t, 4, 0x0A000002, 1000, W);
  // 40s later, one client refreshes; the other has expired.
  touch(t, 4, 0x0A000001, 41000, W);
  TEST_ASSERT_EQUAL_INT(1, activeCount(t, 4, 41000, W));
}

void test_expired_slot_is_reused_not_grown(void) {
  Entry t[2] = {};
  touch(t, 2, 0x0A000001, 1000, W);
  touch(t, 2, 0x0A000002, 1000, W);
  // Both expire; a third client should reuse a slot, not be lost.
  touch(t, 2, 0x0A000003, 60000, W);
  TEST_ASSERT_EQUAL_INT(1, activeCount(t, 2, 60000, W));
}

void test_full_table_evicts_oldest(void) {
  Entry t[2] = {};
  touch(t, 2, 0x0A000001, 1000, W);  // oldest
  touch(t, 2, 0x0A000002, 2000, W);
  touch(t, 2, 0x0A000003, 3000, W);  // all still fresh -> evict ip1
  // ip1 gone, ip2 + ip3 remain.
  TEST_ASSERT_EQUAL_INT(2, activeCount(t, 2, 3000, W));
  // Re-touching ip1 takes a slot again (evicting the now-oldest, ip2).
  touch(t, 2, 0x0A000001, 4000, W);
  TEST_ASSERT_EQUAL_INT(2, activeCount(t, 2, 4000, W));
}

void test_zero_ip_and_null_guards(void) {
  Entry t[2] = {};
  touch(t, 2, 0, 1000, W);            // ip 0 ignored
  TEST_ASSERT_EQUAL_INT(0, activeCount(t, 2, 1000, W));
  touch(nullptr, 2, 0x0A000001, 1000, W); // null table: no crash
  TEST_ASSERT_EQUAL_INT(0, activeCount(nullptr, 2, 1000, W));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_touch_counts_distinct);
  RUN_TEST(test_touch_dedups_same_ip);
  RUN_TEST(test_expired_entries_drop_from_count);
  RUN_TEST(test_expired_slot_is_reused_not_grown);
  RUN_TEST(test_full_table_evicts_oldest);
  RUN_TEST(test_zero_ip_and_null_guards);
  return UNITY_END();
}

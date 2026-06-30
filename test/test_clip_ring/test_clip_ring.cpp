// Host-native unit tests for the pure wrap-around clip-numbering math.
// pio test -e native. Pins the largest-gap oldest/newest logic the cap eviction
// and boot counter seed rely on, including the equidistant-tie fix.

#include <unity.h>
#include "clip_ring.h"

using namespace clipring;

static const uint32_t WRAP = 10000u;

void test_empty_returns_neg_one(void) {
  TEST_ASSERT_EQUAL_INT(-1, runEnd(nullptr, 0, WRAP, false));
  uint32_t idx[1] = {7};
  TEST_ASSERT_EQUAL_INT(-1, runEnd(idx, 0, WRAP, true)); // n=0 even with non-null
}

void test_single_element_oldest_equals_newest(void) {
  uint32_t idx[1] = {42};
  TEST_ASSERT_EQUAL_INT(0, runEnd(idx, 1, WRAP, false)); // oldest
  TEST_ASSERT_EQUAL_INT(0, runEnd(idx, 1, WRAP, true));  // newest
}

// A normal contiguous run {500,501,502,503,504,505}. Oldest=500 (index 0),
// newest=505 (index 5). The caller seeds the next number from newest+1.
void test_contiguous_run(void) {
  uint32_t idx[6] = {500, 501, 502, 503, 504, 505};
  int oldest = runEnd(idx, 6, WRAP, false);
  int newest = runEnd(idx, 6, WRAP, true);
  TEST_ASSERT_EQUAL_INT(0, oldest);          // 500
  TEST_ASSERT_EQUAL_INT(5, newest);          // 505
  // The shell computes the next file number as (newest's number + 1) % wrap.
  TEST_ASSERT_EQUAL_UINT32(506, (idx[newest] + 1) % WRAP);
}

// Run straddling the wrap: {9998,9999,0,1,2}. Oldest must be 9998 (index 0),
// newest must be 2 (index 4) - raw min/max would get this wrong.
void test_straddle_the_wrap(void) {
  uint32_t idx[5] = {9998, 9999, 0, 1, 2};
  int oldest = runEnd(idx, 5, WRAP, false);
  int newest = runEnd(idx, 5, WRAP, true);
  TEST_ASSERT_EQUAL_INT(0, oldest);                 // 9998
  TEST_ASSERT_EQUAL_INT(4, newest);                 // 2
  // next number wraps correctly: (2 + 1) % 10000 = 3
  TEST_ASSERT_EQUAL_UINT32(3, (idx[newest] + 1) % WRAP);
}

// The equidistant tie: {0, 5000} are exactly wrap/2 apart, so both entries tie
// on gap. The tie-break must resolve oldest and newest to DIFFERENT entries
// (newest prefers the larger number -> 5000; oldest prefers the smaller -> 0).
void test_equidistant_tie_resolves_to_different_entries(void) {
  uint32_t idx[2] = {0, 5000};
  int oldest = runEnd(idx, 2, WRAP, false);
  int newest = runEnd(idx, 2, WRAP, true);
  TEST_ASSERT_TRUE(oldest != newest);        // must not collapse to one entry
  TEST_ASSERT_EQUAL_INT(0, oldest);          // smaller number -> 0
  TEST_ASSERT_EQUAL_INT(1, newest);          // larger number  -> 5000
}

// Same tie but with the bigger number first in the array, to confirm the result
// follows the NUMBER (not the array position).
void test_equidistant_tie_order_independent(void) {
  uint32_t idx[2] = {5000, 0};
  int oldest = runEnd(idx, 2, WRAP, false);
  int newest = runEnd(idx, 2, WRAP, true);
  TEST_ASSERT_TRUE(oldest != newest);
  TEST_ASSERT_EQUAL_INT(1, oldest);          // 0 is at index 1
  TEST_ASSERT_EQUAL_INT(0, newest);          // 5000 is at index 0
}

// A run with a small burned-number gap inside it (704 skipped). It is still one
// contiguous block relative to the one big empty gap, so the ends are unchanged:
// oldest=700 (index 0), newest=707 (index 6).
void test_run_with_burned_gap(void) {
  uint32_t idx[7] = {700, 701, 702, 703, 705, 706, 707};
  int oldest = runEnd(idx, 7, WRAP, false);
  int newest = runEnd(idx, 7, WRAP, true);
  TEST_ASSERT_EQUAL_INT(0, oldest);          // 700
  TEST_ASSERT_EQUAL_INT(6, newest);          // 707
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_empty_returns_neg_one);
  RUN_TEST(test_single_element_oldest_equals_newest);
  RUN_TEST(test_contiguous_run);
  RUN_TEST(test_straddle_the_wrap);
  RUN_TEST(test_equidistant_tie_resolves_to_different_entries);
  RUN_TEST(test_equidistant_tie_order_independent);
  RUN_TEST(test_run_with_burned_gap);
  return UNITY_END();
}

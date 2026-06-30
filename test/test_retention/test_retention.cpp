// Host-native unit tests for the pure continuous-recording retention policy.
// pio test -e native

#include <unity.h>
#include "retention.h"

using namespace retention;

// helper: make a segment
static Seg mk(uint32_t num, uint32_t size, uint32_t mtime, bool vid) {
  Seg s; s.num = num; s.size = size; s.mtime = mtime; s.vid = vid; s.evict = false;
  return s;
}

// --- segmentsForMinutes ------------------------------------------------------
void test_segments_for_minutes(void) {
  TEST_ASSERT_EQUAL_UINT32(30, segmentsForMinutes(30, 60)); // 30min @60s = 30
  TEST_ASSERT_EQUAL_UINT32(120, segmentsForMinutes(30, 15)); // 30min @15s = 120
  TEST_ASSERT_EQUAL_UINT32(1, segmentsForMinutes(0, 60));   // 0 minutes floors to 1
  // segS=0 is an invalid config (clamped 15..600 upstream); the guard only
  // prevents a divide-by-zero, treating it as 1s -> 30*60/1.
  TEST_ASSERT_EQUAL_UINT32(1800, segmentsForMinutes(30, 0));
}

// --- sort oldest first (with num tiebreak) -----------------------------------
void test_sort_oldest_first(void) {
  Seg s[3] = { mk(2, 0, 200, false), mk(1, 0, 100, false), mk(3, 0, 100, false) };
  sortOldestFirst(s, 3);
  TEST_ASSERT_EQUAL_UINT32(1, s[0].num); // mtime 100, num 1
  TEST_ASSERT_EQUAL_UINT32(3, s[1].num); // mtime 100, num 3 (tiebreak)
  TEST_ASSERT_EQUAL_UINT32(2, s[2].num); // mtime 200
}

// --- per-stream minutes cap, counted separately ------------------------------
void test_minutes_cap_per_stream_oldest_first(void) {
  // 4 audio + 1 video; keep max 2 per stream -> evict 2 oldest audio, keep video.
  Seg s[5] = {
    mk(1, 100, 10, false), mk(2, 100, 20, false),
    mk(3, 100, 30, false), mk(4, 100, 40, false),
    mk(9, 100, 25, true),
  };
  int ev = planEvictions(s, 5, /*maxSegPerStream*/ 2, /*budget*/ 1ULL << 40, true);
  TEST_ASSERT_EQUAL_INT(2, ev);
  // sorted oldest-first; the two oldest audio (num 1,2) evicted, video kept.
  for (int i = 0; i < 5; i++) {
    if (s[i].num == 1 || s[i].num == 2) TEST_ASSERT_TRUE(s[i].evict);
    else TEST_ASSERT_FALSE(s[i].evict);
  }
}

// --- byte budget cap ---------------------------------------------------------
void test_byte_budget_evicts_oldest_until_under(void) {
  // 4 x 100B = 400B; budget 250B -> evict 2 oldest (down to 200B).
  Seg s[4] = { mk(1,100,10,false), mk(2,100,20,false), mk(3,100,30,false), mk(4,100,40,false) };
  int ev = planEvictions(s, 4, /*maxSeg*/ 1000, /*budget*/ 250, true);
  TEST_ASSERT_EQUAL_INT(2, ev);
  TEST_ASSERT_EQUAL_UINT64(200, survivingBytes(s, 4));
  TEST_ASSERT_EQUAL_INT(2, survivingCount(s, 4));
  TEST_ASSERT_TRUE(s[0].evict);  // oldest two gone (already sorted)
  TEST_ASSERT_TRUE(s[1].evict);
}

// --- both caps interact; whichever bites harder wins -------------------------
void test_both_caps_together(void) {
  // 3 audio x 100B. minutes cap 5 (no effect), budget 150 -> evict oldest 2.
  Seg s[3] = { mk(1,100,10,false), mk(2,100,20,false), mk(3,100,30,false) };
  int ev = planEvictions(s, 3, 5, 150, true);
  TEST_ASSERT_EQUAL_INT(2, ev);
  TEST_ASSERT_EQUAL_UINT64(100, survivingBytes(s, 3));
}

// --- enforce=false (recording paused) plans nothing --------------------------
void test_no_eviction_when_not_enforcing(void) {
  Seg s[3] = { mk(1,100,10,false), mk(2,100,20,false), mk(3,100,30,false) };
  int ev = planEvictions(s, 3, /*maxSeg*/ 0, /*budget*/ 0, /*enforce*/ false);
  TEST_ASSERT_EQUAL_INT(0, ev);
  TEST_ASSERT_EQUAL_UINT64(300, survivingBytes(s, 3));
}

// --- empty set is safe -------------------------------------------------------
void test_empty(void) {
  TEST_ASSERT_EQUAL_INT(0, planEvictions(nullptr, 0, 1, 1, true));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_segments_for_minutes);
  RUN_TEST(test_sort_oldest_first);
  RUN_TEST(test_minutes_cap_per_stream_oldest_first);
  RUN_TEST(test_byte_budget_evicts_oldest_until_under);
  RUN_TEST(test_both_caps_together);
  RUN_TEST(test_no_eviction_when_not_enforcing);
  RUN_TEST(test_empty);
  return UNITY_END();
}

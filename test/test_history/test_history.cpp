// Host-native unit tests for the pure plot-history decimation. pio test -e native

#include <unity.h>
#include "history.h"

using namespace history;

void test_available_buckets(void) {
  TEST_ASSERT_EQUAL_UINT32(100, availableBuckets(100, 0));
  TEST_ASSERT_EQUAL_UINT32(70, availableBuckets(100, 30));
  TEST_ASSERT_EQUAL_UINT32(0, availableBuckets(20, 30)); // end beyond what we have
}

// ring of 8, filled newest-last; w points one past the newest.
// values laid out so index math is checkable.
void test_decimate_identity_when_points_equals_want(void) {
  uint16_t ring[8] = {10, 20, 30, 40, 50, 60, 70, 80};
  uint16_t out[8] = {0};
  // w=8 (wrapped to 0 conceptually), want=4 newest, endBuckets=0 -> last 4: 50,60,70,80
  uint32_t n = decimateMaxPool(ring, 8, 8, 0, 4, 4, out);
  TEST_ASSERT_EQUAL_UINT32(4, n);
  TEST_ASSERT_EQUAL_UINT16(50, out[0]);
  TEST_ASSERT_EQUAL_UINT16(60, out[1]);
  TEST_ASSERT_EQUAL_UINT16(70, out[2]);
  TEST_ASSERT_EQUAL_UINT16(80, out[3]);
}

void test_decimate_max_pools_pairs(void) {
  uint16_t ring[8] = {10, 20, 30, 40, 50, 60, 70, 80};
  uint16_t out[8] = {0};
  // want=4 newest (50,60,70,80), points=2 -> peaks of [50,60] and [70,80]
  uint32_t n = decimateMaxPool(ring, 8, 8, 0, 4, 2, out);
  TEST_ASSERT_EQUAL_UINT32(2, n);
  TEST_ASSERT_EQUAL_UINT16(60, out[0]);
  TEST_ASSERT_EQUAL_UINT16(80, out[1]);
}

void test_decimate_endbuckets_shifts_window(void) {
  uint16_t ring[8] = {10, 20, 30, 40, 50, 60, 70, 80};
  uint16_t out[8] = {0};
  // skip newest 2 (70,80), then take 4 -> 30,40,50,60; points=4 identity
  uint32_t n = decimateMaxPool(ring, 8, 8, 2, 4, 4, out);
  TEST_ASSERT_EQUAL_UINT32(4, n);
  TEST_ASSERT_EQUAL_UINT16(30, out[0]);
  TEST_ASSERT_EQUAL_UINT16(60, out[3]);
}

void test_decimate_wraps_around_ring(void) {
  uint16_t ring[8] = {10, 20, 30, 40, 50, 60, 70, 80};
  uint16_t out[8] = {0};
  // w=2 means newest entry is index 1 (20); want=4 newest -> indices 6,7,0,1 = 70,80,10,20
  uint32_t n = decimateMaxPool(ring, 8, 2, 0, 4, 4, out);
  TEST_ASSERT_EQUAL_UINT32(4, n);
  TEST_ASSERT_EQUAL_UINT16(70, out[0]);
  TEST_ASSERT_EQUAL_UINT16(80, out[1]);
  TEST_ASSERT_EQUAL_UINT16(10, out[2]);
  TEST_ASSERT_EQUAL_UINT16(20, out[3]);
}

void test_decimate_points_clamped_to_want(void) {
  uint16_t ring[8] = {10, 20, 30, 40, 50, 60, 70, 80};
  uint16_t out[8] = {0};
  // asking for 100 points from 3 buckets -> only 3 produced
  uint32_t n = decimateMaxPool(ring, 8, 8, 0, 3, 100, out);
  TEST_ASSERT_EQUAL_UINT32(3, n);
}

void test_decimate_zero_cases(void) {
  uint16_t ring[8] = {1};
  uint16_t out[8] = {0};
  TEST_ASSERT_EQUAL_UINT32(0, decimateMaxPool(ring, 8, 8, 0, 0, 10, out)); // want 0
  TEST_ASSERT_EQUAL_UINT32(0, decimateMaxPool(ring, 8, 8, 0, 4, 0, out));  // points 0
  TEST_ASSERT_EQUAL_UINT32(0, decimateMaxPool(nullptr, 8, 8, 0, 4, 4, out)); // null src
}

void test_decimate_want_clamped_to_ring_len(void) {
  uint16_t ring[8] = {10, 20, 30, 40, 50, 60, 70, 80};
  uint16_t out[8] = {0};
  // want > ringLen must clamp to ringLen so the index math can't alias live data
  uint32_t n = decimateMaxPool(ring, 8, 8, 0, /*want*/ 20, /*points*/ 8, out);
  TEST_ASSERT_EQUAL_UINT32(8, n);
  for (int i = 0; i < 8; i++) TEST_ASSERT_EQUAL_UINT16((uint16_t)((i + 1) * 10), out[i]);
}

void test_decimate_endbuckets_clamped_to_ring_len(void) {
  uint16_t ring[8] = {10, 20, 30, 40, 50, 60, 70, 80};
  uint16_t out[8] = {0};
  // An oversized endBuckets must clamp to ringLen so the unsigned index math
  // can't underflow into a wrong slot. With endBuckets clamped to 8 (== ringLen),
  // (w + ringLen - endBuckets - want + b) % ringLen = (8 + 8 - 8 - 4 + b) % 8 =
  // (4 + b) % 8 -> indices 4,5,6,7 = 50,60,70,80 (the same window as endBuckets=0,
  // which is the well-defined fallback). Just assert it stays in range / defined.
  uint32_t n = decimateMaxPool(ring, 8, 8, /*endBuckets*/ 999, /*want*/ 4, /*points*/ 4, out);
  TEST_ASSERT_EQUAL_UINT32(4, n);
  TEST_ASSERT_EQUAL_UINT16(50, out[0]);
  TEST_ASSERT_EQUAL_UINT16(60, out[1]);
  TEST_ASSERT_EQUAL_UINT16(70, out[2]);
  TEST_ASSERT_EQUAL_UINT16(80, out[3]);
}

// ---- planAlignedPool: stable, grid-aligned decimation ----

// The newest source bucket included by a plan, as an absolute (monotonic) index.
static uint32_t newestIncluded(uint32_t gridIndex, uint32_t endBuckets,
                               const PoolPlan &p) {
  return gridIndex - endBuckets - p.extraEnd;
}

void test_plan_picks_integer_factor_and_uniform_want(void) {
  // want=100, points=30 -> f=ceil(100/30)=4, columns=min(100/4,30)=25, want=100.
  PoolPlan p = planAlignedPool(100, 30, 100000, 0, 1000);
  TEST_ASSERT_EQUAL_UINT32(25, p.columns);
  TEST_ASSERT_EQUAL_UINT32(100, p.want);          // 25 columns * 4 buckets
  TEST_ASSERT_EQUAL_UINT32(0, p.want % p.columns); // exact integer factor
}

void test_plan_right_edge_lands_on_factor_grid(void) {
  // f=4; the newest included bucket index must be a multiple of f for any clock.
  for (uint32_t g = 1000; g < 1000 + 20; g++) {
    PoolPlan p = planAlignedPool(100, 30, 100000, 0, g);
    uint32_t f = p.want / p.columns;
    TEST_ASSERT_EQUAL_UINT32(0, newestIncluded(g, 0, p) % f);
  }
}

void test_plan_is_stable_within_a_grid_cell(void) {
  // As the device clock advances WITHIN one f-cell, the aligned window must cover
  // the exact same absolute buckets (this is what kills the line shimmer).
  PoolPlan base = planAlignedPool(100, 30, 100000, 0, 1000);
  uint32_t f = base.want / base.columns; // 4
  uint32_t anchor = newestIncluded(1000, 0, base);
  for (uint32_t d = 0; d < f; d++) {
    PoolPlan p = planAlignedPool(100, 30, 100000, 0, 1000 + d);
    TEST_ASSERT_EQUAL_UINT32(base.columns, p.columns);
    TEST_ASSERT_EQUAL_UINT32(base.want, p.want);
    TEST_ASSERT_EQUAL_UINT32(anchor, newestIncluded(1000 + d, 0, p)); // unchanged
  }
}

void test_plan_scrolls_one_column_per_factor(void) {
  // Crossing an f-cell boundary advances the window by exactly f buckets (one col).
  PoolPlan a = planAlignedPool(100, 30, 100000, 0, 1000);
  uint32_t f = a.want / a.columns;
  PoolPlan b = planAlignedPool(100, 30, 100000, 0, 1000 + f);
  TEST_ASSERT_EQUAL_UINT32(f, newestIncluded(1000 + f, 0, b) - newestIncluded(1000, 0, a));
}

void test_plan_handles_zoom_endbuckets(void) {
  // With a non-zero end (zoom), alignment still lands on the grid and stays stable.
  PoolPlan p = planAlignedPool(100, 30, 100000, 37, 1000);
  uint32_t f = p.want / p.columns;
  TEST_ASSERT_EQUAL_UINT32(0, newestIncluded(1000, 37, p) % f);
}

void test_plan_zero_cases(void) {
  PoolPlan a = planAlignedPool(0, 30, 100, 0, 10);
  TEST_ASSERT_EQUAL_UINT32(0, a.columns);
  PoolPlan b = planAlignedPool(100, 0, 100, 0, 10);
  TEST_ASSERT_EQUAL_UINT32(0, b.columns);
  // Not enough buckets left after alignment -> nothing to draw, no crash.
  PoolPlan c = planAlignedPool(100, 30, 2, 0, 10);
  TEST_ASSERT_EQUAL_UINT32(0, c.columns);
}

void test_plan_columns_never_exceed_request_or_avail(void) {
  PoolPlan p = planAlignedPool(50, 30, 60, 0, 1000);
  TEST_ASSERT_TRUE(p.columns <= 30);
  TEST_ASSERT_TRUE(p.want <= 60); // fits in availability
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_available_buckets);
  RUN_TEST(test_decimate_identity_when_points_equals_want);
  RUN_TEST(test_decimate_max_pools_pairs);
  RUN_TEST(test_decimate_endbuckets_shifts_window);
  RUN_TEST(test_decimate_wraps_around_ring);
  RUN_TEST(test_decimate_points_clamped_to_want);
  RUN_TEST(test_decimate_zero_cases);
  RUN_TEST(test_decimate_want_clamped_to_ring_len);
  RUN_TEST(test_decimate_endbuckets_clamped_to_ring_len);
  RUN_TEST(test_plan_picks_integer_factor_and_uniform_want);
  RUN_TEST(test_plan_right_edge_lands_on_factor_grid);
  RUN_TEST(test_plan_is_stable_within_a_grid_cell);
  RUN_TEST(test_plan_scrolls_one_column_per_factor);
  RUN_TEST(test_plan_handles_zoom_endbuckets);
  RUN_TEST(test_plan_zero_cases);
  RUN_TEST(test_plan_columns_never_exceed_request_or_avail);
  return UNITY_END();
}

// Host-native unit tests for the pure motion frame-diff. pio test -e native

#include <unity.h>
#include <string.h>
#include "motion.h"

using namespace motion;

// 4x4 luma frame, 2x2 blocks -> a 2x2 grid (4 blocks).
static uint8_t flat[16];
static uint8_t map[2];

void test_no_change_no_blocks(void) {
  memset(flat, 100, sizeof(flat));
  uint8_t cur[16]; memcpy(cur, flat, 16);
  Result r = blockDiff(cur, flat, 4, 4, 2, 5, nullptr, map, sizeof(map));
  TEST_ASSERT_EQUAL_INT(2, r.bx);
  TEST_ASSERT_EQUAL_INT(2, r.by);
  TEST_ASSERT_EQUAL_INT(0, r.changed);
  TEST_ASSERT_EQUAL_INT(4, r.active);
  TEST_ASSERT_EQUAL_UINT32(0, r.diffSum);
}

void test_one_block_changes(void) {
  memset(flat, 100, sizeof(flat));
  uint8_t cur[16]; memcpy(cur, flat, 16);
  // bump the top-left 2x2 block (rows 0-1, cols 0-1) by +50 each px
  cur[0] += 50; cur[1] += 50; cur[4] += 50; cur[5] += 50;
  Result r = blockDiff(cur, flat, 4, 4, 2, 5, nullptr, map, sizeof(map));
  TEST_ASSERT_EQUAL_INT(1, r.changed);                 // only block (0,0)
  TEST_ASSERT_EQUAL_INT(4, r.active);
  TEST_ASSERT_EQUAL_UINT32(200, r.diffSum);            // 4 px * 50
  TEST_ASSERT_TRUE(map[0] & 0x01);                     // bit 0 set
  TEST_ASSERT_EQUAL_INT(0, map[0] & 0x0E);             // others clear
}

void test_threshold_respected(void) {
  memset(flat, 100, sizeof(flat));
  uint8_t cur[16]; memcpy(cur, flat, 16);
  // +4 per px in block (0,0): mean delta = 4, below threshold 5 -> not changed
  cur[0] += 4; cur[1] += 4; cur[4] += 4; cur[5] += 4;
  Result r = blockDiff(cur, flat, 4, 4, 2, 5, nullptr, map, sizeof(map));
  TEST_ASSERT_EQUAL_INT(0, r.changed);
  TEST_ASSERT_EQUAL_UINT32(16, r.diffSum); // still summed
}

void test_mask_excludes_block_from_counts_but_not_diffsum(void) {
  memset(flat, 100, sizeof(flat));
  uint8_t cur[16]; memcpy(cur, flat, 16);
  cur[0] += 50; cur[1] += 50; cur[4] += 50; cur[5] += 50; // block (0,0) changes
  uint8_t mask[2] = {0x01, 0x00};                          // mask block 0
  Result r = blockDiff(cur, flat, 4, 4, 2, 5, mask, map, sizeof(map));
  TEST_ASSERT_EQUAL_INT(0, r.changed);     // masked out
  TEST_ASSERT_EQUAL_INT(3, r.active);      // 4 blocks - 1 masked
  TEST_ASSERT_EQUAL_UINT32(200, r.diffSum); // whole-frame diff still counted
  TEST_ASSERT_EQUAL_UINT32(0, r.activeDiffSum); // ...but the masked block is NOT in the avg-stat sum
  TEST_ASSERT_EQUAL_INT(0, map[0] & 0x01);  // masked block not marked
}

void test_active_diffsum_counts_only_unmasked(void) {
  memset(flat, 100, sizeof(flat));
  uint8_t cur[16]; memcpy(cur, flat, 16);
  cur[2] += 50; cur[3] += 50; cur[6] += 50; cur[7] += 50; // block (1,0) changes (unmasked)
  uint8_t mask[2] = {0x01, 0x00};                          // mask block 0 (unchanged here)
  Result r = blockDiff(cur, flat, 4, 4, 2, 5, mask, map, sizeof(map));
  TEST_ASSERT_EQUAL_INT(1, r.changed);
  TEST_ASSERT_EQUAL_UINT32(200, r.diffSum);       // whole frame
  TEST_ASSERT_EQUAL_UINT32(200, r.activeDiffSum); // unmasked changed block's diff
}

void test_null_frames_safe(void) {
  Result r = blockDiff(nullptr, flat, 4, 4, 2, 5, nullptr, map, sizeof(map));
  TEST_ASSERT_EQUAL_INT(0, r.changed);
  TEST_ASSERT_EQUAL_INT(0, r.active);
}

void test_is_global(void) {
  TEST_ASSERT_TRUE(isGlobal(7, 10, 60));  // 70% > 60%
  TEST_ASSERT_FALSE(isGlobal(6, 10, 60)); // 60% not > 60%
  TEST_ASSERT_FALSE(isGlobal(5, 0, 60));  // no active blocks
}

void test_should_trigger(void) {
  TEST_ASSERT_TRUE(shouldTrigger(5, 3, false, 2, 2));   // enough + persisted
  TEST_ASSERT_FALSE(shouldTrigger(5, 3, true, 2, 2));   // global lighting shift
  TEST_ASSERT_FALSE(shouldTrigger(2, 3, false, 9, 2));  // below block threshold
  TEST_ASSERT_FALSE(shouldTrigger(5, 3, false, 1, 2));  // not persisted yet
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_no_change_no_blocks);
  RUN_TEST(test_one_block_changes);
  RUN_TEST(test_threshold_respected);
  RUN_TEST(test_mask_excludes_block_from_counts_but_not_diffsum);
  RUN_TEST(test_active_diffsum_counts_only_unmasked);
  RUN_TEST(test_null_frames_safe);
  RUN_TEST(test_is_global);
  RUN_TEST(test_should_trigger);
  return UNITY_END();
}

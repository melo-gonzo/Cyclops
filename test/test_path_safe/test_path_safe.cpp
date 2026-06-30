// Host-native unit tests for the pure clip-name path guard. pio test -e native

#include <unity.h>
#include "path_safe.h"

using namespace pathsafe;

void test_accepts_valid_names(void) {
  TEST_ASSERT_TRUE(validClipName("clip_00001_a.wav"));
  TEST_ASSERT_TRUE(validClipName("clip_00355_h.wav"));
  TEST_ASSERT_TRUE(validClipName("vclip_00010_m.avi"));
  TEST_ASSERT_TRUE(validClipName("vclip_00010_m.mot"));
}

void test_rejects_path_traversal(void) {
  TEST_ASSERT_FALSE(validClipName("../clip_1.wav"));
  TEST_ASSERT_FALSE(validClipName("clip_..wav"));     // contains ".."
  TEST_ASSERT_FALSE(validClipName("clip_1/.wav"));    // slash
  TEST_ASSERT_FALSE(validClipName("/etc/passwd"));
}

void test_rejects_wrong_prefix_or_extension(void) {
  TEST_ASSERT_FALSE(validClipName("cont_00001_a.wav")); // cont_ not downloadable
  TEST_ASSERT_FALSE(validClipName("clip_1.txt"));
  TEST_ASSERT_FALSE(validClipName("vclip_1.wav"));      // vclip must be avi/mot
  TEST_ASSERT_FALSE(validClipName("history.bin"));
  TEST_ASSERT_FALSE(validClipName("diag.log"));
}

void test_length_bounds(void) {
  TEST_ASSERT_FALSE(validClipName(""));
  TEST_ASSERT_FALSE(validClipName("a.wav"));            // too short / wrong prefix
  // 33+ chars rejected
  TEST_ASSERT_FALSE(validClipName("clip_000000000000000000000000.wav"));
}

void test_null_safe(void) {
  TEST_ASSERT_FALSE(validClipName(nullptr));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_accepts_valid_names);
  RUN_TEST(test_rejects_path_traversal);
  RUN_TEST(test_rejects_wrong_prefix_or_extension);
  RUN_TEST(test_length_bounds);
  RUN_TEST(test_null_safe);
  return UNITY_END();
}

// Host-native unit tests for the pure WAV header builder. pio test -e native

#include <unity.h>
#include <string.h>
#include "wav.h"

static uint8_t buf[64];

static uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

void test_riff_wave_fmt_data_tags(void) {
  wav::writeHeader(buf, 1000, 16000, 1, 16);
  TEST_ASSERT_EQUAL_INT(0, memcmp(buf, "RIFF", 4));
  TEST_ASSERT_EQUAL_INT(0, memcmp(buf + 8, "WAVEfmt ", 8));
  TEST_ASSERT_EQUAL_INT(0, memcmp(buf + 36, "data", 4));
}
void test_sizes_and_rates_16k_mono(void) {
  uint32_t pcm = 32000;
  wav::writeHeader(buf, pcm, 16000, 1, 16);
  TEST_ASSERT_EQUAL_UINT32(36 + pcm, le32(buf + 4));  // RIFF chunk size
  TEST_ASSERT_EQUAL_UINT32(16, le32(buf + 16));       // fmt chunk size
  TEST_ASSERT_EQUAL_UINT16(1, le16(buf + 20));        // PCM
  TEST_ASSERT_EQUAL_UINT16(1, le16(buf + 22));        // mono
  TEST_ASSERT_EQUAL_UINT32(16000, le32(buf + 24));    // sample rate
  TEST_ASSERT_EQUAL_UINT32(32000, le32(buf + 28));    // byte rate = 16000*1*2
  TEST_ASSERT_EQUAL_UINT16(2, le16(buf + 32));        // block align = 1*2
  TEST_ASSERT_EQUAL_UINT16(16, le16(buf + 34));       // bits
  TEST_ASSERT_EQUAL_UINT32(pcm, le32(buf + 40));      // data size
}
void test_byte_rate_and_align_stereo_8k(void) {
  wav::writeHeader(buf, 0, 8000, 2, 16);
  TEST_ASSERT_EQUAL_UINT16(4, le16(buf + 32));        // block align = 2ch*2B
  TEST_ASSERT_EQUAL_UINT32(8000 * 4, le32(buf + 28)); // byte rate
}
void test_header_is_44_bytes(void) {
  TEST_ASSERT_EQUAL_UINT32(44, wav::HEADER_BYTES);
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_riff_wave_fmt_data_tags);
  RUN_TEST(test_sizes_and_rates_16k_mono);
  RUN_TEST(test_byte_rate_and_align_stereo_8k);
  RUN_TEST(test_header_is_44_bytes);
  return UNITY_END();
}

// Host-native unit tests for the pure metrics encoding/table. pio test -e native

#include <unity.h>
#include "metrics.h"
#include "history.h"

using namespace metrics;

void test_table_is_complete_and_keyed(void) {
  TEST_ASSERT_EQUAL_INT(M_COUNT, count());
  for (int i = 0; i < count(); i++) {
    const Def &d = def(i);
    TEST_ASSERT_NOT_NULL(d.key);
    TEST_ASSERT_NOT_NULL(d.label);
    TEST_ASSERT_NOT_NULL(d.color);
    TEST_ASSERT_TRUE(d.scale > 0.0f); // decode divides by scale
  }
}

void test_encode_clamps_low(void) {
  // RSSI -120 dBm -> (−120+120)*1 = 0; anything lower saturates at 0.
  TEST_ASSERT_EQUAL_UINT16(0, encode(-120.0f, def(M_RSSI)));
  TEST_ASSERT_EQUAL_UINT16(0, encode(-200.0f, def(M_RSSI)));
  TEST_ASSERT_EQUAL_UINT16(0, encode(-5.0f, def(M_TEMP))); // negative temp -> 0
}

void test_encode_clamps_high(void) {
  // temp scale 10 -> 6553.5C saturates; heap KB scale 1 -> 65535 ceiling.
  TEST_ASSERT_EQUAL_UINT16(65535, encode(99999.0f, def(M_TEMP)));
  TEST_ASSERT_EQUAL_UINT16(65535, encode(100000.0f, def(M_HEAP)));
}

void test_encode_decode_roundtrip_temp(void) {
  uint16_t raw = encode(63.4f, def(M_TEMP)); // 63.4*10 = 634
  TEST_ASSERT_EQUAL_UINT16(634, raw);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 63.4f, decode(raw, def(M_TEMP)));
}

void test_encode_decode_roundtrip_rssi(void) {
  uint16_t raw = encode(-67.0f, def(M_RSSI)); // (-67+120)*1 = 53
  TEST_ASSERT_EQUAL_UINT16(53, raw);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -67.0f, decode(raw, def(M_RSSI)));
}

void test_fps_quarter_resolution(void) {
  uint16_t raw = encode(7.5f, def(M_CAMFPS)); // 75
  TEST_ASSERT_EQUAL_UINT16(75, raw);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 7.5f, decode(raw, def(M_CAMFPS)));
}

// A metric ring decodes correctly after decimation (integration with history.h).
void test_ring_decimate_then_decode(void) {
  const Def &d = def(M_TEMP);
  uint16_t ring[8];
  for (int i = 0; i < 8; i++) ring[i] = encode(60.0f + i, d); // 60..67 C
  uint16_t out[4] = {0};
  // newest 4 buckets (64,65,66,67), pooled to 2 -> peaks 65 and 67.
  uint32_t n = history::decimateMaxPool(ring, 8, 8, 0, 4, 2, out);
  TEST_ASSERT_EQUAL_UINT32(2, n);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 65.0f, decode(out[0], d));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 67.0f, decode(out[1], d));
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_table_is_complete_and_keyed);
  RUN_TEST(test_encode_clamps_low);
  RUN_TEST(test_encode_clamps_high);
  RUN_TEST(test_encode_decode_roundtrip_temp);
  RUN_TEST(test_encode_decode_roundtrip_rssi);
  RUN_TEST(test_fps_quarter_resolution);
  RUN_TEST(test_ring_decimate_then_decode);
  return UNITY_END();
}

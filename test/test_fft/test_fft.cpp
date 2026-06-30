// Host-native unit tests for the pure radix-2 FFT core.
// Run with: pio test -e native
//
// Pin the properties the spectrum view relies on: a DC input lands entirely in
// bin 0, a pure tone at an exact bin frequency peaks in that bin, Parseval-ish
// energy lands where expected, and the Hann window behaves (zero at the edges,
// unity at center).

#include <unity.h>
#include <math.h>
#include "fft.h"

static const float PI = 3.14159265358979f;

// DC input -> all energy in bin 0, nothing elsewhere.
void test_dc_goes_to_bin0(void) {
  const int N = 64;
  float re[N], im[N];
  for (int i = 0; i < N; i++) { re[i] = 2.0f; im[i] = 0.0f; }
  fft::forward(re, im, N);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f * N, fft::magnitude(re, im, 0));
  for (int k = 1; k < N / 2; k++)
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.0f, fft::magnitude(re, im, k));
}

// A cosine at exactly bin 4 peaks in bin 4 and is negligible in its neighbours.
void test_tone_peaks_in_its_bin(void) {
  const int N = 64;
  const int K = 4;
  float re[N], im[N];
  for (int i = 0; i < N; i++) { re[i] = cosf(2.0f * PI * K * i / N); im[i] = 0.0f; }
  fft::forward(re, im, N);
  float m4 = fft::magnitude(re, im, K);
  TEST_ASSERT_TRUE(m4 > fft::magnitude(re, im, K - 1) * 50.0f);
  TEST_ASSERT_TRUE(m4 > fft::magnitude(re, im, K + 1) * 50.0f);
  // Real cosine of unit amplitude -> bin magnitude N/2.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, N / 2.0f, m4);
}

// A higher-frequency tone lands in a higher bin than a lower one.
void test_higher_tone_higher_bin(void) {
  const int N = 128;
  auto peakBin = [&](int k) {
    float re[N], im[N];
    for (int i = 0; i < N; i++) { re[i] = cosf(2.0f * PI * k * i / N); im[i] = 0.0f; }
    fft::forward(re, im, N);
    int best = 0; float bm = -1;
    for (int b = 0; b < N / 2; b++) {
      float m = fft::magnitude(re, im, b);
      if (m > bm) { bm = m; best = b; }
    }
    return best;
  };
  TEST_ASSERT_EQUAL_INT(5, peakBin(5));
  TEST_ASSERT_EQUAL_INT(20, peakBin(20));
}

void test_hann_edges_and_center(void) {
  const int N = 64;
  float x[N];
  for (int i = 0; i < N; i++) x[i] = 1.0f;
  fft::hann(x, N);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, x[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, x[N - 1]);
  TEST_ASSERT_FLOAT_WITHIN(2e-2f, 1.0f, x[N / 2]);   // ~peak near the middle
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_dc_goes_to_bin0);
  RUN_TEST(test_tone_peaks_in_its_bin);
  RUN_TEST(test_higher_tone_higher_bin);
  RUN_TEST(test_hann_edges_and_center);
  return UNITY_END();
}

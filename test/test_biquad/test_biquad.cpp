// Host-native unit tests for the pure biquad low-pass core.
// Run with: pio test -e native
//
// These pin the properties we actually depend on: unity DC gain (so the filter
// can't shift levels/RMS), real attenuation of HF "static" near Nyquist, near
// pass-through well below the corner, stability under sustained input, and that
// an out-of-range cutoff is clamped rather than producing NaNs.

#include <unity.h>
#include <math.h>
#include "biquad.h"

using namespace biquad;

static const float FS = 16000.0f;
static const float Q  = 0.70710678f;

// Feed a constant DC level for long enough to settle; a low-pass must pass DC
// through at unity gain (otherwise it would bias every RMS reading).
void test_dc_gain_is_unity(void) {
  Coeffs c = lowpass(4000.0f, FS, Q);
  State s = {0, 0};
  float y = 0;
  for (int i = 0; i < 2000; i++) y = process(s, c, 1000.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 1000.0f, y);
}

// A full-rate Nyquist tone (+A,-A,+A,-A...) is the worst-case "static"; a
// 4kHz low-pass must knock it down hard.
void test_attenuates_nyquist(void) {
  Coeffs c = lowpass(4000.0f, FS, Q);
  State s = {0, 0};
  float sumSq = 0; int cnt = 0;
  for (int i = 0; i < 4000; i++) {
    float x = (i & 1) ? -1000.0f : 1000.0f;
    float y = process(s, c, x);
    if (i >= 2000) { sumSq += y * y; cnt++; }   // measure after settling
  }
  float rms = sqrtf(sumSq / cnt);
  TEST_ASSERT_TRUE(rms < 200.0f);               // >14dB down from 1000 input
}

// A tone well below the corner should come through close to full amplitude.
void test_passes_low_frequency(void) {
  Coeffs c = lowpass(4000.0f, FS, Q);
  State s = {0, 0};
  const float f = 300.0f;                       // far below 4kHz corner
  float peak = 0;
  for (int i = 0; i < 4000; i++) {
    float x = 1000.0f * sinf(2.0f * 3.14159265f * f * i / FS);
    float y = process(s, c, x);
    if (i >= 2000 && fabsf(y) > peak) peak = fabsf(y);
  }
  TEST_ASSERT_FLOAT_WITHIN(120.0f, 1000.0f, peak);
}

// A lower corner must attenuate a given mid tone more than a higher corner.
void test_lower_cutoff_attenuates_more(void) {
  const float f = 3000.0f;
  float rms[2]; float cut[2] = {1500.0f, 6000.0f};
  for (int k = 0; k < 2; k++) {
    Coeffs c = lowpass(cut[k], FS, Q);
    State s = {0, 0};
    float sumSq = 0; int cnt = 0;
    for (int i = 0; i < 4000; i++) {
      float x = 1000.0f * sinf(2.0f * 3.14159265f * f * i / FS);
      float y = process(s, c, x);
      if (i >= 2000) { sumSq += y * y; cnt++; }
    }
    rms[k] = sqrtf(sumSq / cnt);
  }
  TEST_ASSERT_TRUE(rms[0] < rms[1]);            // 1.5kHz corner cuts 3kHz harder
}

// Cutoff above Nyquist must be clamped, not produce NaN/Inf coefficients.
void test_cutoff_above_nyquist_is_finite(void) {
  Coeffs c = lowpass(99999.0f, FS, Q);
  TEST_ASSERT_TRUE(isfinite(c.b0) && isfinite(c.b1) && isfinite(c.b2));
  TEST_ASSERT_TRUE(isfinite(c.a1) && isfinite(c.a2));
}

// Sustained input stays bounded (no IIR blow-up).
void test_stable_under_sustained_input(void) {
  Coeffs c = lowpass(2000.0f, FS, Q);
  State s = {0, 0};
  float y = 0;
  for (int i = 0; i < 10000; i++) y = process(s, c, 5000.0f);
  TEST_ASSERT_TRUE(fabsf(y) < 6000.0f);
}

void test_reset_clears_state(void) {
  State s = {3.3f, -7.7f};
  reset(s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.z1);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.z2);
}

// --- high-pass --------------------------------------------------------------
// A high-pass must reject DC: a constant input settles toward zero.
void test_highpass_blocks_dc(void) {
  Coeffs c = highpass(150.0f, FS, Q);
  State s = {0, 0};
  float y = 0;
  for (int i = 0; i < 4000; i++) y = process(s, c, 1000.0f);
  TEST_ASSERT_TRUE(fabsf(y) < 5.0f);
}

// A high tone well above the corner passes near full amplitude.
void test_highpass_passes_high_frequency(void) {
  Coeffs c = highpass(150.0f, FS, Q);
  State s = {0, 0};
  const float f = 3000.0f;                      // far above 150Hz corner
  float peak = 0;
  for (int i = 0; i < 4000; i++) {
    float x = 1000.0f * sinf(2.0f * 3.14159265f * f * i / FS);
    float y = process(s, c, x);
    if (i >= 2000 && fabsf(y) > peak) peak = fabsf(y);
  }
  TEST_ASSERT_FLOAT_WITHIN(120.0f, 1000.0f, peak);
}

// A higher corner attenuates a given low tone more than a lower corner.
void test_highpass_higher_cutoff_attenuates_more(void) {
  const float f = 200.0f;
  float rms[2]; float cut[2] = {100.0f, 600.0f};
  for (int k = 0; k < 2; k++) {
    Coeffs c = highpass(cut[k], FS, Q);
    State s = {0, 0};
    float sumSq = 0; int cnt = 0;
    for (int i = 0; i < 4000; i++) {
      float x = 1000.0f * sinf(2.0f * 3.14159265f * f * i / FS);
      float y = process(s, c, x);
      if (i >= 2000) { sumSq += y * y; cnt++; }
    }
    rms[k] = sqrtf(sumSq / cnt);
  }
  TEST_ASSERT_TRUE(rms[1] < rms[0]);            // 600Hz corner cuts 200Hz harder
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_dc_gain_is_unity);
  RUN_TEST(test_attenuates_nyquist);
  RUN_TEST(test_passes_low_frequency);
  RUN_TEST(test_lower_cutoff_attenuates_more);
  RUN_TEST(test_cutoff_above_nyquist_is_finite);
  RUN_TEST(test_stable_under_sustained_input);
  RUN_TEST(test_reset_clears_state);
  RUN_TEST(test_highpass_blocks_dc);
  RUN_TEST(test_highpass_passes_high_frequency);
  RUN_TEST(test_highpass_higher_cutoff_attenuates_more);
  return UNITY_END();
}

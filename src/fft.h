// fft.h
//
// Pure, hardware-free radix-2 FFT + windowing - the functional core for the
// live spectrum view. No Arduino / ESP-IDF / FreeRTOS includes, no globals, no
// I/O; just float math on caller-owned arrays, so it unit-tests on the host
// (`pio test -e native`, see test/test_fft).
//
// We run only ~16 transforms/sec on 1024-point frames, so a plain iterative
// Cooley-Tukey FFT in float is far cheaper than its surroundings - no need for
// esp-dsp's assembly path, and this stays portable + testable. The audio task
// (imperative shell) owns the scratch buffers and calls these between chunks.

#pragma once
#include <math.h>

namespace fft {

// In-place Hann window over n real samples. Reduces spectral leakage so a tone
// between bins doesn't smear across the whole spectrum.
inline void hann(float *x, int n) {
  if (n < 2) return;
  for (int i = 0; i < n; i++) {
    float w = 0.5f * (1.0f - cosf(2.0f * 3.14159265358979f * i / (n - 1)));
    x[i] *= w;
  }
}

// In-place iterative radix-2 FFT. n MUST be a power of two; re[] and im[] are
// length n (im[] all-zero for real input). Forward transform, no scaling.
inline void forward(float *re, float *im, int n) {
  // Bit-reversal permutation.
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  // Butterflies, stage by stage.
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * 3.14159265358979f / len;
    float wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = a + len / 2;
        float tr = cr * re[b] - ci * im[b];
        float ti = cr * im[b] + ci * re[b];
        re[b] = re[a] - tr; im[b] = im[a] - ti;
        re[a] += tr;        im[a] += ti;
        float ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = ncr;
      }
    }
  }
}

// Magnitude of bin k from a completed transform: sqrt(re^2 + im^2).
inline float magnitude(const float *re, const float *im, int k) {
  return sqrtf(re[k] * re[k] + im[k] * im[k]);
}

} // namespace fft

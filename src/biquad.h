// biquad.h
//
// Pure, hardware-free biquad IIR filter - a second-order RBJ-cookbook low-pass.
//
// Like audio_dsp.h this is the "functional core": no Arduino / ESP-IDF /
// FreeRTOS includes, no globals, no I/O - just float math on its arguments.
// That makes it unit-testable on the host (`pio test -e native`, see
// test/test_biquad) and keeps the filter math in one audited place. The
// audio task (the imperative shell) owns one State + one Coeffs and calls
// process() per sample, recomputing coefficients only when the cutoff changes.
//
// Used to roll off the broadband HF hiss ("static") on the PDM mic before the
// signal feeds the live stream, the recorded clips, and the RMS level metric.

#pragma once
#include <math.h>

namespace biquad {

// Normalized transfer-function coefficients (a0 divided out).
struct Coeffs { float b0, b1, b2, a1, a2; };

// Per-stream filter memory (Transposed Direct Form II - two state words).
struct State { float z1, z2; };

// RBJ-cookbook low-pass coefficients for a given corner frequency.
//   cutoffHz   - -3dB corner, clamped to (1Hz, 0.49*sampleRate) so cos/sin
//                stay well-defined right up against Nyquist.
//   sampleRate - Hz (e.g. 16000).
//   q          - resonance; 0.70710678 (1/sqrt2) is maximally-flat Butterworth.
inline Coeffs lowpass(float cutoffHz, float sampleRate, float q) {
  const float hi = 0.49f * sampleRate;
  if (cutoffHz > hi) cutoffHz = hi;
  if (cutoffHz < 1.0f) cutoffHz = 1.0f;
  if (q < 0.1f) q = 0.1f;

  const float w0 = 2.0f * 3.14159265358979f * cutoffHz / sampleRate;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);
  const float a0 = 1.0f + alpha;

  Coeffs c;
  c.b0 = ((1.0f - cw) * 0.5f) / a0;
  c.b1 = (1.0f - cw) / a0;
  c.b2 = ((1.0f - cw) * 0.5f) / a0;
  c.a1 = (-2.0f * cw) / a0;
  c.a2 = (1.0f - alpha) / a0;
  return c;
}

// RBJ-cookbook high-pass coefficients (same args/clamping as lowpass). Used to
// roll off sub-bass rumble / HVAC / DC-thermal wander that the ~13Hz one-pole
// DC blocker leaves behind.
inline Coeffs highpass(float cutoffHz, float sampleRate, float q) {
  const float hi = 0.49f * sampleRate;
  if (cutoffHz > hi) cutoffHz = hi;
  if (cutoffHz < 1.0f) cutoffHz = 1.0f;
  if (q < 0.1f) q = 0.1f;

  const float w0 = 2.0f * 3.14159265358979f * cutoffHz / sampleRate;
  const float cw = cosf(w0);
  const float sw = sinf(w0);
  const float alpha = sw / (2.0f * q);
  const float a0 = 1.0f + alpha;

  Coeffs c;
  c.b0 = ((1.0f + cw) * 0.5f) / a0;
  c.b1 = (-(1.0f + cw)) / a0;
  c.b2 = ((1.0f + cw) * 0.5f) / a0;
  c.a1 = (-2.0f * cw) / a0;
  c.a2 = (1.0f - alpha) / a0;
  return c;
}

// One sample through the filter, advancing state. Transposed Direct Form II.
inline float process(State &s, const Coeffs &c, float x) {
  const float y = c.b0 * x + s.z1;
  s.z1 = c.b1 * x - c.a1 * y + s.z2;
  s.z2 = c.b2 * x - c.a2 * y;
  return y;
}

// Clear filter memory (use on enable, so a stale tail can't pop).
inline void reset(State &s) { s.z1 = 0.0f; s.z2 = 0.0f; }

} // namespace biquad

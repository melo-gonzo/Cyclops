// audio_dsp.h
//
// Pure, hardware-free decision logic for adaptive audio-clip triggering.
//
// This is the "functional core": no Arduino / ESP-IDF / FreeRTOS includes, no
// globals, no I/O - just math on its arguments. That makes it (a) unit-testable
// on the host (`pio test -e native`) without a board, and (b) the SINGLE source
// of truth for the threshold/floor math, which was previously duplicated and
// subtly inconsistent across four sites in audio_capture.cpp.
//
// audio_capture.cpp (the "imperative shell") owns the state and timing and calls
// these; tests in test/test_audio_dsp pin the behaviour.

#pragma once

#include <math.h> // sqrtf for the statistical threshold

namespace audsp {

// Adaptive threshold: noise floor x factor, clamped UP to an absolute minimum so
// a dead-quiet room (tiny floor) doesn't become a hair-trigger. Field-tuned via
// the /audio "min thr" control.
inline float adaptiveThreshold(float floor, float factor, float minThr) {
  float t = floor * factor;
  return t < minThr ? minThr : t;
}

// ---- Statistical ("Adaptive mean + sigma") threshold algorithm ----
// The floor x factor model above only knows the level of the floor, not how
// VARIABLE the room is, so a busy/noisy room over-triggers (the floor tracks the
// quiet valleys and factor x that sits below the routine spikes). The statistical
// model instead tracks the running mean AND standard deviation of the level and
// fires only on samples that are k sigma above the ambient DISTRIBUTION - so a
// busy room (large sigma) auto-raises the bar while a quiet room (small sigma)
// stays sensitive. One intuitive knob, k.

// One EWMA step of the running mean+variance. Updated from ALL samples (a slow
// alpha keeps rare loud events from dominating the estimate), so mean + k*sd
// reflects the whole ambient distribution, not just the valleys. Seeds on the
// first sample (mean==0). Standard exponentially-weighted mean/variance.
inline void adaptMeanVar(float &mean, float &var, float rms, float alpha) {
  if (mean <= 0.0f) { mean = rms; var = 0.0f; return; } // seed
  float delta = rms - mean;
  mean += alpha * delta;
  var = (1.0f - alpha) * (var + alpha * delta * delta);
}

// Statistical threshold = ambient mean + k standard deviations, clamped UP to the
// same absolute minimum as the factor model. Lower k = more sensitive.
inline float statisticalThreshold(float mean, float var, float k, float minThr) {
  float sd = var > 0.0f ? sqrtf(var) : 0.0f;
  float t = mean + k * sd;
  return t < minThr ? minThr : t;
}

// Clamp a (possibly stale NVS-loaded) clip length to the valid range the config
// handler enforces. The upper bound matters for memory safety: a clip_ms above
// the slot's capacity would overrun the WAV slot's memcpy. Lower bound is the
// same 500ms minimum the /audio/config handler rejects below.
inline unsigned long clampClipMs(unsigned long clipMs, unsigned long maxClipMs) {
  if (clipMs < 500) return 500;
  if (clipMs > maxClipMs) return maxClipMs;
  return clipMs;
}

// A manual override (>0) wins over the adaptive value; 0 means "use adaptive".
inline float effectiveThreshold(float adaptive, float manualThr) {
  return manualThr > 0.0f ? manualThr : adaptive;
}

// The threshold that actually gates triggering and is reported in the UI - the
// one value every site should agree on.
inline float triggerThreshold(float floor, float factor, float minThr,
                              float manualThr) {
  return effectiveThreshold(adaptiveThreshold(floor, factor, minThr), manualThr);
}

// Scale a per-update EMA rate that was tuned for a nominal update interval to the
// interval that ACTUALLY elapsed, so the effective time constant holds under
// scheduling jitter/contention (chunks/FFT frames don't arrive on an exact
// cadence). dtMs/nominalMs is clamped to [0.25, 8] so a one-off stall can't dump
// a huge step (and the result is capped at 1.0 — a full EMA step). nominalMs and
// the inputs are assumed > 0.
inline float scaleAlphaForDt(float alpha, float dtMs, float nominalMs) {
  float r = dtMs / nominalMs;
  if (r < 0.25f) r = 0.25f;
  if (r > 8.0f) r = 8.0f;
  float a = alpha * r;
  return a > 1.0f ? 1.0f : a;
}

// One step of the noise-floor estimator. Snaps to the input on the first sample
// (floor == 0). Otherwise an EMA: below the (adaptive) threshold it adapts fast
// downward (room got quieter) and slow upward (creeping ambient/thermal drift).
// ABOVE the threshold it applies a very slow "habituation" leak (alphaHab) so a
// *sustained* sound is gradually absorbed as the new background and stops
// re-triggering - the slow-rise/fast-fall noise-floor model used by VAD/anomaly
// detectors. alphaHab == 0 freezes the floor during loud sound (so a persistent
// real event never fades); a small alphaHab habituates over a tunable window.
// alphaDown >> alphaUp >= alphaHab keeps transients firing while steady sounds fade.
inline float adaptFloor(float floor, float rms, float adaptiveThr,
                        float alphaDown, float alphaUp, float alphaHab) {
  if (floor == 0.0f) return rms;
  if (rms < adaptiveThr) {
    float a = (rms < floor) ? alphaDown : alphaUp;
    return floor + a * (rms - floor);
  }
  return alphaHab > 0.0f ? floor + alphaHab * (rms - floor) : floor;
}

// Every gate that must hold to auto-start a clip: feature enabled, past the
// post-boot warm-up, past the retrigger hold-off, and loud enough.
inline bool shouldAutoTrigger(float rms, float triggerThr, bool autoEnabled,
                              bool warmedUp, bool holdoffElapsed) {
  return autoEnabled && warmedUp && holdoffElapsed && rms > triggerThr;
}

// Per-band spectral trigger reducer: the index of the band that most exceeds
// its own threshold, or -1 if none does. Same adaptive model as the scalar
// path, just applied per frequency bucket - the caller precomputes each band's
// threshold with adaptiveThreshold() and learns its floor with adaptFloor(),
// so this stays a trivial, testable reducer. Returns the band with the largest
// (magnitude - threshold) margin so the diag log/UI can name the culprit.
inline int hottestBandOverThreshold(const float *mag, const float *thr, int n) {
  int hot = -1;
  float best = 0.0f;
  for (int b = 0; b < n; b++) {
    float over = mag[b] - thr[b];
    if (over > best) { best = over; hot = b; }
  }
  return hot;
}

} // namespace audsp

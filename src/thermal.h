// thermal.h
//
// Pure, hardware-free thermal-governor logic. No Arduino includes ->
// unit-tested on the host (test/test_thermal).
//
// The ESP32-S3 has NO automatic thermal throttling, and the XIAO's octal SPI
// PSRAM is only rated to 65C ambient (the camera framebuffers + video ring live
// there, so video recording at high die temp corrupts PSRAM and wedges core 1).
// This implements the throttle the chip lacks: pause the PSRAM-heavy video load
// once the die crosses a cutoff. Audio (core 0, light PSRAM) keeps running.
// main.cpp is the shell that reads temperatureRead() and gates the recorder.
//
// The on-die sensor is noisy and spikes under bursty load, so we don't compare
// raw readings against the cutoff (a single hot sample would flap the recorder).
// Instead we smooth with a short moving average and apply ONE hard cutoff - the
// average is the anti-flap mechanism, so there's no resume dead band to reason
// about: video runs whenever the smoothed temp is below the cutoff and pauses
// when it's at or above it.

#pragma once

namespace thermal {

// Fixed-window simple moving average of die-temperature samples. The governor
// feeds one reading per sample tick; averaging over ~30s rejects a transient
// spike before it can trip the cutoff. The average is recomputed from the small
// ring each push (N is tiny), so there's no running-sum drift over long uptimes.
// Pure / host-testable.
template <int N>
class MovingAvg {
 public:
  // Push a sample; return the average over the last min(seen, N) samples.
  float push(float v) {
    buf_[head_] = v;
    head_ = (head_ + 1) % N;
    if (count_ < N) count_++;
    float s = 0.0f;
    for (int i = 0; i < count_; i++) s += buf_[i];
    return s / count_;
  }
  float value() const {
    if (count_ == 0) return 0.0f;
    float s = 0.0f;
    for (int i = 0; i < count_; i++) s += buf_[i];
    return s / count_;
  }
  int samples() const { return count_; }
  void reset() { count_ = 0; head_ = 0; }

 private:
  float buf_[N] = {0};
  int count_ = 0;
  int head_ = 0;
};

// Single hard cutoff: the protected (video) load is paused while the smoothed
// die temp is at or above the cutoff, and runs otherwise. No hysteresis - the
// moving average already prevents flapping.
inline bool overCutoff(float avgTempC, float cutoffC) {
  return avgTempC >= cutoffC;
}

} // namespace thermal

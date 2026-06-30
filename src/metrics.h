// metrics.h
//
// Pure, hardware-free definitions for the diagnostics time-series ("/graphs").
// No Arduino / ESP includes -> unit-tested on the host (test/test_metrics).
//
// One uint16 ring per metric, all sampled on the same time base, so any subset
// can be overlaid on a single normalized plot. This header is the single source
// of truth for the metric set, their fixed-point encoding (float<->uint16), and
// their presentation hints; both the firmware shell (metrics_svc.cpp) and the
// tests include it. The ring decimation lives in history.h (already tested).

#pragma once

#include <stdint.h>

namespace metrics {

// How a metric's per-bucket value is produced by the sampler:
//   GAUGE - snapshot of the latest reported value (temp, rssi, fps, heap, ...)
//   COUNT - number of events accumulated during the bucket (triggers, drops)
enum Kind { GAUGE = 0, COUNT = 1 };

// A metric is stored as a uint16: raw = clamp((value + offset) * scale).
// offset shifts negatives into range (e.g. RSSI dBm), scale sets resolution.
struct Def {
  const char *key;   // stable JSON key
  const char *label; // human label for the legend
  const char *color; // CSS color for the line
  const char *unit;  // unit shown in the legend
  float scale;       // fixed-point multiplier
  float offset;      // added before scaling (to lift negatives >= 0)
  uint8_t kind;      // Kind
};

// Metric ids. Order defines the ring/table layout; M_COUNT terminates it.
enum Id {
  M_TEMP = 0, // die temperature (C)
  M_RSSI,     // WiFi RSSI (dBm, negative)
  M_CAMFPS,   // camera capture fps
  M_NETFPS,   // stream delivery fps
  M_HEAP,     // free heap (KB)
  M_PSRAM,    // free PSRAM (KB)
  M_CLIENTS,  // distinct clients connected (recent authed requests)
  M_AUDIO,    // audio RMS level
  M_MOTION,   // changed motion blocks
  M_ATRIG,    // audio clip triggers (events/bucket)
  M_VTRIG,    // video clip triggers (events/bucket)
  M_STALL,    // camera-DMA stalls recovered (events/bucket)
  M_SDDROP,   // SD card dropouts (events/bucket)
  M_COUNT
};

// Single source of truth for the metric table. Function-local static so the
// header can be included by multiple translation units with no ODR clash.
inline const Def *table() {
  static const Def t[M_COUNT] = {
      {"temp_c",  "Temperature", "#ff6b6b", "\xC2\xB0""C",  10.0f, 0.0f,   GAUGE},
      {"rssi",    "WiFi RSSI",   "#4dd2ff", "dBm",          1.0f,  120.0f, GAUGE},
      {"cam_fps", "Camera FPS",  "#3d9df0", "fps",          10.0f, 0.0f,   GAUGE},
      {"net_fps", "Stream FPS",  "#7c6bff", "fps",          10.0f, 0.0f,   GAUGE},
      {"heap_kb", "Free heap",   "#41d18a", "KB",           1.0f,  0.0f,   GAUGE},
      {"psram_kb","Free PSRAM",  "#2bb673", "KB",           1.0f,  0.0f,   GAUGE},
      {"clients", "Clients",     "#d0d0d0", "",             1.0f,  0.0f,   GAUGE},
      {"audio_rms","Audio level","#ffd24d", "rms",          1.0f,  0.0f,   GAUGE},
      {"motion",  "Motion blocks","#ff9f43","blk",          1.0f,  0.0f,   GAUGE},
      {"audio_trig","Audio triggers","#ffb84d","/bkt",      1.0f,  0.0f,   COUNT},
      {"video_trig","Video triggers","#9b59ff","/bkt",      1.0f,  0.0f,   COUNT},
      {"cam_stall","Cam stalls", "#ff5e7e", "/bkt",         1.0f,  0.0f,   COUNT},
      {"sd_drop", "SD drops",    "#ff7043", "/bkt",         1.0f,  0.0f,   COUNT},
  };
  return t;
}

inline const Def &def(int id) { return table()[id]; }
inline int count() { return (int)M_COUNT; }

// float -> stored uint16 (saturating).
inline uint16_t encode(float value, const Def &d) {
  float s = (value + d.offset) * d.scale;
  if (s <= 0.0f) return 0;
  if (s >= 65535.0f) return 65535;
  return (uint16_t)(s + 0.5f);
}

// stored uint16 -> float (real units).
inline float decode(uint16_t raw, const Def &d) {
  return (float)raw / d.scale - d.offset;
}

} // namespace metrics

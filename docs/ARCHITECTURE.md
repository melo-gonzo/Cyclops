# Cyclops Firmware — Architecture

ESP32-S3 (Seeed XIAO Sense). PlatformIO. MJPEG camera + PDM audio with triggered/continuous clip capture, served over a built-in web UI. Source: `src/`.

## FreeRTOS task layout
Cores: `APP_CPU = 1`, `PRO_CPU = 0` (`src/main.cpp`). Design rule: **camera capture owns core 1 alone (it is the DMA pacemaker); every other I/O — audio, video writes, motion, network TX — runs on core 0** so capture never blocks.

| Task | Core | Prio | Role |
|------|------|------|------|
| `camera` | 1 | 5 | capture → assemble MJPEG chunk → tap JPEG into video ring |
| `stream` | 0 | 5 | fan one assembled chunk to all MJPEG viewers |
| `mjpeg_server` | 1 | 5 | accept stream clients; spawns camera+stream |
| `audio` | 0 | 4 | `i2s_read` 16ms blocks → ring + adaptive trigger detection |
| `audio_stream` | 0 | 3 | live-listen fan-out to `/audio/stream` subscribers |
| `sd_writer` | 0 | 2 | write clip WAVs to SD + SD mount/health state machine |
| `cont_aud` | 0 | 2 | continuous (NVR) audio segments |
| `video_rec` | 0 | 3 | write AVI clips (pre-roll + live) |
| `video_motion` | 0 | 1 | frame-diff motion analysis |
| `cont_prune` | 0 | 1 | prune continuous recordings to retention caps |
| `loop()` | 1 | — | 5s tick: thermal governor, metrics, WDT, stall/OTA/reset checks |

A WDT watches `loop()` and the camera task; a wedged capture (stuck in `esp_camera_fb_get`) is detected and the task reinit'd / chip reset (`main.cpp`).

## Data flow
**(a) Multi-client MJPEG** — the camera task captures one JPEG and assembles `CONTENT_TYPE + len + JPEG + BOUNDARY` **once** into a refcounted send-buffer slot, publishing `latestFrame`. The stream task snapshots that one slot and does a single `client->write()` per viewer, so **N viewers share one assembled chunk**. Back-pressure skips assembly while a consumer is behind; dead clients are dropped on failed write (`main.cpp`).

**(b) Audio ring → clips** — the audio task continuously fills a ~11s PCM ring in PSRAM. A trigger (adaptive RMS breach, per-band spectral, or HTTP) marks a position; after the post-roll the `[trigger−preroll, trigger+postroll]` window is copied into one of `CLIP_SLOTS` WAV buffers in PSRAM. `sd_writer` optionally persists clips to `/clips` on SD; an in-RAM `clip_index` avoids FAT scans on hot paths (`audio_capture.cpp`, `clip_index.h`).

**(c) Triggered MJPEG-AVI video clips** — the camera task taps ~1 frame/period into a PSRAM JPEG ring (pre-roll). Triggers: motion, audio cross-trigger, thermal-resume, or HTTP. `video_rec` opens an AVI on SD, drains the pre-roll from the ring, and writes live frames until the post-roll ends, taking `sdMutex` per frame so audio writes interleave. Audio↔video can cross-trigger each other, rate-limited (`video_record.cpp`, `ratelimit.h`).

## PSRAM vs SD
- **PSRAM (volatile, fast):** camera framebuffers + MJPEG send buffers, audio capture ring + WAV clip slots, video JPEG pre-roll ring, motion buffers, metrics rings. Reserves a 1MB floor (`PSRAM_RESERVE_BYTES`).
- **SD (persistent):** saved triggered clips (`/clips` WAV, `vclip_*.avi`), continuous NVR segments, history/spectrogram bins, diag log. `cont_record` streams straight to SD with rolling retention (minutes + MB cap).

## Thermal governor
`loop()` samples die temp every 5s; `thermal::MovingAvg<6>` (~30s) is compared against a single hard cutoff (`overCutoff`, no hysteresis — the average is the anti-flap). Over cutoff → **pause PSRAM-heavy video only; audio keeps running**, since the octal PSRAM goes out of timing spec at high temp and wedges core 1 (`thermal.h`, `video_record.cpp`).

## Module map (`src/`)
**Shell / app**
- `main.cpp` — boot, camera+MJPEG tasks, streaming fan-out, web routes, thermal/WDT/metrics tick
- `wifi_portal.*` — WiFi bring-up, NVS networks, mDNS, fallback AP, portal (`wifikeys.h` = secret seed)
- `web_auth.*` — HTTP Digest auth (stateless nonces), read-only vs write roles
- `ota_update.*` — ArduinoOTA push + HTTP `/update`, dual-partition
- `branding.h` — device name / AP SSID identity
- `web_ui.*` — shared front-end assets (`/ui.js`, `/caps`): the unified `EventPlot`
  timeline component + `buildNav()` top nav, used by every page on both boards
- `capabilities.h` — `HAS_AUDIO` / `HAS_SD` board flags that gate audio/SD features
- `board_stubs.cpp` — no-op SD/audio shims so the recorder links on boards without
  the audio module (camera-only)

**Audio** — `audio_capture.*` (PDM capture, ring, adaptive trigger, clips, live listen, spectrogram) · `cont_record.*` (continuous NVR segments + retention)

**Video / camera** — `video_record.*` (JPEG pre-roll ring, motion detection + a PSRAM motion-level timeline/event log, motion/audio/thermal-triggered AVI clips, thermal shell). Motion detection runs on **every** board (off the PSRAM frame ring, no SD); clip saving is gated by SD. · `cam_settings.*` (OV2640 runtime tuning + `/camera`) · `OV2640.*`, `camera_pins.h` (driver + pin map)

**Shared Event plot** — both boards render the same `EventPlot` (`web_ui` → `/ui.js`) against one data contract: a binary `Uint16` history series (`/audio/history`, `/video/motion/history`) + JSON events (`/audio/events`, `/video/motion/events`). XIAO plots audio level + triggers; camera-only boards plot motion + motion events. Both are titled "Event plot."

**Diagnostics** — `metrics_svc.*` (PSRAM time-series + `/graphs`) · `diag_log.*` (event log) · `presence.*` (distinct-IP client tracker)

**Pure / host-tested cores** (no Arduino includes, unit-tested in `test/`)
`audio_dsp.h` trigger math · `biquad.h` IIR filter · `fft.h` FFT/window · `history.h` plot decimation · `metrics.h` metric encoding · `thermal.h` governor math · `motion.h` frame-diff · `avi.h` AVI framing · `wav.h` WAV header · `retention.h` NVR eviction · `sd_health.h` mount/backoff FSM · `clip_ring.h` clip numbering · `clip_index.*` in-RAM clip index · `path_safe.h` filename guard · `http_parse.h` header/cookie parse · `ratelimit.h` cooldown

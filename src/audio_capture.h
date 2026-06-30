// audio_capture.h
//
// Continuous PDM-microphone capture for the XIAO ESP32S3 Sense, with
// clip extraction triggered by an adaptive RMS noise threshold or by HTTP
// (e.g. an external motion-detection algorithm watching the MJPEG stream).
//
// Audio runs in its own FreeRTOS task, fully decoupled from the video
// pipeline. The mic feeds a PSRAM ring buffer holding the last several
// seconds; when a trigger fires, the pre-roll already in the ring plus a
// post-roll captured after the trigger are assembled into a WAV clip held
// in PSRAM and served over HTTP. No SD card required.
//
// HTTP endpoints (registered via audioRegisterEndpoints):
//   GET /audio/trigger              start a clip; returns {"id":N,"ready_in_ms":M}
//   GET /audio/clip?id=N            download clip N as WAV (omit id = newest)
//   GET /audio/clips                JSON list of stored clips with age_ms
//   GET /audio/status               rms / noise floor / threshold / state
//   GET /audio/config?en=&clip_ms=&preroll_ms=&auto=&factor=&gain=&min_thr=   runtime tuning
//                                   (en=0 pauses the whole audio side: video-only mode)

#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <freertos/semphr.h>

// Allocates PSRAM buffers and installs the I2S PDM driver.
// Returns false on allocation/driver failure (server keeps running without audio).
bool audioInit();

// Starts the capture task. Call after audioInit() succeeds.
void audioStart();

// Registers the /audio/* HTTP handlers. Call before server.begin().
void audioRegisterEndpoints(WebServer &server);

// SD card state shared with the video recorder: one card, one mutex - all
// SD/SPI access from any module must hold sdGetMutex().
bool sdIsAvailable();
SemaphoreHandle_t sdGetMutex();

// Report an SD write/open failure from another module (e.g. the video recorder):
// marks the card down and schedules an immediate auto-remount, so a drop is
// recovered without waiting for the next liveness probe. Call without the mutex.
void sdNotifyWriteFailed();

// Combined-budget backstop. sdFreeMb() is the card's free space (MB) from the
// cached capacity figures. sdLowSpace() is true (sticky, with hysteresis) when
// free space is below the safety floor: continuous recording pauses NEW segments
// so the card can't fill and write-thrash; triggered event clips still save.
uint32_t sdFreeMb();
bool sdLowSpace();

// HTTP handlers must never block the single-threaded web server indefinitely on
// the SD bus: under continuous recording the recorder/list ops hold the mutex
// for a while, and one stuck handler stalls EVERY queued request (even no-SD
// ones). Server-thread SD ops wait at most this long, then answer 503 "sd busy".
#define SD_HTTP_LOCK_MS 600

// Bounded SD-mutex acquire for HTTP handlers (server thread): returns false if
// the card is busy after `ms`, so the handler can answer 503 instead of hanging
// the single-threaded server. Task-side code keeps using sdGetMutex() directly.
bool sdTryLock(uint32_t ms);
void sdUnlock();

// SD health telemetry for /diag: how many times a mounted card has dropped out
// (and been auto-remounted) this session, and seconds since the last drop
// (UINT32_MAX if it has never dropped).
uint32_t sdDrops();
uint32_t sdSecsSinceDrop();

// SPI clock (MHz) the card actually mounted at: 20 (fast) or 4 (conservative
// fallback for slow/fussy cards - ~5x slower, the prime suspect for slow clip
// loads). 0 when no card is mounted.
uint8_t sdMountClockMhz();

// Cross-trigger: start an audio clip alongside a video clip. Returns clip id.
uint32_t audioTriggerFromVideo();

// Factory reset: delete EVERY file under /clips (audio + video clips, motion
// sidecars, and the plot history). Holds the SD mutex like the other SD ops.
void audioSdWipeAll();

// Continuous (loop) audio recording: write back-to-back cont_NNNNN_a.wav
// segments straight to SD, bypassing the PSRAM clip slots. segMs = segment
// length (0 = leave unchanged). Retention is enforced by cont_record.cpp.
void audioContSet(bool on, uint32_t segMs);
String audioContJson(); // status fields (no braces): "a_on","a_rec","a_file","a_sd"

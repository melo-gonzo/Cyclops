// video_record.h
//
// Triggered video clip recording for the XIAO ESP32S3 Sense.
//
// camCB taps roughly one JPEG per frame period into a PSRAM ring (the
// pre-roll); on trigger a recorder task streams pre-roll plus live frames
// write-through to an MJPEG-AVI on the SD card, so only the pre-roll ever
// lives in RAM. Triggers: HTTP (/video/trigger, e.g. an external motion
// algorithm), a cross-trigger from the audio clip subsystem, and an
// optional on-device motion detector.
//
// HTTP endpoints (registered via videoRegisterEndpoints):
//   GET /video/trigger             start a clip; returns {"id":N,"ready_in_ms":M}
//   GET /video/status              recorder/ring/motion state
//   GET /video/config?en=&fps=&clip_ms=&preroll_ms=&on_audio=&trig_audio=&vmax=
//                     &motion=&motion_blocks=&motion_diff=&motion_block=
//                     &motion_mask=<hex>
//   GET /video/list                JSON list of vclip_*.avi files on SD
//   GET /video/clear               delete all vclip_* files (incl. sidecars)
//   GET /video/motion              live changed-block map for debug overlays;
//                                  polling keeps analysis running ~10s even
//                                  with auto-trigger off

#pragma once

#include <Arduino.h>
#include <WebServer.h>

#define VID_TRIG_HTTP   0
#define VID_TRIG_MOTION 1
#define VID_TRIG_AUDIO  2

// Allocates the PSRAM tap ring (if the master toggle is on) and loads NVS
// settings. w/h must match the camera frame size (written into AVI headers).
// Call after audioInit() so the SD card is already mounted.
bool videoInit(uint16_t w, uint16_t h);

// Starts the recorder task. Call after videoInit() succeeds.
void videoStart();

// Registers the /video/* HTTP handlers. Call before server.begin().
void videoRegisterEndpoints(WebServer &server);

// Called from camCB for every captured JPEG; copies ~1 frame per period
// into the tap ring. Never blocks: drops the tap if the ring can't take it.
void videoTapFrame(const uint8_t *jpg, size_t len);

// Begin a clip (or join the one in flight). Returns clip id, 0 = refused
// (disabled, no SD, or cross-trigger gated off).
uint32_t videoNotifyTrigger(uint8_t source);

// True while the tap ring wants frames - camCB uses this to decide whether
// it may park itself when no MJPEG viewers are connected.
bool videoCaptureActive();

// Software thermal governor. Call periodically with the die temperature; video
// recording (PSRAM-heavy -> wedges core 1 when the octal PSRAM is over temp) is
// paused above the cutoff and resumed once cooled. Audio is unaffected. Returns
// true if the paused state changed (so the caller can log the transition).
bool videoThermalUpdate(float dieTempC);
bool videoThermalPausedNow();

// Callback invoked when video recording is switched on at runtime, so the
// camera task can be woken if it parked while recording was off.
void videoSetWakeCallback(void (*fn)());

// Continuous (loop) video recording: drive the recorder back-to-back into
// cont_NNNNN_v.avi segments (no pre-roll), bringing up the tap ring/camera if
// needed. segMs = segment length (0 = leave unchanged). Triggered clips are
// suppressed while continuous is on. Retention is enforced by cont_record.cpp.
void videoContSet(bool on, uint32_t segMs);
String videoContJson(); // status fields (no braces): "v_on","v_rec","v_file","v_sd"

// Update frame dimensions after a runtime resolution change. Flushes the
// tap ring (frames with the old dimensions must not land in a new clip)
// and resets the motion detector. Returns false while a clip is being
// recorded - the caller should refuse the resolution change.
bool videoSetDims(uint16_t w, uint16_t h);

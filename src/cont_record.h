// cont_record.h
//
// Continuous (loop / NVR-style) recording, layered on top of the trigger-based
// clip system. Records back-to-back fixed-length segments of audio, video, or
// both, straight to SD - bypassing the audio PSRAM clip slots - and prunes the
// oldest so a rolling window is kept. Two retention caps apply together: keep
// at most N minutes per stream AND keep total continuous files under an MB
// budget, whichever is hit first.
//
// The per-stream writers live in audio_capture.cpp (audioContSet/audioContJson)
// and video_record.cpp (videoContSet/videoContJson); this module owns the
// shared settings (NVS "cont"), the rolling pruner, and the /rec dashboard.
//
// HTTP endpoints (XIAO only):
//   GET /rec               management page
//   GET /rec/status        config + per-stream state + disk usage
//   GET /rec/config?aud=&vid=&seg=&min=&mb=   update settings (persisted)
//   GET /rec/clear         delete all continuous (cont_*) files

#pragma once

#if defined(CAMERA_MODEL_XIAO_ESP32S3)
#include <WebServer.h>

// Load settings from NVS, apply them to the writers, and start the pruner.
// Call after audioStart()/videoStart() and before the server begins.
void contInit();

// Register the /rec handlers. Call before server.begin().
void contRegisterEndpoints(WebServer &server);
#endif

// capabilities.h
//
// Board feature flags. In this firmware the PDM microphone and the SD-card layer
// (SPI, CS on GPIO21) are wired for the Seeed XIAO ESP32S3 Sense only; the
// AI-Thinker ESP32-CAM has neither, so audio (clips, monitoring) and SD-backed
// features (saved clips, continuous recording) are gated off there. Everything
// else runs on every board: live MJPEG, the camera page, metrics/graphs,
// diagnostics, WiFi, OTA, and MOTION DETECTION (it runs off the PSRAM frame ring
// with no SD and feeds the Graphs Motion timeline).
//
// CAMERA_MODEL_XIAO_ESP32S3 comes from a -D build flag (platformio.ini), so these
// resolve correctly regardless of include order.

#pragma once

#ifdef CAMERA_MODEL_XIAO_ESP32S3
#define HAS_AUDIO 1
#define HAS_SD    1
#else
#define HAS_AUDIO 0   // no PDM microphone on this board
#define HAS_SD    0   // SD slot not wired for this board in this firmware
#endif

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

#if defined(CAMERA_MODEL_XIAO_ESP32S3)
#define HAS_AUDIO 1
#define HAS_SD    1
#elif defined(CAMERA_MODEL_ESP32P4)
// ESP32-P4-NANO port (firmware-p4/, combined arduino+esp-idf, MIPI-CSI OV5647).
// Video runs via esp_video, not the DVP esp_camera path — see the camera
// abstraction. SD is the NANO's SDMMC slot: audio_capture.cpp's shared SD layer
// with an SDMMC mount backend + the compat SD->SD_MMC alias. Audio is the
// NANO's analog mic through an ES8311 codec (p4_audio.cpp backend; the
// ring/trigger/clip/web pipeline is the same code the XIAO runs).
#define HAS_AUDIO 1   // ES8311 codec + onboard mic
#define HAS_SD    1   // microSD on SDMMC SLOT-0
#else
#define HAS_AUDIO 0   // no PDM microphone on this board (AI-Thinker ESP32-CAM)
#define HAS_SD    0   // SD slot not wired for this board in this firmware
#endif

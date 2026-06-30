// cam_settings.h
//
// Runtime camera sensor tuning, modeled on the canonical esp32-camera
// CameraWebServer example: a /camera settings page plus var/val config
// endpoints for everything the OV2640 exposes (resolution, JPEG quality,
// brightness/contrast/saturation, white balance, exposure, gain, pixel
// pipeline toggles, mirror/flip). Changed values persist in NVS and are
// re-applied at boot on top of the project defaults.
//
// HTTP endpoints (registered via camRegisterEndpoints):
//   GET /camera                    settings page
//   GET /camera/status             JSON of all current sensor values
//   GET /camera/config?var=&val=   set one control (canonical var names)
//   GET /camera/reset              clear overrides, restore code defaults

#pragma once

#include <Arduino.h>
#include <WebServer.h>

// Saved sensor master clock in MHz (the user's /camera override, or the
// board default). Call BEFORE esp_camera init and feed the result into
// camera_config_t.xclk_freq_hz: changing xclk at runtime wedges the
// ESP32-S3 capture DMA, so the setting only takes effect at boot.
int camSavedXclkMhz();

// Applies the project's sensor defaults plus any user overrides saved in
// NVS (including a saved resolution). Call once in setup() after the
// camera driver initializes successfully.
void camSettingsInit();

// Current frame dimensions after camSettingsInit() - pass these to
// videoInit() so AVI headers match what the sensor actually outputs.
void camGetFrameDims(uint16_t *w, uint16_t *h);

// Registers the /camera HTTP handlers. Call before server.begin().
void camRegisterEndpoints(WebServer &server);

// Provide the measured rates shown in /camera/status: capture is the
// sensor's true output rate (camCB), net is how fast frames actually
// leave to stream viewers over WiFi (streamCB).
void camSetFpsSource(float (*fn)());
void camSetNetFpsSource(float (*fn)());

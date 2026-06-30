// ota_update.h
//
// Over-the-air firmware updates for both boards. Two delivery paths share one
// Update.h flash-write core:
//   - ArduinoOTA  : push from PlatformIO during dev
//                   (`pio run -e seeed_xiao_esp32s3_ota -t upload`) - no cable.
//   - HTTP POST /update : upload a firmware.bin from the dashboard, gated by the
//                   same digest auth as every other mutating endpoint.
//
// Requires the dual-OTA partition table (partitions_*_ota.csv): one final USB
// flash installs it, then every update is wireless. The nvs partition keeps its
// offset/size, so WiFi creds + saved settings survive that one re-flash.
//
// mDNS: ArduinoOTA's own mDNS is disabled here; the service is advertised on the
// shared mDNS instance from wifi_portal's startMdns() (MDNS.enableArduino), so
// the existing http service record isn't clobbered by a second MDNS.begin().

#pragma once

#include <Arduino.h>
#include <WebServer.h>

// Compile-time build id, surfaced in /diag as "fw" so you can confirm an update
// actually landed. Override with -DFW_VERSION=... (e.g. a git short hash).
// No parens: used in string-literal concatenation (e.g. "..." FW_VERSION "...").
#ifndef FW_VERSION
#define FW_VERSION __DATE__ " " __TIME__
#endif

// Set up ArduinoOTA (hostname, password, callbacks) and start its listener.
// Call once after WiFi is up and mDNS has been started.
void otaInit();

// Pump ArduinoOTA and service a pending post-update reboot. Call every loop().
void otaHandle();

// Register the HTTP POST /update endpoint. Call during route registration,
// alongside the other *RegisterEndpoints(server) calls (both boards).
void otaRegisterEndpoints(WebServer &server);

// True from the moment an update starts until the reboot. The camera/audio/video
// tasks check this and idle so the flash write owns the bus + CPU and no SD clip
// is left half-written.
bool otaActive();

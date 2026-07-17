// wifi_portal.h
//
// Network bring-up and WiFi management for both boards. Saved networks
// live in NVS (seeded from wifikeys.h on first boot); boot scans and joins
// the strongest known network, and if none is reachable raises a fallback
// access point so the device is always reachable. A persistent
// "standalone" mode skips joining entirely and only hosts the AP - for
// hotels and other places with no usable WiFi.
//
// HTTP endpoints (registered via wifiPortalRegisterEndpoints):
//   GET /wifi                      management page (scan / join / forget)
//   GET /wifi/status               mode, station + AP state, saved networks
//   GET /wifi/scan[?start=1]       async scan; poll until {"nets":[...]}
//   GET /wifi/add?ssid=&pass=[&join=1]   save (and optionally join) a network
//   GET /wifi/join?i=N             join saved network N now
//   GET /wifi/del?i=N              forget saved network N
//   GET /wifi/mode?standalone=0|1  AP-only mode toggle (persistent)
//   GET /wifi/time?epoch=N         set the clock from the browser when NTP
//                                  is unreachable (standalone mode)
//
// Fallback AP: SSID/password and mDNS host come from branding.h (DEVICE_NAME /
// DEVICE_AP_PASS / DEVICE_HOST), served at http://192.168.8.1 (subnet chosen
// not to collide with the 192.168.4.x home LAN). The web UI's digest auth
// applies on the AP exactly as it does on the LAN.

#pragma once

#include <Arduino.h>
#include <WebServer.h>

// Loads saved networks and connects: returns true if joined as a station.
// On failure (or in standalone mode) the fallback AP is up instead, so the
// server should be started regardless of the return value.
bool wifiPortalConnect();

// Registers the /wifi handlers. Call before server.begin().
void wifiPortalRegisterEndpoints(WebServer &server);

// The device's mDNS hostname (per-device, NVS-backed; falls back to DEVICE_HOST).
// The device answers at <hostname>.local. Reused by OTA for ArduinoOTA.setHostname.
const char *wifiPortalHostname();

// Call from loop() (~5s cadence): executes joins requested over HTTP,
// reconnects a dropped station link, raises the fallback AP if it stays
// down, and retries saved networks while nobody is using the AP.
void wifiPortalTick();

// True while the fallback/standalone AP is broadcasting.
bool wifiPortalIsAp();

// Station-link drop stats for /diag: losses since boot, and seconds since the
// most recent one (UINT32_MAX if none) - the sdDrops/sdSecsSinceDrop pattern.
uint32_t wifiPortalDrops();
uint32_t wifiPortalSecsSinceDrop();

// Factory reset of all network/auth state: clears saved networks, the AP
// password, the static-IP octet, standalone mode, and the web-login
// credentials, and wipes the SDK's cached STA creds. Forces 0 saved networks
// (suppressing the wifikeys.h re-seed) so the next boot comes up on the
// fallback AP. Does NOT reboot - the caller does that. Safe to call before
// server start or from the loop task.
void wifiPortalFactoryReset();

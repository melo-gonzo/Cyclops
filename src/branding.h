#pragma once
// Project identity, in one place.
//
//   DEVICE_NAME    - display name (page titles, nav header) and the Wi-Fi AP
//                    SSID raised when no known network is reachable.
//   DEVICE_HOST    - lowercase mDNS hostname; the device answers at
//                    DEVICE_HOST.local. Keep it [a-z0-9-], no spaces.
//   DEVICE_AP_PASS - password for that fallback AP (WPA2 needs >= 8 chars).
//   DEVICE_DEFAULT_USER / DEVICE_DEFAULT_PASS - factory web-login credentials.
//                    DEVICE_DEFAULT_PASS is empty by default: the device ships
//                    PASSWORDLESS (no login prompt) and stays open until you set
//                    a password at /wifi (stored in NVS). Define a non-empty
//                    DEVICE_DEFAULT_PASS to ship a build that requires login.
//
// To rebrand a build without touching tracked source, create a git-ignored
// src/branding_local.h defining any of the above, e.g.:
//
//     #define DEVICE_NAME "Cyclops"
//     #define DEVICE_HOST "cyclops"
//
// Anything left undefined falls back to the defaults below.
#if __has_include("branding_local.h")
#include "branding_local.h"
#endif

#ifndef DEVICE_NAME
#define DEVICE_NAME "Cyclops"
#endif
#ifndef DEVICE_HOST
#define DEVICE_HOST "cyclops"
#endif
#ifndef DEVICE_AP_PASS
#define DEVICE_AP_PASS "cyclops1234"
#endif
#ifndef DEVICE_DEFAULT_USER
#define DEVICE_DEFAULT_USER "admin"
#endif
#ifndef DEVICE_DEFAULT_PASS
#define DEVICE_DEFAULT_PASS "" // empty => passwordless until set at /wifi
#endif

// presence.h
//
// Pure, hardware-free "who's connected" tracker. No Arduino includes ->
// unit-tested on the host (test/test_presence).
//
// The device has no long-lived per-user connection (the dashboard is HTTP
// request/response with polling, and the old "viewers" number was just the
// MJPEG send-queue depth, which reads 0 mid-stream). Instead we count distinct
// client IPs that made an authenticated request within a recent window: a
// browser with the dashboard open polls every few seconds, so it stays counted.
// That is a meaningful "people currently using the device" number.
//
// web_auth.cpp is the shell: it calls presenceTouch() from webAuthCheck() (the
// single auth chokepoint) and presenceActive() feeds /diag + the metrics ring.

#pragma once

#include <stdint.h>

namespace presence {

struct Entry {
  uint32_t ip;     // client IPv4 as uint32 (0 = empty slot)
  uint32_t lastMs; // millis() of last activity
};

// Record activity from `ip` at `now`. Updates an existing entry, reuses an
// empty/expired slot, or (table full of fresh entries) evicts the oldest.
// Unsigned (now - lastMs) is wraparound-safe for the comparison. Pure.
inline void touch(Entry *t, int n, uint32_t ip, uint32_t now, uint32_t windowMs) {
  if (!t || n <= 0 || ip == 0) return;
  int reuse = -1, oldest = 0;
  for (int i = 0; i < n; i++) {
    if (t[i].ip == ip) { t[i].lastMs = now; return; }       // existing client
    bool free = t[i].ip == 0 || (now - t[i].lastMs) > windowMs;
    if (free && reuse < 0) reuse = i;                        // first reusable slot
    if (t[i].lastMs < t[oldest].lastMs) oldest = i;         // fallback eviction
  }
  int slot = reuse >= 0 ? reuse : oldest;
  t[slot].ip = ip;
  t[slot].lastMs = now;
}

// Distinct clients active within `windowMs` of `now`. Pure.
inline int activeCount(const Entry *t, int n, uint32_t now, uint32_t windowMs) {
  if (!t || n <= 0) return 0;
  int c = 0;
  for (int i = 0; i < n; i++)
    if (t[i].ip != 0 && (now - t[i].lastMs) <= windowMs) c++;
  return c;
}

} // namespace presence

// ---- Shell API (defined in presence.cpp; not built in host tests) ----

// Record an authenticated request from a client IP (uint32 IPv4).
void presenceTouch(uint32_t ip);

// Distinct clients seen within the recent window (for /diag + metrics).
int presenceActive();

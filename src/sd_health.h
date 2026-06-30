// sd_health.h
//
// Pure, hardware-free SD mount/recovery POLICY (the timing + backoff state
// machine), separated from the SD I/O it drives. No Arduino includes ->
// unit-tested on the host (test/test_sd_health). This is the exact logic that
// caused an earlier field regression (holding the bus too long / wrong backoff),
// so pinning the transitions with tests is the point. audio_capture.cpp is the
// shell: it asks nextAction(), performs the probe/remount I/O, then feeds the
// result back via onProbeResult()/onRemountResult().

#pragma once

#include <stdint.h>

namespace sdhealth {

enum Action { NONE, PROBE, REMOUNT };

struct State {
  bool available;          // is the card currently mounted/usable
  uint32_t nextProbeMs;    // millis() when the next probe/remount is due
  uint32_t recoverDelayMs; // current backoff between remount attempts
};

struct Config {
  uint32_t probeMs;        // liveness-probe cadence while mounted
  uint32_t recoverMinMs;   // first remount delay after a drop
  uint32_t recoverMaxMs;   // backoff ceiling between remount attempts
};

// Wraparound-safe "is it time to act yet" (millis() wraps ~every 49 days).
inline bool due(const State &s, uint32_t now) {
  return (int32_t)(now - s.nextProbeMs) >= 0;
}

// What the shell should do this tick: probe the live card, attempt a remount,
// or nothing (not due yet).
inline Action nextAction(const State &s, uint32_t now) {
  if (!due(s, now)) return NONE;
  return s.available ? PROBE : REMOUNT;
}

// Result of a liveness probe of a mounted card.
inline void onProbeResult(State &s, const Config &c, uint32_t now, bool alive) {
  if (alive) {
    s.nextProbeMs = now + c.probeMs;
  } else { // silent dropout -> mark down and retry ASAP
    s.available = false;
    s.recoverDelayMs = c.recoverMinMs;
    s.nextProbeMs = now;
  }
}

// Result of a remount attempt of a down card.
inline void onRemountResult(State &s, const Config &c, uint32_t now, bool ok) {
  if (ok) {
    s.available = true;
    s.recoverDelayMs = c.recoverMinMs;
    s.nextProbeMs = now + c.probeMs;
  } else { // exponential backoff up to the ceiling
    // Guard a zero floor: if recoverMinMs is 0 the seed is 0 and 0*2 == 0
    // forever, so the backoff never grows. Treat a 0 current delay as 1 so it
    // can start doubling.
    uint32_t cur = s.recoverDelayMs ? s.recoverDelayMs : 1;
    s.recoverDelayMs = cur < c.recoverMaxMs ? cur * 2 : c.recoverMaxMs;
    s.nextProbeMs = now + s.recoverDelayMs;
  }
}

// External "a write just failed" signal (card pulled/wedged): drop and retry ASAP.
inline void markDown(State &s, const Config &c, uint32_t now) {
  s.available = false;
  s.recoverDelayMs = c.recoverMinMs;
  s.nextProbeMs = now;
}

// Combined-budget free-space backstop with hysteresis. When free space drops
// below floorMb, continuous recording pauses (triggered clips keep saving) so the
// card can't fill and write-thrash; it resumes only once free recovers past
// floorMb + hystMb (the continuous pruner frees space in segment-sized chunks, so
// a bare threshold would oscillate). Pure so the transition is unit-tested;
// `low` is the previous sticky state, the return value the next.
inline bool lowSpaceNext(bool low, uint32_t freeMb, uint32_t floorMb,
                         uint32_t hystMb) {
  return low ? (freeMb < floorMb + hystMb) : (freeMb < floorMb);
}

} // namespace sdhealth

// ratelimit.h
//
// Pure, hardware-free "has the cooldown elapsed" check. No Arduino includes ->
// unit-tested on the host (test/test_ratelimit). Used to throttle the
// audio->video cross-trigger so a noisy room can't drive back-to-back video
// clips (the sustained camera/SD load that wedges core 1 -> watchdog reboot).

#pragma once

#include <stdint.h>

namespace ratelimit {

// True if an event is allowed now:
//   - cooldownMs == 0 disables limiting (always allow)
//   - the first event (fired == false) is always allowed
//   - otherwise only once cooldownMs has elapsed since lastMs
// Unsigned subtraction makes this correct across the millis() wrap (elapsed is
// always < 2^32 for any sane cooldown).
inline bool allow(bool fired, uint32_t lastMs, uint32_t now, uint32_t cooldownMs) {
  if (cooldownMs == 0) return true;
  if (!fired) return true;
  return (uint32_t)(now - lastMs) >= cooldownMs;
}

} // namespace ratelimit

// history.h
//
// Pure, hardware-free decimation for the amplitude/threshold plot history ring.
// No Arduino includes -> unit-tested on the host (test/test_history). The
// wraparound index math here is easy to get wrong by hand, so it lives in one
// tested place; audio_capture.cpp's /audio/history handler is the shell that
// reads the args and streams the result.

#pragma once

#include <stdint.h>

namespace history {

// Buckets available in the ring after skipping the newest `endBuckets` (the
// dashboard's "shift the right edge into the past" zoom).
inline uint32_t availableBuckets(uint32_t count, uint32_t endBuckets) {
  return count > endBuckets ? count - endBuckets : 0;
}

// Max-pool `want` ring buckets (ending `endBuckets` before the newest) down to
// at most `points` output peaks, writing into out[]. src is a ring of ringLen
// uint16 with w = next-write index (newest entry at w-1). Returns the number of
// output points actually produced (= min(points, want)). Pure.
inline uint32_t decimateMaxPool(const uint16_t *src, uint32_t ringLen, uint32_t w,
                                uint32_t endBuckets, uint32_t want,
                                uint32_t points, uint16_t *out) {
  if (!src || out == 0 || want == 0 || points == 0 || ringLen == 0) return 0;
  // Never decimate more than one ring's worth: a want > ringLen would make the
  // index expression alias live data (or wrap past the start). Both callers
  // clamp upstream, but guard here too so the core is safe in isolation.
  if (want > ringLen) want = ringLen;
  // Likewise clamp endBuckets: an oversized "shift the right edge into the past"
  // value underflows the unsigned index expression below and reads a wrong (but
  // in-bounds) slot. The header claims callers clamp, but only `want` was
  // guarded - cap it to ringLen so the modulo index stays well-defined.
  if (endBuckets > ringLen) endBuckets = ringLen;
  if (points > want) points = want;
  for (uint32_t p = 0; p < points; p++) {
    uint32_t b0 = (uint64_t)p * want / points;
    uint32_t b1 = (uint64_t)(p + 1) * want / points;
    uint16_t peak = 0;
    for (uint32_t b = b0; b < b1; b++) {
      uint32_t idx = (w + ringLen - endBuckets - want + b) % ringLen;
      if (src[idx] > peak) peak = src[idx];
    }
    out[p] = peak;
  }
  return points;
}

// Plan a STABLE decimation: pick an integer buckets-per-column factor and back
// the window's right edge up to the last factor-grid boundary, so a given output
// column always max-pools the SAME source buckets between refreshes (its value is
// then byte-identical frame to frame - no shimmer). Only the newest column turns
// over as time advances; the window scrolls one whole column per `f` buckets.
//
//   want       - buckets the caller wants to show (already clamped to availability)
//   points     - columns the caller asked for (canvas width-ish)
//   count      - buckets currently in the ring (saturating is fine)
//   endBuckets - caller's "shift right edge into the past" (0 = live)
//   gridIndex  - a MONOTONIC bucket index from the device clock (millis/bucketMs);
//                only its value mod f matters, so any constant phase offset from
//                the ring's own write index is harmless.
//
// Returns {want, columns, extraEnd}: feed `want` and `columns` to decimateMaxPool
// (want == columns*f makes its grouping uniform) with endBuckets += extraEnd.
// columns == 0 means "nothing to draw".
struct PoolPlan { uint32_t want, columns, extraEnd; };
inline PoolPlan planAlignedPool(uint32_t want, uint32_t points, uint32_t count,
                                uint32_t endBuckets, uint32_t gridIndex) {
  PoolPlan z = {0, 0, 0};
  if (want == 0 || points == 0) return z;
  if (points > want) points = want;
  uint32_t f = (want + points - 1) / points; // ceil: buckets per column
  if (f < 1) f = 1;
  // Back up to the last grid boundary: ((gridIndex - endBuckets) mod f), done with
  // mod-only arithmetic so it never underflows regardless of the phase offset.
  uint32_t extra = ((gridIndex % f) + f - (endBuckets % f)) % f;
  uint32_t avail = count > endBuckets + extra ? count - (endBuckets + extra) : 0;
  // Cap columns so the window covers no MORE than requested (want/f) and no more
  // than the data left after alignment (avail/f); want/f <= points by construction.
  uint32_t usable = want < avail ? want : avail;
  uint32_t cols = usable / f;
  if (cols > points) cols = points; // belt-and-suspenders
  if (cols == 0) return z;
  PoolPlan r = {cols * f, cols, extra};
  return r;
}

} // namespace history

// clip_ring.h
//
// Pure, hardware-free largest-gap math for the wrap-around clip numbering. No
// Arduino/FreeRTOS includes -> unit-tested on the host (test/test_clip_ring).
// The live clip numbers of one kind form a contiguous run in the 0..wrap-1 ring
// (created sequentially, evicted oldest-first), so there is exactly one big empty
// gap. Finding the run's ends by raw min/max breaks when the run straddles the
// wrap (e.g. {9998,9999,0,1,2}: the oldest is 9998, not 0), so we find them by
// the gap instead. clip_index.cpp's eviction (oldest) and boot counter seed
// (newest) are the shells that filter one kind's numbers and call this.

#pragma once

#include <stdint.h>

namespace clipring {

// Find the array index of the run's OLDEST (wantNewest=false) or NEWEST
// (wantNewest=true) entry in `idx[0..n)`, a plain array of clip numbers already
// filtered to one kind. Returns -1 if n==0.
//
// Each entry's "gap" is the distance to its nearest neighbour on the relevant
// side of the ring (the higher side for newest, the lower side for oldest);
// neighbours at distance 0 (duplicate numbers) are ignored. The run end is the
// entry with the LARGEST such gap: the newest sits just before the big empty
// gap, the oldest just after it.
//
// Tie rule (the fix for the equidistant-pair degeneracy): when two entries are
// exactly wrap/2 apart they tie on gap, and a strict "greater than" would let
// the SAME entry win both passes -> eviction/seed pick the wrong end. So on a
// gap tie we break deterministically by the number itself: the NEWEST pass
// prefers the LARGER idx value, the OLDEST pass the SMALLER. That guarantees the
// two passes resolve to DIFFERENT entries in the equidistant case.
inline int runEnd(const uint32_t *idx, int n, uint32_t wrap, bool wantNewest) {
  if (!idx || n <= 0) return -1;
  int best = -1;
  uint32_t bestGap = 0;
  for (int i = 0; i < n; i++) {
    uint32_t gap = wrap; // distance to the nearest neighbour on one side
    for (int j = 0; j < n; j++) {
      if (j == i) continue;
      uint32_t d = wantNewest
          ? (idx[j] + wrap - idx[i]) % wrap   // higher side (newest)
          : (idx[i] + wrap - idx[j]) % wrap;  // lower side (oldest)
      if (d != 0 && d < gap) gap = d;
    }
    if (best < 0 || gap > bestGap) {
      bestGap = gap;
      best = i;
    } else if (gap == bestGap) {
      // Tie-break deterministically so the two passes can't pick the same entry.
      bool prefer = wantNewest ? (idx[i] > idx[best]) : (idx[i] < idx[best]);
      if (prefer) best = i;
    }
  }
  return best;
}

} // namespace clipring

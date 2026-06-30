// motion.h
//
// Pure, hardware-free frame-difference motion analysis. No Arduino includes ->
// unit-tested on the host (test/test_motion). The block loop + masking + bitmap
// packing is fiddly and drives false triggers, so it lives in one tested place.
// video_record.cpp is the shell that decodes JPEG->luma and owns the timing /
// consecutive-hit state.

#pragma once

#include <stdint.h>
#include <string.h>

namespace motion {

struct Result {
  int changed;          // unmasked blocks whose mean abs delta >= diffThresh
  int active;           // unmasked blocks considered
  uint32_t diffSum;     // total abs luma delta over the WHOLE frame (incl. masked)
  uint32_t activeDiffSum; // abs luma delta over UNMASKED blocks only (for avg stat)
  int bx, by;           // grid dimensions actually used
};

// Compare two mw x mh 8-bit luma frames in blk x blk blocks. For each block the
// mean absolute delta is sum|cur-prev| / (blk*blk); >= diffThresh marks it
// changed and sets its bit in outMap (bit index = j*bx + i, LSB-first bytes).
// Masked blocks (bit set in mask) are skipped for changed/active counts but
// still contribute to diffSum. mask/outMap may be null. Returns counts + grid.
inline Result blockDiff(const uint8_t *cur, const uint8_t *prev, int mw, int mh,
                        int blk, uint32_t diffThresh, const uint8_t *mask,
                        uint8_t *outMap, size_t mapBytes) {
  Result r = {0, 0, 0, 0, 0, 0};
  if (!cur || !prev || blk <= 0 || mw <= 0 || mh <= 0) return r;
  r.bx = mw / blk;
  r.by = mh / blk;
  if (outMap) memset(outMap, 0, mapBytes);
  uint32_t blkPx = (uint32_t)blk * blk;

  for (int j = 0; j < r.by; j++) {
    for (int i = 0; i < r.bx; i++) {
      uint32_t diff = 0;
      const uint8_t *c0 = cur + j * blk * mw + i * blk;
      const uint8_t *p0 = prev + j * blk * mw + i * blk;
      for (int y = 0; y < blk; y++) {
        const uint8_t *c = c0 + y * mw;
        const uint8_t *p = p0 + y * mw;
        for (int x = 0; x < blk; x++) {
          int d = (int)c[x] - (int)p[x];
          diff += d < 0 ? -d : d;
        }
      }
      r.diffSum += diff;
      int bit = j * r.bx + i;
      size_t byteIdx = (size_t)(bit >> 3);
      if (mask && byteIdx < mapBytes && (mask[byteIdx] & (1 << (bit & 7))))
        continue; // user-masked: ignore for triggering
      r.active++;
      r.activeDiffSum += diff; // unmasked only -> the "avg motion" stat ignores masked regions
      if (diff / blkPx >= diffThresh) {
        r.changed++;
        if (outMap && byteIdx < mapBytes) outMap[byteIdx] |= 1 << (bit & 7);
      }
    }
  }
  return r;
}

// A scene-wide delta (> pct% of active blocks changed) is lighting/AEC, not
// real motion - the caller suppresses triggering on it.
inline bool isGlobal(int changed, int active, int pct) {
  return active > 0 && changed * 100 > active * pct;
}

// Trigger gate: enough blocks changed, not a global lighting shift, and it has
// persisted for consecNeeded consecutive checks.
inline bool shouldTrigger(int changed, int blocksThreshold, bool global,
                          int consecutive, int consecNeeded) {
  return !global && changed >= blocksThreshold && consecutive >= consecNeeded;
}

} // namespace motion

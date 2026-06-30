// retention.h
//
// Pure, hardware-free rolling-retention policy for continuous (NVR) recording.
// No Arduino/SD includes, so it is unit-tested on the host (test/test_retention).
// cont_record.cpp is the shell: it scans the card into Seg[], calls planEvictions,
// then deletes the segments the planner marked. Keeping the policy here makes the
// "delete oldest until under both caps" logic a single tested source of truth.

#pragma once

#include <stdint.h>

namespace retention {

struct Seg {
  uint32_t num;   // file index (tiebreak for equal mtimes)
  uint32_t size;  // bytes
  uint32_t mtime; // epoch seconds; primary sort key (oldest first)
  bool vid;       // true = video stream, false = audio stream
  bool evict;     // OUTPUT: planner sets true for segments to delete
};

// How many segments fit in keepMin minutes at segS-second segments (>= 1).
inline uint32_t segmentsForMinutes(uint32_t keepMin, uint32_t segS) {
  uint32_t m = keepMin * 60 / (segS ? segS : 1);
  return m < 1 ? 1 : m;
}

// Sort oldest-first by (mtime, num). Small n (<= a few thousand); simple
// insertion sort keeps this dependency-free and stable.
inline void sortOldestFirst(Seg *s, int n) {
  for (int i = 1; i < n; i++) {
    Seg key = s[i];
    int j = i - 1;
    while (j >= 0 && (s[j].mtime > key.mtime ||
                      (s[j].mtime == key.mtime && s[j].num > key.num))) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = key;
  }
}

// Plan evictions to satisfy BOTH caps, oldest-first:
//   1. per-stream count cap maxSegPerStream (audio and video counted separately)
//   2. total byte budget budgetBytes (across both streams)
// Sets seg.evict on the chosen segments and returns how many were evicted.
// enforce=false (recording paused) plans nothing. Pure: no I/O, no globals.
inline int planEvictions(Seg *s, int n, uint32_t maxSegPerStream,
                         uint64_t budgetBytes, bool enforce) {
  for (int i = 0; i < n; i++) s[i].evict = false;
  if (!enforce || n <= 0) return 0;

  sortOldestFirst(s, n);

  int aRem = 0, vRem = 0;
  uint64_t total = 0;
  for (int i = 0; i < n; i++) {
    (s[i].vid ? vRem : aRem)++;
    total += s[i].size;
  }

  int evicted = 0;
  // 1. per-stream minutes (segment-count) cap.
  for (int i = 0; i < n; i++) {
    int &rem = s[i].vid ? vRem : aRem;
    if ((uint32_t)rem > maxSegPerStream) {
      s[i].evict = true;
      total -= s[i].size;
      rem--;
      evicted++;
    }
  }
  // 2. total disk budget.
  for (int i = 0; i < n && total > budgetBytes; i++) {
    if (s[i].evict) continue;
    s[i].evict = true;
    total -= s[i].size;
    evicted++;
  }
  return evicted;
}

inline uint64_t survivingBytes(const Seg *s, int n) {
  uint64_t b = 0;
  for (int i = 0; i < n; i++)
    if (!s[i].evict) b += s[i].size;
  return b;
}

inline int survivingCount(const Seg *s, int n) {
  int c = 0;
  for (int i = 0; i < n; i++)
    if (!s[i].evict) c++;
  return c;
}

} // namespace retention

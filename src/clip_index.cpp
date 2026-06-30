// clip_index.cpp - see clip_index.h.
// Pure in-RAM clip index (no hardware): compiled on every board so the video
// recorder/list links even where SD isn't present.

#include "clip_index.h"
#include "clip_ring.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include <string.h>

// Fixed-capacity index in PSRAM. 2048 entries x ~40B = ~80KB, far above any sane
// clip-count cap; if it ever fills, further adds are dropped from the index (the
// file is still written) and the next mount re-scan resyncs - graceful, not fatal.
#define CLIP_IDX_CAP 2048

struct ClipRec {
  uint32_t idx;     // number parsed from the name (for oldest-first eviction)
  uint32_t size;    // bytes
  uint32_t mtime;   // epoch seconds (0 if the clock was unset at write time)
  uint8_t  kind;    // CLIP_AUDIO / CLIP_VIDEO
  char     name[28];
};

static ClipRec *g_idx = nullptr;
static uint32_t g_count = 0;            // entries in use
static SemaphoreHandle_t g_mtx = nullptr;

// Parse the numeric index out of "clip_00042_a.wav" / "vclip_00042_m.avi".
// %u stops at the first non-digit, so the prefix length is all that differs.
static uint32_t parseIdx(const char *name) {
  const char *p = name;
  while (*p && (*p < '0' || *p > '9')) p++; // skip the "clip_"/"vclip_" prefix
  unsigned v = 0;
  return (sscanf(p, "%u", &v) == 1) ? v : 0;
}

void clipIndexInit() {
  if (g_idx) return;
  g_mtx = xSemaphoreCreateMutex();
  g_idx = (ClipRec *)ps_malloc(sizeof(ClipRec) * CLIP_IDX_CAP);
  if (!g_idx) g_idx = (ClipRec *)malloc(sizeof(ClipRec) * CLIP_IDX_CAP);
  g_count = 0;
  if (!g_idx) Serial.println("[clipidx] alloc failed - lists/caps will fall back to scans");
}

// Find an entry by name; returns index in g_idx or -1. Caller holds g_mtx.
static int findByName(const char *name) {
  for (uint32_t i = 0; i < g_count; i++)
    if (strcmp(g_idx[i].name, name) == 0) return (int)i;
  return -1;
}

void clipIndexClear(int kind) {
  if (!g_idx || !g_mtx) return;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  if (kind < 0) {
    g_count = 0;
  } else {
    uint32_t w = 0;
    for (uint32_t i = 0; i < g_count; i++)
      if (g_idx[i].kind != (uint8_t)kind) g_idx[w++] = g_idx[i];
    g_count = w;
  }
  xSemaphoreGive(g_mtx);
}

void clipIndexAdd(int kind, const char *name, uint32_t size, uint32_t mtime) {
  if (!g_idx || !g_mtx || !name) return;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  int at = findByName(name);
  if (at < 0) {
    if (g_count >= CLIP_IDX_CAP) { xSemaphoreGive(g_mtx); return; } // full: drop (re-scan resyncs)
    at = (int)g_count++;
    g_idx[at].kind = (uint8_t)kind;
    g_idx[at].idx = parseIdx(name);
    strlcpy(g_idx[at].name, name, sizeof(g_idx[at].name));
  }
  g_idx[at].size = size;
  g_idx[at].mtime = mtime;
  xSemaphoreGive(g_mtx);
}

void clipIndexRemove(const char *name) {
  if (!g_idx || !g_mtx || !name) return;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  int at = findByName(name);
  if (at >= 0) g_idx[at] = g_idx[--g_count]; // swap-with-last
  xSemaphoreGive(g_mtx);
}

uint32_t clipIndexCount(int kind) {
  if (!g_idx || !g_mtx) return 0;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  uint32_t n = 0;
  for (uint32_t i = 0; i < g_count; i++) if (g_idx[i].kind == (uint8_t)kind) n++;
  xSemaphoreGive(g_mtx);
  return n;
}

// The live numbers of one kind form a contiguous run in the 0..CLIP_NUM_WRAP-1
// ring (created sequentially, evicted oldest-first), so there is one big empty
// gap. The OLDEST sits just after that gap, the NEWEST just before it; the
// wrap-aware largest-gap math (and its tie-break) lives in clipring::runEnd
// (clip_ring.h) so it can be host-tested. Here we just filter the in-RAM entries
// of the requested kind into a stack scratch of their numbers (with a parallel
// map back to the g_idx position), call it, and translate the result back.
// Caller holds g_mtx. wantNewest=false returns the oldest position, true the newest.
static int clipRunEnd(int kind, bool wantNewest) {
  static uint32_t nums[CLIP_IDX_CAP];
  static int back[CLIP_IDX_CAP];
  int n = 0;
  for (uint32_t i = 0; i < g_count; i++) {
    if (g_idx[i].kind != (uint8_t)kind) continue;
    nums[n] = g_idx[i].idx;
    back[n] = (int)i;
    n++;
  }
  int r = clipring::runEnd(nums, n, CLIP_NUM_WRAP, wantNewest);
  return r >= 0 ? back[r] : -1;
}

bool clipIndexOldest(int kind, char *out, size_t cap) {
  if (!g_idx || !g_mtx || !out || cap == 0) return false;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  int best = clipRunEnd(kind, false);
  bool found = best >= 0;
  if (found) strlcpy(out, g_idx[best].name, cap);
  xSemaphoreGive(g_mtx);
  return found;
}

uint32_t clipIndexNextNum(int kind) {
  if (!g_idx || !g_mtx) return 0;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  int newest = clipRunEnd(kind, true);
  uint32_t next = (newest >= 0) ? (g_idx[newest].idx + 1) % CLIP_NUM_WRAP : 0;
  xSemaphoreGive(g_mtx);
  return next;
}

void clipIndexListJson(int kind, uint32_t maxN, String &out) {
  out += "[";
  if (!g_idx || !g_mtx) { out += "]"; return; }
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  uint32_t emitted = 0;
  for (uint32_t i = 0; i < g_count && emitted < maxN; i++) {
    if (g_idx[i].kind != (uint8_t)kind) continue;
    if (emitted) out += ",";
    out += "{\"name\":\"";
    out += g_idx[i].name;
    out += "\",\"size\":";
    out += String(g_idx[i].size);
    out += ",\"mtime\":";
    out += String((unsigned long)g_idx[i].mtime);
    out += "}";
    emitted++;
  }
  xSemaphoreGive(g_mtx);
  out += "]";
}


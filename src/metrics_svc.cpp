// metrics_svc.cpp - imperative shell for the diagnostics time-series.
//
// Per-metric uint16 PSRAM rings on one shared time base, a tiny set/event API
// any module can feed, a sampler that snapshots one bucket per tick, and the
// HTTP endpoints behind the "/graphs" page. The encoding/metric set is the pure,
// unit-tested metrics.h; the ring decimation is the unit-tested history.h.

#include "metrics_svc.h"
#include "web_assets.gen.h"
#include "history.h"
#include "web_auth.h"
#include "branding.h"
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace metrics;

// 1440 buckets, fixed (13 metrics x 1440 x 2B ~= 37 KB PSRAM). The per-bucket
// duration is derived from a tunable retention window (g_windowMin) spread over
// those fixed buckets - the spectrogram's "fixed columns, stretch the bucket"
// model - so /graphs retention is adjustable at constant memory. Default 2h@5s.
#define MET_BUCKETS   1440
#define MET_MAX_POINTS 600 // cap on decimated points per series (bounds response)
#define DEFAULT_MET_WIN_MIN 120u  // 2h  -> 5s  buckets (unchanged default)
#define MET_WIN_MIN_MIN     120u  // 2h  -> 5s  buckets (finest; ~matches loop tick)
#define MET_WIN_MAX_MIN     1440u // 24h -> 60s buckets (coarsest)

static uint16_t *g_ring[M_COUNT];          // one ring per metric (PSRAM)
static uint32_t g_write = 0, g_count = 0;  // shared write index / fill level
static uint32_t g_windowMin = DEFAULT_MET_WIN_MIN; // retention window (minutes), NVS-backed

// Per-bucket duration for the current window: minutes spread over the fixed ring.
static inline uint32_t metBucketMs() {
  uint32_t b = (uint32_t)((uint64_t)g_windowMin * 60000ULL / MET_BUCKETS);
  return b < 1000 ? 1000 : b;
}
static volatile float g_latest[M_COUNT];   // last reported GAUGE value
static volatile uint32_t g_evt[M_COUNT];   // accumulated COUNT events this bucket
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static WebServer *g_server = nullptr;

bool metricsInit() {
  // One contiguous PSRAM block carved into M_COUNT rings.
  size_t per = (size_t)MET_BUCKETS * sizeof(uint16_t);
  uint16_t *blk = (uint16_t *)ps_calloc((size_t)M_COUNT * MET_BUCKETS, sizeof(uint16_t));
  if (!blk) {
    Serial.println("[metrics] PSRAM ring alloc failed");
    return false;
  }
  for (int i = 0; i < M_COUNT; i++) g_ring[i] = blk + (size_t)i * MET_BUCKETS;
  (void)per;

  Preferences p; // restore the tunable retention window
  if (p.begin("metrics", true)) {
    g_windowMin = p.getUInt("win", g_windowMin);
    if (g_windowMin < MET_WIN_MIN_MIN || g_windowMin > MET_WIN_MAX_MIN) g_windowMin = DEFAULT_MET_WIN_MIN;
    p.end();
  }
  Serial.printf("[metrics] %d metrics x %d buckets (%uKB), window %umin (%ums/bucket)\n",
                (int)M_COUNT, MET_BUCKETS,
                (unsigned)((size_t)M_COUNT * MET_BUCKETS * sizeof(uint16_t) / 1024),
                (unsigned)g_windowMin, (unsigned)metBucketMs());
  return true;
}

uint32_t metricsBucketMs() { return metBucketMs(); }
uint32_t metricsWindowMin() { return g_windowMin; }

// Change the retention window (minutes). Existing buckets were captured at the
// old cadence, so each ring is re-bucketed (history.h resampleMaxPool) to the
// new one - collected history survives the change instead of being wiped.
// Persisted to NVS.
void metricsSetWindowMin(uint32_t m) {
  if (m < MET_WIN_MIN_MIN || m > MET_WIN_MAX_MIN || m == g_windowMin) return;
  uint32_t oldMs = metBucketMs();
  g_windowMin = m;
  uint32_t newMs = metBucketMs();
  if (oldMs != newMs) {
    // The loop-task sampler may land one bucket mid-rebucket (no lock: holding
    // g_mux across a 37KB rewrite would stall it far longer); worst case is one
    // glitched column vs. the full wipe this replaced. Scratch alloc failure
    // falls back to that old restart-the-ring behavior (n stays 0).
    uint32_t n = 0;
    uint16_t *tmp = (g_ring[0] && g_count)
                        ? (uint16_t *)ps_malloc((size_t)MET_BUCKETS * sizeof(uint16_t))
                        : NULL;
    if (tmp) {
      for (int i = 0; i < M_COUNT; i++) {
        n = history::resampleMaxPool(g_ring[i], MET_BUCKETS, g_write, g_count,
                                     oldMs, newMs, tmp, MET_BUCKETS);
        memcpy(g_ring[i], tmp, (size_t)n * sizeof(uint16_t));
      }
      free(tmp);
    }
    g_write = n % MET_BUCKETS;
    g_count = n;
  }
  Preferences p;
  if (p.begin("metrics", false)) { p.putUInt("win", g_windowMin); p.end(); }
}

void metricsSet(Id id, float value) {
  if ((unsigned)id < M_COUNT) g_latest[id] = value; // unsigned: also rejects negative id
}

void metricsEvent(Id id) {
  if ((unsigned)id >= M_COUNT) return;
  portENTER_CRITICAL(&g_mux);
  g_evt[id]++;
  portEXIT_CRITICAL(&g_mux);
}

void metricsSample() {
  if (!g_ring[0]) return;
  for (int i = 0; i < M_COUNT; i++) {
    float v;
    if (def(i).kind == COUNT) {
      portENTER_CRITICAL(&g_mux);
      v = (float)g_evt[i];
      g_evt[i] = 0;
      portEXIT_CRITICAL(&g_mux);
    } else {
      v = g_latest[i];
    }
    g_ring[i][g_write] = encode(v, def(i));
  }
  g_write = (g_write + 1) % MET_BUCKETS;
  if (g_count < MET_BUCKETS) g_count++;
}

// ---- HTTP ----

static void handleMeta() {
  if (!webAuthCheck(*g_server)) return;
  String out;
  out.reserve(900);
  out += "{\"bucket_ms\":";
  out += metBucketMs();
  out += ",\"max_buckets\":";
  out += MET_BUCKETS;
  out += ",\"win_min\":";
  out += g_windowMin;
  out += ",\"viewer\":";
  out += webAuthIsViewer() ? "true" : "false";
  out += ",\"metrics\":[";
  for (int i = 0; i < M_COUNT; i++) {
    const Def &d = def(i);
    if (i) out += ',';
    out += "{\"key\":\"";  out += d.key;
    out += "\",\"label\":\""; out += d.label;
    out += "\",\"color\":\""; out += d.color;
    out += "\",\"unit\":\"";  out += d.unit;
    out += "\",\"scale\":";   out += String(d.scale, 4);
    out += ",\"offset\":";    out += String(d.offset, 4);
    out += ",\"count\":";     out += (d.kind == COUNT ? "true" : "false");
    out += '}';
  }
  out += "]}";
  g_server->send(200, "application/json", out);
}

static void handleSeries() {
  if (!webAuthCheck(*g_server)) return;
  uint32_t secs = g_server->hasArg("secs") ? (uint32_t)g_server->arg("secs").toInt() : 0;
  uint32_t points = g_server->hasArg("points") ? (uint32_t)g_server->arg("points").toInt() : 240;

  uint32_t want = g_count;
  if (secs > 0) {
    uint32_t wb = (uint32_t)((uint64_t)secs * 1000 / metBucketMs());
    if (wb < want) want = wb;
  }
  if (points > want) points = want;
  if (points > MET_MAX_POINTS) points = MET_MAX_POINTS;

  static uint16_t buf[MET_MAX_POINTS]; // server task only (single-threaded)

  size_t reserve = (size_t)points * 6 * M_COUNT + 256;
  // Building this String can transiently want ~47KB; bail before allocating on
  // a low-heap device (warns at <30KB) rather than risk a fragmenting failure.
  if (ESP.getFreeHeap() < reserve + 16384) {
    g_server->send(503, "text/plain", "low memory, try again");
    return;
  }

  String out;
  out.reserve(reserve);
  uint32_t bucketMs = metBucketMs();
  out += "{\"bucket_ms\":";
  out += bucketMs;
  out += ",\"want\":";   out += want;
  out += ",\"points\":"; out += points;
  out += ",\"span_ms\":"; out += (uint32_t)((uint64_t)want * bucketMs);
  out += ",\"series\":{";
  for (int m = 0; m < M_COUNT; m++) {
    uint32_t n = (points && g_ring[m])
                     ? history::decimateMaxPool(g_ring[m], MET_BUCKETS, g_write, 0, want, points, buf)
                     : 0;
    if (m) out += ',';
    out += '"'; out += def(m).key; out += "\":[";
    for (uint32_t i = 0; i < n; i++) {
      if (i) out += ',';
      out += buf[i];
    }
    out += ']';
  }
  out += "}}";
  g_server->send(200, "application/json", out);
}

// WEB_GRAPHS_HTML now lives in web/graphs.html (compiled in as WEB_GRAPHS_HTML via web/ codegen)

static void handlePage() {
  if (!webAuthCheck(*g_server)) return;
  g_server->send_P(200, "text/html", WEB_GRAPHS_HTML);
}

// GET /metrics/config?win_min=N - set the retention window (minutes). Clamped,
// persisted, and restarts the ring (old buckets used the prior cadence).
static void handleConfig() {
  if (!webAuthRequireWrite(*g_server)) return;
  if (g_server->hasArg("win_min"))
    metricsSetWindowMin((uint32_t)g_server->arg("win_min").toInt());
  String out = "{\"win_min\":";
  out += metricsWindowMin();
  out += ",\"bucket_ms\":";
  out += metBucketMs();
  out += "}";
  g_server->send(200, "application/json", out);
}

void metricsRegisterEndpoints(WebServer &server) {
  g_server = &server;
  server.on("/graphs", HTTP_GET, handlePage);
  server.on("/metrics/meta", HTTP_GET, handleMeta);
  server.on("/metrics/series", HTTP_GET, handleSeries);
  server.on("/metrics/config", HTTP_GET, handleConfig);
}

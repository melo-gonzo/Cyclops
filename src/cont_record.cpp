// cont_record.cpp - see cont_record.h

#include "capabilities.h" // HAS_SD gate (any board with a card records)
#if HAS_SD

#include "cont_record.h"
#include "web_assets.gen.h"

#include <Arduino.h>
#include <Preferences.h>
#include <SD.h>
#include <stdlib.h>

#include "audio_capture.h" // audioContSet/audioContJson, sdGetMutex/sdIsAvailable
#include "video_record.h"  // videoContSet/videoContJson
#include "web_auth.h"
#include "branding.h"
#include "retention.h"     // pure eviction policy (unit-tested in test/test_retention)

#define CLIP_DIR "/clips"

// ---- settings (NVS "cont") ----
static bool     cAudOn = false;
static bool     cVidOn = false;
static uint32_t cSegS = 60;     // segment length, seconds (15..600)
static uint32_t cKeepMin = 30;  // minutes kept per stream (1..240)
static uint32_t cKeepMb = 2000; // total MB cap for continuous files (50..100000)

// ---- usage cache, refreshed by the pruner ----
static volatile uint32_t cUsedKb = 0;
static volatile uint32_t cFileCnt = 0;

static WebServer *cServer = NULL;

// Scratch for the pruner; PSRAM so it never pressures the heap.
#define MAX_CF 1024
static retention::Seg *cf = NULL;

static void contLoad() {
  Preferences p;
  if (!p.begin("cont", true)) return;
  cAudOn = p.getBool("a", cAudOn);
  cVidOn = p.getBool("v", cVidOn);
  cSegS = p.getUInt("seg", cSegS);
  cKeepMin = p.getUInt("min", cKeepMin);
  cKeepMb = p.getUInt("mb", cKeepMb);
  p.end();
}

static void contSave() {
  Preferences p;
  if (!p.begin("cont", false)) return;
  p.putBool("a", cAudOn);
  p.putBool("v", cVidOn);
  p.putUInt("seg", cSegS);
  p.putUInt("min", cKeepMin);
  p.putUInt("mb", cKeepMb);
  p.end();
}

static void contApply() {
#if HAS_AUDIO
  audioContSet(cAudOn, cSegS * 1000);
#endif
  videoContSet(cVidOn, cSegS * 1000);
}

// ---- rolling pruner ----

static void delSeg(const retention::Seg &f) {
  char path[48];
  snprintf(path, sizeof(path), CLIP_DIR "/cont_%05u_%s", (unsigned)f.num,
           f.vid ? "v.avi" : "a.wav");
  SD.remove(path); // caller holds the SD mutex
}

// Scan continuous files, refresh the usage cache, and (when recording is on)
// enforce the minutes-per-stream and total-MB caps, deleting oldest first.
static void contPrune() {
  if (!sdIsAvailable() || cf == NULL) return;
  SemaphoreHandle_t m = sdGetMutex();

  xSemaphoreTake(m, portMAX_DELAY);
  int n = 0;
  File dir = SD.open(CLIP_DIR);
  if (dir) {
    File e;
    while (n < MAX_CF && (e = dir.openNextFile())) {
      const char *b = strrchr(e.name(), '/');
      b = b ? b + 1 : e.name();
      unsigned idx;
      bool match = false, vid = false;
      if (sscanf(b, "cont_%u_v.avi", &idx) == 1) { match = true; vid = true; }
      else if (sscanf(b, "cont_%u_a.wav", &idx) == 1) { match = true; }
      if (match) {
        cf[n].num = idx;
        cf[n].size = e.size();
        cf[n].mtime = (uint32_t)e.getLastWrite();
        cf[n].vid = vid;
        cf[n].evict = false;
        n++;
      }
      e.close();
    }
    dir.close();
  }

  // Pure policy decides what to evict; this shell just carries out the deletes.
  retention::planEvictions(cf, n, retention::segmentsForMinutes(cKeepMin, cSegS),
                           (uint64_t)cKeepMb * 1024 * 1024, cAudOn || cVidOn);
  for (int i = 0; i < n; i++)
    if (cf[i].evict) delSeg(cf[i]);

  uint64_t bytes = retention::survivingBytes(cf, n);
  uint32_t cnt = (uint32_t)retention::survivingCount(cf, n);
  xSemaphoreGive(m);

  cFileCnt = cnt;
  cUsedKb = (uint32_t)(bytes / 1024);
}

static void contPruneTask(void *pv) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    contPrune();
  }
}

// ---- HTTP handlers ----

static bool authOk() { return webAuthCheck(*cServer); }
static bool authWriteOk() { return webAuthRequireWrite(*cServer); } // 403s the viewer

static void handleStatus() {
  if (!authOk()) return;
  String j;
  j.reserve(320);
  j += "{\"aud\":"; j += cAudOn ? "true" : "false";
  j += ",\"vid\":"; j += cVidOn ? "true" : "false";
  j += ",\"seg_s\":"; j += String(cSegS);
  j += ",\"keep_min\":"; j += String(cKeepMin);
  j += ",\"keep_mb\":"; j += String(cKeepMb);
#if HAS_AUDIO
  j += ',' + audioContJson();
#else
  // Same keys audioContJson emits, pinned off: no mic on this board.
  j += ",\"a_on\":false,\"a_rec\":false,\"a_file\":0,\"a_sd\":true";
#endif
  j += ',' + videoContJson();
  j += ",\"sd\":"; j += sdIsAvailable() ? "true" : "false";
  j += ",\"used_mb\":"; j += String(cUsedKb / 1024);
  j += ",\"files\":"; j += String(cFileCnt);
  // Card-level free-space backstop: when low, continuous pauses (triggered clips
  // still save). Surfaced so the page can show "paused (disk full)".
  j += ",\"sd_low\":"; j += sdLowSpace() ? "true" : "false";
  j += ",\"sd_free_mb\":"; j += String(sdFreeMb());
  j += ",\"viewer\":"; j += webAuthIsViewer() ? "true" : "false";
  j += '}';
  cServer->send(200, "application/json", j);
}

static void handleConfig() {
  if (!authWriteOk()) return;
  if (cServer->hasArg("aud")) cAudOn = cServer->arg("aud") == "1";
  if (cServer->hasArg("vid")) cVidOn = cServer->arg("vid") == "1";
  if (cServer->hasArg("seg")) {
    long v = atol(cServer->arg("seg").c_str());
    if (v < 15) v = 15;
    if (v > 600) v = 600;
    cSegS = v;
  }
  if (cServer->hasArg("min")) {
    long v = atol(cServer->arg("min").c_str());
    if (v < 1) v = 1;
    if (v > 240) v = 240;
    cKeepMin = v;
  }
  if (cServer->hasArg("mb")) {
    long v = atol(cServer->arg("mb").c_str());
    if (v < 50) v = 50;
    if (v > 100000) v = 100000;
    cKeepMb = v;
  }
  contSave();
  contApply();
  handleStatus();
}

static void handleClear() {
  if (!authWriteOk()) return;
  // Turn recording off first so writers don't recreate files mid-wipe.
  cAudOn = cVidOn = false;
  contSave();
  contApply();
  if (!sdIsAvailable()) { cServer->send(503, "text/plain", "no sd card"); return; }
  SemaphoreHandle_t m = sdGetMutex();
  int deleted = 0;
  for (;;) {
    char names[16][32];
    int n = 0;
    xSemaphoreTake(m, portMAX_DELAY);
    File dir = SD.open(CLIP_DIR);
    if (dir) {
      File e;
      while (n < 16 && (e = dir.openNextFile())) {
        const char *b = strrchr(e.name(), '/');
        b = b ? b + 1 : e.name();
        if (strncmp(b, "cont_", 5) == 0) strlcpy(names[n++], b, sizeof(names[0]));
        e.close();
      }
      dir.close();
    }
    for (int i = 0; i < n; i++)
      if (SD.remove(String(CLIP_DIR) + "/" + names[i])) deleted++;
    xSemaphoreGive(m);
    if (n == 0) break;
  }
  cUsedKb = 0;
  cFileCnt = 0;
  char json[48];
  snprintf(json, sizeof(json), "{\"deleted\":%d}", deleted);
  cServer->send(200, "application/json", json);
}

// ---- page ----

// WEB_REC_HTML now lives in web/rec.html (compiled in as WEB_REC_HTML via web/ codegen)

static void handlePage() {
  if (!authOk()) return;
  cServer->send_P(200, "text/html", WEB_REC_HTML);
}

// ---- public API ----

void contInit() {
  if (psramFound()) cf = (retention::Seg *)ps_malloc(sizeof(retention::Seg) * MAX_CF);
  if (cf == NULL) cf = (retention::Seg *)malloc(sizeof(retention::Seg) * MAX_CF);
  contLoad();
  contApply();
  xTaskCreatePinnedToCore(contPruneTask, "cont_prune", 4096, NULL, 1, NULL, 0);
}

void contRegisterEndpoints(WebServer &server) {
  cServer = &server;
  server.on("/rec", HTTP_GET, handlePage);
  server.on("/rec/status", HTTP_GET, handleStatus);
  server.on("/rec/config", HTTP_GET, handleConfig);
  server.on("/rec/clear", HTTP_GET, handleClear);
}

#endif // HAS_SD

// video_record.cpp
// Triggered MJPEG-AVI clip recording. See video_record.h.
//
// Buffering model:
//   camCB --(videoTapFrame, ~1 frame per period)--> JPEG byte ring (PSRAM)
//   trigger marks a time; the recorder task writes the pre-roll frames
//   already in the ring and then live frames straight to an AVI on SD.
//   Strict SPSC: camCB is the only producer, the recorder the only
//   consumer. The producer never blocks - if a tap would overwrite a frame
//   the recorder still needs (or one pinned by the motion detector), the
//   tap is dropped and counted instead.

// Compiled on every board: motion detection runs off the PSRAM frame ring with
// no SD. SD-backed clip saving is skipped at runtime where sdIsAvailable() is
// false (boards without an SD card). The SD/audio shims it links against are
// provided by audio_capture.cpp (XIAO) or board_stubs.cpp (no-audio boards).

#include "video_record.h"
#include <WiFi.h>       // WiFiClient for raw binary responses (arduino 3.x no
                        // longer provides it transitively via WebServer.h)
#include "audio_capture.h"
#include "clip_index.h" // in-RAM /clips index: video list + cap never scan the card
#include "ota_update.h" // otaActive(): pause recorder/motion tasks while firmware flashes
#include "web_auth.h"
#include "avi.h"       // pure MJPEG-AVI framing (unit-tested in test/test_avi)
#include "motion.h"    // pure frame-diff motion analysis (unit-tested in test/test_motion)
#include "ratelimit.h" // pure cross-trigger cooldown (unit-tested in test/test_ratelimit)
#include "thermal.h"   // pure thermal-governor hysteresis (unit-tested in test/test_thermal)
#include "metrics_svc.h" // diagnostics time-series feed (/graphs)
#include "diag_log.h"    // dlog: RTC-backed ring, survives the reboot it predicts
#include "history.h"     // pure plot-history decimation (unit-tested in test/test_history)
#include <Preferences.h>
#include <FS.h>
#include <SD.h>
#include <esp_task_wdt.h> // feed the boot WDT during the (bounded) clip scan
#include <img_converters.h>
#include <esp_jpg_decode.h>

#define VID_CLIP_DIR   "/clips"
#define VID_LIST_MAX   200

// ---- Tap ring sizing (degradation ladder if PSRAM is tight) ----
static const size_t VRING_TRY[] = {1536 * 1024, 1024 * 1024, 768 * 1024};
#define VRING_PSRAM_FLOOR (1024 * 1024) // same reserve audio honors
#define VQ_MAX         96               // frame metadata FIFO entries
#define VID_IDX_MAX    600              // frames per clip hard cap (60s @ 10fps)

// ---- Clip sizing (runtime-tunable via /video/config) ----
#define VID_MIN_CLIP_MS        2000
#define VID_MAX_CLIP_MS        60000
#define VID_MAX_PREROLL_MS     8000
#define DEFAULT_VID_CLIP_MS    10000
#define DEFAULT_VID_PREROLL_MS 4000
#define DEFAULT_VID_FPS        5
#define DEFAULT_VID_MAX_FILES  40 // delete-oldest cap; AVIs run ~0.5-3MB each

// ---- Per-clip key-frame thumbnail (vclip_*.jpg / clip_*.jpg sidecar) ----
// Reuses the motion detector's cheap 1/8-scale JPEG decode, so an XGA frame
// becomes 128x96; re-encoded at this quality it lands ~3-4KB. Small enough to
// store next to every clip and load a whole table's worth over the LAN.
#define THUMB_JPEG_QUALITY 14

// ---- On-device motion detection ----
// 1/8-scale JPEG decode (DC-only, ~40-80ms) of the newest tapped frame,
// converted to 8-bit luma, then per-block diff against the previous check.
// Block size is runtime-tunable (motBlockSz luma px per side, so a block
// covers 8*motBlockSz sensor px; 8 -> 64x64 at XGA). Two consecutive hits
// are required so auto-exposure flicker can't trigger alone, and a
// scene-wide delta is classed as a lighting change rather than motion.
#define MOT_CHECK_MS     500
#define MOT_HOLDOFF_MS   5000   // min gap after a clip before motion can re-trigger
#define MOT_WARMUP_MS    15000  // let AEC/AWB settle after boot
#define DEFAULT_MOT_BLOCKS 12   // changed blocks needed to trigger
#define DEFAULT_MOT_DIFF 12     // avg per-pixel luma delta for a block to count
#define DEFAULT_MOT_COOLDOWN_S 5 // refractory: ignore new motion triggers this many
                                 // seconds after one fires (limits repeat triggers;
                                 // applies on every board, unlike the clip holdoff)
#define MOT_COOLDOWN_MAX_S 60
#define DEFAULT_MOT_BLKSZ 8     // luma px per block side (16/8/4/2)
#define MOT_GLOBAL_PCT   60     // >this % of blocks changed = lighting shift, ignore
#define MOT_POLL_KEEPALIVE_MS 10000 // /video/motion polls keep analysis running
#define MOT_GRID_MAX_X   64     // XGA at the finest grid: 1024/8/2
#define MOT_GRID_MAX_Y   48     // XGA at the finest grid: 768/8/2
#define MOT_MAP_BYTES    ((MOT_GRID_MAX_X * MOT_GRID_MAX_Y + 7) / 8)
#define MOT_HIST_MAX     64     // ~32s of maps kept for clip sidecars

struct VFrame {
  uint32_t off;  // byte offset in vring
  uint32_t len;
  uint32_t tsMs;
};

struct VidIdx {
  uint32_t off;  // '00dc' fourcc position relative to the 'movi' fourcc
  uint32_t len;
};

static portMUX_TYPE videoMux = portMUX_INITIALIZER_UNLOCKED;

// Tap ring (guarded by videoMux; frame bytes are stable outside the mux
// because eviction of unconsumed/pinned frames is forbidden)
static uint8_t *vring = NULL;
static size_t   vringSize = 0;
static uint32_t vWriteOff = 0;
static VFrame   vq[VQ_MAX];
static uint32_t vqTail = 0;            // index of oldest entry
static uint32_t vqCount = 0;
static uint32_t vqSeqNext = 0;         // seq of the next frame to enqueue
static uint32_t vqPinSeq = UINT32_MAX; // motion detector pins one frame
static volatile int vringUsers = 0;    // readers touching vring right now
static uint32_t vRingGen = 0;          // bumped whenever the ring is reset
                                       // (videoSetDims); a tap whose memcpy spans
                                       // a reset discards its frame
static uint32_t vDrops = 0;            // taps dropped (ring pressure)
static uint32_t vLastTapMs = 0;

// Recorder state
static volatile bool videoEnabled = true;
static volatile bool recActive = false;
static volatile bool recAbort = false; // disable requested mid-recording
static volatile bool trigPending = false;
static uint8_t  trigSource = VID_TRIG_HTTP;
static uint32_t trigTsMs = 0;
static uint32_t recNextSeq = 0;        // next frame seq the recorder will write
static uint32_t recFileNum = 0;        // file number reserved for the clip in flight
static uint32_t vFileIndex = 1;        // next vclip_NNNNN number (1+: 0 is the
                                       // "busy" sentinel returned by videoNotifyTrigger)
static uint32_t lastClipEndMs = 0;
static VidIdx  *vIdx = NULL;

// Continuous (loop) recording: drive the recorder back-to-back into
// cont_NNNNN_v.avi segments. Retention is enforced by cont_record.cpp.
static volatile bool     contVidOn = false;
static volatile uint32_t contVidSegMs = 60000;
static volatile bool     contVidRec = false;
static volatile bool     contVidSdFail = false;
static volatile uint32_t contVidFile = 0;
static uint32_t          contVidCounter = 0; // next number (seeded from SD)
static TaskHandle_t tVideoRec = NULL;

// Runtime config
static volatile uint32_t vClipMs = DEFAULT_VID_CLIP_MS;
static volatile uint32_t vPrerollMs = DEFAULT_VID_PREROLL_MS;
static volatile uint32_t vFps = DEFAULT_VID_FPS;
static volatile bool videoOnAudio = true;  // audio clip also starts video
static volatile bool audioOnVideo = true;  // video trigger also starts audio
static volatile uint32_t vidMaxFiles = DEFAULT_VID_MAX_FILES;
// Cross-trigger cooldown: audio may start a video clip at most once per this
// many ms. Caps the back-to-back recording a noisy room would otherwise drive -
// the sustained camera/SD load that wedges core 1. 0 = unlimited (old behaviour).
static volatile uint32_t vXtrigCdMs = 30000;
// Minimum gap after any automatic clip before motion/audio may trigger another
// (the global clip-rate limiter; 0 = off). HTTP/manual triggers bypass it.
static volatile uint32_t vMinGapMs = 3000;
static bool vXtrigFired = false;
static uint32_t vLastXtrigMs = 0;
// Thermal governor: video (PSRAM-heavy, wedges core 1 when the octal PSRAM is
// over its ~65C-ambient spec) is paused when the die temp is at or above
// vTempCutoffC and runs below it - a single hard cutoff against a 30s moving
// average of the sensor (no resume dead band; the average rejects spikes).
// Audio (core 0) is unaffected. Driven by main.cpp's loop via videoThermalUpdate().
static volatile float vTempCutoffC = 100.0f;
static volatile bool vThermalEnabled = true; // governor on/off (NVS "temp_en")
static volatile bool videoThermalPaused = false;

static uint16_t vidW = 1024, vidH = 768;
static void (*videoWakeCb)() = NULL;

// Motion detector state (working buffers in PSRAM)
static volatile bool motionEnabled = false;
static volatile uint32_t motionBlocks = DEFAULT_MOT_BLOCKS;
static volatile uint32_t motionDiff = DEFAULT_MOT_DIFF;
static volatile uint32_t motCooldownMs = DEFAULT_MOT_COOLDOWN_S * 1000; // trigger refractory
static uint32_t lastMotTrigMs = 0; // millis() of the last motion trigger (cooldown gate)
static volatile uint8_t motBlockSz = DEFAULT_MOT_BLKSZ; // luma px per block side
static volatile int motionLevel = 0;     // changed blocks at the last check
static volatile bool motionGlobal = false; // last check looked like a lighting shift
static uint8_t *motPrev = NULL; // 1/8-scale 8-bit luma frames
static uint8_t *motCur = NULL;
static bool motPrimed = false;
static int motConsec = 0;
static uint16_t motW = 0, motH = 0;
static volatile uint32_t motLastPollMs = 0; // overlay polls keep analysis alive
static TaskHandle_t tVideoMotion = NULL;

// Per-check changed-block bitmaps: the newest (for the live overlay) plus a
// short history ring the recorder snapshots into a clip sidecar file.
struct MotRec {
  uint32_t ts;     // millis() of the check, 0 = never ran
  int16_t  level;  // changed block count
  uint8_t  bx, by; // grid dims at the time
  bool     global; // scene-wide delta (lighting), excluded from triggering
  uint8_t  luma;   // mean frame luma (low light tanks the OV2640 frame rate)
  uint8_t  avgDiff; // mean per-pixel abs luma delta across the frame
  uint8_t  chgDiff; // mean per-pixel abs luma delta over CHANGED blocks only
  uint8_t  map[MOT_MAP_BYTES]; // bit j*bx+i set = block (i,j) changed
};
static MotRec motNow;          // guarded by videoMux
static MotRec *motHist = NULL; // MOT_HIST_MAX entries in PSRAM, videoMux-guarded
static uint32_t motHistWrite = 0, motHistCount = 0;

// Long-run motion-level timeline for the dashboard plot: peak changed-blocks per
// fixed bucket in a PSRAM ring (24h @ 30s = 5.6KB). Written only by the motion
// task; the /video/motion/history endpoint snapshots wr/cnt under videoMux and
// decimates (history.h, same max-pool the audio level plot uses).
#define MOT_LVL_HIST_N     2880u   // 24h of 30s buckets
#define MOT_LVL_BUCKET_MS  30000u
static uint16_t *motLvlHist = NULL;
static uint32_t motLvlWr = 0, motLvlCnt = 0;
// Buckets closed since boot, bumped atomically with motLvlWr: the pooling-grid
// index for /video/motion/history. Must tick with the write index, not the wall
// clock, or polls regroup every column (see history.h planAlignedPool).
static uint32_t motLvlSeq = 0;
static uint16_t motLvlPeak = 0;       // peak within the bucket being filled
static uint32_t motLvlBucketMs = 0;   // start of the current bucket (millis)

// Motion trigger event log (Event-plot markers). Logged when motion crosses the
// threshold, even with no SD — so the timeline shows WHEN motion happened.
// Mirrors the /audio/events shape so the shared Event plot consumes both.
#define MOT_EVT_N 64
struct MotEvt { uint32_t tsMs; uint16_t level; uint16_t thr; uint32_t id; };
static MotEvt *motEvt = NULL;
static uint32_t motEvtWr = 0, motEvtCnt = 0, motEvtNextId = 1;

// User-drawn exclusion mask: bit j*bx+i set = block (i,j) never counts as
// motion (swaying trees, a TV, ...). Edited by clicking blocks on the
// /camera preview overlay. Persisted in NVS together with the grid dims it was
// drawn for; a grid or resolution change remaps it (interpolateMask) so the
// exclusion zone survives.
static uint8_t motMask[MOT_MAP_BYTES]; // guarded by videoMux

static WebServer *videoServer = NULL;

// ---- Settings persistence (NVS, same pattern as audio) ----

static void loadVidSettings() {
  Preferences p;
  if (!p.begin("video", true)) return; // first boot: namespace doesn't exist yet
  videoEnabled = p.getBool("en", videoEnabled);
  vFps = p.getUInt("fps", vFps);
  if (vFps < 2) vFps = 2;
  if (vFps > 10) vFps = 10;
  vClipMs = p.getUInt("clip_ms", vClipMs);
  vPrerollMs = p.getUInt("pre_ms", vPrerollMs);
  if (vPrerollMs >= vClipMs) vPrerollMs = vClipMs / 2;
  videoOnAudio = p.getBool("on_aud", videoOnAudio);
  audioOnVideo = p.getBool("trg_aud", audioOnVideo);
  vXtrigCdMs = p.getUInt("xtrig_cd", vXtrigCdMs);
  vMinGapMs = p.getUInt("min_gap", vMinGapMs);
  vTempCutoffC = p.getFloat("temp_max", vTempCutoffC);
  vThermalEnabled = p.getBool("temp_en", vThermalEnabled);
  vidMaxFiles = p.getUInt("vmax", vidMaxFiles);
  motionEnabled = p.getBool("mot", motionEnabled);
  motionBlocks = p.getUInt("mot_blk", motionBlocks);
  motionDiff = p.getUInt("mot_diff", motionDiff);
  motCooldownMs = p.getUInt("mot_cd", motCooldownMs);
  uint8_t bsz = p.getUChar("mot_bsz", motBlockSz);
  if (bsz == 2 || bsz == 4 || bsz == 8 || bsz == 16) motBlockSz = bsz;
  // Restore the mask, remapping it if it was drawn for a different grid (a
  // grid-fineness or framesize change since it was saved) so an exclusion zone
  // survives across those changes instead of being silently dropped.
  uint8_t gx = (uint8_t)((vidW / 8) / motBlockSz);
  uint8_t gy = (uint8_t)((vidH / 8) / motBlockSz);
  uint8_t mbx = p.getUChar("mot_mask_bx", 0), mby = p.getUChar("mot_mask_by", 0);
  if (mbx > 0 && mby > 0) {
    static uint8_t stored[MOT_MAP_BYTES];
    p.getBytes("mot_mask", stored, MOT_MAP_BYTES);
    if (mbx == gx && mby == gy)
      memcpy(motMask, stored, MOT_MAP_BYTES);
    else
      motion::interpolateMask(stored, mbx, mby, motMask, gx, gy, MOT_MAP_BYTES);
  }
  p.end();
}

static void saveVidSettings() {
  Preferences p;
  if (!p.begin("video", false)) return;
  p.putBool("en", videoEnabled);
  p.putUInt("fps", vFps);
  p.putUInt("clip_ms", vClipMs);
  p.putUInt("pre_ms", vPrerollMs);
  p.putBool("on_aud", videoOnAudio);
  p.putBool("trg_aud", audioOnVideo);
  p.putUInt("xtrig_cd", vXtrigCdMs);
  p.putUInt("min_gap", vMinGapMs);
  p.putFloat("temp_max", vTempCutoffC);
  p.putBool("temp_en", vThermalEnabled);
  p.putUInt("vmax", vidMaxFiles);
  p.putBool("mot", motionEnabled);
  p.putUInt("mot_blk", motionBlocks);
  p.putUInt("mot_diff", motionDiff);
  p.putUInt("mot_cd", motCooldownMs);
  p.putUChar("mot_bsz", motBlockSz);
  p.putBytes("mot_mask", (const void *)motMask, MOT_MAP_BYTES);
  p.putUChar("mot_mask_bx", (uint8_t)(motW / motBlockSz));
  p.putUChar("mot_mask_by", (uint8_t)(motH / motBlockSz));
  p.end();
}

// ---- Tap ring ----

static bool vringAlloc() {
  if (vring != NULL) return true;
  for (size_t i = 0; i < sizeof(VRING_TRY) / sizeof(VRING_TRY[0]); i++) {
    size_t want = VRING_TRY[i];
    if (ESP.getFreePsram() < want + VRING_PSRAM_FLOOR) continue;
    uint8_t *buf = (uint8_t *)ps_malloc(want);
    if (buf == NULL) continue;
    portENTER_CRITICAL(&videoMux);
    vring = buf;
    vringSize = want;
    vWriteOff = 0;
    vqTail = vqCount = 0;
    portEXIT_CRITICAL(&videoMux);
    Serial.printf("[video] tap ring: %uKB PSRAM (~%us pre-roll at %ufps)\n",
                  (unsigned)(want / 1024),
                  (unsigned)(want / (45 * 1024) / vFps), (unsigned)vFps);
    return true;
  }
  Serial.println("[video] tap ring allocation failed - not enough PSRAM");
  return false;
}

/** Free the ring. Caller must ensure no recording is active. */
static void vringFree() {
  uint8_t *buf;
  portENTER_CRITICAL(&videoMux);
  buf = vring;
  vring = NULL;
  vringSize = 0;
  vqCount = 0;
  portEXIT_CRITICAL(&videoMux);
  if (buf == NULL) return;
  while (vringUsers > 0) vTaskDelay(pdMS_TO_TICKS(10)); // let readers finish
  free(buf);
}

void videoTapFrame(const uint8_t *jpg, size_t len) {
  if ((!videoEnabled && !contVidOn) || vring == NULL || len == 0) return;
  uint32_t now = millis();
  if (now - vLastTapMs < 1000 / vFps) return;

  portENTER_CRITICAL(&videoMux);
  if (vring == NULL || len > vringSize) {
    portEXIT_CRITICAL(&videoMux);
    return;
  }
  uint32_t off = vWriteOff;
  if (off + len > vringSize) off = 0; // wrap; tail gap goes unused

  // Evict the oldest frames overlapping the landing zone. Frames the
  // recorder hasn't consumed (or the motion detector pinned) are
  // untouchable: drop this tap instead so camCB never blocks or corrupts.
  bool ok = true;
  while (vqCount > 0) {
    VFrame &f = vq[vqTail];
    bool overlap = (vqCount == VQ_MAX) ||
                   (f.off < off + len && off < f.off + f.len);
    if (!overlap) break;
    uint32_t oldestSeq = vqSeqNext - vqCount;
    if ((recActive && oldestSeq >= recNextSeq) || oldestSeq == vqPinSeq) {
      ok = false;
      break;
    }
    vqTail = (vqTail + 1) % VQ_MAX;
    vqCount--;
  }
  if (!ok) {
    vDrops++;
    portEXIT_CRITICAL(&videoMux);
    return;
  }
  vringUsers++; // covers the memcpy below against vringFree()
  uint32_t gen = vRingGen; // snapshot: if the ring is reset mid-copy, drop this frame
  portEXIT_CRITICAL(&videoMux);

  memcpy(vring + off, jpg, len);

  portENTER_CRITICAL(&videoMux);
  vringUsers--;
  // Skip if the ring was freed (toggle off) OR reset (videoSetDims) during the
  // copy: re-adding at the stale `off`/vWriteOff would corrupt the reset ring.
  if (vring != NULL && vRingGen == gen) {
    VFrame &nf = vq[(vqTail + vqCount) % VQ_MAX];
    nf.off = off;
    nf.len = len;
    nf.tsMs = now;
    vqCount++;
    vqSeqNext++;
    vWriteOff = off + len;
  }
  portEXIT_CRITICAL(&videoMux);

  vLastTapMs = now;
  if (recActive && tVideoRec != NULL) xTaskNotifyGive(tVideoRec);
}

// ---- AVI muxing ----
// Fixed 224-byte header stub; frame count / sizes are patched at close.
// Layout: RIFF @0, LIST hdrl @12 (avih @24, LIST strl @88: strh @100,
// strf @164), LIST movi @212, frames from 224, idx1 appended at the end.

static bool aviWriteHeader(File &f) {
  uint8_t h[avi::HEADER_BYTES];
  avi::buildHeader(h, vidW, vidH, vFps); // pure layout (avi.h, unit-tested)
  return f.write(h, sizeof(h)) == sizeof(h);
}

static bool aviWriteFrame(File &f, const uint8_t *jpg, uint32_t len, uint32_t *relOff) {
  uint8_t hdr[8];
  avi::frameChunkHeader(hdr, len);
  *relOff = (uint32_t)f.position() - avi::MOVI_FOURCC_POS;
  if (f.write(hdr, 8) != 8) return false;
  if (f.write(jpg, len) != len) return false;
  if (len & 1) { // RIFF chunks are word-aligned
    uint8_t z = 0;
    if (f.write(&z, 1) != 1) return false;
  }
  return true;
}

// moviEnd is the file position just past the last FULLY-written frame (the
// caller's goodPos). On a clean finish that's where the cursor already sits;
// after a mid-clip write failure it's behind a dangling partial chunk, so we
// seek back to it before appending idx1 and derive the movi/RIFF length fields
// from it - the partial chunk and any trailing slack fall outside the lengths
// a compliant player reads.
static void aviFinalize(File &f, uint32_t frames, uint32_t moviEnd) {
  f.seek(moviEnd);
  uint8_t hdr[8];
  avi::idx1Header(hdr, frames);
  f.write(hdr, 8);
  for (uint32_t i = 0; i < frames; i++) {
    uint8_t e[16];
    avi::indexEntry(e, vIdx[i].off, vIdx[i].len);
    f.write(e, 16);
  }
  uint32_t fileEnd = f.position();
  avi::Patch patches[4]; // pure size/count derivation (avi.h, unit-tested)
  avi::finalizePatches(frames, moviEnd, fileEnd, patches);
  uint8_t v[4];
  for (auto &pt : patches) { f.seek(pt.off); avi::w32(v, pt.val); f.write(v, 4); }
}

// ---- SD helpers ----

/**
 * Delete oldest vclip files (lowest number) until at most vidMaxFiles remain.
 * Index-based, NO directory scan (see sdEnforceCap): the in-RAM clip index names
 * the oldest .avi; we remove it (and its .mot motion sidecar) and drop it from
 * the index. A per-write full /clips scan held the SD mutex for seconds on a slow
 * card.
 */
static void vidEnforceCap() {
  if (!sdIsAvailable() || vidMaxFiles == 0) return;
  char name[28];
  while (clipIndexCount(CLIP_VIDEO) > vidMaxFiles &&
         clipIndexOldest(CLIP_VIDEO, name, sizeof(name))) {
    String avi = String(VID_CLIP_DIR) + "/" + name;
    xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
    bool removed = SD.remove(avi);
    String mot = avi;
    mot.replace(".avi", ".mot");
    SD.remove(mot); // motion sidecar rides along, if one exists
    String thumb = avi;
    thumb.replace(".avi", ".jpg");
    SD.remove(thumb); // key-frame thumbnail sidecar too
    xSemaphoreGive(sdGetMutex());
    clipIndexRemove(name);
    if (removed) Serial.printf("[video] SD cap (%u): removed %s\n", vidMaxFiles, name);
  }
}

// ---- Recorder task ----

/**
 * Motion-map sidecar (vclip_NNNNN_x.mot): JSON of the changed-block maps
 * observed across the clip window, used by the dashboard player to overlay
 * where motion was during playback. A sidecar because burning boxes into
 * the frames would mean re-encoding every JPEG on-device. Timestamps are
 * ms relative to the clip's first frame. Skipped when no analysis ran.
 */
static void writeMotSidecar(const char *aviPath, uint32_t firstTs, uint32_t lastTs) {
  if (motHist == NULL) return;
  // recorder task only; one entry copied under the mux at a time, the file
  // written incrementally (a fine grid's maps are too big to assemble in RAM)
  static MotRec rec;
  static char hex[MOT_MAP_BYTES * 2 + 1];

  char path[48];
  strlcpy(path, aviPath, sizeof(path));
  char *dot = strrchr(path, '.');
  if (dot == NULL) return;
  strcpy(dot, ".mot");

  portENTER_CRITICAL(&videoMux);
  uint32_t cnt = motHistCount, wr = motHistWrite;
  portEXIT_CRITICAL(&videoMux);

  xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
  File f;
  uint32_t written = 0;
  uint8_t bx = 0, by = 0;
  for (uint32_t i = 0; i < cnt; i++) {
    portENTER_CRITICAL(&videoMux);
    rec = motHist[(wr + MOT_HIST_MAX - cnt + i) % MOT_HIST_MAX];
    portEXIT_CRITICAL(&videoMux);
    if ((int32_t)(rec.ts - (firstTs - 600)) < 0 ||
        (int32_t)((lastTs + 600) - rec.ts) < 0) {
      continue;
    }
    if (written == 0) { // header carries one grid size: open lazily
      f = SD.open(path, FILE_WRITE);
      if (!f) break;
      bx = rec.bx;
      by = rec.by;
      f.printf("{\"bx\":%u,\"by\":%u,\"maps\":[", bx, by);
    } else if (rec.bx != bx || rec.by != by) {
      continue; // grid changed mid-history; skip mismatched entries
    }
    int bytes = (rec.bx * rec.by + 7) / 8;
    for (int b = 0; b < bytes; b++) sprintf(hex + b * 2, "%02x", rec.map[b]);
    f.printf("%s[%ld,%d,\"%s\"]", written ? "," : "",
             (long)(int32_t)(rec.ts - firstTs), rec.level, hex);
    written++;
  }
  if (written > 0) {
    f.print("]}");
    f.close();
  }
  xSemaphoreGive(sdGetMutex());
}

static bool encodeThumbToSd(const char *jpgPath, const uint8_t *src, size_t srcLen,
                            uint16_t w, uint16_t h); // defined below (near the decoders)

/**
 * One clip, write-through: open the AVI, drain the pre-roll from the ring,
 * then follow the producer frame by frame until the post-roll window ends.
 * sdMutex is taken per frame so audio clip writes interleave cleanly.
 */
static void recordClip(uint8_t source, uint32_t trigTs) {
  // Key-frame thumbnail: copy the first frame at/after the trigger out of the
  // ring while we hold it (it may be evicted before the clip finishes), then
  // encode a small .jpg sidecar once the AVI is closed. keyJpg==NULL => not yet
  // captured; if the loop never reaches trigTs (camera stopped) we fall back to
  // the last written frame so a clip always gets a thumbnail.
  uint8_t *keyJpg = NULL;
  uint32_t keyLen = 0;
  uint16_t keyW = 0, keyH = 0;
  bool keyFinal = false; // true once we've grabbed a frame at/after the trigger
  // Pre-roll start: oldest ring frame within vPrerollMs of the trigger.
  portENTER_CRITICAL(&videoMux);
  uint32_t oldestSeq = vqSeqNext - vqCount;
  uint32_t seq = vqSeqNext;
  for (uint32_t i = 0; i < vqCount; i++) {
    const VFrame &f = vq[(vqTail + i) % VQ_MAX];
    // signed: a frame tapped just after the trigger counts too
    if ((int32_t)(trigTs - f.tsMs) <= (int32_t)vPrerollMs) {
      seq = oldestSeq + i;
      break;
    }
  }
  recNextSeq = seq;
  portEXIT_CRITICAL(&videoMux);

  uint32_t fileNum = recFileNum; // reserved at trigger time in videoNotifyTrigger
  char path[48];
  snprintf(path, sizeof(path), VID_CLIP_DIR "/vclip_%05u_%c.avi", fileNum,
           source == VID_TRIG_HTTP ? 'h' : source == VID_TRIG_MOTION ? 'm' : 'a');

  xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
  File f = SD.open(path, FILE_WRITE);
  bool ok = f && aviWriteHeader(f);
  xSemaphoreGive(sdGetMutex());
  if (!ok) {
    if (f) f.close();
    Serial.printf("[video] failed to open %s\n", path);
    return;
  }

  uint32_t postEndTs = trigTs + (vClipMs - vPrerollMs);
  uint32_t frames = 0;
  uint32_t lastTs = 0;
  uint32_t firstTs = 0;
  uint32_t goodPos = avi::HEADER_BYTES; // file position past the last good frame

  while (frames < VID_IDX_MAX && !recAbort) {
    bool have = false;
    uint32_t off = 0, len = 0, ts = 0;
    portENTER_CRITICAL(&videoMux);
    uint32_t oseq = vqSeqNext - vqCount;
    if (recNextSeq < oseq) {
      recNextSeq = oseq; // frame lost before recording pinned it; skip ahead
    }
    if (recNextSeq < vqSeqNext) {
      const VFrame &fr = vq[(vqTail + (recNextSeq - oseq)) % VQ_MAX];
      off = fr.off;
      len = fr.len;
      ts = fr.tsMs;
      have = true;
      vringUsers++;
    }
    portEXIT_CRITICAL(&videoMux);

    if (!have) {
      if (lastTs != 0 && (int32_t)(lastTs - postEndTs) >= 0) break; // done
      if (millis() > postEndTs + 3000) break; // frames stopped arriving
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
      continue;
    }
    if ((int32_t)(ts - postEndTs) > 0) { // first frame past the window
      portENTER_CRITICAL(&videoMux);
      vringUsers--;
      portEXIT_CRITICAL(&videoMux);
      break;
    }

    uint32_t relOff = 0;
    xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
    bool wrote = aviWriteFrame(f, vring + off, len, &relOff);
    uint32_t pos = f.position();
    xSemaphoreGive(sdGetMutex());

    // Grab the key frame while it's still pinned: the first frame at/after the
    // trigger, keeping earlier (pre-roll) frames only as a fallback until then.
    if (wrote && !keyFinal) {
      bool atTrigger = (int32_t)(ts - trigTs) >= 0;
      if (keyJpg == NULL || atTrigger) {
        uint8_t *nb = (uint8_t *)ps_malloc(len);
        if (nb) {
          memcpy(nb, vring + off, len);
          free(keyJpg);
          keyJpg = nb; keyLen = len; keyW = motW; keyH = motH; keyFinal = atTrigger;
        }
      }
    }

    portENTER_CRITICAL(&videoMux);
    vringUsers--;
    recNextSeq++;
    portEXIT_CRITICAL(&videoMux);

    if (!wrote) {
      // A partial '00dc' chunk may sit past goodPos; finalize from goodPos so
      // the movi/RIFF lengths exclude it (SD has no truncate to drop it).
      Serial.printf("[video] SD write failed mid-clip (%s)\n", path);
      sdNotifyWriteFailed(); // card gone/wedged: let the health tick recover it
      break;
    }
    vIdx[frames].off = relOff;
    vIdx[frames].len = len;
    frames++;
    goodPos = pos; // only advances on a fully-written frame
    if (firstTs == 0) firstTs = ts;
    lastTs = ts;
  }

  xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
  if (frames == 0) {
    // no frames made it in (camera down / aborted) - don't leave junk files
    f.close();
    SD.remove(path);
  } else {
    aviFinalize(f, frames, goodPos);
    f.close();
  }
  xSemaphoreGive(sdGetMutex());

  if (frames == 0) {
    free(keyJpg); // aborted clip: no sidecar to write
    Serial.printf("[video] clip %u aborted, no frames (%s)\n", fileNum, path);
    return;
  }
  Serial.printf("[video] clip %u -> %s (%u frames, %s trigger)\n",
                fileNum, path, frames,
                source == VID_TRIG_HTTP ? "http" : source == VID_TRIG_MOTION ? "motion" : "audio");
  // Key-frame thumbnail sidecar (vclip_*.jpg): decode the captured trigger frame
  // to a small JPEG. Non-fatal - a clip without one just shows no thumbnail.
  if (keyJpg) {
    char tpath[48];
    strlcpy(tpath, path, sizeof(tpath));
    char *dot = strrchr(tpath, '.');
    if (dot) { strcpy(dot, ".jpg"); encodeThumbToSd(tpath, keyJpg, keyLen, keyW, keyH); }
    free(keyJpg);
    keyJpg = NULL;
  }
  writeMotSidecar(path, firstTs, lastTs);
  // Index the finished clip (file size = movi end + idx1 header + index entries).
  time_t nowt = time(NULL);
  clipIndexAdd(CLIP_VIDEO, path + sizeof(VID_CLIP_DIR), goodPos + 8 + frames * 16,
               nowt > 1600000000 ? (uint32_t)nowt : 0);
  vidEnforceCap();
}

// One continuous segment: record live frames (no pre-roll) into a fresh
// cont_NNNNN_v.avi until contVidSegMs elapses or the AVI frame cap is hit, then
// finalize. Reuses the same AVI muxer and frame tap as recordClip. Returns
// false on SD failure so the caller can pause the loop.
static bool recordContSegment() {
  portENTER_CRITICAL(&videoMux);
  recNextSeq = vqSeqNext; // start from the next frame to arrive (no pre-roll)
  portEXIT_CRITICAL(&videoMux);

  uint32_t num = contVidCounter;
  char path[48];
  snprintf(path, sizeof(path), VID_CLIP_DIR "/cont_%05u_v.avi", num);

  xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
  File f = SD.open(path, FILE_WRITE);
  bool ok = f && aviWriteHeader(f);
  xSemaphoreGive(sdGetMutex());
  if (!ok) {
    if (f) f.close();
    contVidSdFail = true;
    return false;
  }
  contVidSdFail = false;
  contVidFile = num;
  contVidRec = true;

  uint32_t endMs = millis() + contVidSegMs;
  uint32_t frames = 0;
  uint32_t goodPos = avi::HEADER_BYTES; // file position past the last good frame
  bool sdFail = false;
  while (frames < VID_IDX_MAX && !recAbort && contVidOn &&
         (int32_t)(millis() - endMs) < 0 && sdIsAvailable()) {
    bool have = false;
    uint32_t off = 0, len = 0;
    portENTER_CRITICAL(&videoMux);
    uint32_t oseq = vqSeqNext - vqCount;
    if (recNextSeq < oseq) recNextSeq = oseq; // lost frames: skip ahead
    if (recNextSeq < vqSeqNext) {
      const VFrame &fr = vq[(vqTail + (recNextSeq - oseq)) % VQ_MAX];
      off = fr.off;
      len = fr.len;
      have = true;
      vringUsers++;
    }
    portEXIT_CRITICAL(&videoMux);

    if (!have) {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(250));
      continue;
    }

    uint32_t relOff = 0;
    xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
    bool wrote = aviWriteFrame(f, vring + off, len, &relOff);
    uint32_t pos = f.position();
    xSemaphoreGive(sdGetMutex());

    portENTER_CRITICAL(&videoMux);
    vringUsers--;
    recNextSeq++;
    portEXIT_CRITICAL(&videoMux);

    if (!wrote) { sdFail = true; sdNotifyWriteFailed(); break; } // card gone: recover
    vIdx[frames].off = relOff;
    vIdx[frames].len = len;
    frames++;
    goodPos = pos; // only advances on a fully-written frame
  }

  xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
  if (frames == 0) {
    f.close();
    SD.remove(path);
  } else {
    aviFinalize(f, frames, goodPos);
    f.close();
  }
  xSemaphoreGive(sdGetMutex());

  contVidRec = false;
  if (sdFail) { contVidSdFail = true; return false; }
  if (frames > 0) contVidCounter++;
  return true;
}

static void videoRecTask(void *pvParameters) {
  for (;;) {
    if (otaActive()) { vTaskDelay(pdMS_TO_TICKS(200)); continue; } // idle during OTA flash
    // Continuous mode takes over the recorder: loop segments back to back.
    // Triggered clips are suppressed while it runs (one camera, one recorder).
    if (contVidOn && (videoEnabled || vring != NULL) && sdIsAvailable() &&
        !videoThermalPaused &&   // thermal governor pauses continuous video too
        !sdLowSpace()) {         // combined-budget backstop: pause new segments
                                 // when the card is low on space (see sdLowSpace)
      recActive = true;
      bool okSeg = recordContSegment();
      recActive = false;
      recAbort = false;
      lastClipEndMs = millis();
      if (!okSeg) vTaskDelay(pdMS_TO_TICKS(2000)); // SD trouble: back off, retry
      continue;
    }

    // Wait for a trigger; short timeout so we re-check contVidOn promptly.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) == 0 && !contVidOn) continue;

    uint8_t source;
    uint32_t trigTs;
    portENTER_CRITICAL(&videoMux);
    bool go = trigPending;
    trigPending = false;
    source = trigSource;
    trigTs = trigTsMs;
    if (go) recActive = true;
    portEXIT_CRITICAL(&videoMux);
    if (!go) continue;

    if (videoEnabled && vring != NULL && sdIsAvailable()) {
      recordClip(source, trigTs);
    } else {
      // Trigger was reserved (recFileNum, returned to the caller) but video got
      // disabled / lost SD / lost the ring in the gap before we picked it up.
      // Reclaim the file number so /video/list doesn't advertise a vclip_<id>
      // that never gets written. Safe: once recActive latched above, further
      // triggers join rather than reserve, so nothing took a number after this.
      portENTER_CRITICAL(&videoMux);
      if (vFileIndex == recFileNum + 1) vFileIndex = recFileNum;
      portEXIT_CRITICAL(&videoMux);
    }

    recActive = false;
    recAbort = false;
    lastClipEndMs = millis();
    // disabled mid-recording (but keep the ring if continuous or motion needs it)
    if (!videoEnabled && !contVidOn && !motionEnabled && vring != NULL) vringFree();
  }
}

// ---- On-device motion detection ----

// Direct JPEG -> 1/8-scale luma decode via esp_jpg_decode. We avoid
// jpg2rgb565 because recent esp32-camera builds dither its output (random
// noise per decode that read as avg ~17/pixel "motion" on a static scene).
struct LumaDec {
  const uint8_t *src;
  size_t len;
  uint8_t *luma;        // outW*outH bytes
  uint16_t expW, expH;  // expected scaled dims; mismatch aborts the decode
  uint32_t lumaSum;
};

static size_t lumaReadCb(void *arg, size_t index, uint8_t *buf, size_t len) {
  LumaDec *d = (LumaDec *)arg;
  if (index + len > d->len) len = d->len - index;
  if (buf) memcpy(buf, d->src + index, len);
  return len;
}

static bool lumaWriteCb(void *arg, uint16_t x, uint16_t y, uint16_t w,
                        uint16_t h, uint8_t *data) {
  LumaDec *d = (LumaDec *)arg;
  if (data == NULL) { // start (dims announce) or end marker
    if (x == 0 && y == 0 && w > 0 && (w != d->expW || h != d->expH)) return false;
    return true;
  }
  if ((uint32_t)x + w > d->expW || (uint32_t)y + h > d->expH) return false;
  for (uint16_t iy = 0; iy < h; iy++) {
    uint8_t *o = d->luma + (size_t)(y + iy) * d->expW + x;
    const uint8_t *s = data + (size_t)iy * w * 3;
    for (uint16_t ix = 0; ix < w; ix++, s += 3) {
      uint8_t l = (uint8_t)((s[0] * 77 + s[1] * 150 + s[2] * 29) >> 8);
      o[ix] = l;
      d->lumaSum += l;
    }
  }
  return true;
}

static bool decodeLuma(const uint8_t *jpg, size_t len, uint8_t *out,
                       uint16_t w, uint16_t h, uint32_t *lumaSum) {
  LumaDec d = {jpg, len, out, w, h, 0};
  bool ok = esp_jpg_decode(len, JPG_SCALE_8X, lumaReadCb, lumaWriteCb, &d) == ESP_OK;
  *lumaSum = d.lumaSum;
  return ok;
}

// ---- Key-frame thumbnails ----
// Same 1/8-scale esp_jpg_decode as motion, but kept in colour (RGB888) and
// re-encoded to a tiny JPEG. Colour is worth the extra 2 bytes/px here: a
// thumbnail is for a human deciding which clip to open, not for frame-diffing,
// so jpg2rgb565's dithering (why motion avoids it) is irrelevant - we use the
// same dimension-checked decoder either way.
struct RgbDec {
  const uint8_t *src;
  size_t len;
  uint8_t *rgb;         // expW*expH*3 bytes
  uint16_t expW, expH;  // scaled dims; mismatch aborts (framesize changed)
};

static bool rgbWriteCb(void *arg, uint16_t x, uint16_t y, uint16_t w,
                       uint16_t h, uint8_t *data) {
  RgbDec *d = (RgbDec *)arg;
  if (data == NULL) { // start (dims announce) or end marker
    if (x == 0 && y == 0 && w > 0 && (w != d->expW || h != d->expH)) return false;
    return true;
  }
  if ((uint32_t)x + w > d->expW || (uint32_t)y + h > d->expH) return false;
  for (uint16_t iy = 0; iy < h; iy++) {
    uint8_t *o = d->rgb + ((size_t)(y + iy) * d->expW + x) * 3;
    memcpy(o, data + (size_t)iy * w * 3, (size_t)w * 3); // decoder emits R,G,B
  }
  return true;
}

// Decode `src` at 1/8 scale into an RGB888 buffer, re-encode a small JPEG, and
// write it to `jpgPath` on SD. w/h are the expected 1/8 dims (framesize/8).
// Returns false without touching SD on any failure. ~60-100ms of CPU.
static bool encodeThumbToSd(const char *jpgPath, const uint8_t *src, size_t srcLen,
                            uint16_t w, uint16_t h) {
  if (!src || srcLen == 0 || w == 0 || h == 0) return false;
  size_t rgbLen = (size_t)w * h * 3;
  uint8_t *rgb = (uint8_t *)ps_malloc(rgbLen);
  if (rgb == NULL) return false;
  RgbDec d = {src, srcLen, rgb, w, h};
  bool ok = esp_jpg_decode(srcLen, JPG_SCALE_8X, lumaReadCb, rgbWriteCb, &d) == ESP_OK;
  if (!ok) { free(rgb); return false; }

  uint8_t *jout = NULL;
  size_t jlen = 0;
  ok = fmt2jpg(rgb, rgbLen, w, h, PIXFORMAT_RGB888, THUMB_JPEG_QUALITY, &jout, &jlen);
  free(rgb);
  if (!ok || jout == NULL) { if (jout) free(jout); return false; }

  bool wrote = false;
  xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
  File f = SD.open(jpgPath, FILE_WRITE);
  if (f) { wrote = f.write(jout, jlen) == jlen; f.close(); }
  xSemaphoreGive(sdGetMutex());
  free(jout);
  if (!wrote) SD.remove(jpgPath); // don't leave a half-written thumbnail
  return wrote;
}

// Public: thumbnail the NEWEST ring frame. Snapshots the frame's offset/len
// under the mux, pins it (vringUsers) across a brief memcpy out of the ring, then
// runs the ~80ms decode/encode on the copy so the producer is never stalled.
// Used by the audio clip writer.
bool videoSaveKeyThumb(const char *jpgPath) {
  if (!sdIsAvailable() || vring == NULL) return false;
  uint16_t w = 0, h = 0;
  uint32_t off = 0, len = 0;
  bool have = false;
  portENTER_CRITICAL(&videoMux);
  if (vqCount > 0 && motW > 0 && motH > 0) {
    const VFrame &fr = vq[(vqTail + vqCount - 1) % VQ_MAX];
    off = fr.off; len = fr.len; w = motW; h = motH;
    vringUsers++;              // forbid eviction of these bytes while we copy
    have = true;
  }
  portEXIT_CRITICAL(&videoMux);
  if (!have) return false;

  uint8_t *copy = (uint8_t *)ps_malloc(len);
  if (copy) memcpy(copy, vring + off, len); // off/len stable while pinned
  portENTER_CRITICAL(&videoMux);
  vringUsers--;
  portEXIT_CRITICAL(&videoMux);

  if (copy == NULL) return false;
  bool ok = encodeThumbToSd(jpgPath, copy, len, w, h);
  free(copy);
  return ok;
}

/**
 * Decode the newest tapped frame at 1/8 scale, convert to luma, and count
 * 8x8 blocks whose average per-pixel luma delta against the previous check
 * exceeds motionDiff (luma so white-balance/chroma drift can't masquerade
 * as motion). Runs at priority 1 so it can never starve capture, streaming,
 * or audio; the frame is pinned in the ring during the decode (producer
 * drops a tap rather than overwrite it). Analysis keeps running while a
 * clip records (the overlay and sidecar want the maps) and whenever the
 * /video/motion endpoint is being polled, even with triggering disarmed,
 * so sensitivity can be tuned visually before enabling auto-trigger.
 */
static void videoMotionTask(void *pvParameters) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(MOT_CHECK_MS));
    if (otaActive()) continue; // idle during OTA flash
    bool wantAnalysis = motionEnabled ||
                        millis() - motLastPollMs < MOT_POLL_KEEPALIVE_MS;
    if (!wantAnalysis || !videoEnabled || vring == NULL || motPrev == NULL) {
      motConsec = 0;
      motPrimed = false;
      motionLevel = 0;
      motionGlobal = false;
      continue;
    }

    bool have = false;
    uint32_t off = 0, len = 0;
    uint16_t mw = 0, mh = 0; // snapshot the grid dims: a /camera framesize
                             // change can move motW/motH mid-check, and decode
                             // and diff must agree on the buffer stride.
    portENTER_CRITICAL(&videoMux);
    if (vqCount > 0) {
      const VFrame &f = vq[(vqTail + vqCount - 1) % VQ_MAX];
      off = f.off;
      len = f.len;
      mw = motW;
      mh = motH;
      vqPinSeq = vqSeqNext - 1;
      vringUsers++;
      have = true;
    }
    portEXIT_CRITICAL(&videoMux);
    if (!have) continue;

    uint32_t lumaSum = 0;
    bool decoded = decodeLuma(vring + off, len, motCur, mw, mh, &lumaSum);

    portENTER_CRITICAL(&videoMux);
    vqPinSeq = UINT32_MAX;
    vringUsers--;
    portEXIT_CRITICAL(&videoMux);

    if (!decoded) {
      motPrimed = false;
      continue;
    }

    if (motPrimed) {
      const int blk = motBlockSz; // snapshot: config can change it mid-check
      const int bx = mw / blk, by = mh / blk;
      static uint8_t map[MOT_MAP_BYTES]; // statics: 384B each, this task only
      static uint8_t mask[MOT_MAP_BYTES];
      portENTER_CRITICAL(&videoMux);
      memcpy(mask, (const void *)motMask, MOT_MAP_BYTES);
      portEXIT_CRITICAL(&videoMux);
      // Pure block-diff (motion.h, unit-tested); shell owns timing + consec state.
      motion::Result mr = motion::blockDiff(motCur, motPrev, mw, mh, blk,
                                            motionDiff, mask, map, MOT_MAP_BYTES);
      int changed = mr.changed, active = mr.active;
      bool global = motion::isGlobal(changed, active, MOT_GLOBAL_PCT);
      motionLevel = changed;
      metricsSet(metrics::M_MOTION, (float)changed);
      motionGlobal = global;

      // Fold into the motion-level timeline ring (peak changed-blocks per bucket).
      // Single-writer (this task); the endpoint snapshots wr/cnt under videoMux.
      if (motLvlHist) {
        uint32_t now = millis();
        if (motLvlBucketMs == 0) motLvlBucketMs = now;
        if ((uint16_t)changed > motLvlPeak) motLvlPeak = (uint16_t)changed;
        while ((int32_t)(now - motLvlBucketMs) >= (int32_t)MOT_LVL_BUCKET_MS) {
          portENTER_CRITICAL(&videoMux);
          motLvlHist[motLvlWr] = motLvlPeak;
          motLvlWr = (motLvlWr + 1) % MOT_LVL_HIST_N;
          if (motLvlCnt < MOT_LVL_HIST_N) motLvlCnt++;
          motLvlSeq++;
          portEXIT_CRITICAL(&videoMux);
          motLvlPeak = 0;
          motLvlBucketMs += MOT_LVL_BUCKET_MS;
        }
      }

      // Mean per-pixel delta over UNMASKED blocks only, so a masked region (e.g.
      // a TV) doesn't inflate the displayed "avg motion". Falls back to 0 when
      // every block is masked.
      uint32_t activePx = (uint32_t)active * blk * blk;
      uint32_t avg = activePx ? mr.activeDiffSum / activePx : 0;
      // Mean per-pixel delta over the CHANGED blocks only (the ones drawn red):
      // "how hard is the motion where it's happening", vs avg which dilutes it
      // over the whole scene. 0 when nothing changed.
      uint32_t changedPx = (uint32_t)changed * blk * blk;
      uint32_t chgAvg = changedPx ? mr.changedDiffSum / changedPx : 0;
      portENTER_CRITICAL(&videoMux);
      motNow.ts = millis();
      motNow.level = changed;
      motNow.bx = bx;
      motNow.by = by;
      motNow.global = global;
      motNow.luma = (uint8_t)(lumaSum / ((uint32_t)mw * mh));
      motNow.avgDiff = (uint8_t)(avg > 255 ? 255 : avg);
      motNow.chgDiff = (uint8_t)(chgAvg > 255 ? 255 : chgAvg);
      memcpy(motNow.map, map, MOT_MAP_BYTES);
      if (motHist != NULL) {
        motHist[motHistWrite] = motNow;
        motHistWrite = (motHistWrite + 1) % MOT_HIST_MAX;
        if (motHistCount < MOT_HIST_MAX) motHistCount++;
      }
      portEXIT_CRITICAL(&videoMux);

      // A scene-wide delta is AEC/lighting, not motion: never trigger on it.
      if (global) motConsec = 0;
      else motConsec = changed >= (int)motionBlocks ? motConsec + 1 : 0;
      if (motionEnabled && !recActive && motConsec >= 2 &&
          millis() > MOT_WARMUP_MS &&
          millis() - lastClipEndMs > MOT_HOLDOFF_MS &&
          (lastMotTrigMs == 0 || millis() - lastMotTrigMs >= motCooldownMs)) {
        motConsec = 0;
        lastMotTrigMs = millis(); // start the refractory window (limits repeat triggers)
        Serial.printf("[video] motion trigger: %d/%d blocks changed\n", changed, bx * by);
        // Log the event for the Event-plot timeline (markers) even if no SD clip
        // gets recorded (camera-only boards still want the "when" timeline).
        if (motEvt != NULL) {
          portENTER_CRITICAL(&videoMux);
          motEvt[motEvtWr] = {millis(), (uint16_t)changed, (uint16_t)motionBlocks, motEvtNextId++};
          motEvtWr = (motEvtWr + 1) % MOT_EVT_N;
          if (motEvtCnt < MOT_EVT_N) motEvtCnt++;
          portEXIT_CRITICAL(&videoMux);
        }
        videoNotifyTrigger(VID_TRIG_MOTION);
      }
    }

    uint8_t *t = motPrev;
    motPrev = motCur;
    motCur = t;
    motPrimed = true;
  }
}

// ---- Triggers ----

uint32_t videoNotifyTrigger(uint8_t source) {
  if (!videoEnabled || vring == NULL || !sdIsAvailable()) return 0;
  if (videoThermalPaused) return 0; // too hot for PSRAM-heavy video (see thermal.h)
  if (source == VID_TRIG_AUDIO) {
    if (!videoOnAudio) return 0;
    // Cross-trigger cooldown: cap how often audio spawns a video clip so a
    // noise storm can't drive near-continuous recording (the core-1 wedge).
    if (!ratelimit::allow(vXtrigFired, vLastXtrigMs, millis(), vXtrigCdMs))
      return 0;
  }
  // Holdoff between automatic clips so a trigger storm (e.g. sustained
  // noise) can't chain recordings back to back; manual HTTP always wins.
  if (source != VID_TRIG_HTTP && vMinGapMs != 0 && lastClipEndMs != 0 &&
      millis() - lastClipEndMs < vMinGapMs) {
    return 0;
  }

  // Continuous mode owns the recorder and intentionally suppresses triggered
  // clips (one camera, one recorder). recActive is also true during a cont
  // segment, so don't mistake that for a triggered clip in flight: return a
  // sentinel 0 so callers don't get a phantom id naming a cont_* segment.
  if (contVidOn) return 0;

  uint32_t id;
  bool started = false;
  portENTER_CRITICAL(&videoMux);
  if (recActive || trigPending) {
    id = recFileNum; // join the TRIGGERED clip in flight (same file)
  } else {
    trigPending = true;
    trigSource = source;
    trigTsMs = millis();
    // Reserve the file number now so the returned id names the actual file
    // the recorder will produce (/video/list shows vclip_<id>_*.avi). Wraps at
    // CLIP_NUM_WRAP, skipping 0 (the busy sentinel) so ids stay 1..wrap-1.
    recFileNum = vFileIndex;
    vFileIndex++;
    if (vFileIndex >= CLIP_NUM_WRAP) vFileIndex = 1;
    id = recFileNum;
    started = true;
  }
  portEXIT_CRITICAL(&videoMux);

  if (started) {
    // Record the cooldown anchor only on a real new-clip start (joining an
    // in-flight clip adds no extra load, so it shouldn't reset the timer).
    if (source == VID_TRIG_AUDIO) { vLastXtrigMs = trigTsMs; vXtrigFired = true; }
    metricsEvent(metrics::M_VTRIG);
    if (source != VID_TRIG_AUDIO && audioOnVideo) audioTriggerFromVideo();
    if (tVideoRec != NULL) xTaskNotifyGive(tVideoRec);
  }
  return id;
}

// ---------------- HTTP handlers ----------------

static void handleVideoStatus() {
  uint32_t frames, spanMs = 0;
  portENTER_CRITICAL(&videoMux);
  frames = vqCount;
  if (vqCount > 1) {
    spanMs = vq[(vqTail + vqCount - 1) % VQ_MAX].tsMs - vq[vqTail].tsMs;
  }
  portEXIT_CRITICAL(&videoMux);

  char json[600];
  snprintf(json, sizeof(json),
           "{\"enabled\":%s,\"recording\":%s,\"fps\":%u,\"clip_ms\":%u,"
           "\"preroll_ms\":%u,\"on_audio\":%s,\"trig_audio\":%s,\"xtrig_cd_ms\":%u,"
           "\"min_gap_ms\":%u,"
           "\"temp_max\":%.0f,\"thermal_en\":%s,\"thermal_paused\":%s,"
           "\"ring_kb\":%u,\"ring_frames\":%u,\"ring_span_ms\":%u,"
           "\"drops\":%u,\"vmax\":%u,\"sd\":%s,"
           "\"motion\":%s,\"motion_blocks\":%u,\"motion_level\":%d,"
           "\"motion_diff\":%u,\"motion_global\":%s}",
           videoEnabled ? "true" : "false",
           recActive ? "true" : "false",
           vFps, vClipMs, vPrerollMs,
           videoOnAudio ? "true" : "false",
           audioOnVideo ? "true" : "false", (unsigned)vXtrigCdMs,
           (unsigned)vMinGapMs,
           vTempCutoffC, vThermalEnabled ? "true" : "false",
           videoThermalPaused ? "true" : "false",
           (unsigned)(vringSize / 1024), frames, spanMs,
           vDrops, vidMaxFiles,
           sdIsAvailable() ? "true" : "false",
           motionEnabled ? "true" : "false",
           motionBlocks, motionLevel,
           motionDiff, motionGlobal ? "true" : "false");
  videoServer->send(200, "application/json", json);
}

static void handleVideoTrigger() {
  if (!videoEnabled) {
    videoServer->send(409, "text/plain", "video recording is disabled");
    return;
  }
  if (!sdIsAvailable()) {
    videoServer->send(503, "text/plain", "no sd card - video clips need one");
    return;
  }
  uint32_t id = videoNotifyTrigger(VID_TRIG_HTTP);
  if (id == 0) {
    // No clip reserved: report the actual reason rather than always blaming
    // continuous recording. The thermal governor pauses PSRAM-heavy video when
    // the die is too hot; continuous mode owns the single recorder otherwise.
    videoServer->send(409, "text/plain",
                      videoThermalPaused ? "busy: video paused (too hot) - cooling down"
                                         : "busy: continuous recording active");
    return;
  }
  char json[96];
  snprintf(json, sizeof(json), "{\"id\":%u,\"ready_in_ms\":%u}",
           id, vClipMs - vPrerollMs);
  videoServer->send(200, "application/json", json);
}

static void handleVideoConfig() {
  if (videoServer->hasArg("en")) {
    bool want = videoServer->arg("en") == "1";
    if (want && !videoEnabled) {
      if (!vringAlloc()) {
        videoServer->send(507, "text/plain", "not enough free PSRAM for the tap ring");
        return;
      }
      videoEnabled = true;
      if (videoWakeCb != NULL) videoWakeCb(); // camera may be parked
    } else if (!want && videoEnabled) {
      videoEnabled = false;
      if (recActive) {
        recAbort = true; // recorder finalizes the clip and frees the ring
      } else {
        vringFree(); // reclaim the PSRAM while off
      }
    }
  }
  if (videoServer->hasArg("fps")) {
    uint32_t v = strtoul(videoServer->arg("fps").c_str(), NULL, 10);
    if (v < 2 || v > 10) {
      videoServer->send(400, "text/plain", "fps must be 2..10");
      return;
    }
    vFps = v;
  }
  if (videoServer->hasArg("clip_ms")) {
    if (recActive) {
      videoServer->send(409, "text/plain", "busy: clip recording in progress");
      return;
    }
    uint32_t v = strtoul(videoServer->arg("clip_ms").c_str(), NULL, 10);
    if (v < VID_MIN_CLIP_MS || v > VID_MAX_CLIP_MS) {
      videoServer->send(400, "text/plain",
                        "clip_ms must be " + String(VID_MIN_CLIP_MS) + ".." + String(VID_MAX_CLIP_MS));
      return;
    }
    vClipMs = v;
    if (vPrerollMs >= vClipMs) vPrerollMs = vClipMs / 2;
  }
  if (videoServer->hasArg("preroll_ms")) {
    uint32_t v = strtoul(videoServer->arg("preroll_ms").c_str(), NULL, 10);
    if (v >= vClipMs || v > VID_MAX_PREROLL_MS) {
      videoServer->send(400, "text/plain",
                        "preroll_ms must be < clip_ms and <= " + String(VID_MAX_PREROLL_MS));
      return;
    }
    vPrerollMs = v;
  }
  if (videoServer->hasArg("on_audio")) {
    videoOnAudio = videoServer->arg("on_audio") == "1";
  }
  if (videoServer->hasArg("trig_audio")) {
    audioOnVideo = videoServer->arg("trig_audio") == "1";
  }
  if (videoServer->hasArg("xtrig_cd")) { // audio->video cooldown, seconds (0=off)
    uint32_t v = strtoul(videoServer->arg("xtrig_cd").c_str(), NULL, 10);
    if (v > 3600) v = 3600;
    vXtrigCdMs = v * 1000;
  }
  if (videoServer->hasArg("min_gap")) { // min seconds between auto clips (0=off)
    uint32_t v = strtoul(videoServer->arg("min_gap").c_str(), NULL, 10);
    if (v > 3600) v = 3600;
    vMinGapMs = v * 1000;
  }
  if (videoServer->hasArg("temp_max")) { // thermal-pause cutoff, deg C (60..125)
    float v = atof(videoServer->arg("temp_max").c_str());
    if (v < 60.0f) v = 60.0f;
    if (v > 125.0f) v = 125.0f;
    vTempCutoffC = v;
  }
  if (videoServer->hasArg("temp_en")) { // enable/disable the thermal governor
    vThermalEnabled = videoServer->arg("temp_en") == "1";
    if (!vThermalEnabled) videoThermalPaused = false; // resume video at once
  }
  if (videoServer->hasArg("vmax")) {
    vidMaxFiles = strtoul(videoServer->arg("vmax").c_str(), NULL, 10);
    vidEnforceCap();
  }
  if (videoServer->hasArg("motion")) {
    if (motPrev == NULL && videoServer->arg("motion") == "1") {
      videoServer->send(507, "text/plain", "motion buffers unavailable");
      return;
    }
    motionEnabled = videoServer->arg("motion") == "1";
  }
  if (videoServer->hasArg("motion_block")) { // grid fineness: luma px per side
    uint32_t v = strtoul(videoServer->arg("motion_block").c_str(), NULL, 10);
    if (v != 2 && v != 4 && v != 8 && v != 16) {
      videoServer->send(400, "text/plain", "motion_block must be 2, 4, 8 or 16");
      return;
    }
    if ((uint8_t)v != motBlockSz) {
      static uint8_t oldm[MOT_MAP_BYTES]; // single-threaded web handler
      portENTER_CRITICAL(&videoMux);
      memcpy(oldm, motMask, MOT_MAP_BYTES);
      int obx = motW / motBlockSz, oby = motH / motBlockSz;
      motBlockSz = (uint8_t)v;
      // Remap the mask onto the new grid so a painted exclusion zone survives a
      // fineness change instead of being wiped.
      motion::interpolateMask(oldm, obx, oby, motMask,
                              motW / motBlockSz, motH / motBlockSz, MOT_MAP_BYTES);
      portEXIT_CRITICAL(&videoMux);
    }
  }
  if (videoServer->hasArg("motion_blocks")) {
    uint32_t v = strtoul(videoServer->arg("motion_blocks").c_str(), NULL, 10);
    uint32_t maxBlocks = (motW / motBlockSz) * (motH / motBlockSz);
    if (v >= 1 && v <= maxBlocks) motionBlocks = v;
  }
  if (videoServer->hasArg("motion_diff")) {
    uint32_t v = strtoul(videoServer->arg("motion_diff").c_str(), NULL, 10);
    if (v >= 3 && v <= 60) motionDiff = v;
  }
  if (videoServer->hasArg("motion_cooldown")) { // refractory seconds after a trigger
    uint32_t v = strtoul(videoServer->arg("motion_cooldown").c_str(), NULL, 10);
    if (v <= MOT_COOLDOWN_MAX_S) motCooldownMs = v * 1000;
  }
  { // a coarser grid can leave the trigger threshold unreachable
    uint32_t maxBlocks = (motW / motBlockSz) * (motH / motBlockSz);
    if (motionBlocks > maxBlocks) motionBlocks = maxBlocks;
  }
  if (videoServer->hasArg("motion_mask")) { // hex bitmap, "" or "00..." clears
    String v = videoServer->arg("motion_mask");
    if (v.length() > MOT_MAP_BYTES * 2 || v.length() % 2 != 0) {
      videoServer->send(400, "text/plain", "bad motion_mask");
      return;
    }
    uint8_t m[MOT_MAP_BYTES] = {0};
    for (size_t i = 0; i < v.length(); i += 2) {
      char b[3] = {v[i], v[i + 1], 0};
      char *end;
      long val = strtol(b, &end, 16);
      if (end != b + 2) {
        videoServer->send(400, "text/plain", "bad motion_mask");
        return;
      }
      m[i / 2] = (uint8_t)val;
    }
    portENTER_CRITICAL(&videoMux);
    memcpy((void *)motMask, m, MOT_MAP_BYTES);
    portEXIT_CRITICAL(&videoMux);
  }
  saveVidSettings();
  handleVideoStatus();
}

/**
 * The live changed-block map for debug overlays. Polling this keeps the
 * analysis loop running for MOT_POLL_KEEPALIVE_MS even when auto-trigger
 * is off, so the overlay doubles as a tuning tool. map is hex-packed bits,
 * bit j*bx+i = block (i,j); empty until two checks have run.
 */
static void handleVideoMotion() {
  motLastPollMs = millis();
  // statics: ~3KB of map/mask hex at the finest grid, single-threaded server
  static MotRec r;
  static uint8_t mask[MOT_MAP_BYTES];
  static char map[MOT_MAP_BYTES * 2 + 1];
  static char maskHex[MOT_MAP_BYTES * 2 + 1];
  static char json[256 + MOT_MAP_BYTES * 4];
  portENTER_CRITICAL(&videoMux);
  r = motNow;
  memcpy(mask, (const void *)motMask, MOT_MAP_BYTES);
  portEXIT_CRITICAL(&videoMux);

  bool analyzing = motPrev != NULL && videoEnabled && vring != NULL;
  // The analysis task is priority 1 on core 0; an active live stream (priority
  // 5, same core) starves it, so checks can slip well past one MOT_CHECK_MS -
  // with one slow phone viewer attached ~40% of polls used to catch the map
  // >5s stale. Blanking it then made the overlay flap between the live map and
  // "warming up", so instead we ALWAYS send the last map (as long as its grid
  // still applies) plus its age_ms, and the overlay reports staleness honestly.
  // "fresh" (<5s) is kept for the level/trigger fields shown as live numbers.
  bool haveMap = r.ts != 0 && r.bx == motW / motBlockSz && r.by == motH / motBlockSz;
  bool fresh = haveMap && millis() - r.ts < 5000;
  map[0] = 0;
  if (haveMap) {
    int bytes = (r.bx * r.by + 7) / 8;
    for (int i = 0; i < bytes; i++) sprintf(map + i * 2, "%02x", r.map[i]);
  }
  // bx/by/mask reflect the current grid even while maps warm up, so the
  // mask editor works immediately
  uint16_t bx = motW / motBlockSz, by = motH / motBlockSz;
  int maskBytes = (bx * by + 7) / 8;
  for (int i = 0; i < maskBytes; i++) sprintf(maskHex + i * 2, "%02x", mask[i]);
  maskHex[maskBytes * 2] = 0;

  snprintf(json, sizeof(json),
           "{\"enabled\":%s,\"analyzing\":%s,\"fresh\":%s,\"age_ms\":%lu,\"bx\":%u,\"by\":%u,"
           "\"blk\":%u,\"level\":%d,\"thresh\":%u,\"diff\":%u,\"cooldown\":%u,\"global\":%s,"
           "\"luma\":%u,\"avg_diff\":%u,\"chg_diff\":%u,\"map\":\"%s\",\"mask\":\"%s\"}",
           motionEnabled ? "true" : "false", analyzing ? "true" : "false",
           fresh ? "true" : "false",
           haveMap ? (unsigned long)(millis() - r.ts) : 0UL, bx, by, motBlockSz,
           haveMap ? r.level : 0,
           motionBlocks, motionDiff, motCooldownMs / 1000,
           haveMap && r.global ? "true" : "false", haveMap ? r.luma : 0,
           haveMap ? r.avgDiff : 0, haveMap ? r.chgDiff : 0, map, maskHex);
  videoServer->send(200, "application/json", json);
}

// Motion-level timeline for the shared Event plot — SAME binary contract as
// /audio/history: a Uint16 array (peak changed-blocks per decimated bucket) with
// an X-Bucket-Ms header. ?secs= window (60..86400), ?points= resolution
// (16..1440), ?end= ms-ago of the right edge (for drag-zoom), ?thr=1 returns the
// (flat) trigger-threshold series instead of the level.
static void handleMotionHistory() {
  motLastPollMs = millis(); // keep analysis alive while the plot is open
  static uint16_t out[1440];
  uint32_t secs = videoServer->hasArg("secs")
                      ? strtoul(videoServer->arg("secs").c_str(), NULL, 10) : 3600;
  if (secs < 60) secs = 60;
  if (secs > 86400) secs = 86400;
  uint32_t points = videoServer->hasArg("points")
                        ? strtoul(videoServer->arg("points").c_str(), NULL, 10) : 720;
  if (points < 16) points = 16;
  if (points > 1440) points = 1440;
  uint32_t endMs = videoServer->hasArg("end")
                       ? strtoul(videoServer->arg("end").c_str(), NULL, 10) : 0;
  bool thr = videoServer->hasArg("thr") && videoServer->arg("thr") == "1";

  uint32_t cnt, w, seq;
  portENTER_CRITICAL(&videoMux);
  cnt = motLvlCnt; w = motLvlWr; seq = motLvlSeq;
  portEXIT_CRITICAL(&videoMux);

  const uint32_t bucketMs = MOT_LVL_BUCKET_MS;
  uint32_t endBuckets = endMs / bucketMs;
  uint32_t avail = history::availableBuckets(cnt, endBuckets);
  uint32_t want = secs * 1000UL / bucketMs;
  if (want > avail) want = avail;

  // Grid-aligned, integer-factor pooling (history.h) so the line's historical
  // columns are stable between refreshes - no shimmer. The grid index is the
  // ring's own close counter (ticks with the write index; a wall-clock index
  // regrouped every column when a poll split the two ticks). Same plan for both
  // the level and threshold series so they stay 1:1 aligned on the client.
  history::PoolPlan pl =
      history::planAlignedPool(want, points, cnt, endBuckets, seq);
  want = pl.want;
  points = pl.columns;
  endBuckets += pl.extraEnd;

  if (thr) {
    // Flat threshold series (motion cutoff is a scalar): one value per aligned col.
    for (uint32_t i = 0; i < points; i++) out[i] = (uint16_t)motionBlocks;
  } else {
    points = motLvlHist ? history::decimateMaxPool(motLvlHist, MOT_LVL_HIST_N, w,
                                                   endBuckets, want, points, out)
                        : 0;
  }

  WiFiClient client = videoServer->client();
  char hdr[192];
  int hl = snprintf(hdr, sizeof(hdr),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/octet-stream\r\n"
                    "Content-Length: %u\r\n"
                    "X-Bucket-Ms: %u\r\n"
                    "Connection: close\r\n\r\n",
                    (unsigned)(points * 2),
                    points ? (unsigned)(want * bucketMs / points) : 0);
  client.write(hdr, hl);
  if (points > 0) client.write((uint8_t *)out, points * 2);
  client.stop();
}

// Motion trigger events for the Event-plot markers — same JSON shape as
// /audio/events: newest first, [{age_ms,id,sd,rms,thr,hz,source}]. (sd=-1, hz=0;
// rms carries the changed-block count so the plot's "×over" readout works.)
static void handleMotionEvents() {
  uint32_t secs = videoServer->hasArg("secs")
                      ? strtoul(videoServer->arg("secs").c_str(), NULL, 10) : 3600;
  if (secs < 60) secs = 60;
  if (secs > 86400) secs = 86400;
  uint32_t cnt, w, now = millis();
  portENTER_CRITICAL(&videoMux);
  cnt = motEvtCnt; w = motEvtWr;
  portEXIT_CRITICAL(&videoMux);

  String json = "[";
  json.reserve(cnt * 80 + 4);
  int emitted = 0;
  for (uint32_t k = 0; k < cnt && motEvt != NULL; k++) {
    const MotEvt &ev = motEvt[(w + MOT_EVT_N - 1 - k) % MOT_EVT_N];
    uint32_t age = now - ev.tsMs;
    if (age > secs * 1000UL) break;
    if (emitted++) json += ",";
    json += "{\"age_ms\":" + String(age);
    json += ",\"id\":" + String(ev.id);
    json += ",\"sd\":-1,\"rms\":" + String(ev.level);
    json += ",\"thr\":" + String(ev.thr);
    json += ",\"hz\":0,\"source\":\"motion\"}";
  }
  json += "]";
  videoServer->send(200, "application/json", json);
}

static void handleVideoList() {
  if (!sdIsAvailable()) {
    videoServer->send(503, "application/json", "{\"error\":\"no sd card\"}");
    return;
  }
  // Served from the in-RAM index - NO SD access, so it can't 503 on bus
  // contention and is near-instant even on a slow card.
  String json;
  json.reserve(4096);
  clipIndexListJson(CLIP_VIDEO, VID_LIST_MAX, json);
  videoServer->send(200, "application/json", json);
}

static void handleVideoClear() {
  if (!sdIsAvailable()) {
    videoServer->send(503, "text/plain", "no sd card");
    return;
  }
  // Collect-then-delete in batches, same as the audio /sd/clear.
  int deleted = 0;
  for (;;) {
    char names[16][32];
    int n = 0;
    xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
    File dir = SD.open(VID_CLIP_DIR);
    if (dir) {
      File entry;
      while (n < 16 && (entry = dir.openNextFile())) {
        const char *base = strrchr(entry.name(), '/');
        base = base ? base + 1 : entry.name();
        if (strncmp(base, "vclip_", 6) == 0) {
          strlcpy(names[n++], base, sizeof(names[0]));
        }
        entry.close();
      }
      dir.close();
    }
    for (int i = 0; i < n; i++) {
      if (SD.remove(String(VID_CLIP_DIR) + "/" + names[i])) deleted++;
    }
    xSemaphoreGive(sdGetMutex());
    if (n == 0) break;
  }
  clipIndexClear(CLIP_VIDEO); // all video clips gone
  char json[48];
  snprintf(json, sizeof(json), "{\"deleted\":%d}", deleted);
  videoServer->send(200, "application/json", json);
}

// ---------------- Public API ----------------

bool videoInit(uint16_t w, uint16_t h) {
  vidW = w;
  vidH = h;
  loadVidSettings();

  vIdx = (VidIdx *)ps_malloc(VID_IDX_MAX * sizeof(VidIdx));
  if (vIdx == NULL) {
    Serial.println("[video] frame index allocation failed");
    return false;
  }

  if (videoEnabled && !vringAlloc()) {
    videoEnabled = false; // recoverable later via /video/config?en=1
  }

  // Motion detector working buffers (two 1/8-scale luma frames), sized for
  // the largest runtime framesize (XGA -> 128x96) so resolution changes
  // via /camera/config never need a realloc.
  motW = vidW / 8;
  motH = vidH / 8;
  motPrev = (uint8_t *)ps_malloc((size_t)(1024 / 8) * (768 / 8));
  motCur = (uint8_t *)ps_malloc((size_t)(1024 / 8) * (768 / 8));
  motHist = (MotRec *)ps_calloc(MOT_HIST_MAX, sizeof(MotRec)); // ~25KB
  motLvlHist = (uint16_t *)ps_calloc(MOT_LVL_HIST_N, sizeof(uint16_t)); // ~5.6KB timeline
  motEvt = (MotEvt *)ps_calloc(MOT_EVT_N, sizeof(MotEvt));              // Event-plot markers
  if (motPrev == NULL || motCur == NULL) {
    Serial.println("[video] motion buffer alloc failed - motion detection disabled");
    free(motPrev);
    free(motCur);
    motPrev = motCur = NULL;
    motionEnabled = false;
  }

  // Resume vclip numbering from whatever is already on the card AND populate the
  // in-RAM video clip index so the list + cap never scan again.
  clipIndexClear(CLIP_VIDEO);
  if (sdIsAvailable()) {
    xSemaphoreTake(sdGetMutex(), portMAX_DELAY);
    File dir = SD.open(VID_CLIP_DIR);
    if (dir) {
      File entry;
      // Bounded like the audio boot scan: a corrupt FAT directory chain can
      // loop, feeding entries forever and hanging boot.
      uint32_t scanned = 0;
      while (scanned < 20000 && (entry = dir.openNextFile())) {
        scanned++;
        esp_task_wdt_reset(); // don't let a slow healthy-card scan trip the boot WDT
        unsigned idx = 0;
        const char *base = strrchr(entry.name(), '/');
        base = base ? base + 1 : entry.name();
        const char *ext = strrchr(base, '.');
        if (sscanf(base, "vclip_%u", &idx) == 1 && ext && strcmp(ext, ".avi") == 0) {
          clipIndexAdd(CLIP_VIDEO, base, entry.size(), (uint32_t)entry.getLastWrite());
        }
        unsigned cidx = 0;
        if (sscanf(base, "cont_%u_v.avi", &cidx) == 1 && cidx + 1 > contVidCounter) {
          contVidCounter = cidx + 1; // resume continuous numbering too
        }
        entry.close();
      }
      if (scanned >= 20000)
        dlog(DLOG_ERR, "video boot scan stopped at %u entries - FAT directory "
             "chain suspect, reformat advised", (unsigned)scanned);
      dir.close();
    }
    xSemaphoreGive(sdGetMutex());
  }
  // Resume just past the newest existing vclip (wrap-aware). Never 0: that's the
  // "busy/suppressed" sentinel videoNotifyTrigger returns, so clip ids stay 1+.
  vFileIndex = clipIndexNextNum(CLIP_VIDEO);
  if (vFileIndex == 0) vFileIndex = 1;

  Serial.printf("[video] recorder ready: %ux%u @ %ufps, clip %us (pre-roll %us), next file %u\n",
                vidW, vidH, (unsigned)vFps, (unsigned)(vClipMs / 1000),
                (unsigned)(vPrerollMs / 1000), vFileIndex);
  return true;
}

bool videoSetDims(uint16_t w, uint16_t h) {
  if (recActive || trigPending) return false;
  static uint8_t oldm[MOT_MAP_BYTES]; // single-threaded (web/config path)
  portENTER_CRITICAL(&videoMux);
  int obx = motW / motBlockSz, oby = motH / motBlockSz; // grid before the change
  memcpy(oldm, motMask, MOT_MAP_BYTES);
  vidW = w;
  vidH = h;
  motW = w / 8;
  motH = h / 8;
  motPrimed = false; // previous 1/8-scale frame has the old dimensions
  // Remap the mask onto the new (resolution-derived) grid so it survives a
  // framesize change instead of being wiped.
  motion::interpolateMask(oldm, obx, oby, motMask,
                          motW / motBlockSz, motH / motBlockSz, MOT_MAP_BYTES);
  vqTail = 0;
  vqCount = 0;
  vWriteOff = 0;
  vRingGen++; // invalidate any tap memcpy in flight (see videoTapFrame)
  portEXIT_CRITICAL(&videoMux);
  return true;
}

// Capture must keep running (camera unparked) for recording OR motion detection.
bool videoCaptureActive() { return (videoEnabled || contVidOn || motionEnabled) && vring != NULL; }

// Thermal governor (pure logic in thermal.h): smooth the noisy die sensor with
// a 30s moving average, then apply the single hard cutoff. main.cpp samples on
// its 5s system tick, so 6 samples ~= 30s. Returns true on a state change so
// the caller logs it.
#define THERMAL_AVG_SAMPLES 6
static thermal::MovingAvg<THERMAL_AVG_SAMPLES> vTempAvg;
bool videoThermalUpdate(float dieTempC) {
  bool was = videoThermalPaused;
  float avg = vTempAvg.push(dieTempC); // keep the average warm even when disabled
  // Governor disabled: never pause on temperature (user opt-out of the throttle).
  videoThermalPaused =
      vThermalEnabled ? thermal::overCutoff(avg, vTempCutoffC) : false;
  return videoThermalPaused != was;
}
bool videoThermalPausedNow() { return videoThermalPaused; }

void videoSetWakeCallback(void (*fn)()) { videoWakeCb = fn; }

void videoContSet(bool on, uint32_t segMs) {
  if (segMs) contVidSegMs = segMs;
  if (on && !contVidOn) {
    // Continuous needs the frame tap; bring it up (and wake a parked camera)
    // even if the trigger-video master toggle is off.
    if (vring == NULL) vringAlloc();
    contVidOn = true;
    if (tVideoRec != NULL) xTaskNotifyGive(tVideoRec); // kick the loop
    if (videoWakeCb != NULL) videoWakeCb();
  } else if (!on && contVidOn) {
    contVidOn = false;
    if (recActive) recAbort = true; // end the current segment promptly
  }
}

String videoContJson() {
  String s;
  s.reserve(80);
  s += "\"v_on\":";   s += contVidOn ? "true" : "false";
  s += ",\"v_rec\":"; s += contVidRec ? "true" : "false";
  s += ",\"v_file\":"; s += String(contVidFile);
  s += ",\"v_sd\":";  s += contVidSdFail ? "false" : "true";
  return s;
}

void videoStart() {
  // Core 0 with the other I/O tasks; above sdWriter (2), below audio (4).
  xTaskCreatePinnedToCore(videoRecTask, "video_rec", 4096, NULL, 3, &tVideoRec, 0);
  // An audio cross-trigger can land between audioStart() and here (boot
  // transients do this); without the task to notify, trigPending would
  // stay latched forever. Deliver the missed wakeup.
  if (trigPending) xTaskNotifyGive(tVideoRec);
  if (motPrev != NULL) {
    // Priority 1: motion checks only ever use otherwise-idle cycles. Pinned to
    // core 1 (APP_CPU), NOT core 0: on core 0 it is the lowest-priority task
    // under the stream sender (streamCB, 5) + audio (4), so an active live
    // stream starved it and the debug overlay froze mid-scene. Core 1 hosts
    // only camCB (5, blocked on sensor DMA ~70% of each frame) and the WebServer
    // tick (mjpegCB, 5, sleeps between 8ms handles) + the delay(100) Arduino
    // loop, so there is ample idle time here EVEN while core 0 streams — the
    // overlay now refreshes in real time regardless of viewers. Cross-core safe:
    // ring access is under the videoMux portMUX spinlock and esp_jpg_decode runs
    // on per-call local state with its own output buffers (motCur/motPrev).
    xTaskCreatePinnedToCore(videoMotionTask, "video_motion", 6144, NULL, 1, &tVideoMotion, 1);
  }
}

static WebServer::THandlerFunction vidGuarded(void (*h)()) {
  return [h]() {
    if (!webAuthCheck(*videoServer)) return;
    h();
  };
}

// Mutating routes: reject the read-only viewer with 403.
static WebServer::THandlerFunction vidGuardedW(void (*h)()) {
  return [h]() {
    if (!webAuthRequireWrite(*videoServer)) return;
    h();
  };
}

void videoRegisterEndpoints(WebServer &server) {
  videoServer = &server;
  server.on("/video/trigger", HTTP_GET, vidGuardedW(handleVideoTrigger));
  server.on("/video/status", HTTP_GET, vidGuarded(handleVideoStatus));
  server.on("/video/config", HTTP_GET, vidGuardedW(handleVideoConfig));
  server.on("/video/list", HTTP_GET, vidGuarded(handleVideoList));
  server.on("/video/clear", HTTP_GET, vidGuardedW(handleVideoClear));
  server.on("/video/motion", HTTP_GET, vidGuarded(handleVideoMotion));
  server.on("/video/motion/history", HTTP_GET, vidGuarded(handleMotionHistory));
  server.on("/video/motion/events", HTTP_GET, vidGuarded(handleMotionEvents));
}


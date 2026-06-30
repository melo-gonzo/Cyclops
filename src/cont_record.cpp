// cont_record.cpp - see cont_record.h

#if defined(CAMERA_MODEL_XIAO_ESP32S3)

#include "cont_record.h"

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
  audioContSet(cAudOn, cSegS * 1000);
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
  j += ',' + audioContJson();
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

static const char REC_PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>)rawliteral" DEVICE_NAME R"rawliteral( - Continuous</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#14171c;color:#dde3ea;margin:0;padding:16px;max-width:820px;margin:auto}
a{color:#6fb3ff}
.card{background:#1d2229;border-radius:10px;padding:14px;margin-bottom:14px}
h3{font-size:.85em;color:#8a94a0;font-weight:500;margin:0 0 10px;text-transform:uppercase;letter-spacing:.05em}
#nav{display:flex;gap:6px;align-items:center;margin:4px 0 16px;flex-wrap:wrap}
#nav b{margin-right:10px}
#nav a{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}
#nav a.cur{background:#1d2229;color:#dde3ea}
button{background:#3d9df0;color:#fff;border:0;border-radius:8px;padding:8px 14px;font-size:.9em;cursor:pointer}
button.warn{background:#2a313a}
label{font-size:.9em;color:#aab4c0}
input[type=number]{background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:7px;width:80px}
.row{display:flex;flex-wrap:wrap;gap:14px;align-items:center}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
.on{background:#5df0a0}.off{background:#3a424d}.rec{background:#f05d5d}
.note{font-size:.85em;color:#8a94a0}
.bad{color:#f05d5d}
table{width:100%;border-collapse:collapse;font-size:.9em}
td,th{padding:7px 6px;text-align:left;border-bottom:1px solid #2a313a}
th{color:#8a94a0;font-weight:500;font-size:.85em}
.badge{font-size:.75em;padding:1px 7px;border-radius:10px;background:#2a313a;color:#aab4c0}
.badge.v{background:#3a2a4d;color:#c08cff}.badge.a{background:#243a4d;color:#6fb3ff}
audio{height:30px;vertical-align:middle}
body.viewer input,body.viewer select,body.viewer button{pointer-events:none;opacity:.4}
#vbadge{display:none;background:#2a313a;color:#7fd6a0;border-radius:8px;padding:3px 9px;font-size:.8em}
body.viewer #vbadge{display:inline-block}
</style></head><body>
<div id="nav"></div>

<div class="card"><h3>Continuous recording</h3>
  <div id="state" class="note">-</div>
  <div id="sdwarn" class="bad" style="margin-top:6px;display:none">SD card unavailable - recording paused.</div>
  <div id="lowwarn" class="bad" style="margin-top:6px;display:none">Card nearly full - continuous recording paused; triggered clips still save.</div>
</div>

<div class="card"><h3>Settings</h3>
  <div class="row" style="margin-bottom:12px">
    <label><input type="checkbox" id="caud"> record audio</label>
    <label><input type="checkbox" id="cvid"> record video</label>
  </div>
  <div class="row">
    <label>segment length <input type="number" id="seg" min="15" max="600" step="5"> s</label>
    <label>keep <input type="number" id="min" min="1" max="240"> min/stream</label>
    <label>disk cap <input type="number" id="mb" min="50" max="100000" step="50"> MB</label>
    <button onclick="apply()">apply</button>
  </div>
  <div class="note" style="margin-top:10px;line-height:1.5">
    Segments write straight to SD (bypassing the clip slots); oldest are deleted
    once either cap is reached. Audio &asymp; 1.9 MB/min; video &asymp; 30 MB/min at the
    current resolution/fps. Video segments cap at ~120 s regardless of the
    setting (AVI frame limit), and enabling continuous video takes over the
    recorder, suspending motion/audio-triggered clips while it runs.
  </div>
</div>

<div class="card"><h3>Segments <span id="usage" style="float:right;font-weight:400" class="note"></span></h3>
  <table><thead><tr><th>time</th><th>type</th><th>size</th><th>play / download</th><th></th></tr></thead>
  <tbody id="segs"></tbody></table>
  <div class="row" style="margin-top:10px">
    <button class="warn" onclick="clrAll()">delete all continuous files</button>
  </div>
</div>

<script>
const esc=s=>s.replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/"/g,"&quot;");
const fmtTs=ms=>{const d=new Date(ms);return d.toLocaleString([],{month:"short",day:"numeric",hour:"2-digit",minute:"2-digit",second:"2-digit"})};
const fmtSz=b=>b>1048576?(b/1048576).toFixed(1)+" MB":(b/1024).toFixed(0)+" KB";
function dotState(on,rec,sd,low){
  if(!on)return "<span class='dot off'></span>off";
  if(!sd)return "<span class='dot rec'></span>paused (SD error)";
  if(low)return "<span class='dot rec'></span>paused (disk full)";
  return rec?"<span class='dot rec'></span>recording":"<span class='dot on'></span>armed";
}
async function refreshStatus(){
  try{
    const s=await(await fetch("/rec/status")).json();
    document.body.classList.toggle('viewer',!!s.viewer); // read-only: lock controls
    if(document.activeElement!==caud)caud.checked=s.aud;
    if(document.activeElement!==cvid)cvid.checked=s.vid;
    if(document.activeElement!==seg)seg.value=s.seg_s;
    if(document.activeElement!==min)min.value=s.keep_min;
    if(document.activeElement!==mb)mb.value=s.keep_mb;
    state.innerHTML="<b>Audio</b> "+dotState(s.aud,s.a_rec,s.a_sd,s.sd_low)
      +" &nbsp;&nbsp; <b>Video</b> "+dotState(s.vid,s.v_rec,s.v_sd,s.sd_low);
    sdwarn.style.display=s.sd?"none":"";
    lowwarn.style.display=(s.sd&&s.sd_low)?"":"none";
    usage.textContent=s.files+" files · "+s.used_mb+" MB on disk (cap "+s.keep_mb
      +" MB) · "+s.sd_free_mb+" MB free";
  }catch(e){}
}
async function refreshSegs(){
  try{
    const list=await(await fetch("/sd/list")).json();
    const segs=list.filter(f=>/^cont_\d+_[av]\./.test(f.name))
      .sort((a,b)=>b.mtime-a.mtime);
    segs_.innerHTML=segs.map(f=>{
      const vid=f.name.endsWith(".avi");
      const url="/sd/file?name="+encodeURIComponent(f.name);
      const dur=vid?"":" · "+Math.round((f.size-44)/32000)+"s";
      const play=vid
        ?"<a href='"+url+"'>download .avi</a>"
        :"<audio controls preload='none' src='"+url+"'></audio> <a href='"+url+"'>wav</a>";
      return "<tr><td>"+(f.mtime>1600000000?fmtTs(f.mtime*1000):"—")+dur+"</td>"
        +"<td><span class='badge "+(vid?"v":"a")+"'>"+(vid?"VIDEO":"AUDIO")+"</span></td>"
        +"<td>"+fmtSz(f.size)+"</td><td>"+play+"</td>"
        +"<td style='text-align:right'><a href='#' onclick='del(\""+esc(f.name)+"\");return false'>delete</a></td></tr>";
    }).join("")||"<tr><td class='note' colspan='5'>no continuous segments yet</td></tr>";
  }catch(e){}
}
async function apply(){
  const q="?aud="+(caud.checked?1:0)+"&vid="+(cvid.checked?1:0)
    +"&seg="+(+seg.value||60)+"&min="+(+min.value||30)+"&mb="+(+mb.value||2000);
  await fetch("/rec/config"+q);refreshStatus();
}
async function del(name){
  if(!confirm("Delete "+name+"?"))return;
  await fetch("/sd/delete?name="+encodeURIComponent(name));refreshSegs();
}
async function clrAll(){
  if(!confirm("Delete ALL continuous segments and stop continuous recording?"))return;
  await fetch("/rec/clear");refreshStatus();refreshSegs();
}
const segs_=document.getElementById("segs");
refreshStatus();refreshSegs();
setInterval(refreshStatus,3000);
setInterval(refreshSegs,5000);
</script><script src="/ui.js"></script>
<script>buildNav('/rec')</script>
</body></html>
)rawliteral";

static void handlePage() {
  if (!authOk()) return;
  cServer->send_P(200, "text/html", REC_PAGE);
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

#endif // CAMERA_MODEL_XIAO_ESP32S3

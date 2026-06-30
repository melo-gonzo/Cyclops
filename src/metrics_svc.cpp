// metrics_svc.cpp - imperative shell for the diagnostics time-series.
//
// Per-metric uint16 PSRAM rings on one shared time base, a tiny set/event API
// any module can feed, a sampler that snapshots one bucket per tick, and the
// HTTP endpoints behind the "/graphs" page. The encoding/metric set is the pure,
// unit-tested metrics.h; the ring decimation is the unit-tested history.h.

#include "metrics_svc.h"
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

// Change the retention window (minutes). Restarts the ring because existing
// buckets were captured at the old cadence, and persists to NVS.
void metricsSetWindowMin(uint32_t m) {
  if (m < MET_WIN_MIN_MIN || m > MET_WIN_MAX_MIN || m == g_windowMin) return;
  g_windowMin = m;
  g_write = 0;
  g_count = 0;
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

static const char GRAPHS_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>)HTML" DEVICE_NAME R"HTML( - Graphs</title>
<style>
*{box-sizing:border-box}
body{margin:0;background:#11151b;color:#dde3ea;font-family:system-ui,sans-serif;padding:14px}
#nav{display:flex;gap:6px;align-items:center;margin:0 0 14px;flex-wrap:wrap}
#nav b{margin-right:10px}
#nav a{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}
#nav a.cur{background:#1d2229;color:#dde3ea}
.card{background:#171c23;border:1px solid #232a33;border-radius:12px;padding:12px;margin-bottom:14px}
.bar{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-bottom:10px}
.rng{padding:5px 12px;border:1px solid #2a313a;background:#1d2229;color:#cdd6df;border-radius:8px;cursor:pointer;font-size:.85em}
.rng.cur{background:#3d9df0;border-color:#3d9df0;color:#06121f}
.spacer{flex:1}
.lnk{color:#8a94a0;cursor:pointer;font-size:.82em;text-decoration:underline}
#wrap{position:relative}
canvas{width:100%;height:340px;display:block;background:#0d1116;border-radius:8px}
#legend{display:flex;flex-wrap:wrap;gap:6px 14px;margin-top:12px}
.mi{display:flex;align-items:center;gap:7px;cursor:pointer;font-size:.85em;padding:3px 6px;border-radius:7px;user-select:none}
.mi.off{opacity:.4}
.sw{width:13px;height:13px;border-radius:3px;flex:none}
.mi .lab{color:#cdd6df}
.mi .val{color:#8a94a0;font-variant-numeric:tabular-nums}
#tip{position:absolute;pointer-events:none;display:none;background:#06121f;border:1px solid #2a313a;border-radius:8px;padding:7px 9px;font-size:.78em;z-index:5;min-width:120px}
#tip .tr{display:flex;justify-content:space-between;gap:12px}
#tip .tt{color:#8a94a0;margin-bottom:4px}
.hint{color:#5d6773;font-size:.78em;margin-top:6px}
body.viewer select,body.viewer input,body.viewer button{pointer-events:none;opacity:.4}
#vbadge{display:none;background:#2a313a;color:#7fd6a0;border-radius:8px;padding:3px 9px;font-size:.8em}
body.viewer #vbadge{display:inline-block}
</style></head><body>
<div id="nav"></div>
<div class="card">
  <div class="bar">
    <span class="rng" data-s="300">5m</span>
    <span class="rng" data-s="900">15m</span>
    <span class="rng cur" data-s="3600">1h</span>
    <span class="rng" data-s="0">All</span>
    <select id="retwin" onchange="setRetWin(this.value)" title="device retention: how long graph history is kept. Sets per-bucket resolution at fixed memory; changing it clears stored history."
      style="margin-left:10px;background:#222a33;color:#9aa4b0;border:1px solid #2a313a;border-radius:6px;font-size:.85em;padding:1px 4px">
      <option value="120">keep 2h</option>
      <option value="360">keep 6h</option>
      <option value="720">keep 12h</option>
      <option value="1440">keep 24h</option>
    </select>
    <span class="spacer"></span>
    <span class="lnk" id="all">all</span>
    <span class="lnk" id="none">none</span>
  </div>
  <div id="wrap"><canvas id="cv"></canvas><div id="tip"></div></div>
  <div class="hint">Each series is normalized to its own min/max over the window, so every metric shares one plot. Hover for real values; click a metric to toggle it.</div>
  <div id="legend"></div>
</div>
<script>
let META=[],SEL={},secs=3600,DATA=null,hoverX=-1;
const cv=document.getElementById("cv"),tip=document.getElementById("tip"),wrap=document.getElementById("wrap");
const DEFAULT_ON=["temp_c","rssi","cam_fps","motion","audio_trig","video_trig"];
function fmt(v,u){let s=Math.abs(v)>=100?v.toFixed(0):Math.abs(v)>=10?v.toFixed(1):v.toFixed(2);return s+(u?" "+u:"")}
async function setRetWin(v){await fetch("/metrics/config?win_min="+v);await loadMeta();await poll();}
async function loadMeta(){
  const j=await(await fetch("/metrics/meta")).json();META=j.metrics;
  document.body.classList.toggle('viewer',!!j.viewer); // read-only: lock controls
  const rw=document.getElementById("retwin");if(rw&&j.win_min)rw.value=j.win_min;
  for(const m of META)SEL[m.key]=DEFAULT_ON.includes(m.key);
  renderLegend();
}
function renderLegend(){
  const L=document.getElementById("legend");L.innerHTML="";
  for(const m of META){
    const d=document.createElement("div");d.className="mi"+(SEL[m.key]?"":" off");
    d.innerHTML='<span class="sw" style="background:'+m.color+'"></span><span class="lab">'+m.label+'</span><span class="val" id="v_'+m.key+'">-</span>';
    d.onclick=()=>{SEL[m.key]=!SEL[m.key];d.className="mi"+(SEL[m.key]?"":" off");draw()};
    L.appendChild(d);
  }
}
function metaOf(k){return META.find(m=>m.key===k)}
function decode(raw,m){return raw/m.scale-m.offset}
async function poll(){
  try{
    const w=cv.clientWidth||700;
    const r=await fetch("/metrics/series?secs="+secs+"&points="+Math.min(Math.round(w),600));
    if(r.ok){DATA=await r.json();draw()}
  }catch(e){}
}
function draw(){
  cv.width=cv.clientWidth||700;cv.height=340;
  const ctx=cv.getContext("2d"),W=cv.width,H=cv.height,PT=10,PB=22,PL=4,PR=4;
  ctx.clearRect(0,0,W,H);
  // grid
  ctx.strokeStyle="#1b2129";ctx.lineWidth=1;ctx.beginPath();
  for(let g=0;g<=4;g++){const y=PT+(H-PT-PB)*g/4;ctx.moveTo(PL,y);ctx.lineTo(W-PR,y)}ctx.stroke();
  if(!DATA){ctx.fillStyle="#8a94a0";ctx.font="12px sans-serif";ctx.fillText("collecting data...",10,24);return}
  const span=DATA.span_ms||0;
  // time axis labels
  ctx.fillStyle="#5d6773";ctx.font="11px sans-serif";ctx.textBaseline="top";
  const mins=Math.round(span/60000);
  ctx.textAlign="left";ctx.fillText("-"+(mins>=60?(mins/60).toFixed(mins%60?1:0)+"h":mins+"m"),PL+2,H-16);
  ctx.textAlign="right";ctx.fillText("now",W-PR-2,H-16);
  const gx=W-PL-PR,plotW=gx,x0=PL,yT=PT,yH=H-PT-PB;
  let any=false,latest={};
  for(const m of META){
    if(!SEL[m.key])continue;
    const arr=DATA.series[m.key];if(!arr||arr.length<1)continue;
    any=true;
    let lo=Infinity,hi=-Infinity;
    for(const r of arr){const v=decode(r,m);if(v<lo)lo=v;if(v>hi)hi=v}
    latest[m.key]=decode(arr[arr.length-1],m);
    const rng=hi-lo;
    const xs=arr.length>1?plotW/(arr.length-1):0;
    ctx.strokeStyle=m.color;ctx.lineWidth=1.6;ctx.beginPath();
    for(let i=0;i<arr.length;i++){
      const v=decode(arr[i],m);
      const norm=rng>1e-6?(v-lo)/rng:0.5;
      const x=x0+i*xs,y=yT+yH*(1-norm);
      i?ctx.lineTo(x,y):ctx.moveTo(x,y);
    }
    ctx.stroke();
  }
  if(!any){ctx.fillStyle="#8a94a0";ctx.font="12px sans-serif";ctx.fillText("no metrics selected",10,24)}
  // legend live values
  for(const m of META){
    const el=document.getElementById("v_"+m.key);if(!el)continue;
    el.textContent=(m.key in latest)?fmt(latest[m.key],m.unit):"-";
  }
  // crosshair + tooltip
  if(hoverX>=x0&&hoverX<=x0+plotW&&any){
    const ks=Object.keys(DATA.series).find(k=>SEL[k]&&DATA.series[k]&&DATA.series[k].length);
    const n=ks?DATA.series[ks].length:0;
    if(n>0){
      const idx=Math.round((hoverX-x0)/(plotW/(n-1||1)));
      const ci=Math.max(0,Math.min(n-1,idx));
      const cx=x0+ci*(plotW/(n-1||1));
      ctx.strokeStyle="#3a434f";ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(cx,yT);ctx.lineTo(cx,yT+yH);ctx.stroke();ctx.setLineDash([]);
      const ageMs=span*(1-ci/(n-1||1));
      const am=Math.round(ageMs/60000),as=Math.round(ageMs/1000)%60;
      let rows='<div class="tt">-'+(am?am+"m":"")+as+'s</div>';
      for(const m of META){
        if(!SEL[m.key])continue;const arr=DATA.series[m.key];if(!arr||ci>=arr.length)continue;
        rows+='<div class="tr"><span style="color:'+m.color+'">'+m.label+'</span><span>'+fmt(decode(arr[ci],m),m.unit)+'</span></div>';
      }
      tip.innerHTML=rows;tip.style.display="block";
      let tx=cx+12;if(tx+150>W)tx=cx-150;tip.style.left=tx+"px";tip.style.top=(yT+6)+"px";
    }
  }else tip.style.display="none";
}
cv.addEventListener("mousemove",e=>{const r=cv.getBoundingClientRect();hoverX=(e.clientX-r.left)*(cv.width/r.width);draw()});
cv.addEventListener("mouseleave",()=>{hoverX=-1;draw()});
document.querySelectorAll(".rng").forEach(x=>x.onclick=()=>{
  document.querySelectorAll(".rng").forEach(y=>y.className="rng");x.className="rng cur";
  secs=+x.dataset.s;poll();
});
document.getElementById("all").onclick=()=>{for(const m of META)SEL[m.key]=true;renderLegend();draw()};
document.getElementById("none").onclick=()=>{for(const m of META)SEL[m.key]=false;renderLegend();draw()};
window.addEventListener("resize",draw);
(async()=>{await loadMeta();await poll();setInterval(poll,5000)})();
</script><script src="/ui.js"></script>
<script>buildNav('/graphs')</script>
</body></html>)HTML";

static void handlePage() {
  if (!webAuthCheck(*g_server)) return;
  g_server->send_P(200, "text/html", GRAPHS_PAGE);
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

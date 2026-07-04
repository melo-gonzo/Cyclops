// cam_settings.cpp - see cam_settings.h

#include "cam_settings.h"

#include <Preferences.h>
#include <esp_camera.h>

#include "web_auth.h"
#include "branding.h"

#if defined(CAMERA_MODEL_XIAO_ESP32S3)
#include "video_record.h" // resolution changes must update AVI dimensions
#endif

// The camera driver allocates its frame buffers for FRAMESIZE_XGA at init,
// so runtime resolution changes may only go downward from there. Frames
// above XGA would also break main.cpp's MAX_FRAME_SIZE (200KB) send pool.
#define CAM_MAX_FRAMESIZE FRAMESIZE_XGA

// Sensor master clock, runtime-tunable: at 10MHz and below the esp32-camera
// driver engages the OV2640's internal clock doubler (decided inside
// set_framesize), which often yields a HIGHER frame rate than 20MHz.
#if defined(CAMERA_MODEL_XIAO_ESP32S3)
#define CAM_DEFAULT_XCLK_MHZ 20
#else
#define CAM_DEFAULT_XCLK_MHZ 8
#endif
#define CAM_XCLK_MIN_MHZ 4
#define CAM_XCLK_MAX_MHZ 24

static WebServer *camServer = NULL;
// Current frame dimensions. Written from camSettingsInit (setup/recover) and the
// /camera/config + /camera/reset handlers (all the HTTP server task), read by
// camGetFrameDims from the video-init path. No lock: 16-bit aligned loads/stores
// are atomic on Xtensa, so a reader sees a whole old or new value, never a tear.
static uint16_t camW = 1024, camH = 768;
static int camXclk = CAM_DEFAULT_XCLK_MHZ;
static float (*camFpsSource)() = NULL;
static float (*camNetFpsSource)() = NULL;

void camSetFpsSource(float (*fn)()) { camFpsSource = fn; }
void camSetNetFpsSource(float (*fn)()) { camNetFpsSource = fn; }

int camSavedXclkMhz() {
  Preferences p;
  if (p.begin("cam", true)) {
    camXclk = p.getInt("xclk", CAM_DEFAULT_XCLK_MHZ);
    p.end();
  }
  if (camXclk < CAM_XCLK_MIN_MHZ || camXclk > CAM_XCLK_MAX_MHZ) camXclk = CAM_DEFAULT_XCLK_MHZ;
  return camXclk;
}

// ---- Control table (canonical CameraWebServer var names) ----
// Order matters at boot: auto/manual mode toggles (aec, agc) are applied
// before their manual values (aec_value, agc_gain).

struct CamVar {
  const char *name; // query arg and NVS key
  int16_t minV, maxV;
  int (*set)(sensor_t *, int);
  int (*get)(const camera_status_t &);
};

#define CAM_SET(fn) [](sensor_t *s, int v) -> int { return s->fn(s, v); }
#define CAM_GET(field) [](const camera_status_t &st) -> int { return (int)st.field; }

static const CamVar CAM_VARS[] = {
  {"quality",        10, 63, CAM_SET(set_quality),        CAM_GET(quality)},
  {"brightness",     -2,  2, CAM_SET(set_brightness),     CAM_GET(brightness)},
  {"contrast",       -2,  2, CAM_SET(set_contrast),       CAM_GET(contrast)},
  {"saturation",     -2,  2, CAM_SET(set_saturation),     CAM_GET(saturation)},
  {"special_effect",  0,  6, CAM_SET(set_special_effect), CAM_GET(special_effect)},
  {"awb",             0,  1, CAM_SET(set_whitebal),       CAM_GET(awb)},
  {"awb_gain",        0,  1, CAM_SET(set_awb_gain),       CAM_GET(awb_gain)},
  {"wb_mode",         0,  4, CAM_SET(set_wb_mode),        CAM_GET(wb_mode)},
  {"aec",             0,  1, CAM_SET(set_exposure_ctrl),  CAM_GET(aec)},
  {"aec2",            0,  1, CAM_SET(set_aec2),           CAM_GET(aec2)},
  {"ae_level",       -2,  2, CAM_SET(set_ae_level),       CAM_GET(ae_level)},
  {"aec_value",       0, 1200, CAM_SET(set_aec_value),    CAM_GET(aec_value)},
  {"agc",             0,  1, CAM_SET(set_gain_ctrl),      CAM_GET(agc)},
  {"agc_gain",        0, 30, CAM_SET(set_agc_gain),       CAM_GET(agc_gain)},
  {"gainceiling",     0,  6,
   [](sensor_t *s, int v) -> int { return s->set_gainceiling(s, (gainceiling_t)v); },
   CAM_GET(gainceiling)},
  {"bpc",             0,  1, CAM_SET(set_bpc),            CAM_GET(bpc)},
  {"wpc",             0,  1, CAM_SET(set_wpc),            CAM_GET(wpc)},
  {"raw_gma",         0,  1, CAM_SET(set_raw_gma),        CAM_GET(raw_gma)},
  {"lenc",            0,  1, CAM_SET(set_lenc),           CAM_GET(lenc)},
  {"hmirror",         0,  1, CAM_SET(set_hmirror),        CAM_GET(hmirror)},
  {"vflip",           0,  1, CAM_SET(set_vflip),          CAM_GET(vflip)},
  {"dcw",             0,  1, CAM_SET(set_dcw),            CAM_GET(dcw)},
  {"colorbar",        0,  1, CAM_SET(set_colorbar),       CAM_GET(colorbar)},
};
#define CAM_VAR_COUNT (sizeof(CAM_VARS) / sizeof(CAM_VARS[0]))

static bool framesizeDims(int fs, uint16_t *w, uint16_t *h) {
  switch (fs) {
    case FRAMESIZE_QVGA: *w = 320;  *h = 240; return true;
    case FRAMESIZE_CIF:  *w = 400;  *h = 296; return true;
    case FRAMESIZE_HVGA: *w = 480;  *h = 320; return true;
    case FRAMESIZE_VGA:  *w = 640;  *h = 480; return true;
    case FRAMESIZE_SVGA: *w = 800;  *h = 600; return true;
    case FRAMESIZE_XGA:  *w = 1024; *h = 768; return true;
    default: return false;
  }
}

// Project sensor defaults (moved here from setup() so /camera/reset can
// re-apply them). Tuned for the indoor scene: banding filter on, auto exposure.
// NOTE on flicker banding (the horizontal stripes under mains lighting): it is
// reduced by LONGER, band-aligned exposures (each row then integrates over whole
// 50/60Hz flicker cycles), NOT by shorter ones - a short exposure makes the bands
// WORSE. The real fix is the OV2640 banding filter snapping exposure to flicker
// multiples; auto AEC (set_exposure_ctrl=1) lets it do that.
static void camApplyDefaults(sensor_t *sensor) {
  sensor->set_brightness(sensor, 0);
  sensor->set_contrast(sensor, 0);
  sensor->set_saturation(sensor, 0);
  sensor->set_special_effect(sensor, 0);
  sensor->set_whitebal(sensor, 1);
  sensor->set_awb_gain(sensor, 1);
  sensor->set_wb_mode(sensor, 1);
  sensor->set_exposure_ctrl(sensor, 1);
  sensor->set_aec2(sensor, 1);        // enable DSP AEC (needed for banding filter)
  sensor->set_ae_level(sensor, 0);
  sensor->set_aec_value(sensor, 600); // AEC seed/manual fallback (auto AEC overrides). Longer
                                       // than before: a longer exposure averages more flicker
                                       // cycles, so it reduces banding when AEC is manual.
  sensor->set_gain_ctrl(sensor, 1);
  sensor->set_agc_gain(sensor, 0);
  sensor->set_gainceiling(sensor, (gainceiling_t)2);
  sensor->set_bpc(sensor, 1);
  sensor->set_wpc(sensor, 1);
  sensor->set_raw_gma(sensor, 1);
  sensor->set_lenc(sensor, 1);
  sensor->set_dcw(sensor, 1);
  sensor->set_colorbar(sensor, 0);

  // Enable the OV2640 50/60 Hz banding filter.
  // Register 0x13 bit 5 in sensor bank (0xFF=0x01) turns on auto banding.
  sensor->set_reg(sensor, 0xFF, 0xFF, 0x01); // select sensor register bank
  sensor->set_reg(sensor, 0x13, 0x20, 0x20); // COM8: banding filter enable
}

void camSettingsInit() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) return;

  camApplyDefaults(sensor);

  int fs = CAM_MAX_FRAMESIZE;
  bool badFs = false;
  Preferences p;
  if (p.begin("cam", true)) { // namespace missing on first boot
    for (size_t i = 0; i < CAM_VAR_COUNT; i++) {
      if (p.isKey(CAM_VARS[i].name)) {
        // Clamp to the same range the /camera/config handler enforces: a value
        // from older firmware or corrupted NVS must not reach the sensor unchecked
        // (only the HTTP path range-checked before).
        int v = p.getInt(CAM_VARS[i].name);
        if (v < CAM_VARS[i].minV) v = CAM_VARS[i].minV;
        if (v > CAM_VARS[i].maxV) v = CAM_VARS[i].maxV;
        CAM_VARS[i].set(sensor, v);
      }
    }
    if (p.isKey("framesize")) {
      int v = p.getInt("framesize");
      uint16_t w, h;
      if (v <= CAM_MAX_FRAMESIZE && framesizeDims(v, &w, &h)) fs = v;
      else badFs = true; // unusable stored value (out of range / unsupported)
    }
    p.end();
  }
  if (badFs) { // drop the bad key so it doesn't linger and get re-read every boot
    Preferences pw;
    if (pw.begin("cam", false)) { pw.remove("framesize"); pw.end(); }
  }
  // Always (re-)apply the framesize: the driver picks the sensor clock
  // divider/doubler inside set_framesize based on the current xclk.
  uint16_t w, h;
  framesizeDims(fs, &w, &h);
  sensor->set_framesize(sensor, (framesize_t)fs);
  camW = w;
  camH = h;
  Serial.printf("[cam] sensor settings applied: %ux%u, xclk %dMHz\n", camW, camH, camXclk);
}

void camGetFrameDims(uint16_t *w, uint16_t *h) {
  *w = camW;
  *h = camH;
}

// ---- HTTP handlers ----

static void handleCamStatus() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    camServer->send(503, "text/plain", "camera not initialized");
    return;
  }
  String json = "{\"framesize\":" + String((int)sensor->status.framesize) +
                ",\"width\":" + String(camW) + ",\"height\":" + String(camH) +
                ",\"xclk\":" + String(camXclk) +
                ",\"fps\":" + String(camFpsSource ? camFpsSource() : 0.0f, 1) +
                ",\"net_fps\":" + String(camNetFpsSource ? camNetFpsSource() : 0.0f, 1);
  for (size_t i = 0; i < CAM_VAR_COUNT; i++) {
    json += ",\"" + String(CAM_VARS[i].name) + "\":" +
            String(CAM_VARS[i].get(sensor->status));
  }
  json += ",\"viewer\":" + String(webAuthIsViewer() ? "true" : "false");
  json += "}";
  camServer->send(200, "application/json", json);
}

static void handleCamConfig() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    camServer->send(503, "text/plain", "camera not initialized");
    return;
  }
  String var = camServer->arg("var");
  int val = camServer->arg("val").toInt();

  if (var == "framesize") {
    uint16_t w, h;
    if (val > CAM_MAX_FRAMESIZE || !framesizeDims(val, &w, &h)) {
      camServer->send(400, "text/plain", "unsupported framesize");
      return;
    }
#if defined(CAMERA_MODEL_XIAO_ESP32S3)
    if (!videoSetDims(w, h)) {
      camServer->send(409, "text/plain", "busy: video clip recording in progress");
      return;
    }
#endif
    if (sensor->set_framesize(sensor, (framesize_t)val) != 0) {
      camServer->send(500, "text/plain", "set_framesize failed");
      return;
    }
    camW = w;
    camH = h;
  } else if (var == "xclk") {
    if (val < CAM_XCLK_MIN_MHZ || val > CAM_XCLK_MAX_MHZ) {
      camServer->send(400, "text/plain", "xclk must be 4..24 MHz");
      return;
    }
    // Saved only: runtime set_xclk wedges the S3 capture DMA, so the new
    // clock is picked up by camera init at the next boot.
    camXclk = val;
  } else {
    const CamVar *cv = NULL;
    for (size_t i = 0; i < CAM_VAR_COUNT; i++) {
      if (var == CAM_VARS[i].name) { cv = &CAM_VARS[i]; break; }
    }
    if (cv == NULL) {
      camServer->send(400, "text/plain", "unknown var");
      return;
    }
    if (val < cv->minV || val > cv->maxV) {
      camServer->send(400, "text/plain",
                      var + " must be " + String(cv->minV) + ".." + String(cv->maxV));
      return;
    }
    if (cv->set(sensor, val) != 0) {
      camServer->send(500, "text/plain", "sensor rejected " + var);
      return;
    }
  }

  Preferences p;
  if (p.begin("cam", false)) {
    p.putInt(var.c_str(), val);
    p.end();
  }
  handleCamStatus();
}

static void handleCamReset() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    camServer->send(503, "text/plain", "camera not initialized");
    return;
  }
#if defined(CAMERA_MODEL_XIAO_ESP32S3)
  if (!videoSetDims(1024, 768)) {
    camServer->send(409, "text/plain", "busy: video clip recording in progress");
    return;
  }
#endif
  Preferences p;
  if (p.begin("cam", false)) {
    p.clear();
    p.end();
  }
  camApplyDefaults(sensor);
  camXclk = CAM_DEFAULT_XCLK_MHZ; // takes effect at next boot
  sensor->set_framesize(sensor, CAM_MAX_FRAMESIZE);
  camW = 1024;
  camH = 768;
  handleCamStatus();
}

// ---- Settings page ----

static const char CAM_PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>)rawliteral" DEVICE_NAME R"rawliteral( - Camera Settings</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#14171c;color:#dde3ea;margin:0;padding:16px;max-width:760px;margin:auto}
a{color:#6fb3ff}
.card{background:#1d2229;border-radius:10px;padding:14px;margin-bottom:14px}
h3{font-size:.85em;color:#8a94a0;font-weight:500;margin:0 0 8px;text-transform:uppercase;letter-spacing:.05em}
#nav{display:flex;gap:6px;align-items:center;margin:4px 0 16px}
#nav b{margin-right:10px}
#nav a{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}
#nav a.cur{background:#1d2229;color:#dde3ea}
button{background:#3d9df0;color:#fff;border:0;border-radius:8px;padding:10px 16px;font-size:1em;cursor:pointer}
label{font-size:.9em;color:#aab4c0}
select{background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px}
.ctl{display:grid;grid-template-columns:110px 1fr 60px;gap:10px;align-items:center;margin:8px 0}
.ctl span{font-size:.85em;color:#8a94a0;text-align:right}
input[type=range]{width:100%;accent-color:#3d9df0}
input[type=range]:disabled{opacity:.35}
.ctl input[type=number]{width:100%;box-sizing:border-box;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:4px;font-size:.85em;text-align:right}
.ctl input[type=number]:disabled{opacity:.35}
.cbs{display:flex;flex-wrap:wrap;gap:6px 16px;margin:4px 0}
.cbs label{padding:4px 0}
#pv{width:100%;border-radius:6px;background:#000;min-height:120px}
.note{font-size:.85em;color:#8a94a0}
@media(max-width:600px){
  body{padding:10px}
  .card{padding:12px;margin-bottom:10px}
  #nav{gap:4px;flex-wrap:wrap}#nav b{width:100%;margin:0 0 2px}
  input[type=number],select{font-size:16px}
  .ctl{grid-template-columns:96px 1fr 56px;gap:8px}
}
/* Read-only viewer: lock every control; the live preview stays visible. */
body.viewer input,body.viewer select,body.viewer button,body.viewer textarea{pointer-events:none;opacity:.4}
#vbadge{display:none;background:#2a313a;color:#7fd6a0;border-radius:8px;padding:3px 9px;font-size:.8em}
body.viewer #vbadge{display:inline-block}
</style></head><body>
<div id="nav"></div>
<div class="card">
  <h3>Preview <span id="fps" style="color:#5df0a0;text-transform:none;letter-spacing:0;margin-left:8px"></span>
    <label style="float:right;text-transform:none;letter-spacing:0"><input type="checkbox" id="pvon" checked onchange="pvTick()"> live</label>
    <label id="motwrap" style="float:right;text-transform:none;letter-spacing:0;margin-right:14px"><input type="checkbox" id="moton" onchange="motPoll()"> motion</label></h3>
  <div style="position:relative">
    <img id="pv" alt="preview">
    <canvas id="mot" style="position:absolute;inset:0;width:100%;height:100%;pointer-events:none;display:none"></canvas>
  </div>
</div>
<div class="card" id="motcard" style="display:none">
  <h3>Motion detection <span id="motstat" style="color:#8a94a0;text-transform:none;letter-spacing:0;margin-left:8px;font-weight:normal"></span></h3>
  <div class="cbs">
    <label><input type="checkbox" id="mot_en" onchange="motSend('motion='+(this.checked?1:0))"> auto-trigger clips</label>
    <label><input type="checkbox" id="mot_edit" onchange="motEditTick()"> edit mask (click blocks)</label>
    <button onclick="motClearMask()" style="background:#2a313a;color:#dde3ea;border:0;border-radius:6px;padding:4px 10px;font-size:.85em;cursor:pointer">clear mask</button>
  </div>
  <div class="ctl"><label title="finer grids localize motion better, but each block averages fewer pixels so they read noisier; changing the grid clears the mask">grid</label>
    <select id="mot_blk" onchange="motSend('motion_block='+this.value)">
      <option value="16">coarse</option>
      <option value="8">normal</option>
      <option value="4">fine</option>
      <option value="2">ultra</option>
    </select><span id="mot_blk_v"></span></div>
  <div class="ctl"><label title="changed blocks needed to trigger; lower = more sensitive">blocks to trigger</label>
    <input type="range" id="mot_blocks" min="1" max="192" oninput="lbl(this)" onchange="motSend('motion_blocks='+this.value)"><input type="number" id="mot_blocks_v" onchange="numIn(this)"></div>
  <div class="ctl"><label title="avg luma delta for a block to count as changed; lower = more sensitive">pixel diff</label>
    <input type="range" id="mot_diff" min="3" max="60" oninput="lbl(this)" onchange="motSend('motion_diff='+this.value)"><input type="number" id="mot_diff_v" onchange="numIn(this)"></div>
  <div class="ctl"><label title="after a trigger, ignore new triggers for this many seconds (motion is still graphed; only repeat triggers are suppressed). 0 = no limit">trigger cooldown</label>
    <input type="range" id="mot_cd" min="0" max="60" oninput="lbl(this)" onchange="motSend('motion_cooldown='+this.value)"><input type="number" id="mot_cd_v" onchange="numIn(this)"><span class="note">s</span></div>
  <div class="note" style="line-height:1.6;margin-top:8px">
    Every 0.5s each block's average brightness is compared against the previous check.<br>
    <b>grid</b> &mdash; how finely the frame is split into blocks (across&times;down shown at right). Finer pinpoints motion better but each block averages fewer pixels, so it reads noisier.<br>
    <b>blocks to trigger</b> &mdash; how many blocks must change, on 2 checks in a row, to record a clip. Lower = more sensitive.<br>
    <b>pixel diff</b> &mdash; brightness change (0&ndash;255) a block needs before it counts as changed. Lower = more sensitive; keep it above "avg diff" or noise will count.<br>
    <b>level</b> &mdash; how many blocks changed in the last check; a clip triggers when level &ge; blocks-to-trigger twice in a row.<br>
    <b>luma</b> &mdash; average frame brightness (0&ndash;255). In the dark the sensor slows down and noise rises.<br>
    <b>avg diff</b> &mdash; frame-wide average per-pixel change between checks: the noise floor of the scene. A static scene reads 1&ndash;3.
  </div>
  <div class="note" style="margin-top:6px">Turn on the "motion" overlay above to tune visually: red = changing now, blue = masked out (never counts). The mask clears if the resolution or grid changes.</div>
</div>
<div class="card"><h3>Image</h3>
  <div class="ctl"><label>resolution</label>
    <select id="framesize" onchange="send('framesize',this.value)">
      <option value="5">QVGA 320&times;240</option>
      <option value="6">CIF 400&times;296</option>
      <option value="7">HVGA 480&times;320</option>
      <option value="8">VGA 640&times;480</option>
      <option value="9">SVGA 800&times;600</option>
      <option value="10">XGA 1024&times;768</option>
    </select><span id="res"></span></div>
  <div class="ctl"><label title="sensor master clock; 10 MHz and below engages the OV2640 clock doubler; applied at boot because runtime changes stall the capture DMA">XCLK</label>
    <select id="xclk" onchange="send('xclk',this.value)">
      <option value="8">8 MHz</option>
      <option value="10">10 MHz</option>
      <option value="12">12 MHz</option>
      <option value="16">16 MHz</option>
      <option value="20">20 MHz</option>
      <option value="24">24 MHz</option>
    </select><span class="note" style="text-align:right">on boot</span></div>
  <div class="ctl"><label title="lower number = better quality, bigger frames">jpeg quality</label><input type="range" id="quality" min="10" max="63" oninput="lbl(this)" onchange="send('quality',this.value)"><input type="number" id="quality_v" onchange="numIn(this)"></div>
  <div class="ctl"><label>brightness</label><input type="range" id="brightness" min="-2" max="2" oninput="lbl(this)" onchange="send('brightness',this.value)"><input type="number" id="brightness_v" onchange="numIn(this)"></div>
  <div class="ctl"><label>contrast</label><input type="range" id="contrast" min="-2" max="2" oninput="lbl(this)" onchange="send('contrast',this.value)"><input type="number" id="contrast_v" onchange="numIn(this)"></div>
  <div class="ctl"><label>saturation</label><input type="range" id="saturation" min="-2" max="2" oninput="lbl(this)" onchange="send('saturation',this.value)"><input type="number" id="saturation_v" onchange="numIn(this)"></div>
  <div class="ctl"><label>special effect</label>
    <select id="special_effect" onchange="send('special_effect',this.value)">
      <option value="0">none</option><option value="1">negative</option>
      <option value="2">grayscale</option><option value="3">red tint</option>
      <option value="4">green tint</option><option value="5">blue tint</option>
      <option value="6">sepia</option>
    </select><span></span></div>
  <div class="cbs">
    <label><input type="checkbox" id="hmirror" onchange="send('hmirror',this.checked?1:0)"> h-mirror</label>
    <label><input type="checkbox" id="vflip" onchange="send('vflip',this.checked?1:0)"> v-flip</label>
  </div>
</div>
<div class="card"><h3>Exposure &amp; gain</h3>
  <div class="cbs">
    <label><input type="checkbox" id="aec" onchange="send('aec',this.checked?1:0)"> auto exposure</label>
    <label><input type="checkbox" id="aec2" onchange="send('aec2',this.checked?1:0)" title="DSP-side AEC; needed for the banding filter"> AEC DSP</label>
    <label><input type="checkbox" id="agc" onchange="send('agc',this.checked?1:0)"> auto gain</label>
  </div>
  <div class="ctl"><label title="exposure bias when auto exposure is on">AE level</label><input type="range" id="ae_level" min="-2" max="2" oninput="lbl(this)" onchange="send('ae_level',this.value)"><input type="number" id="ae_level_v" onchange="numIn(this)"></div>
  <div class="ctl"><label title="manual exposure; only used when auto exposure is off">exposure</label><input type="range" id="aec_value" min="0" max="1200" oninput="lbl(this)" onchange="send('aec_value',this.value)"><input type="number" id="aec_value_v" onchange="numIn(this)"></div>
  <div class="ctl"><label title="manual gain; only used when auto gain is off">gain</label><input type="range" id="agc_gain" min="0" max="30" oninput="lbl(this)" onchange="send('agc_gain',this.value)"><input type="number" id="agc_gain_v" onchange="numIn(this)"></div>
  <div class="ctl"><label title="max sensor gain auto-gain may reach; higher = brighter but noisier">gain ceiling</label>
    <select id="gainceiling" onchange="send('gainceiling',this.value)">
      <option value="0">2x</option><option value="1">4x</option>
      <option value="2">8x</option><option value="3">16x</option>
      <option value="4">32x</option><option value="5">64x</option>
      <option value="6">128x</option>
    </select><span></span></div>
</div>
<div class="card"><h3>White balance</h3>
  <div class="cbs">
    <label><input type="checkbox" id="awb" onchange="send('awb',this.checked?1:0)"> auto white balance</label>
    <label><input type="checkbox" id="awb_gain" onchange="send('awb_gain',this.checked?1:0)"> AWB gain</label>
  </div>
  <div class="ctl"><label>WB mode</label>
    <select id="wb_mode" onchange="send('wb_mode',this.value)">
      <option value="0">auto</option><option value="1">sunny</option>
      <option value="2">cloudy</option><option value="3">office</option>
      <option value="4">home</option>
    </select><span></span></div>
</div>
<div class="card"><h3>Pixel pipeline</h3>
  <div class="cbs">
    <label><input type="checkbox" id="bpc" onchange="send('bpc',this.checked?1:0)" title="black pixel cancellation"> BPC</label>
    <label><input type="checkbox" id="wpc" onchange="send('wpc',this.checked?1:0)" title="white pixel cancellation"> WPC</label>
    <label><input type="checkbox" id="raw_gma" onchange="send('raw_gma',this.checked?1:0)" title="raw gamma correction"> raw gamma</label>
    <label><input type="checkbox" id="lenc" onchange="send('lenc',this.checked?1:0)" title="lens shading correction"> lens corr</label>
    <label><input type="checkbox" id="dcw" onchange="send('dcw',this.checked?1:0)" title="downsize cropping window"> DCW</label>
    <label><input type="checkbox" id="colorbar" onchange="send('colorbar',this.checked?1:0)" title="sensor test pattern"> colorbar</label>
  </div>
</div>
<div class="card"><div style="display:flex;gap:16px;align-items:center;flex-wrap:wrap">
  <button onclick="resetCam()" style="background:#2a313a">restore defaults</button>
  <span class="note">changes apply live and persist across reboots</span>
</div></div>
<script src="/ui.js"></script>
<script>
const SLIDERS=["quality","brightness","contrast","saturation","ae_level","aec_value","agc_gain"];
const CHECKS=["hmirror","vflip","aec","aec2","agc","awb","awb_gain","bpc","wpc","raw_gma","lenc","dcw","colorbar"];
const SELECTS=["framesize","special_effect","wb_mode","gainceiling","xclk"];
function lbl(el){
  const t=document.getElementById(el.id+"_v");
  if(document.activeElement!==t)t.value=el.value;
}
// Exact-value entry: each slider's readout is a number input; typing a value
// clamps it to the slider's range and fires the slider's own change handler.
function numIn(el){
  const s=document.getElementById(el.id.slice(0,-2));
  let v=Math.round(+el.value);
  if(isNaN(v))v=+s.value;
  v=Math.min(+s.max,Math.max(+s.min,v));
  el.value=v;s.value=v;
  el.blur();
  s.dispatchEvent(new Event("change"));
}
function sync(s){
  document.body.classList.toggle('viewer',!!s.viewer); // read-only: lock controls
  SLIDERS.forEach(k=>{const e=document.getElementById(k);e.value=s[k];lbl(e)});
  CHECKS.forEach(k=>{document.getElementById(k).checked=!!s[k]});
  SELECTS.forEach(k=>{document.getElementById(k).value=s[k]});
  document.getElementById("res").textContent=s.width+"×"+s.height;
  // manual sliders only act when their auto mode is off
  [["aec_value",!!s.aec],["agc_gain",!!s.agc],["ae_level",!s.aec]].forEach(([k,d])=>{
    document.getElementById(k).disabled=d;
    document.getElementById(k+"_v").disabled=d;
  });
}
function refresh(){return fetch("/camera/status").then(r=>r.json()).then(sync)}
function send(k,v){
  fetch("/camera/config?var="+k+"&val="+v).then(r=>{
    if(!r.ok)return r.text().then(t=>{alert(t);refresh()});
    return r.json().then(sync).then(()=>setTimeout(pvNow,400));
  });
}
function resetCam(){
  fetch("/camera/reset").then(r=>{
    if(!r.ok)return r.text().then(t=>alert(t));
    return r.json().then(sync).then(()=>setTimeout(pvNow,400));
  });
}
// Live = ride the MJPEG stream via the robust reader (fetch()-based, auto-
// reconnects on stall/drop and self-pauses when the tab is hidden, instead of a
// bare <img> that freezes on a half-decoded frame); unchecked = a single still,
// refreshed after each config change. pvNow only reloads in stills mode so config
// changes don't restart the stream connection.
let camStream=null;
function pvStream(){if(!camStream)camStream=mjpegStream(document.getElementById("pv"),"/mjpeg/1");return camStream}
function pvSet(){
  const pv=document.getElementById("pv");
  if(document.getElementById("pvon").checked)pvStream().start();
  else{pvStream().stop();pv.src="/jpg?t="+Date.now()}
}
function pvNow(){
  if(!document.hidden&&!document.getElementById("pvon").checked)pvSet();
}
function pvTick(){if(!document.hidden)pvSet()}
// The live stream's own visibility pause/resume is handled inside mjpegStream;
// here we only refresh the still when returning to a stills-mode tab.
document.addEventListener("visibilitychange",()=>{
  if(!document.hidden&&!document.getElementById("pvon").checked)pvSet();
});
setInterval(()=>{ // fps poll only: must not yank controls mid-adjustment
  if(document.hidden)return;
  fetch("/camera/status").then(r=>r.json()).then(s=>{
    document.getElementById("fps").textContent=
      s.fps>0?"cam "+s.fps.toFixed(1)+(s.net_fps>0?" · net "+s.net_fps.toFixed(1):"")+" fps":"";
  }).catch(()=>{});
},2000);
// Motion detection: debug overlay + interactive tuning. Polling
// /video/motion keeps analysis alive on the device even with auto-trigger
// off, so overlay + sliders are the tuning loop. In mask-edit mode the
// overlay canvas takes clicks and toggles the block under the cursor in
// the exclusion mask. Everything hides if this board has no video module
// (endpoint 404s).
let _motMiss=0,_lastMot=null,_motTick=0,_motMap=null,_motMapAt=0;
const _mbit=(hex,b)=>(parseInt(hex.substr((b>>3)*2,2),16)>>(b&7))&1;
function drawMot(m){
  const pv=document.getElementById("pv"),mot=document.getElementById("mot");
  mot.style.display="block";
  mot.width=pv.clientWidth;mot.height=pv.clientHeight;
  const ctx=mot.getContext("2d");
  ctx.clearRect(0,0,mot.width,mot.height);
  ctx.font="13px monospace";
  // Hold the last good map briefly so a missed analysis slot under streaming
  // load doesn't flicker the overlay to "warming up".
  if(m.map){_motMap=m.map;_motMapAt=Date.now()}
  else if(_motMap&&Date.now()-_motMapAt<3000)m.map=_motMap;
  if(!m.bx){ctx.fillStyle="#8a94a0";ctx.fillText("motion: warming up",10,20);return}
  const bw=mot.width/m.bx,bh=mot.height/m.by;
  if(document.getElementById("mot_edit").checked){ // block boundaries
    ctx.strokeStyle="rgba(221,227,234,.3)";ctx.lineWidth=1;
    ctx.beginPath();
    for(let i=1;i<m.bx;i++){ctx.moveTo(i*bw,0);ctx.lineTo(i*bw,mot.height)}
    for(let j=1;j<m.by;j++){ctx.moveTo(0,j*bh);ctx.lineTo(mot.width,j*bh)}
    ctx.stroke();
  }
  if(m.mask){
    ctx.fillStyle="rgba(61,157,240,.3)";
    for(let b=0;b<m.bx*m.by;b++)
      if(_mbit(m.mask,b))ctx.fillRect((b%m.bx)*bw,((b/m.bx)|0)*bh,bw,bh);
  }
  if(!m.map){ctx.fillStyle="#8a94a0";ctx.fillText("motion: warming up",10,20);return}
  ctx.fillStyle="rgba(240,93,93,.22)";ctx.strokeStyle="rgba(240,93,93,.85)";ctx.lineWidth=2;
  for(let b=0;b<m.bx*m.by;b++){
    if(_mbit(m.map,b)){
      const x=(b%m.bx)*bw,y=((b/m.bx)|0)*bh;
      ctx.fillRect(x,y,bw,bh);ctx.strokeRect(x,y,bw,bh);
    }
  }
  ctx.fillStyle=m.global?"#f0c95d":m.level>=m.thresh?"#f05d5d":"#5df0a0";
  ctx.fillText("motion "+m.level+"/"+m.thresh+(m.global?" lighting change":"")
    +(m.enabled?"":" (trigger off)"),10,20);
}
function motSync(m){
  document.getElementById("motcard").style.display="";
  const en=document.getElementById("mot_en"),
        bl=document.getElementById("mot_blocks"),
        df=document.getElementById("mot_diff"),
        cd=document.getElementById("mot_cd"),
        bz=document.getElementById("mot_blk");
  if(document.activeElement!==en)en.checked=m.enabled;
  if(m.bx)bl.max=m.bx*m.by;
  if(document.activeElement!==bl){bl.value=m.thresh;lbl(bl)}
  if(document.activeElement!==df){df.value=m.diff;lbl(df)}
  if(document.activeElement!==cd&&m.cooldown!=null){cd.value=m.cooldown;lbl(cd)}
  if(document.activeElement!==bz&&m.blk)bz.value=m.blk;
  document.getElementById("mot_blk_v").textContent=m.bx?(m.bx+"×"+m.by):"";
  // Only overwrite when the check is fresh (m.map present). A stale or
  // warming-up poll otherwise blanked these fields until the next good one,
  // which read as them flickering or vanishing and needing a refresh.
  if(m.map)document.getElementById("motstat").textContent=
    "level "+m.level+" · luma "+m.luma+" · avg diff "+m.avg_diff;
}
function motPoll(){
  if(document.hidden)return;
  const on=document.getElementById("moton").checked,
        mot=document.getElementById("mot");
  // overlay off: keep a slow poll so the tuning card stays live
  if(!on){mot.style.display="none";if(_motTick++%4)return}
  fetch("/video/motion").then(r=>{
    if(!r.ok)throw 0;
    return r.json();
  }).then(m=>{
    if(_paint&&_lastMot)m.mask=_lastMot.mask; // mid-stroke: keep local edits
    _lastMot=m;_motMiss=0;
    motSync(m);
    if(on)drawMot(m);
  }).catch(()=>{
    if(++_motMiss>2){
      document.getElementById("motwrap").style.display="none";
      document.getElementById("motcard").style.display="none";
      mot.style.display="none";
    }
  });
}
function motSend(q){fetch("/video/config?"+q).then(r=>{
  if(!r.ok)return r.text().then(t=>alert(t));
}).then(()=>motPoll())}
function motEditTick(){
  const edit=document.getElementById("mot_edit").checked,
        mot=document.getElementById("mot");
  mot.style.pointerEvents=edit?"auto":"none";
  mot.style.touchAction=edit?"none":""; // don't scroll the page while painting
  if(edit&&!document.getElementById("moton").checked){
    document.getElementById("moton").checked=true;
  }
  motPoll(); // redraw immediately so the grid outline appears/disappears
}
function motClearMask(){motSend("motion_mask=")}
// Drag-to-paint mask editing: the first block touched decides whether the
// stroke masks or unmasks; one config request per stroke, on release.
let _paint=null;
function _blockAt(e){
  const r=document.getElementById("mot").getBoundingClientRect();
  const i=Math.min(_lastMot.bx-1,Math.max(0,Math.floor((e.clientX-r.left)/r.width*_lastMot.bx)));
  const j=Math.min(_lastMot.by-1,Math.max(0,Math.floor((e.clientY-r.top)/r.height*_lastMot.by)));
  return j*_lastMot.bx+i;
}
function _paintApply(b){
  if(_paint.set)_paint.bytes[b>>3]|=1<<(b&7);
  else _paint.bytes[b>>3]&=~(1<<(b&7));
  _lastMot.mask=[..._paint.bytes].map(v=>v.toString(16).padStart(2,"0")).join("");
  drawMot(_lastMot);
}
const _motCv=document.getElementById("mot");
_motCv.onpointerdown=e=>{
  if(!_lastMot||!_lastMot.bx)return;
  const n=Math.ceil(_lastMot.bx*_lastMot.by/8),bytes=new Uint8Array(n);
  for(let k=0;k<n;k++)bytes[k]=parseInt(_lastMot.mask.substr(k*2,2),16)||0;
  const b=_blockAt(e);
  _paint={set:!((bytes[b>>3]>>(b&7))&1),bytes:bytes};
  _motCv.setPointerCapture(e.pointerId);
  _paintApply(b);
  e.preventDefault();
};
_motCv.onpointermove=e=>{if(_paint&&_lastMot)_paintApply(_blockAt(e))};
_motCv.onpointerup=_motCv.onpointercancel=e=>{
  if(!_paint)return;
  _paint=null;
  motSend("motion_mask="+_lastMot.mask);
};
setInterval(motPoll,500);
refresh();pvTick();
</script>
<script>buildNav('/camera')</script>
</body></html>)rawliteral";

static void handleCamPage() {
  camServer->send_P(200, "text/html", CAM_PAGE);
}

static WebServer::THandlerFunction camGuarded(void (*h)()) {
  return [h]() {
    if (!webAuthCheck(*camServer)) return;
    h();
  };
}

// Mutating routes: reject the read-only viewer with 403.
static WebServer::THandlerFunction camGuardedW(void (*h)()) {
  return [h]() {
    if (!webAuthRequireWrite(*camServer)) return;
    h();
  };
}

void camRegisterEndpoints(WebServer &server) {
  camServer = &server;
  server.on("/camera", HTTP_GET, camGuarded(handleCamPage));
  server.on("/camera/status", HTTP_GET, camGuarded(handleCamStatus));
  server.on("/camera/config", HTTP_GET, camGuardedW(handleCamConfig));
  server.on("/camera/reset", HTTP_GET, camGuardedW(handleCamReset));
}

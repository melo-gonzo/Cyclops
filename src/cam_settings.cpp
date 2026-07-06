// cam_settings.cpp - see cam_settings.h

#include "cam_settings.h"
#include "web_assets.gen.h"

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

// WEB_CAM_HTML now lives in web/cam.html (compiled in as WEB_CAM_HTML via web/ codegen)

static void handleCamPage() {
  camServer->send_P(200, "text/html", WEB_CAM_HTML);
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

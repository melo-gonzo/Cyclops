// web_ui.cpp - see web_ui.h. Board-agnostic shared web assets.

#include "web_ui.h"
#include "capabilities.h"
#include "web_auth.h"
#include "branding.h"
#include "web_assets.gen.h"  // WEB_UI_JS, generated from web/ui.js at build time

static WebServer *uiServer = NULL;

// Shared front-end components. EventPlot is the unified level/event timeline used
// by BOTH the audio dashboard and the camera-only motion page: same code, same
// data contract (binary Uint16 history + JSON events), so the two boards converge
// on one plot. A host element gets the range buttons, canvas, zoom + pan sliders,
// and an info line; drag to zoom, double-click to reset, tap a point/marker for
// details. opts: {historyUrl(secs,points,end,thr), eventsUrl(secs), hasLog,
// markerColor(source), onMarker(ev), defSecs, refreshMs}.
// The shared front-end (EventPlot, mjpegStream, buildNav) documented above
// lives in web/ui.js and is compiled in as WEB_UI_JS by the web/ codegen
// (tools/gen_web_assets.py). Edit the asset there, not here.

static void handleUiJs() {
  if (!webAuthCheck(*uiServer)) return;
  // The UI changes with every firmware release; a heuristically-cached stale
  // copy (this server sends no validators) can outlive an OTA and leave pages
  // driving a UI that no longer matches the device. Always refetch (~16KB, LAN).
  uiServer->sendHeader("Cache-Control", "no-cache");
  uiServer->send_P(200, "application/javascript", WEB_UI_JS);
}

static void handleCaps() {
  if (!webAuthCheck(*uiServer)) return;
  char j[128];
  snprintf(j, sizeof(j),
           "{\"name\":\"%s\",\"has_audio\":%s,\"has_sd\":%s,\"has_video\":%s}",
           DEVICE_NAME, HAS_AUDIO ? "true" : "false", HAS_SD ? "true" : "false", "true");
  uiServer->send(200, "application/json", j);
}

void webUiRegisterEndpoints(WebServer &server) {
  uiServer = &server;
  server.on("/ui.js", HTTP_GET, handleUiJs);
  server.on("/caps", HTTP_GET, handleCaps);
}

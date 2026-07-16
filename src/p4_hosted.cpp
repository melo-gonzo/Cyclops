// p4_hosted.cpp — see p4_hosted.h.
#if defined(CAMERA_MODEL_ESP32P4)

#include "p4_hosted.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include "esp32-hal-hosted.h"  // hostedGet*Version / hostedBeginUpdate / ...
#include "web_auth.h"
#include "diag_log.h"

// Auth helpers defined in main.cpp (wrap webAuthCheck on the global server).
bool webAuthOk();
bool webAuthWriteOk();

static void handleC6Version(WebServer &server) {
  uint32_t hM, hm, hp, sM, sm, sp;
  hostedGetHostVersion(&hM, &hm, &hp);
  hostedGetSlaveVersion(&sM, &sm, &sp);
  char j[192];
  snprintf(j, sizeof(j),
           "{\"host\":\"%lu.%lu.%lu\",\"slave\":\"%lu.%lu.%lu\",\"slave_target\":\"%s\","
           "\"update_available\":%s}",
           (unsigned long)hM, (unsigned long)hm, (unsigned long)hp,
           (unsigned long)sM, (unsigned long)sm, (unsigned long)sp,
           hostedGetSlaveTargetName() ? hostedGetSlaveTargetName() : "unknown",
           hostedHasUpdate() ? "true" : "false");
  server.send(200, "application/json", j);
}

// Stream `url` (plain HTTP, LAN) into the C6's passive OTA slot, activate, and
// restart the P4 so the esp-hosted transport re-initializes on the new slave.
static void handleC6Update(WebServer &server) {
  if (!server.hasArg("url")) {
    server.send(400, "text/plain", "missing ?url=<http bin url>\n");
    return;
  }
  String url = server.arg("url");
  dlog(DLOG_INFO, "C6 update from %s", url.c_str());

  HTTPClient http;
  if (!http.begin(url)) { server.send(500, "text/plain", "http begin failed\n"); return; }
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    http.end();
    char m[64]; snprintf(m, sizeof(m), "GET failed: %d\n", code);
    server.send(502, "text/plain", m);
    return;
  }
  int total = http.getSize();
  if (total <= 0) { http.end(); server.send(502, "text/plain", "no content length\n"); return; }

  if (!hostedBeginUpdate()) { http.end(); server.send(500, "text/plain", "hostedBeginUpdate failed\n"); return; }

  NetworkClient *stream = http.getStreamPtr();
  static uint8_t buf[4096];
  int written = 0;
  bool ok = true;
  while (written < total) {
    size_t avail = stream->available();
    if (!avail) {
      if (!stream->connected()) { ok = false; break; }
      delay(5);
      continue;
    }
    int n = stream->readBytes(buf, avail > sizeof(buf) ? sizeof(buf) : avail);
    if (n <= 0) { ok = false; break; }
    if (!hostedWriteUpdate(buf, n)) { ok = false; break; }
    written += n;
  }
  http.end();

  if (!ok || written != total) {
    dlog(DLOG_ERR, "C6 update aborted at %d/%d bytes", written, total);
    server.send(500, "text/plain", "transfer/flash failed - old slave kept\n");
    return;
  }
  if (!hostedEndUpdate()) { server.send(500, "text/plain", "hostedEndUpdate failed\n"); return; }
  if (!hostedActivateUpdate()) { server.send(500, "text/plain", "activate failed\n"); return; }

  dlog(DLOG_INFO, "C6 slave updated (%d bytes) - restarting", written);
  server.send(200, "text/plain", "C6 updated - device restarting\n");
  delay(750);  // let the response flush
  ESP.restart();
}

void p4HostedRegisterEndpoints(WebServer &server) {
  server.on("/c6/version", HTTP_GET, [&server]() {
    if (!webAuthOk()) return;
    handleC6Version(server);
  });
  server.on("/c6/update", HTTP_GET, [&server]() {
    if (!webAuthWriteOk()) return;  // mutates the co-processor: write role only
    handleC6Update(server);
  });
}

#endif // CAMERA_MODEL_ESP32P4

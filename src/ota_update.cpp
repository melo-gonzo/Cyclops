// ota_update.cpp - see ota_update.h for the design overview.

#include "ota_update.h"

#include <ArduinoOTA.h>
#include <Update.h>
#include <esp_task_wdt.h>

#include "branding.h"     // DEVICE_AP_PASS (default OTA secret)
#include "web_auth.h"     // webAuthCheck()
#include "wifi_portal.h"  // wifiPortalHostname()
#include "diag_log.h"     // dlog()

// OTA auth secret. Defaults to the fallback-AP password (already a per-build
// secret seeded from wifikeys.h via branding.h). Override -DOTA_PASSWORD=... for
// a distinct one. LAN-only, plain-HTTP threat model is accepted for this device.
#ifndef OTA_PASSWORD
#define OTA_PASSWORD DEVICE_AP_PASS
#endif

#define OTA_PORT 3232 // ArduinoOTA default; advertised via MDNS.enableArduino()

static volatile bool     otaInProgress = false;
static volatile uint32_t otaRebootAt   = 0; // restart once this millis() is reached (0 = none)

static WebServer *otaServer = nullptr; // captured at registration, like the other modules
static bool       otaAuthFail = false; // upload rejected at start: skip writes, send no 2nd reply

bool otaActive() { return otaInProgress; }

// Quiesce the heavy tasks before the flash write: set the flag, then give them a
// beat to notice (each checks otaActive() at the top of its loop and idles). This
// frees the SPI bus / PSRAM-DMA / CPU and stops new SD clips so none is left
// half-written across the reboot. Idempotent.
static void otaBeginGuard() {
  if (otaInProgress) return;
  otaInProgress = true;
  vTaskDelay(pdMS_TO_TICKS(150));
}

// ---------------- ArduinoOTA (dev push) ----------------

void otaInit() {
  ArduinoOTA.setHostname(wifiPortalHostname());
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setPort(OTA_PORT);
  // wifi_portal's startMdns() already advertises the OTA service on the shared
  // mDNS instance; let ArduinoOTA NOT re-init mDNS (that would drop the http
  // service record).
  ArduinoOTA.setMdnsEnabled(false);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;            // camera/audio/video tasks idle from here
    esp_task_wdt_delete(NULL);       // loop task won't feed the WDT mid-transfer
    dlog(DLOG_WARN, "OTA: ArduinoOTA start (%s)",
         ArduinoOTA.getCommand() == U_FLASH ? "flash" : "fs");
  });
  ArduinoOTA.onEnd([]() {
    dlog(DLOG_WARN, "OTA: ArduinoOTA complete - rebooting");
  });
  ArduinoOTA.onError([](ota_error_t err) {
    dlog(DLOG_ERR, "OTA: ArduinoOTA error %u", (unsigned)err);
    otaInProgress = false;
    esp_task_wdt_add(NULL);          // transfer failed; keep running, re-arm WDT
  });
  ArduinoOTA.begin();
}

void otaHandle() {
  ArduinoOTA.handle();
  if (otaRebootAt && (int32_t)(millis() - otaRebootAt) >= 0) {
    Serial.flush();
    ESP.restart();
  }
}

// ---------------- HTTP POST /update (fleet / dashboard) ----------------

// Streams the multipart firmware body straight into the OTA partition. Runs on
// the server task; blocks it for the ~2-10s transfer (the live stream stalls -
// acceptable for a deliberate update). Auth is checked once at FILE_START.
static void handleUpdateUpload() {
  HTTPUpload &up = otaServer->upload();
  switch (up.status) {
    case UPLOAD_FILE_START:
      if (!webAuthRequireWrite(*otaServer)) { otaAuthFail = true; return; } // 401/403 already sent
      otaAuthFail = false;
      otaBeginGuard();
      dlog(DLOG_WARN, "OTA: /update upload start (%s)", up.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        dlog(DLOG_ERR, "OTA: Update.begin failed (%u)", (unsigned)Update.getError());
      }
      break;
    case UPLOAD_FILE_WRITE:
      if (otaAuthFail || Update.hasError()) return;
      if (Update.write(up.buf, up.currentSize) != up.currentSize) {
        dlog(DLOG_ERR, "OTA: write error (%u)", (unsigned)Update.getError());
      }
      break;
    case UPLOAD_FILE_END:
      if (otaAuthFail) return;
      if (Update.end(true)) dlog(DLOG_WARN, "OTA: /update wrote %u bytes - rebooting", up.totalSize);
      else                  dlog(DLOG_ERR, "OTA: Update.end failed (%u)", (unsigned)Update.getError());
      break;
    case UPLOAD_FILE_ABORTED:
      Update.abort();
      otaInProgress = false;
      dlog(DLOG_ERR, "OTA: /update aborted");
      break;
    default:
      break;
  }
}

static void handleUpdateDone() {
  if (otaAuthFail) { otaAuthFail = false; return; } // webAuthCheck already sent 401
  bool ok = !Update.hasError();
  otaServer->sendHeader("Connection", "close");
  otaServer->send(ok ? 200 : 500, "application/json",
                  ok ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"update failed\"}");
  if (ok) otaRebootAt = millis() + 800; // let the response flush, then reboot (see otaHandle)
  else    otaInProgress = false;        // failed: resume normal operation
}

void otaRegisterEndpoints(WebServer &server) {
  otaServer = &server;
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateUpload);
}

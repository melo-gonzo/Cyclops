// main.cpp
/*
  ESP32-CAM MJPEG Streaming Server

  Hardware: ESP32-CAM (AI-Thinker model)
  Required: PSRAM for frame buffering

  Architecture:
  - camCB assembles the full MJPEG chunk (part headers + JPEG + boundary)
    directly into a PSRAM buffer. One memcpy of the JPEG, zero malloc/free
    on the hot path.
  - A 3-slot buffer pool with per-slot refcounts lets the camera publish
    new frames while the streaming task is still writing the previous one.
  - streamCB drains all queued clients per frame with a single write each,
    so N clients share one assembled chunk.
*/

#include "OV2640.h"
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include <esp_bt.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_pm.h>
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include "lwip/sockets.h" // raw send()/select() for the bounded MJPEG frame write

// Camera model comes from the platformio env build flags
// (-DCAMERA_MODEL_XIAO_ESP32S3 for the XIAO env); default to AI-Thinker.
#if !defined(CAMERA_MODEL_XIAO_ESP32S3) && !defined(CAMERA_MODEL_AI_THINKER)
#define CAMERA_MODEL_AI_THINKER
#endif
#include "camera_pins.h"
// Optional home-WiFi seed. With no wifikeys.h the device boots straight to the
// fallback AP for onboarding at /wifi (no secrets in source required).
#if __has_include("wifikeys.h")
#include "wifikeys.h"
#else
const char *ssid = "";
const char *password = "";
#endif
#include "branding.h"
#include "capabilities.h"  // HAS_AUDIO / HAS_SD board feature flags
#include "web_ui.h"        // shared /ui.js (EventPlot) + /caps
#include "diag_log.h"
#include "metrics_svc.h"
#include "presence.h"
#include "web_auth.h"
#include "wifi_portal.h"
#include "ota_update.h"
#include "cam_settings.h"

#include "video_record.h"  // motion detector + video recorder run on every board
#ifdef CAMERA_MODEL_XIAO_ESP32S3
#include "audio_capture.h"
#include "cont_record.h"
#include <Wire.h>

/**
 * Camera probe failed: scan the SCCB bus directly to distinguish a dead or
 * unplugged sensor (bus silent) from a software/config problem (sensor ACKs,
 * OV2640 = 0x30). The sensor only answers SCCB while XCLK is running, so
 * feed it 20MHz first.
 */
static void sccbDiagnosticScan() {
  ledcSetup(0, 20000000, 1);
  ledcAttachPin(XCLK_GPIO_NUM, 0);
  ledcWrite(0, 1); // 50% duty at 1-bit resolution
  delay(50);

  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM, 100000);
  int found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[diag] I2C ACK at 0x%02X%s\n", addr,
                    addr == 0x30 ? " <- OV2640 is alive, problem is software" : "");
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[diag] SCCB scan: bus silent - camera electrically disconnected (ribbon/connector)");
  }
  Wire.end();
  ledcDetachPin(XCLK_GPIO_NUM);
}
#endif

// CPU core assignments
#define APP_CPU 1
#define PRO_CPU 0

// Pacing between server.handleClient() calls. Kept short so dashboard polls
// (/diag, /audio/status, /video/status, /video/motion, /camera/status) aren't
// each delayed up to a full tick; core 1 is otherwise idle while camCB blocks
// on the sensor DMA, so this costs ~nothing.
const int SERVER_HANDLE_INTERVAL = 8;

const unsigned long RESTART_INTERVAL = 2 * 60 * 60 * 1000; // 2 hours

// Frame buffer configuration
#define BUFFER_COUNT 3
#define MAX_FRAME_SIZE (200 * 1024)
#define CHUNK_OVERHEAD 256
#define SEND_BUFFER_SIZE (MAX_FRAME_SIZE + CHUNK_OVERHEAD)
#define MAX_CLIENTS 8

// Global variables
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Set at the end of setup(); scheduledRestart's AI-Thinker uptime baseline.
unsigned long startTimeAGAIN = 0;

// Hardware objects
OV2640 cam;
WebServer server(80);
static bool cameraOk = false;
// Master live-stream switch (audio-only mode). When off, /mjpeg/1 and /jpg are
// refused so no viewer can wake the camera - combined with video recording off,
// the camera task parks and the PSRAM/heat load stops. NVS-persisted.
static volatile bool liveStreamOn = true;

// Task handles
TaskHandle_t tMjpeg;
TaskHandle_t tCam;
TaskHandle_t tStream;

// Client queue
QueueHandle_t streamingClients;

// Pre-assembled MJPEG chunk buffers.
// Each slot holds the complete per-frame payload:
//   "Content-Type: image/jpeg\r\nContent-Length: N\r\n\r\n" + JPEG + BOUNDARY
// Refcount > 0 means a consumer is currently reading from this slot, so
// the producer must not overwrite it. latestFrame names the newest ready slot.
static char*   sendBuffers[BUFFER_COUNT];
static size_t  sendLens[BUFFER_COUNT];
static volatile int sendRefcount[BUFFER_COUNT] = {0};
static volatile int latestFrame = -1;
// True between publishing a frame and streamCB picking it up. When the network
// is the bottleneck (slow/many viewers) the camera produces frames far faster
// than they drain; assembling each one is a wasted ~80KB PSRAM memcpy that
// contends with streamCB's TX reads. camCB skips assembly while this is set.
static volatile bool framePending = false;
// True while streamCB is parked with no viewers (blocked on its notification).
// camCB reads this to decide whether the camera itself may park.
static volatile bool streamTaskIdle = false;

// Pre-calculated HTTP response headers
static const char STREAM_HEADER[] = "HTTP/1.1 200 OK\r\n"
                                   "Access-Control-Allow-Origin: *\r\n"
                                   "Content-Type: multipart/x-mixed-replace; "
                                   "boundary=123456789000000000000987654321\r\n";

static const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
static const char CONTENT_TYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";

static const char JPEG_HEADER[] = "HTTP/1.1 200 OK\r\n"
                                 "Content-disposition: inline; filename=capture.jpg\r\n"
                                 "Content-type: image/jpeg\r\n\r\n";

static const int streamHeaderLen = sizeof(STREAM_HEADER) - 1;
static const int boundaryLen     = sizeof(BOUNDARY) - 1;
static const int contentTypeLen  = sizeof(CONTENT_TYPE) - 1;
static const int jpegHeaderLen   = sizeof(JPEG_HEADER) - 1;

/**
 * Disable Bluetooth to free up memory and CPU resources
 */
void disableBluetoothForPerformance() {
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
}

/**
 * Lock CPU at maximum frequency and disable light sleep.
 */
void configurePowerManagement() {
#if CONFIG_IDF_TARGET_ESP32S3
  esp_pm_config_esp32s3_t pm_config;
#else
  esp_pm_config_esp32_t pm_config;
#endif
  pm_config.max_freq_mhz = 240;
  pm_config.min_freq_mhz = 240;
  pm_config.light_sleep_enable = false;
  esp_pm_configure(&pm_config);
}

void scheduledRestart() {
// AI-Thinker stability hack; on the XIAO it would wipe RAM clips and the
// 24h amplitude history, and the S3 doesn't need it.
#ifndef CAMERA_MODEL_XIAO_ESP32S3
  if (millis() - startTimeAGAIN >= RESTART_INTERVAL) {
    Serial.println("Performing scheduled restart for system maintenance...");
    ESP.restart();
  }
#endif
}

/**
 * Allocate the PSRAM buffer pool.
 */
void initializeBufferPool() {
  for (int i = 0; i < BUFFER_COUNT; i++) {
    sendBuffers[i] = (char*)ps_malloc(SEND_BUFFER_SIZE);
    if (sendBuffers[i] == NULL) {
      Serial.println("CRITICAL: Failed to allocate frame buffer - insufficient PSRAM");
      ESP.restart();
    }
    sendLens[i] = 0;
    sendRefcount[i] = 0;
  }
  Serial.printf("Frame buffer pool: %d x %dKB\n", BUFFER_COUNT, SEND_BUFFER_SIZE / 1024);
}

/**
 * Pick a buffer slot safe to overwrite: refcount 0 and not the newest
 * frame (so a consumer that sees latestFrame has a moment to refcount it).
 * Returns -1 only if all slots are held by consumers, which shouldn't
 * happen with one streamCB consumer holding at most one slot.
 */
int acquireWriteSlot() {
  int chosen = -1;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < BUFFER_COUNT; i++) {
    if (sendRefcount[i] == 0 && i != latestFrame) {
      chosen = i;
      break;
    }
  }
  if (chosen == -1) {
    for (int i = 0; i < BUFFER_COUNT; i++) {
      if (sendRefcount[i] == 0) { chosen = i; break; }
    }
  }
  portEXIT_CRITICAL(&mux);
  return chosen;
}

// Measured capture rate: EMA of the inter-frame interval, updated by camCB.
// This is the sensor's true output rate (cam.run() blocks on DMA), not the
// stream or tap rate.
static volatile float camFrameEmaMs = 0;
static volatile uint32_t camLastFrameMs = 0;
static volatile uint32_t camStalls = 0; // sensor-DMA wedges recovered this session

// WDT-culprit forensics. The task watchdog watches two tasks: loop() and camCB.
// Each stamps its own heartbeat into RTC memory every iteration; the values
// survive a watchdog reset. After a TASK-watchdog reboot, whichever heartbeat
// lagged the other is the task that stopped running (hung or starved). camBeat==0
// is the sentinel for "camera was parked" (unwatched) -> loop is the culprit.
static RTC_NOINIT_ATTR uint32_t g_loopBeat;
static RTC_NOINIT_ATTR uint32_t g_camBeat;

float cameraMeasuredFps() {
  uint32_t last = camLastFrameMs;
  float ema = camFrameEmaMs;
  if (last == 0 || ema <= 0 || millis() - last > 3000) return 0; // parked/stale
  return 1000.0f / ema;
}

// Measured delivery rate: how fast frames actually leave over WiFi,
// counted over ~4s windows because TCP delivery is bursty. The gap
// between this and cameraMeasuredFps() is the network bottleneck
// (~100KB XGA frames over a ~150KB/s TCP link ≈ 1.5fps displayed).
static volatile float netFpsValue = 0;
static volatile uint32_t netLastFrameMs = 0;

float streamMeasuredFps() {
  if (netLastFrameMs == 0 || millis() - netLastFrameMs > 5000) return 0; // no viewers
  return netFpsValue;
}

/**
 * Camera capture task - runs on APP_CPU.
 * Captures a JPEG, builds the full MJPEG chunk in-place, and publishes it.
 */
void camCB(void *pvParameters) {
  // Watch this task: if cam.run() ever wedges on the sensor DMA, the WDT
  // resets the chip instead of leaving a dead stream (the XIAO has no
  // scheduled restart to fall back on). Unsubscribed around the idle
  // self-suspend below so parking never trips it.
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    g_camBeat = millis(); // WDT-culprit heartbeat (see g_camBeat decl)
    if (otaActive()) { vTaskDelay(pdMS_TO_TICKS(100)); continue; } // freeze DMA/PSRAM during OTA
    // cam.run() blocks until the sensor DMA completes, so it is the
    // natural pacemaker — no artificial FPS cap.
    cam.run();
    size_t frameSize = cam.getSize();
    char*  src = (char*)cam.getfb();

    if (src == NULL || frameSize == 0 || frameSize > MAX_FRAME_SIZE) {
      continue;
    }

    uint32_t fnow = millis();
    if (camLastFrameMs != 0) {
      float d = (float)(fnow - camLastFrameMs);
      camFrameEmaMs = camFrameEmaMs <= 0 ? d : camFrameEmaMs + 0.1f * (d - camFrameEmaMs);
    }
    camLastFrameMs = fnow;

    // Tap ~1 frame per period into the video pre-roll ring; never blocks. Runs on
    // every board: the motion detector reads from this ring (no SD needed). Done
    // before the skip below so recording keeps every frame even when throttled.
    videoTapFrame((const uint8_t *)src, frameSize);

    // If viewers exist but streamCB hasn't picked up the last frame we
    // published, the network is the bottleneck: don't burn PSRAM bandwidth
    // assembling a frame the consumer won't reach before a newer one. The
    // camera stays paced (cam.run above) and the tap already ran. Gated on
    // viewers>0 so the no-viewer path still falls through to self-suspend.
    if (framePending && uxQueueMessagesWaiting(streamingClients) > 0) {
      continue;
    }

    int idx = acquireWriteSlot();
    if (idx < 0) {
      continue;
    }

    // Assemble: CONTENT_TYPE + "<len>\r\n\r\n" + JPEG + BOUNDARY into one buffer.
    char* dst = sendBuffers[idx];
    int pos = 0;
    memcpy(dst + pos, CONTENT_TYPE, contentTypeLen);
    pos += contentTypeLen;
    pos += sprintf(dst + pos, "%u\r\n\r\n", (unsigned)frameSize);
    memcpy(dst + pos, src, frameSize);
    pos += frameSize;
    memcpy(dst + pos, BOUNDARY, boundaryLen);
    pos += boundaryLen;

    portENTER_CRITICAL(&mux);
    sendLens[idx] = pos;
    latestFrame = idx;
    framePending = true;
    portEXIT_CRITICAL(&mux);

    xTaskNotifyGive(tStream);

    // The video pre-roll ring + motion detector need frames even with no stream
    // viewers, so only park the camera when capture (recording or motion) is
    // inactive too. videoCaptureActive() is true on every board now.
    if (streamTaskIdle && !videoCaptureActive()) {
      esp_task_wdt_delete(NULL); // a parked task can't feed the WDT
      g_camBeat = 0;             // sentinel: parked/unwatched, not a WDT culprit
      vTaskSuspend(NULL);
      esp_task_wdt_add(NULL);    // resumed (new viewer / recording re-enabled)
      camLastFrameMs = millis(); // fresh heartbeat so the stall check doesn't
                                 // false-fire on the stale pre-park timestamp
    }
  }
}

// Re-enabling video recording (or motion) must wake a camera parked while idle.
static void wakeCameraTask() {
  if (tCam != NULL && eTaskGetState(tCam) == eSuspended) vTaskResume(tCam);
}

// No frame for this long while the camera should be running = sensor DMA wedged
// (esp_camera_fb_get never returns). Well above the ~2.5s/frame low-light worst
// case, and far under the 45s task-WDT so we recover before it reboots us.
#define CAM_STALL_MS 12000

// Recover a wedged camera WITHOUT rebooting the whole chip (the old behaviour:
// let the 45s WDT reset everything, which also drops WiFi and re-runs the
// SD-mount race). Runs from loop() - which feeds the WDT - never from camCB.
// The wedged camCB is blocked inside fb_get; vTaskDelete cleanly unlinks a task
// blocked on a queue, so it is safe to delete, then tear down and re-create the
// driver and the task. After repeated failures, fall back to a reboot.
static void recoverCamera() {
  static uint8_t fails = 0;
  camStalls++;
  metricsEvent(metrics::M_STALL);
  dlog(DLOG_ERR, "camera stalled %lus - reinitialising (stall #%lu)",
       (unsigned long)((millis() - camLastFrameMs) / 1000), (unsigned long)camStalls);

  if (tCam != NULL) {
    esp_task_wdt_delete(tCam); // stop watching the task we're about to delete
    vTaskDelete(tCam);         // safe even though it's blocked in fb_get
    tCam = NULL;
  }
  esp_task_wdt_reset();

  esp_err_t err = cam.reinit();
  esp_task_wdt_reset();
  // Pace retries (camLastFrameMs gates the next attempt CAM_STALL_MS later) and
  // give up to a reboot if the sensor is truly gone.
  camLastFrameMs = millis();
  if (err != ESP_OK) {
    if (++fails >= 3) {
      dlog(DLOG_ERR, "camera reinit failed 0x%x x%u - rebooting", err, fails);
      delay(50);
      ESP.restart();
    }
    dlog(DLOG_ERR, "camera reinit failed 0x%x (try %u/3)", err, fails);
    return; // tCam stays NULL; loop() retries after CAM_STALL_MS
  }

  camSettingsInit(); // re-apply saved sensor settings (resolution, exposure, …)
  xTaskCreatePinnedToCore(camCB, "camera", 6144, NULL, 5, &tCam, APP_CPU);
  fails = 0;
  dlog(DLOG_INFO, "camera recovered");
}

// Called every loop() tick. Fires only when the camera should be producing
// frames (task exists and isn't parked) but hasn't for CAM_STALL_MS.
static void checkCameraStall() {
  if (!cameraOk) return;
  bool shouldRun = (tCam == NULL) || (eTaskGetState(tCam) != eSuspended);
  if (shouldRun && camLastFrameMs != 0 &&
      (int32_t)(millis() - camLastFrameMs) > CAM_STALL_MS) {
    recoverCamera();
  }
}

// Digest auth: the password never crosses the wire in cleartext, so this
// holds up on plain HTTP. Credentials are NVS-backed; the device ships
// passwordless (open) until a password is set at /wifi. See web_auth.cpp.
bool webAuthOk() {
  return webAuthCheck(server);
}

// For mutating handlers: also rejects the read-only viewer (403).
bool webAuthWriteOk() {
  return webAuthRequireWrite(server);
}

/**
 * Add a streaming client. Called from the HTTP server task.
 */
void handleJPGSstream(void) {
  if (!webAuthOk()) return;
  if (!cameraOk) {
    server.send(503, "text/plain", "camera not available");
    return;
  }
  if (!liveStreamOn) {
    server.send(503, "text/plain", "live stream off (audio-only mode)");
    return;
  }
  if (!uxQueueSpacesAvailable(streamingClients)) {
    // Genuinely at the viewer cap (streamCB reaps dead clients every cycle, so a
    // full queue now means real viewers, not stale sockets). Answer with a real
    // 503 instead of silently closing - a silent close shows as a blank stream /
    // curl "got nothing" and is impossible to tell apart from a crash.
    server.send(503, "text/plain", "too many viewers");
    return;
  }

  WiFiClient *client = new WiFiClient();
  *client = server.client();
  client->setNoDelay(true);

  client->write(STREAM_HEADER, streamHeaderLen);
  client->write(BOUNDARY, boundaryLen);

  // Drop the client cleanly if the queue lost its slot between the
  // spaces-available check and here (shouldn't happen with one producer,
  // but a 0-timeout send can still fail - don't leak the WiFiClient).
  if (xQueueSend(streamingClients, (void *)&client, 0) != pdTRUE) {
    delete client;
    return;
  }

  if (eTaskGetState(tCam) == eSuspended) vTaskResume(tCam);
  // streamCB parks on its notification (never vTaskSuspend), so this give both
  // wakes an idle task and requests an immediate fan-out - no resume needed.
  xTaskNotifyGive(tStream);
}

/**
 * Streaming task - fans out each published frame to every connected client.
 */
// Bounded, frame-atomic send for the MJPEG fan-out. A full JPEG (tens of KB) is
// bigger than the TCP send buffer, so it normally takes several non-blocking
// send() passes as the window drains - that's fine for a healthy client (drains
// in well under the deadline on a LAN). But a half-open / wedged client never
// drains: WiFiClient::write would then block ~10s (10 retries x 1s select),
// stalling EVERY other viewer. Here we cap the total push at STREAM_SEND_DEADLINE_MS
// and bail, so one stuck viewer is dropped in <1s instead of jamming the stream.
// Returns true only if the WHOLE frame went out (a partial frame would desync the
// multipart boundary, so the caller drops the client on false).
#define STREAM_SEND_DEADLINE_MS 800
static bool streamSendFrame(int fd, const char *buf, size_t len) {
  if (fd < 0) return false;
  size_t sent = 0;
  uint32_t start = millis();
  while (sent < len) {
    int r = send(fd, buf + sent, len - sent, MSG_DONTWAIT);
    if (r > 0) {
      sent += (size_t)r;
      continue;
    }
    if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false; // dead
    if (millis() - start > STREAM_SEND_DEADLINE_MS) return false;       // not draining
    // Send buffer full: wait briefly for it to open, but never past the deadline.
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    struct timeval tv = {0, 100000}; // 100ms
    select(fd + 1, NULL, &set, NULL, &tv);
  }
  return true;
}

void streamCB(void *pvParameters) {
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  for (;;) {
    // Wake on a fresh frame from camCB OR at least every 500ms. The timeout is a
    // liveness floor: camCB only notifies while framePending is clear, so if a send
    // ever wedged and left framePending stuck, a pure portMAX_DELAY wait here would
    // deadlock - the queue would fill with stale clients and every new viewer would
    // get rejected. Waking on the timeout guarantees we always drain the latest
    // frame (clearing framePending) and reap dead clients no matter what.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

    UBaseType_t activeClients = uxQueueMessagesWaiting(streamingClients);
    if (activeClients == 0) {
      // Race-free park: block on the task notification, NOT vTaskSuspend. A
      // give from handleJPGSstream that lands between the queue check above and
      // blocking here is latched, so the wakeup can't be lost. The old
      // suspend-based park had exactly that hole: a viewer could be enqueued in
      // the window before vTaskSuspend (the handler saw a still-running task and
      // skipped vTaskResume), leaving the client queued but never served - a
      // black stream that only healed when the NEXT viewer connected.
      streamTaskIdle = true;
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      streamTaskIdle = false;
      continue;
    }

    // Snapshot the latest frame and take a read reference so camCB can't
    // overwrite it while we're iterating clients.
    int idx;
    size_t len;
    portENTER_CRITICAL(&mux);
    idx = latestFrame;
    if (idx >= 0) {
      sendRefcount[idx]++;
      len = sendLens[idx];
      framePending = false; // consumer has moved on; camCB may assemble the next
    } else {
      len = 0;
    }
    portEXIT_CRITICAL(&mux);

    if (idx < 0 || len == 0) continue;

    const char* buf = sendBuffers[idx];
    bool wrote = false;

    int toProcess = activeClients;
    for (int i = 0; i < toProcess; i++) {
      WiFiClient *client;
      if (xQueueReceive(streamingClients, &client, 0) != pdTRUE) break;

      if (!client->connected()) {
        delete client;
        continue;
      }

      // Bounded send: a stuck client is dropped within STREAM_SEND_DEADLINE_MS
      // instead of blocking the whole fan-out for ~10s inside WiFiClient::write.
      if (!streamSendFrame(client->fd(), buf, len)) {
        client->stop();
        delete client;
        continue;
      }

      xQueueSend(streamingClients, &client, 0);
      wrote = true;
    }

    if (wrote) {
      static uint32_t winStart = 0;
      static uint32_t winFrames = 0;
      uint32_t nnow = millis();
      if (winStart == 0 || nnow - netLastFrameMs > 5000) { // fresh viewer
        winStart = nnow;
        winFrames = 0;
      }
      winFrames++;
      if (nnow - winStart >= 4000) {
        netFpsValue = winFrames * 1000.0f / (nnow - winStart);
        winStart = nnow;
        winFrames = 0;
      }
      netLastFrameMs = nnow;
    }

    portENTER_CRITICAL(&mux);
    sendRefcount[idx]--;
    portEXIT_CRITICAL(&mux);
  }
}

/**
 * Single JPEG capture endpoint.
 */
void handleJPG(void) {
  if (!webAuthOk()) return;
  if (!cameraOk) {
    server.send(503, "text/plain", "camera not available");
    return;
  }
  if (!liveStreamOn) {
    server.send(503, "text/plain", "live stream off (audio-only mode)");
    return;
  }
  WiFiClient client = server.client();
  if (!client.connected()) return;

  client.setNoDelay(true);

  // camCB owns the camera while it runs; calling cam.run() here too would
  // race on the driver's frame buffer (intermittent NULL frames). Serve the
  // newest published frame instead. We must NEVER call the blocking cam.run()
  // from this webserver task: in low light a capture can stall for seconds,
  // freezing the MJPEG stream and every dashboard poll (single-threaded server).
  // If no published frame is ready, return a quick 503 and let the client retry.
  if (tCam != NULL && eTaskGetState(tCam) != eSuspended) {
    int idx;
    size_t len = 0;
    portENTER_CRITICAL(&mux);
    idx = latestFrame;
    if (idx >= 0) {
      sendRefcount[idx]++;
      len = sendLens[idx];
    }
    portEXIT_CRITICAL(&mux);
    if (idx >= 0) {
      // Slot layout: CONTENT_TYPE + "<len>\r\n\r\n" + JPEG + BOUNDARY
      const char *buf = sendBuffers[idx];
      const char *end = buf + len - boundaryLen;
      const char *jpg = buf + contentTypeLen;
      while (jpg + 4 < end && memcmp(jpg, "\r\n\r\n", 4) != 0) jpg++;
      jpg += 4;
      bool ok = jpg < end;
      if (ok) {
        client.write(JPEG_HEADER, jpegHeaderLen);
        client.write(jpg, end - jpg);
      }
      portENTER_CRITICAL(&mux);
      sendRefcount[idx]--;
      portEXIT_CRITICAL(&mux);
      if (ok) return;
    }
    // No published frame yet (just woke up) - fall through to 503.
  }

  // No recently-published frame available; do not block the server capturing one.
  server.send(503, "text/plain", "camera warming up, try again");
}

void handleDiag() {
  if (!webAuthOk()) return;
  uint16_t camW, camH;
  camGetFrameDims(&camW, &camH);
  sensor_t *sensor = cameraOk ? esp_camera_sensor_get() : NULL;
  char json[384];
  snprintf(json, sizeof(json),
           "{\"rssi\":%d,\"ip\":\"%s\",\"uptime_ms\":%lu,"
           "\"free_heap\":%u,\"free_psram\":%u,\"camera\":%s,\"clients\":%u,"
           "\"temp_c\":%.1f,\"fps\":%.1f,\"net_fps\":%.1f,"
           "\"res\":\"%ux%u\",\"quality\":%d,\"xclk\":%d,\"cam_stalls\":%lu,"
           "\"stream_on\":%s%s}",
           WiFi.RSSI(),
           (WiFi.status() == WL_CONNECTED ? WiFi.localIP() : WiFi.softAPIP()).toString().c_str(),
           (unsigned long)millis(), ESP.getFreeHeap(), ESP.getFreePsram(),
           cameraOk ? "true" : "false",
           (unsigned)presenceActive(),
           temperatureRead(), // internal die temp, runs ~20-30C above ambient
           cameraMeasuredFps(), streamMeasuredFps(),
           camW, camH,
           sensor ? sensor->status.quality : 0,
           camSavedXclkMhz(), (unsigned long)camStalls,
           liveStreamOn ? "true" : "false"
#ifdef CAMERA_MODEL_XIAO_ESP32S3
           , videoThermalPausedNow() ? ",\"thermal_paused\":true" : ",\"thermal_paused\":false"
#else
           , ""
#endif
  );

  String out = json;
#ifdef CAMERA_MODEL_XIAO_ESP32S3
  // SD diagnostic: connected state + how flaky the card has been this session.
  char sd[112];
  uint32_t since = sdSecsSinceDrop();
  snprintf(sd, sizeof(sd), ",\"sd\":%s,\"sd_drops\":%u,\"sd_drop_age_s\":%ld,\"sd_mhz\":%u",
           sdIsAvailable() ? "true" : "false", (unsigned)sdDrops(),
           since == UINT32_MAX ? -1L : (long)since, (unsigned)sdMountClockMhz());
  out.remove(out.length() - 1); // splice before the closing brace
  out += sd;
  out += "}";
#endif
  // Firmware build id (both boards) - confirm an OTA actually took effect.
  out.remove(out.length() - 1); // splice before the closing brace
  out += ",\"fw\":\"" FW_VERSION "\"";
  out += webAuthIsViewer() ? ",\"viewer\":true}" : ",\"viewer\":false}";
  server.send(200, "application/json", out);
}

// Camera hardware probe: distinguishes a control-bus (SCCB) fault from a
// data-bus (DVP) fault without ever calling the blocking esp_camera_fb_get().
// - sensor id (PID/VER/MIDH/MIDL) is read over SCCB at init and cached here:
//   a correct PID (OV2640 = 0x26) proves SIOD/SIOC + XCLK + power are good.
// - ever_got_frame=false means camCB has never received a single frame since
//   boot, i.e. cam.run() is blocked waiting on DMA that never completes ->
//   the DVP parallel data lines (D0-D7/PCLK/VSYNC/HREF) are not delivering.
void handleCameraProbe() {
  if (!webAuthOk()) return;
  sensor_t *s = cameraOk ? esp_camera_sensor_get() : NULL;
  uint32_t age = camLastFrameMs ? (millis() - camLastFrameMs) : 0;
  char j[288];
  snprintf(j, sizeof(j),
           "{\"sensor_present\":%s,\"pid\":\"0x%02x\",\"ver\":\"0x%02x\","
           "\"midh\":\"0x%02x\",\"midl\":\"0x%02x\",\"sccb_ok\":%s,"
           "\"ever_got_frame\":%s,\"last_frame_age_ms\":%ld,"
           "\"cam_fps\":%.1f,\"cam_stalls\":%lu}",
           s ? "true" : "false",
           s ? s->id.PID : 0, s ? s->id.VER : 0,
           s ? s->id.MIDH : 0, s ? s->id.MIDL : 0,
           (s && s->id.PID) ? "true" : "false",
           camLastFrameMs ? "true" : "false",
           camLastFrameMs ? (long)age : -1L,
           cameraMeasuredFps(), (unsigned long)camStalls);
  server.send(200, "application/json", j);
}

// Plain-text event log (RTC ring). /log?clear wipes it. Survives reboots, so
// after the device resets itself this shows what led up to it.
void handleLog() {
  if (server.hasArg("clear")) {
    if (!webAuthWriteOk()) return;   // clearing mutates: viewer is rejected (403); 401 handled inside
    dlogClear();
    dlog(DLOG_INFO, "log cleared via /log?clear");
    server.send(200, "text/plain", "log cleared\n");
    return;
  }
  if (!webAuthOk()) return;          // dump is read-only
  static char buf[4096]; // server task only
  dlogDump(buf, sizeof(buf)); // NUL-terminated
  server.send(200, "text/plain", buf);
}

// Master live-stream switch (audio-only mode), NVS-persisted in "sys".
static void loadStreamPref() {
  Preferences p;
  if (p.begin("sys", true)) { liveStreamOn = p.getBool("stream", true); p.end(); }
}
void handleStreamPower() {
  if (!webAuthWriteOk()) return;
  if (server.hasArg("on")) {
    liveStreamOn = server.arg("on") == "1";
    Preferences p;
    if (p.begin("sys", false)) { p.putBool("stream", liveStreamOn); p.end(); }
    dlog(DLOG_INFO, "live stream %s via /camera/power", liveStreamOn ? "ON" : "OFF");
  }
  char j[40];
  snprintf(j, sizeof(j), "{\"stream_on\":%s}", liveStreamOn ? "true" : "false");
  server.send(200, "application/json", j);
}

// Static quick-reference for the tunable settings. XIAO-only subsystems (audio,
// spectral trigger, video) are #ifdef'd out of the page on the esp32cam build so
// the docs always match the firmware actually running.
static const char DOCS_PAGE[] PROGMEM = R"DOC(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>)DOC" DEVICE_NAME R"DOC( - Docs</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#14171c;color:#dde3ea;margin:0;padding:16px;max-width:760px;margin:auto}
a{color:#6fb3ff}
.card{background:#1d2229;border-radius:10px;padding:14px 16px;margin-bottom:14px}
h3{font-size:.85em;color:#8a94a0;font-weight:500;margin:0 0 10px;text-transform:uppercase;letter-spacing:.05em}
#nav{display:flex;gap:6px;align-items:center;margin:4px 0 12px;flex-wrap:wrap}
#nav b{margin-right:10px}
#nav a,#nav span{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}
#nav .cur{background:#1d2229;color:#dde3ea}
#tabs{display:flex;gap:6px;flex-wrap:wrap;margin:0 0 16px}
#tabs button{background:#1d2229;color:#8a94a0;border:0;padding:7px 13px;border-radius:8px;font-size:.9em;cursor:pointer}
#tabs button.cur{background:#2b6cb0;color:#fff}
section{display:none}
section.on{display:block}
dl{margin:0;display:grid;grid-template-columns:160px 1fr;gap:6px 16px}
dt{color:#dde3ea;font-weight:600;font-size:.9em}
dd{margin:0;color:#aab4c0;font-size:.9em;line-height:1.45}
.r{color:#6f7a86;font-size:.82em;white-space:nowrap}
.lead{color:#8a94a0;font-size:.9em;margin:0 0 4px}
.p{color:#aab4c0;font-size:.9em;line-height:1.5;margin:0 0 10px}
.p+.p{margin-top:-2px}
code{background:#14171c;border-radius:4px;padding:1px 5px;font-size:.85em;color:#cbd5e1}
.card ul{margin:6px 0 0;padding-left:18px;color:#aab4c0;font-size:.9em;line-height:1.5}
.card li{margin-bottom:4px}
@media(max-width:560px){dl{grid-template-columns:1fr}dt{margin-top:8px}}
</style></head><body>
<div id="nav"></div>
<div id="tabs">
  <button data-s="ov" class="cur">Overview</button>
  <button data-s="arch">Architecture</button>
)DOC"
#ifdef CAMERA_MODEL_XIAO_ESP32S3
R"DOC(  <button data-s="trig">Triggers</button>
)DOC"
#endif
R"DOC(  <button data-s="set">Settings</button>
  <button data-s="net">Setup &amp; network</button>
</div>

<section id="ov">
<div class="card"><h3>What it is</h3>
<p class="p">)DOC" DEVICE_NAME R"DOC( is a trigger-based audio + video capture node built on the Seeed XIAO ESP32-S3 Sense (the firmware also builds for the ESP32-CAM, video only). It listens and watches; when sound or motion crosses a threshold it saves a short clip — with the seconds <em>before</em> the event included — to the SD card.</p>
<p class="p">Everything is served from an on-device web dashboard over your LAN or the device's own WiFi hotspot. There is no cloud: capture, storage, and playback are entirely local.</p>
</div>
<div class="card"><h3>The pages</h3><dl>
<dt>Clips</dt><dd>Saved audio/video clips (RAM + SD), the level plot, and the live spectrum.</dd>
<dt>Live</dt><dd>Live MJPEG video and the live audio stream.</dd>
<dt>Camera</dt><dd>Sensor controls — resolution, exposure, gain, white balance.</dd>
<dt>Record</dt><dd>Continuous (untriggered) recording to SD, plus the trigger settings.</dd>
<dt>Graphs</dt><dd>System-metrics history: RSSI, FPS, heap/PSRAM, temperature, triggers.</dd>
<dt>WiFi</dt><dd>Network join, hostname, accounts, and over-the-air firmware updates.</dd>
</dl></div>
</section>

<section id="arch">
<div class="card"><h3>Capture pipeline</h3>
<p class="p"><strong>Audio:</strong> PDM mic → I2S DMA → DSP (high/low-pass filters, RMS level, FFT) → trigger decision → PSRAM pre-roll ring → SD writer → <code>clip_NNNNN_x.wav</code>.</p>
<p class="p"><strong>Video:</strong> camera → JPEG frame ring (PSRAM) → motion frame-diff → AVI writer → <code>vclip_NNNNN_x.avi</code>. An audio trigger can also record a paired video clip and vice-versa.</p>
</div>
<div class="card"><h3>Concurrency &amp; storage</h3>
<ul>
<li>Independent FreeRTOS tasks (audio capture, camera, MJPEG stream, SD writer, web server) pinned across both cores.</li>
<li>All SD/SPI access is serialized through one mutex. The clip list and cap-eviction hot paths read an in-RAM index of <code>/clips</code> instead of scanning the card, so a slow card never stalls the UI.</li>
<li>Settings live in NVS and survive reboots. The level / heatmap / metrics history is mirrored to SD, so the plots survive power loss too.</li>
</ul>
</div>
<div class="card"><h3>Clip storage &amp; retention</h3>
<ul>
<li>Clip numbers run <code>00000</code>–<code>09999</code> and wrap, keeping names tidy. Each kind (audio, video) keeps the newest N; the oldest is evicted first (wrap-aware).</li>
<li>Per-clip over-threshold data (level / threshold / Hz) is persisted in <code>clipmeta.csv</code> so the Clips table's "Over thr" column survives reboots.</li>
<li>Continuous segments (Record page) roll off by a size/time retention cap, independent of triggers.</li>
</ul>
</div>
<div class="card"><h3>Code shape</h3>
<p class="p">"Functional core / imperative shell": the decision math — filters, thresholds, FFT, retention, parsing — lives in pure C++ headers unit-tested on the host (<code>pio test -e native</code>). The <code>.cpp</code> shells own the hardware and timing. XIAO-only subsystems are compiled out of the ESP32-CAM build, so this page always matches the running firmware.</p>
</div>
</section>
)DOC"
#ifdef CAMERA_MODEL_XIAO_ESP32S3
R"DOC(<section id="trig">
<div class="card"><h3>Level trigger — adaptive</h3>
<p class="p">The device continuously learns the room's noise floor with a slow-rise / fast-fall estimator: it follows a sudden quieting quickly but creeping ambient drift slowly. The trigger fires when the level exceeds <strong>floor × sensitivity</strong>, clamped up to the "min threshold" so a silent room can't become a hair-trigger.</p>
<p class="p">A <em>sustained</em> loud sound is gradually absorbed into the floor (habituation) so it stops re-firing, while brief transients keep triggering. A manual threshold (&gt; 0) overrides the adaptive value entirely.</p>
</div>
<div class="card"><h3>Level trigger — statistical</h3>
<p class="p">An alternative model that tracks the running <strong>mean and standard deviation (σ)</strong> of the level over a window (in minutes) and fires on samples <strong>k·σ above the mean</strong>. A busy, variable room raises its own bar automatically; a quiet room stays sensitive. One knob: <code>k</code> (lower = more sensitive). <code>k</code> is the σ multiplier — distinct from the measured σ shown live.</p>
</div>
<div class="card"><h3>Spectral (band) trigger</h3>
<p class="p">Runs an FFT and learns a per-band floor with the same adaptive model. It fires when any single frequency band crosses its own threshold — catching narrow-band sounds (beeps, alarms, whines) that broadband level misses — and names the hottest band in the log and tooltip.</p>
</div>
<div class="card"><h3>Motion &amp; pre-roll</h3>
<p class="p"><strong>Motion:</strong> an on-device block frame-diff; a video clip records when enough of the frame changes. Off by default because it costs CPU.</p>
<p class="p"><strong>Pre-roll:</strong> a rolling buffer continuously holds the last few seconds, so every clip starts <em>before</em> the trigger. <strong>Gain:</strong> thresholds scale with mic gain, so changing gain doesn't change how sensitive the trigger feels.</p>
</div>
</section>
)DOC"
#endif
R"DOC(<section id="set">
<p class="lead">Tunable settings. Ranges in grey.</p>
)DOC"
#ifdef CAMERA_MODEL_XIAO_ESP32S3
R"DOC(<div class="card"><h3>Audio capture</h3><dl>
<dt>Clip length</dt><dd>Total length of each saved WAV, pre-roll included. <span class="r">0.5–10 s</span></dd>
<dt>Pre-roll</dt><dd>Seconds kept from <em>before</em> the trigger fired. <span class="r">&lt; clip length</span></dd>
<dt>Mic gain</dt><dd>Digital gain on the PDM mic. Trigger thresholds scale with it, so loudness sensitivity stays put. <span class="r">1–64×</span></dd>
<dt>Auto-trigger</dt><dd>Start a clip automatically when the level crosses the threshold.</dd>
<dt>Sensitivity</dt><dd>Adaptive threshold = learned noise-floor × this. <strong>Lower = more sensitive.</strong> <span class="r">1.2–20</span></dd>
<dt>Threshold algorithm</dt><dd>Floor × factor, or statistical (mean + k·σ). <span class="r">see Triggers</span></dd>
<dt>k (σ multiplier)</dt><dd>Statistical mode: fire k standard deviations above the mean. <strong>Lower = more sensitive.</strong></dd>
<dt>Averaging window</dt><dd>Statistical mode: minutes of level history the mean/σ are computed over. <span class="r">0.25–240 min</span></dd>
<dt>Min threshold</dt><dd>Absolute floor for the adaptive threshold; stops a dead-quiet room becoming a hair-trigger. <span class="r">0–32767</span></dd>
<dt>Manual threshold</dt><dd>Fixed trigger level; overrides the adaptive value when set. <span class="r">0 = adaptive</span></dd>
<dt>Low-pass filter</dt><dd>Rolls off high-frequency hiss above the cutoff. Affects the live stream, saved clips, and the level meter. <span class="r">200–7800 Hz</span></dd>
<dt>High-pass filter</dt><dd>Rolls off low-frequency rumble / HVAC below the cutoff. <span class="r">30–2000 Hz</span></dd>
<dt>Level retention</dt><dd>How far back the amplitude/threshold plot is kept. Longer = coarser buckets, same memory. <span class="r">6–72 h</span></dd>
</dl></div>
<div class="card"><h3>Spectral trigger &amp; heatmap</h3><dl>
<dt>Band trigger</dt><dd>Fire when any single frequency band crosses its own adaptive threshold (catches narrow-band sounds the level meter misses).</dd>
<dt>Band sensitivity</dt><dd>Per-band threshold = band floor × this. <strong>Higher = LESS sensitive.</strong> <span class="r">1.5–100</span></dd>
<dt>Min band magnitude</dt><dd>Advanced floor: bands quieter than this never trigger, regardless of sensitivity.</dd>
<dt>Heatmap window</dt><dd>Time span of the frequency heatmap. Longer = coarser columns, same memory. Scale is dBFS (0 = full-scale). <span class="r">1–120 min</span></dd>
</dl></div>
<div class="card"><h3>Video clips</h3><dl>
<dt>Enable</dt><dd>Master on/off for clip recording; frees the PSRAM frame ring when off.</dd>
<dt>FPS</dt><dd>Frames per second captured into clips. <span class="r">2–10</span></dd>
<dt>Clip length / Pre-roll</dt><dd>Same meaning as audio, applied to the MJPEG-AVI clip.</dd>
<dt>Record on audio</dt><dd>An audio trigger also records a paired video clip (and vice-versa).</dd>
<dt>Motion detect</dt><dd>On-device frame-diff trigger. Off by default — it costs CPU.</dd>
<dt>Motion sensitivity</dt><dd>How much frame change counts as motion; higher diff / more blocks = less twitchy (ignores AEC flicker).</dd>
</dl></div>
)DOC"
#endif
R"DOC(<div class="card"><h3>Camera</h3><dl>
<dt>Resolution</dt><dd>Sensor frame size. Larger = sharper but slower and heavier on PSRAM/SD.</dd>
<dt>Quality / exposure / gain</dt><dd>Standard OV-sensor controls (JPEG quality, auto/manual exposure, AGC, white balance). Tune live on the Camera page.</dd>
<dt>Live stream</dt><dd>Master on/off for the MJPEG stream; capture keeps running for triggers when off.</dd>
</dl></div>
<div class="card"><h3>Diagnostics (Graphs)</h3><dl>
<dt>Metrics retention</dt><dd>How far back the system-metrics graphs are kept. Longer = coarser buckets, same memory. <span class="r">2–24 h</span></dd>
<dt>Series</dt><dd>RSSI, FPS, heap/PSRAM, temperature, clients, trigger/stall/drop counts. Each is normalized to its own range; click the legend to toggle, hover for real values.</dd>
</dl></div>
<div class="card"><h3>Continuous record</h3><dl>
<dt>Continuous record</dt><dd>Back-to-back segments to SD on the Record page, independent of triggers; old segments roll off by a retention cap.</dd>
</dl></div>
<p class="lead">Settings persist across reboots (NVS). History plots also survive power loss via the SD card.</p>
</section>

<section id="net">
<div class="card"><h3>First boot</h3>
<p class="p">Out of the box the device raises a WiFi hotspot. Connect to it, open the dashboard, and on the <b>WiFi</b> page enter your home network. Set a hostname — the device is then reachable at <code>&lt;host&gt;.local</code> via mDNS — and optionally a static-IP last octet, which lets a whole fleet run one identical binary while staying individually addressable. Reboot to apply network changes; other settings apply live.</p>
</div>
<div class="card"><h3>Accounts</h3><dl>
<dt>Admin</dt><dd>Full control: every setting, clip deletion, firmware updates.</dd>
<dt>Viewer (optional)</dt><dd>Read-only: can watch, listen, and download clips, but cannot change any setting. Enable it on the WiFi page.</dd>
</dl></div>
<div class="card"><h3>Firmware (OTA)</h3>
<p class="p">Update over the air from the <b>Firmware</b> card on the WiFi page: upload a <code>firmware.bin</code> and the device verifies it, writes the spare OTA partition, and reboots into it. The version line shows the running build. The dual-OTA layout means a bad image can roll back to the previous one.</p>
</div>
</section>

<script>
(function(){
var tb=document.querySelectorAll('#tabs button'),sc=document.querySelectorAll('section');
function show(id){var i;for(i=0;i<sc.length;i++)sc[i].classList.toggle('on',sc[i].id===id);
for(i=0;i<tb.length;i++)tb[i].classList.toggle('cur',tb[i].dataset.s===id);
if(history.replaceState)history.replaceState(null,'','#'+id);else location.hash=id;}
for(var k=0;k<tb.length;k++)tb[k].addEventListener('click',function(){show(this.dataset.s);});
var h=location.hash.slice(1);show(document.getElementById(h)&&document.getElementById(h).tagName=='SECTION'?h:'ov');
})();
</script>
<script src="/ui.js"></script><script>buildNav('/docs')</script>
</body></html>)DOC";

void handleDocs() {
  if (!webAuthOk()) return;
  server.send_P(200, "text/html", DOCS_PAGE);
}

#if !HAS_AUDIO
// Camera-only boards (no mic, no SD in this firmware) get purpose-built pages
// instead of the audio dashboard: live video still works, and motion detection
// runs off the PSRAM frame ring (its history lives on the Graphs timeline). Only
// SD-backed continuous recording is genuinely unavailable. Shared dark-theme
// chrome + nav so these match the rest of the UI.
static String pageOpen(const char *cur) {
  String h;
  h.reserve(1500);
  h += F("<!DOCTYPE html><html><head><meta charset=utf-8>"
         "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
         "<title>" DEVICE_NAME "</title><style>"
         "body{font-family:-apple-system,system-ui,sans-serif;background:#14171c;color:#dde3ea;margin:0;padding:16px;max-width:760px;margin:auto}"
         ".card{background:#1d2229;border-radius:10px;padding:14px 16px;margin-bottom:14px}"
         "h3{margin:0 0 8px}.p{color:#aab4c0;font-size:.95em;line-height:1.5;margin:0}"
         ".p a,a.lnk{color:#6fb3ff}img{display:block;width:100%;border-radius:8px;background:#000}"
         "label{display:inline-flex;align-items:center;gap:7px;font-size:.95em}"
         ".big{font-size:1.5em;font-weight:600}.mut{color:#8a94a0;font-size:.85em}"
         "button.sm{background:#222933;color:#cdd6df;border:1px solid #2a313a;border-radius:6px;padding:5px 10px;font-size:.85em;cursor:pointer}"
         "button.sm.cur{background:#2b6cb0;color:#fff;border-color:#2b6cb0}"
         "canvas{display:block;width:100%}"
         "#nav{display:flex;gap:6px;align-items:center;margin:4px 0 12px;flex-wrap:wrap}"
         "#nav b{margin-right:10px}"
         "#nav a,#nav span{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}"
         "#nav .cur{background:#1d2229;color:#dde3ea}"
         "</style></head><body><div id=nav></div>"
         "<script src=/ui.js></script><script>buildNav('");
  h += cur;
  h += F("')</script>");
  return h;
}

// "/" on a camera-only board: the shared Event plot (same component the XIAO
// audio dashboard uses) bound to the motion timeline + motion events.
void handleMotionPage() {
  if (!webAuthOk()) return;
  String h = pageOpen("/");
  h += F("<div class=card><h3>Event plot</h3>"
         "<label><input type=checkbox id=mot onchange=tog()> motion detection</label>"
         "<span class=mut style=margin-left:12px><b id=lvl>—</b> blocks changed now</span>"
         "<div id=ep style=margin-top:10px></div>"
         "<p class=p style=margin-top:8px>Peak motion (changed blocks) over time; "
         "dashed line is the trigger threshold, triangles are motion events. No SD "
         "on this board, so nothing is saved &mdash; <a class=lnk href=/live>Live</a> "
         "for video, <a class=lnk href=/graphs>Graphs</a> for more series.</p></div>"
         "<script src=/ui.js></script>"
         "<script>"
         "EventPlot(document.getElementById('ep'),{"
         "historyUrl:q=>'/video/motion/history'+q,"
         "eventsUrl:s=>'/video/motion/events?secs='+s});"
         "async function tick(){try{const s=await(await fetch('/video/status')).json();"
         "mot.checked=!!s.motion;lvl.textContent=s.motion?s.motion_level:'off';}catch(e){}}"
         "async function tog(){await fetch('/video/config?motion='+(mot.checked?1:0));tick();}"
         "tick();setInterval(()=>{if(!document.hidden)tick()},1500);"
         "document.addEventListener('visibilitychange',()=>{if(!document.hidden)tick()});"
         "</script></body></html>");
  server.send(200, "text/html", h);
}

// "/live" on a camera-only board: the live MJPEG video (no audio on this board).
void handleLiveVideoPage() {
  if (!webAuthOk()) return;
  String h = pageOpen("/live");
  h += F("<div class=card><h3>Live video</h3>"
         "<img id=cam alt=\"live stream\">"
         "<p class=p style=margin-top:10px>Live MJPEG stream. This board has no "
         "microphone, so there's no audio. Motion detection and its timeline are on "
         "the <a class=lnk href=/>Clips</a> and <a class=lnk href=/graphs>Graphs</a> "
         "tabs.</p></div>"
         // Robust MJPEG: fetch()-based reader that auto-reconnects on stall/drop
         // instead of freezing on a half-decoded frame like a bare <img> does.
         "<script>mjpegStream(document.getElementById('cam'),'/mjpeg/1').start()</script>"
         "</body></html>");
  server.send(200, "text/html", h);
}

void handleFeatureRecNA() {
  if (!webAuthOk()) return;
  String h = pageOpen("/rec");
  h += F("<div class=card><h3>Recording needs an SD card</h3>"
         "<p class=p>Continuous recording writes to an SD card, which isn't wired "
         "on this board in this firmware. Motion detection still runs (no SD "
         "needed) — see <a class=lnk href=/>Clips</a> and "
         "<a class=lnk href=/graphs>Graphs</a>.</p></div></body></html>");
  server.send(200, "text/html", h);
}
#endif // !HAS_AUDIO

void handleNotFound() {
  // Minimal 404: do not enumerate routes (that leaks the endpoint map pre-auth)
  // and do not reflect the requested URI back as content.
  server.send(404, "text/plain", "Not found");
}

/**
 * HTTP server task - spawns capture/stream workers and runs the server loop.
 */
void mjpegCB(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(SERVER_HANDLE_INTERVAL);

  streamingClients = xQueueCreate(MAX_CLIENTS, sizeof(WiFiClient *));

  if (cameraOk) {
    xTaskCreatePinnedToCore(camCB,    "camera", 6144, NULL, 5, &tCam,    APP_CPU);
    xTaskCreatePinnedToCore(streamCB, "stream", 6144, NULL, 5, &tStream, PRO_CPU);
  }

  server.on("/mjpeg/1", HTTP_GET, handleJPGSstream);
  server.on("/jpg",     HTTP_GET, handleJPG);
  server.on("/diag",    HTTP_GET, handleDiag);
  server.on("/camera/probe", HTTP_GET, handleCameraProbe); // SCCB vs DVP fault isolation
  server.on("/log",     HTTP_GET, handleLog);
  server.on("/camera/power", HTTP_GET, handleStreamPower); // master live-stream on/off
  server.on("/docs",    HTTP_GET, handleDocs);             // static settings reference
  metricsInit();                    // diagnostics history rings (PSRAM)
  metricsRegisterEndpoints(server); // /graphs diagnostics time-series page
  webUiRegisterEndpoints(server);   // shared /ui.js (EventPlot) + /caps (all boards)
  camSetFpsSource(cameraMeasuredFps);
  camSetNetFpsSource(streamMeasuredFps);
  camRegisterEndpoints(server);
  wifiPortalRegisterEndpoints(server);
  otaRegisterEndpoints(server); // POST /update firmware upload (both boards)
#if HAS_AUDIO
  audioRegisterEndpoints(server);   // also registers "/", "/audio", "/live"
#else
  // Camera-only board: no audio dashboard. Serve a motion overview at "/" and a
  // live-video page at "/live"; motion + its Graphs timeline work without SD.
  server.on("/",     HTTP_GET, handleMotionPage);
  server.on("/live", HTTP_GET, handleLiveVideoPage);
#endif
  videoRegisterEndpoints(server);   // motion detection + /video/* on every board
#if HAS_SD
  contRegisterEndpoints(server);    // "/rec" continuous recording (needs SD)
#else
  server.on("/rec",  HTTP_GET, handleFeatureRecNA);
#endif
  webAuthBegin(server); // collect the Cookie header for remember-me sessions
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("HTTP server started");

  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    server.handleClient();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void printSystemInfo() {
  Serial.println("\n========== ESP32-CAM System Diagnostics ==========");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("XTAL Frequency: %d MHz\n", getXtalFrequencyMhz());
  Serial.printf("APB Frequency: %d MHz\n", getApbFrequency() / 1000000);
  Serial.printf("Flash Speed: %d MHz\n", ESP.getFlashChipSpeed() / 1000000);
  Serial.printf("Flash Size: %d MB\n", ESP.getFlashChipSize() / (1024 * 1024));

  FlashMode_t flashMode = ESP.getFlashChipMode();
  Serial.print("Flash Mode: ");
  switch(flashMode) {
    case FM_QIO:  Serial.println("QIO (Fastest)"); break;
    case FM_QOUT: Serial.println("QOUT"); break;
    case FM_DIO:  Serial.println("DIO"); break;
    default:      Serial.println("DOUT"); break;
  }

  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Total Heap: %d bytes\n", ESP.getHeapSize());

  if (psramFound()) {
    Serial.println("PSRAM: Available");
    Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
    Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
  } else {
    Serial.println("PSRAM: NOT FOUND - This will severely limit performance!");
  }

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  Serial.printf("Chip: ESP32 with %d cores, revision %d\n", chip_info.cores, chip_info.revision);
  Serial.println("==================================================\n");
}

#ifdef CAMERA_MODEL_XIAO_ESP32S3
// BOOT button (GPIO0) long-press = factory reset, the recovery path when the
// web password is forgotten (every HTTP endpoint is auth-gated, so software
// recovery is impossible). Holding BOOT during a *reset* enters ROM download
// mode instead, so this only fires from a deliberate hold on a running device.
#define RESET_BTN_GPIO 0
#define RESET_HOLD_MS  5000
#endif

void setup() {
#ifdef CAMERA_MODEL_AI_THINKER
  // Disable brownout detector: ESP32-CAM's AMS1117 regulator dips on WiFi TX
  // bursts and the BOD will reset the chip mid-frame. Stopgap until a 1000uF
  // cap on 5V is added.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

  Serial.begin(115200);
  delay(300);

  // Earliest point with Serial up: capture why we (re)started. The RTC ring
  // survived the reboot, so this records whether the last run crashed/hung.
  dlogBegin();

  // If the last run died to the task watchdog, name the task that stopped. The
  // RTC heartbeats are valid here because a TASK-watchdog reset implies the
  // previous run executed and stamped them.
  if (esp_reset_reason() == ESP_RST_TASK_WDT) {
    uint32_t lb = g_loopBeat, cb = g_camBeat;
    if (cb == 0) {
      dlog(DLOG_ERR, "WDT culprit: loop (camera was parked); loopBeat=%lus",
           (unsigned long)(lb / 1000));
    } else {
      uint32_t lag = lb < cb ? cb - lb : lb - cb;
      dlog(DLOG_ERR, "WDT culprit: %s hung ~%lus (loopBeat=%lu camBeat=%lu)",
           lb < cb ? "loop" : "camera", (unsigned long)(lag / 1000),
           (unsigned long)lb, (unsigned long)cb);
    }
  }

  loadStreamPref(); // master live-stream switch (audio-only mode), from NVS

  Serial.println("ESP32-CAM MJPEG Streaming Server");
  Serial.println("Initializing system...");

  printSystemInfo();

  disableBluetoothForPerformance();
  configurePowerManagement();
  initializeBufferPool();

#ifdef CAMERA_MODEL_XIAO_ESP32S3
  pinMode(RESET_BTN_GPIO, INPUT_PULLUP); // BOOT button for factory-reset hold
#endif

  camera_config_t config = {}; // zero-init: driver has fields we don't set (e.g. sccb_i2c_port)
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  // Board default (XIAO 20MHz, AI-Thinker 8MHz) or the user's saved /camera
  // override. Runtime set_xclk wedges the S3 capture DMA, so xclk is only
  // ever applied here at init.
  config.xclk_freq_hz = camSavedXclkMhz() * 1000000;
  config.frame_size   = FRAMESIZE_XGA; // 1024x768
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST; // always serve newest frame
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12; // lower number = better quality; on the OV2640 dropping
                            // from ~30 to ~12 sharpens the stream for only ~10% more
                            // bytes (measured), so the link cost is negligible
  config.fb_count     = 2;

  // Camera failure is non-fatal: keep WiFi/audio up so the rest of the
  // system can be exercised while the sensor connection is sorted out.
  cameraOk = (cam.init(config) == ESP_OK);
  if (!cameraOk) {
    Serial.println("WARNING: Camera init failed - continuing without video");
#ifdef CAMERA_MODEL_XIAO_ESP32S3
    sccbDiagnosticScan();
#endif
  }

  if (cameraOk) {
  delay(100);

  // Project sensor defaults + any user overrides saved from /camera,
  // including a saved resolution (see cam_settings.cpp).
  camSettingsInit();

  delay(50);
  } // cameraOk

  esp_task_wdt_init(45, true);
  esp_task_wdt_add(NULL);

  // Joins the strongest saved network; if none is reachable (or standalone
  // mode is set) the fallback AP comes up instead - never restart, the
  // server must stay reachable either way.
  if (!wifiPortalConnect()) {
    Serial.println("WiFi: not joined - reachable via the " DEVICE_NAME " access point");
  }

  // Over-the-air updates: ArduinoOTA push (dev) + HTTP /update (fleet). After
  // WiFi/mDNS are up so the hostname resolves for espota. Works on the AP too.
  otaInit();

#if HAS_AUDIO
  if (audioInit()) {
    audioStart();
  } else {
    Serial.println("WARNING: audio init failed - continuing video-only");
  }
#endif
  // Video recorder + motion detector run on EVERY board: motion detection works
  // off the PSRAM frame ring with no SD and feeds the Graphs timeline (M_MOTION).
  // Clip *saving* is skipped without an SD card (all SD paths guard internally).
  {
    uint16_t camW, camH;
    camGetFrameDims(&camW, &camH); // whatever framesize camSettingsInit applied
    if (videoInit(camW, camH)) {
      videoSetWakeCallback(wakeCameraTask);
      videoStart();
    } else {
      Serial.println("WARNING: video recorder init failed - live stream unaffected");
    }
  }
#if HAS_SD
  contInit(); // continuous-recording settings + rolling pruner (needs SD)
#endif

  xTaskCreatePinnedToCore(mjpegCB, "mjpeg_server", 12288, NULL, 5, &tMjpeg, APP_CPU);

  startTimeAGAIN = millis();

  Serial.println("ESP32-CAM streaming server started successfully!");
  Serial.println("System ready for streaming clients");
}

#ifdef CAMERA_MODEL_XIAO_ESP32S3
// Full factory reset: wipe the SD card, clear all network + web-login settings,
// then reboot. Comes up on the fallback AP, passwordless (no login) by default.
static void factoryReset() {
  Serial.println("[reset] BOOT held - performing factory reset");
  audioSdWipeAll();         // delete everything under /clips
  wifiPortalFactoryReset(); // clear network + web-login NVS -> AP on next boot
  Serial.flush();
  delay(300);
  ESP.restart();
}

// Polled from loop() (~100ms). Fires once after the BOOT button is held for
// RESET_HOLD_MS; releasing before then cancels.
static void checkFactoryResetButton() {
  static uint32_t pressedSince = 0;
  static bool fired = false;
  if (digitalRead(RESET_BTN_GPIO) == LOW) { // active-low BOOT button
    if (pressedSince == 0) {
      pressedSince = millis();
    } else if (!fired && millis() - pressedSince >= RESET_HOLD_MS) {
      fired = true;
      factoryReset();
    }
  } else {
    pressedSince = 0;
  }
}
#endif

/**
 * Main loop handles light system monitoring; heavy work runs in tasks.
 * Polls at 100ms so the BOOT-button hold is timed accurately; the status
 * print and wifi tick stay on their own 5s cadence.
 */
void loop() {
  static unsigned long lastSystemCheck = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastSystemCheck >= 5000) {
    wifiPortalTick();
    scheduledRestart();
    lastSystemCheck = currentTime;

    uint32_t heap = ESP.getFreeHeap();
    Serial.printf("Status - Clients: %d, Free Heap: %d, Free PSRAM: %d, WiFi RSSI: %d dBm\n",
                  uxQueueMessagesWaiting(streamingClients),
                  heap, ESP.getFreePsram(), WiFi.RSSI());

    // Log breadcrumbs to the diag ring: an immediate warning if heap runs low
    // (a leak/fragmentation is a prime suspect for the server going unresponsive),
    // plus a sparse heartbeat so the ring keeps useful history without flooding.
    static unsigned long lastBeat = 0;
    static bool heapWarned = false;
    if (heap < 30000 && !heapWarned) {
      dlog(DLOG_ERR, "LOW HEAP %uK - server may stall", (unsigned)(heap / 1024));
      heapWarned = true;
    } else if (heap > 50000) {
      heapWarned = false; // re-arm once it recovers
    }
    if (currentTime - lastBeat >= 300000) { // every 5 min
      lastBeat = currentTime;
      dlog(DLOG_INFO, "alive: heap %uK, psram %uK, rssi %d, clients %d",
           (unsigned)(heap / 1024), (unsigned)(ESP.getFreePsram() / 1024),
           WiFi.RSSI(), (int)uxQueueMessagesWaiting(streamingClients));
    }

#ifdef CAMERA_MODEL_XIAO_ESP32S3
    // Software thermal governor: pause PSRAM-heavy video when the die runs too
    // hot (octal PSRAM goes out of timing spec -> core-1 wedge); audio is fine.
    float dieC = temperatureRead();
    if (videoThermalUpdate(dieC)) {
      dlog(videoThermalPausedNow() ? DLOG_WARN : DLOG_INFO,
           videoThermalPausedNow() ? "THERMAL: video paused at %.0fC (too hot for PSRAM)"
                                   : "THERMAL: video resumed at %.0fC",
           dieC);
    }
#endif

  }

  // Diagnostics sampler on its own (tunable) cadence so /graphs retention is
  // adjustable at fixed memory. The gauges are cheap to re-read here; COUNT
  // events accumulate in their modules and are snapshot+reset by metricsSample.
  static unsigned long lastMetricsSample = 0;
  if (currentTime - lastMetricsSample >= metricsBucketMs()) {
    lastMetricsSample = currentTime;
    uint32_t h = ESP.getFreeHeap();
    metricsSet(metrics::M_RSSI, WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0);
    metricsSet(metrics::M_CAMFPS, cameraMeasuredFps());
    metricsSet(metrics::M_NETFPS, streamMeasuredFps());
    metricsSet(metrics::M_HEAP, h / 1024.0f);
    metricsSet(metrics::M_PSRAM, ESP.getFreePsram() / 1024.0f);
    metricsSet(metrics::M_CLIENTS, (float)presenceActive());
    metricsSet(metrics::M_TEMP, temperatureRead()); // die temp (both builds)
    metricsSample(); // snapshot one bucket of every diagnostics metric
  }

#ifdef CAMERA_MODEL_XIAO_ESP32S3
  checkFactoryResetButton();
#endif

  checkCameraStall(); // reinit a wedged sensor instead of riding to a WDT reboot

  otaHandle(); // pump ArduinoOTA + service a pending post-update reboot

  g_loopBeat = millis(); // WDT-culprit heartbeat (see g_loopBeat decl)
  esp_task_wdt_reset();
  delay(100);
}

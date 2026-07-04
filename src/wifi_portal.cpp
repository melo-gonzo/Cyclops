// wifi_portal.cpp - see wifi_portal.h

#include "wifi_portal.h"

#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <time.h>

#include "web_auth.h"
#include "branding.h"
#include "diag_log.h"

// Compiled-in home network from wifikeys.h (included once by main.cpp).
extern const char *ssid;
extern const char *password;

// Fallback AP credentials (see branding.h). WPA2 needs >= 8 characters. The
// /wifi page shows these (it sits behind digest auth like everything else).
#define WIFI_AP_SSID DEVICE_NAME
#define WIFI_AP_PASS DEVICE_AP_PASS

// Static IP is fully configurable at /wifi (address/netmask/gateway/DNS) and,
// when enabled, applies to whatever network the device joins. Off => DHCP.

#define MAX_NETS              6
#define CONNECT_TIMEOUT_MS    10000
#define BOOT_MAX_ATTEMPTS     4      // bounded so setup() stays under the 45s WDT
#define FALLBACK_AP_AFTER_MS  45000  // station down this long -> raise the AP
#define STA_RETRY_PERIOD_MS   60000  // retry saved networks while the AP idles

static char netSsid[MAX_NETS][33];
static char netPass[MAX_NETS][65];
static int  netCount = 0;
// Guards every read/write of the net table above. HTTP handlers (add/del) run on
// the server task while the loop task reads the table in tryKnown/tryJoin/tick;
// a /wifi/del mid-scan could otherwise read a half-shifted entry or stale count.
// A FreeRTOS mutex (not a portMUX) because we may briefly hold it across strlcpy.
static SemaphoreHandle_t netMtx = NULL;
static bool standalone = false;
static bool ipStatic = false;                 // false => DHCP on every network
static char ipAddr[16] = "";                  // static IP config (when ipStatic)
static char ipMask[16] = "255.255.255.0";
static char ipGw[16]   = "";
static char ipDns[16]  = "";
static char apPass[65] = WIFI_AP_PASS;        // fallback-AP password (editable)
static char deviceHost[33] = DEVICE_HOST;     // mDNS hostname (<host>.local), per-device

static bool apUp = false;
static volatile int  pendingJoin = -1;   // saved-net index, executed in tick
static volatile bool pendingMode = false;
static volatile bool pendingDrop = false; // forgot the active net: drop link in tick
static volatile uint32_t rebootAt = 0;    // restart once this millis() is reached
static volatile bool joining = false;
static char joinMsg[96] = "";
static uint32_t staLostMs = 0;
static uint32_t lastStaRetryMs = 0;

static WebServer *wifiServer = NULL;

// ---------------- saved networks (NVS) ----------------

static void saveNets() {
  Preferences p;
  if (!p.begin("wifi", false)) return;
  p.putInt("n", netCount);
  p.putBool("ap", standalone);
  p.putBool("ipst", ipStatic);
  p.putString("ipaddr", ipAddr);
  p.putString("ipmask", ipMask);
  p.putString("ipgw", ipGw);
  p.putString("ipdns", ipDns);
  p.putString("appass", apPass);
  p.putString("host", deviceHost);
  char key[4];
  for (int i = 0; i < netCount; i++) {
    snprintf(key, sizeof(key), "s%d", i); p.putString(key, netSsid[i]);
    snprintf(key, sizeof(key), "p%d", i); p.putString(key, netPass[i]);
  }
  p.end();
}

static void loadNets() {
  Preferences p;
  int n = -1;
  if (p.begin("wifi", true)) {
    n = p.getInt("n", -1);
    standalone = p.getBool("ap", false);
    ipStatic = p.getBool("ipst", false);
    p.getString("ipaddr", ipAddr, sizeof(ipAddr));
    if (p.getString("ipmask", ipMask, sizeof(ipMask)) == 0 || ipMask[0] == 0)
      strlcpy(ipMask, "255.255.255.0", sizeof(ipMask));
    p.getString("ipgw", ipGw, sizeof(ipGw));
    p.getString("ipdns", ipDns, sizeof(ipDns));
    if (p.getString("appass", apPass, sizeof(apPass)) == 0 || apPass[0] == 0)
      strlcpy(apPass, WIFI_AP_PASS, sizeof(apPass));
    if (p.getString("host", deviceHost, sizeof(deviceHost)) == 0 || deviceHost[0] == 0)
      strlcpy(deviceHost, DEVICE_HOST, sizeof(deviceHost));
    char key[4];
    for (int i = 0; i < n && i < MAX_NETS; i++) {
      snprintf(key, sizeof(key), "s%d", i);
      p.getString(key, netSsid[i], sizeof(netSsid[i]));
      snprintf(key, sizeof(key), "p%d", i);
      p.getString(key, netPass[i], sizeof(netPass[i]));
    }
    p.end();
  }
  if (n < 0) {
    // First boot: seed the home network from wifikeys.h if one is compiled in,
    // otherwise start empty so the device comes up on the AP for onboarding.
    if (ssid && ssid[0]) {
      strlcpy(netSsid[0], ssid, sizeof(netSsid[0]));
      strlcpy(netPass[0], password, sizeof(netPass[0]));
      netCount = 1;
    } else {
      netCount = 0;
    }
    saveNets();
  } else {
    netCount = n > MAX_NETS ? MAX_NETS : n;
  }
}

// ---------------- connection machinery ----------------

// When a static IP is configured (at /wifi) it applies to whatever network the
// device joins; otherwise every network uses DHCP. Set just before WiFi.begin().
static void applyIpConfig(const char *s) {
  (void)s; // static config is global now, not tied to a specific SSID
  IPAddress ip, gw, mask, dns;
  if (ipStatic && ip.fromString(ipAddr)) {
    if (!mask.fromString(ipMask)) mask.fromString("255.255.255.0");
    gw.fromString(ipGw);                          // 0.0.0.0 if blank/invalid
    if (!(ipDns[0] && dns.fromString(ipDns))) dns = gw; // DNS defaults to gateway
    WiFi.config(ip, gw, mask, dns);
  } else {
    WiFi.config(IPAddress((uint32_t)0), IPAddress((uint32_t)0), IPAddress((uint32_t)0)); // DHCP
  }
}

static void startMdns() {
  MDNS.end();
  const char *h = deviceHost[0] ? deviceHost : DEVICE_HOST;
  if (MDNS.begin(h)) {
    MDNS.addService("http", "tcp", 80);
    // Advertise the ArduinoOTA service on THIS mDNS instance so espota can find
    // <host>.local. ArduinoOTA's own mDNS is disabled (otaInit) so it doesn't
    // re-init mDNS and drop the http service above. auth=true (OTA has a password).
    MDNS.enableArduino(3232, true);
  }
}

const char *wifiPortalHostname() {
  return deviceHost[0] ? deviceHost : DEVICE_HOST;
}

static void onStaUp() {
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // max TX power -> fewer retransmits
  // NTP (UTC): gives SD files real mtimes; browser renders local time
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  startMdns();
  staLostMs = 0;
  bool isStatic = ssid && ssid[0] && WiFi.SSID() == ssid; // static IP only on home net
  Serial.printf("[wifi] connected to \"%s\", IP %s\n",
                WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  dlog(DLOG_INFO, "wifi up \"%s\" %s (%s) rssi %d",
       WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
       isStatic ? "static" : "DHCP", WiFi.RSSI());
  if (!isStatic && ssid && ssid[0])
    dlog(DLOG_WARN, "static IP NOT applied: joined net != home \"%s\"", ssid);
}

static void startAp() {
  if (apUp) return;
  // AP_STA (not plain AP) so scans and join attempts still work underneath.
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.softAPConfig(IPAddress(192, 168, 8, 1), IPAddress(192, 168, 8, 1),
                    IPAddress(255, 255, 255, 0));
  apUp = WiFi.softAP(WIFI_AP_SSID, apPass);
  if (apUp) {
    startMdns();
    Serial.printf("[wifi] AP \"%s\" up at http://%s\n",
                  WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[wifi] softAP start FAILED");
  }
}

static void stopAp() {
  if (!apUp) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apUp = false;
  Serial.println("[wifi] AP stopped (station link restored)");
}

// Blocking join attempt; resets the task WDT while it waits. Runs in the
// loop task (boot and tick) only - never from an HTTP handler.
static bool tryJoin(int slot) {
  // Copy the chosen credentials out of the net table under the lock, then drop
  // it BEFORE the blocking WiFi work below - we must not hold netMtx across a
  // slow WiFi call, and a concurrent /wifi/del could shift the table meanwhile.
  char ssidBuf[33], passBuf[65];
  xSemaphoreTake(netMtx, portMAX_DELAY);
  if (slot < 0 || slot >= netCount) { xSemaphoreGive(netMtx); return false; }
  strlcpy(ssidBuf, netSsid[slot], sizeof(ssidBuf));
  strlcpy(passBuf, netPass[slot], sizeof(passBuf));
  xSemaphoreGive(netMtx);

  joining = true;
  snprintf(joinMsg, sizeof(joinMsg), "joining \"%s\"...", ssidBuf);
  Serial.printf("[wifi] trying \"%s\"\n", ssidBuf);
  WiFi.disconnect();
  applyIpConfig(ssidBuf);
  WiFi.begin(ssidBuf, passBuf);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < CONNECT_TIMEOUT_MS) {
    delay(100);
    esp_task_wdt_reset();
  }
  joining = false;
  if (WiFi.status() == WL_CONNECTED) {
    onStaUp();
    snprintf(joinMsg, sizeof(joinMsg), "connected to \"%s\" - %s",
             ssidBuf, WiFi.localIP().toString().c_str());
    return true;
  }
  snprintf(joinMsg, sizeof(joinMsg), "could not join \"%s\" (wrong password or out of range)",
           ssidBuf);
  Serial.printf("[wifi] \"%s\" failed\n", ssidBuf);
  return false;
}

// Scan and try saved networks, strongest first; unseen ones (hidden SSIDs)
// are tried last. Attempt count is bounded for the WDT's sake.
static bool tryKnown() {
  // Snapshot the net table under the lock so the (blocking) scan + ordering below
  // run against a stable copy, never a half-shifted table from a concurrent
  // /wifi/del. tryJoin() re-locks and re-validates the slot when it actually joins.
  char snapSsid[MAX_NETS][33];
  int  snapCount;
  xSemaphoreTake(netMtx, portMAX_DELAY);
  snapCount = netCount;
  for (int i = 0; i < snapCount; i++) strlcpy(snapSsid[i], netSsid[i], sizeof(snapSsid[i]));
  xSemaphoreGive(netMtx);
  if (snapCount == 0) return false;

  int16_t found = WiFi.scanNetworks(); // blocking, ~2-3s
  esp_task_wdt_reset();

  int order[MAX_NETS], rssi[MAX_NETS], cand = 0;
  for (int i = 0; i < snapCount; i++) {
    for (int j = 0; j < found; j++) {
      if (WiFi.SSID(j) == snapSsid[i]) {
        order[cand] = i;
        rssi[cand++] = WiFi.RSSI(j);
        break;
      }
    }
  }
  for (int a = 0; a < cand; a++) // strongest first
    for (int b = a + 1; b < cand; b++)
      if (rssi[b] > rssi[a]) {
        int t = order[a]; order[a] = order[b]; order[b] = t;
        t = rssi[a]; rssi[a] = rssi[b]; rssi[b] = t;
      }
  for (int i = 0; i < snapCount && cand < MAX_NETS; i++) { // hidden SSIDs
    bool seen = false;
    for (int a = 0; a < cand; a++) seen |= (order[a] == i);
    if (!seen) order[cand++] = i;
  }
  WiFi.scanDelete();

  int tries = cand < BOOT_MAX_ATTEMPTS ? cand : BOOT_MAX_ATTEMPTS;
  for (int a = 0; a < tries; a++) {
    if (tryJoin(order[a])) return true;
  }
  return false;
}

// ---------------- public API ----------------

bool wifiPortalIsAp() { return apUp; }

void wifiPortalFactoryReset() {
  Preferences p;
  // Clear our network store, then pin n=0 so loadNets() on the next boot does
  // NOT re-seed the home network from wifikeys.h - the device comes up on the
  // fallback AP, which is the whole point of a "force AP" recovery.
  if (p.begin("wifi", false)) {
    p.clear();
    p.putInt("n", 0);
    p.end();
  }
  // Clear the web-login override so it reverts to the passwordless default.
  if (p.begin("auth", false)) {
    p.clear();
    p.end();
  }
  // Drop the SDK's cached STA creds too, so it can't silently rejoin.
  WiFi.disconnect(false, true);
  Serial.println("[reset] network + auth settings cleared (AP on next boot)");
}

bool wifiPortalConnect() {
  if (!netMtx) netMtx = xSemaphoreCreateMutex(); // net-table lock, created once
  loadNets();
  WiFi.persistent(false); // creds live in our own NVS namespace, not the SDK's
  // Wipe any credentials the SDK cached on a previous run. Otherwise the SDK
  // auto-reconnects to the last network on boot - even one the user forgot, so
  // it never falls back to the AP. We only ever join networks from our list
  // (tryKnown calls WiFi.begin explicitly, which re-caches the chosen one).
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true); // eraseap: clear stored STA credentials
  esp_wifi_set_ps(WIFI_PS_NONE);
  if (standalone) {
    Serial.println("[wifi] standalone mode: hosting AP only");
    startAp();
    return false;
  }
  if (tryKnown()) return true;
  Serial.println("[wifi] no saved network reachable - raising fallback AP");
  startAp();
  lastStaRetryMs = millis();
  return false;
}

void wifiPortalTick() {
  // Reboot requested over HTTP: wait out a short grace period so the response
  // has flushed to the browser, then restart.
  if (rebootAt && (int32_t)(millis() - rebootAt) >= 0) {
    Serial.println("[wifi] reboot requested - restarting");
    Serial.flush();
    ESP.restart();
  }

  // Mode change requested over HTTP (response was already sent).
  if (pendingMode) {
    pendingMode = false;
    if (standalone) {
      WiFi.disconnect();
      staLostMs = 0;
      startAp();
      snprintf(joinMsg, sizeof(joinMsg), "standalone mode: AP \"%s\" only", WIFI_AP_SSID);
    } else {
      if (!tryKnown()) startAp();
      else if (WiFi.softAPgetStationNum() == 0) stopAp();
    }
    return;
  }

  // Forgot the network we were on (over HTTP): drop the live link. eraseap
  // clears the SDK's stored credentials so auto-reconnect can't silently
  // rejoin it. With nothing left to join, raise the fallback AP immediately
  // rather than waiting out FALLBACK_AP_AFTER_MS.
  if (pendingDrop) {
    pendingDrop = false;
    WiFi.disconnect(false, true);
    staLostMs = 0;
    xSemaphoreTake(netMtx, portMAX_DELAY);
    int nLeft = netCount; // consistent read of the (HTTP-mutated) net count
    xSemaphoreGive(netMtx);
    if (nLeft == 0) {
      startAp();
      snprintf(joinMsg, sizeof(joinMsg), "network forgotten - AP \"%s\" up", WIFI_AP_SSID);
    } else if (!tryKnown()) {
      startAp();
    } else if (apUp && WiFi.softAPgetStationNum() == 0) {
      stopAp();
    }
    return;
  }

  // Join requested over HTTP.
  if (pendingJoin >= 0) {
    int slot = pendingJoin;
    pendingJoin = -1;
    if (!tryJoin(slot) && !standalone) {
      // restore whatever was reachable before the attempt
      if (!tryKnown()) startAp();
    }
    return;
  }

  if (standalone) return;

  if (WiFi.status() == WL_CONNECTED) {
    staLostMs = 0;
    // drop a leftover fallback AP once nobody is using it
    if (apUp && WiFi.softAPgetStationNum() == 0) stopAp();
    return;
  }

  uint32_t now = millis();
  if (staLostMs == 0) {
    staLostMs = now;
    Serial.println("[wifi] station link lost, reconnecting...");
    WiFi.reconnect();
    return;
  }
  if (!apUp && now - staLostMs > FALLBACK_AP_AFTER_MS) {
    startAp();
    lastStaRetryMs = now;
    return;
  }
  // Periodic full retry - but never while someone is browsing via the AP,
  // because join attempts hop channels and would kick them off.
  if (now - lastStaRetryMs > STA_RETRY_PERIOD_MS &&
      (!apUp || WiFi.softAPgetStationNum() == 0)) {
    lastStaRetryMs = now;
    if (tryKnown() && apUp && WiFi.softAPgetStationNum() == 0) stopAp();
  }
}

// ---------------- HTTP handlers ----------------

static bool wifiAuthOk() { return webAuthCheck(*wifiServer); }
static bool wifiAuthWriteOk() { return webAuthRequireWrite(*wifiServer); } // 403s the viewer

static String jsonEsc(const String &s) {
  String o;
  o.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c >= 0x20) o += c;
  }
  return o;
}

static void handleWifiStatus() {
  if (!wifiAuthOk()) return;
  // A viewer must never see the admin/viewer usernames or the AP WPA2 password:
  // the /wifi page only CSS-hides the admin cards, so blank these out in the JSON
  // itself. Keep the keys present (empty string) so the page JS shape is unchanged.
  bool vw = webAuthIsViewer();
  bool sta = WiFi.status() == WL_CONNECTED;
  String j;
  j.reserve(512);
  j += "{\"standalone\":"; j += standalone ? "true" : "false";
  j += ",\"joining\":"; j += (joining || pendingJoin >= 0) ? "true" : "false";
  j += ",\"msg\":\""; j += jsonEsc(joinMsg); j += '"';
  j += ",\"sta\":{\"up\":"; j += sta ? "true" : "false";
  if (sta) {
    j += ",\"ssid\":\""; j += jsonEsc(WiFi.SSID());
    j += "\",\"ip\":\""; j += WiFi.localIP().toString();
    j += "\",\"rssi\":"; j += String(WiFi.RSSI());
  }
  j += "},\"ipstatic\":"; j += ipStatic ? "true" : "false";
  j += ",\"ipaddr\":\""; j += jsonEsc(ipAddr); j += '"';
  j += ",\"ipmask\":\""; j += jsonEsc(ipMask); j += '"';
  j += ",\"ipgw\":\""; j += jsonEsc(ipGw); j += '"';
  j += ",\"ipdns\":\""; j += jsonEsc(ipDns); j += '"';
  j += ",\"host\":\""; j += jsonEsc(deviceHost); j += '"';
  j += ",\"user\":\""; j += vw ? "" : jsonEsc(webAuthGetUser()); j += '"';
  j += ",\"authon\":"; j += webAuthEnabled() ? "true" : "false"; // false = passwordless
  j += ",\"vuser\":\""; j += vw ? "" : jsonEsc(webAuthGetViewerUser()); j += '"'; // "" = viewer off
  j += ",\"viewer\":"; j += webAuthIsViewer() ? "true" : "false";       // this session's role
  j += ",\"ap\":{\"up\":"; j += apUp ? "true" : "false";
  j += ",\"ssid\":\"" WIFI_AP_SSID "\",\"pass\":\""; j += vw ? "" : jsonEsc(apPass); j += '"';
  if (apUp) {
    j += ",\"ip\":\""; j += WiFi.softAPIP().toString();
    j += "\",\"stations\":"; j += String(WiFi.softAPgetStationNum());
  }
  j += "},\"time_ok\":"; j += (time(NULL) > 1750000000) ? "true" : "false";
  j += ",\"nets\":[";
  for (int i = 0; i < netCount; i++) {
    if (i) j += ',';
    j += "{\"i\":"; j += String(i);
    j += ",\"ssid\":\""; j += jsonEsc(netSsid[i]);
    j += "\",\"home\":"; j += (ssid && ssid[0] && strcmp(netSsid[i], ssid) == 0) ? "true" : "false";
    j += ",\"cur\":"; j += (sta && WiFi.SSID() == netSsid[i]) ? "true" : "false";
    j += '}';
  }
  j += "]}";
  wifiServer->send(200, "application/json", j);
}

static void handleWifiScan() {
  if (!wifiAuthOk()) return;
  if (wifiServer->hasArg("start")) {
    WiFi.scanDelete();
    WiFi.scanNetworks(true); // async; poll this endpoint until done
    wifiServer->send(200, "application/json", "{\"scanning\":true}");
    return;
  }
  int16_t n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) {
    wifiServer->send(200, "application/json", "{\"scanning\":true}");
    return;
  }
  if (n < 0) n = 0;
  String j;
  j.reserve(1024);
  j += "{\"scanning\":false,\"nets\":[";
  bool first = true;
  for (int i = 0; i < n; i++) {
    String s = WiFi.SSID(i);
    if (s.length() == 0) continue;
    bool dup = false; // keep only the strongest BSSID per SSID
    for (int k = 0; k < i; k++) dup |= (WiFi.SSID(k) == s);
    if (dup) continue;
    bool known = false;
    for (int k = 0; k < netCount; k++) known |= (s == netSsid[k]);
    if (!first) j += ',';
    first = false;
    j += "{\"ssid\":\""; j += jsonEsc(s);
    j += "\",\"rssi\":"; j += String(WiFi.RSSI(i));
    j += ",\"sec\":"; j += (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) ? "true" : "false";
    j += ",\"known\":"; j += known ? "true" : "false";
    j += '}';
  }
  j += "]}";
  wifiServer->send(200, "application/json", j);
}

static void handleWifiAdd() {
  if (!wifiAuthWriteOk()) return;
  String s = wifiServer->arg("ssid");
  String pw = wifiServer->arg("pass");
  if (s.length() == 0 || s.length() > 32 || pw.length() > 64) {
    wifiServer->send(400, "text/plain", "ssid must be 1..32 chars, pass <= 64");
    return;
  }
  // Hold netMtx across the lookup + count change + copy so a loop-task reader
  // (tryKnown/tick) never sees a partially written slot or a stale count.
  xSemaphoreTake(netMtx, portMAX_DELAY);
  int slot = -1;
  for (int i = 0; i < netCount; i++)
    if (s == netSsid[i]) slot = i; // re-adding updates the password
  if (slot < 0) {
    if (netCount >= MAX_NETS) {
      xSemaphoreGive(netMtx);
      wifiServer->send(507, "text/plain", "all " + String(MAX_NETS) + " slots used - forget one first");
      return;
    }
    slot = netCount++;
  }
  strlcpy(netSsid[slot], s.c_str(), sizeof(netSsid[slot]));
  strlcpy(netPass[slot], pw.c_str(), sizeof(netPass[slot]));
  xSemaphoreGive(netMtx);
  saveNets();
  if (wifiServer->arg("join") == "1") {
    standalone = false; // joining is an explicit exit from standalone mode
    saveNets();
    pendingJoin = slot;
    snprintf(joinMsg, sizeof(joinMsg), "joining \"%s\"...", netSsid[slot]);
  }
  wifiServer->send(200, "application/json", "{\"saved\":" + String(slot) + "}");
}

static void handleWifiJoin() {
  if (!wifiAuthWriteOk()) return;
  int i = wifiServer->hasArg("i") ? atoi(wifiServer->arg("i").c_str()) : -1;
  if (i < 0 || i >= netCount) {
    wifiServer->send(400, "text/plain", "bad index");
    return;
  }
  standalone = false;
  saveNets();
  pendingJoin = i;
  snprintf(joinMsg, sizeof(joinMsg), "joining \"%s\"...", netSsid[i]);
  wifiServer->send(200, "application/json", "{\"joining\":true}");
}

static void handleWifiDel() {
  if (!wifiAuthWriteOk()) return;
  int i = wifiServer->hasArg("i") ? atoi(wifiServer->arg("i").c_str()) : -1;
  if (i < 0 || i >= netCount) {
    wifiServer->send(400, "text/plain", "bad index");
    return;
  }
  // Forgetting the network we're currently on should actually drop it (the
  // expected behavior), not silently keep the live link. The tick does the
  // disconnect + AP fallback; doing WiFi state changes here (HTTP task) is
  // unsafe. staying connected on a different DHCP lease is what we're avoiding.
  // Hold netMtx across the whole shift-down + count decrement so a loop-task
  // reader (tryKnown/tick) never iterates a half-shifted table or a stale count.
  bool wasActive = WiFi.status() == WL_CONNECTED && WiFi.SSID() == netSsid[i];
  xSemaphoreTake(netMtx, portMAX_DELAY);
  for (int k = i; k < netCount - 1; k++) {
    strlcpy(netSsid[k], netSsid[k + 1], sizeof(netSsid[k]));
    strlcpy(netPass[k], netPass[k + 1], sizeof(netPass[k]));
  }
  netCount--;
  xSemaphoreGive(netMtx);
  saveNets();
  if (wasActive) pendingDrop = true;
  wifiServer->send(200, "application/json", "{\"ok\":true}");
}

static void handleWifiMode() {
  if (!wifiAuthWriteOk()) return;
  if (wifiServer->hasArg("standalone")) {
    bool want = wifiServer->arg("standalone") == "1";
    if (want != standalone) {
      standalone = want;
      saveNets();
      pendingMode = true; // applied from tick, after this response is out
    }
  }
  wifiServer->send(200, "application/json",
                   String("{\"standalone\":") + (standalone ? "true" : "false") + "}");
}

// Browser-supplied clock for standalone mode, where NTP can't be reached;
// only accepted while the clock is clearly unset so it never fights NTP.
static void handleWifiTime() {
  if (!wifiAuthWriteOk()) return;
  time_t epoch = (time_t)strtoull(wifiServer->arg("epoch").c_str(), NULL, 10);
  if (epoch > 1750000000 && time(NULL) < 1750000000) {
    struct timeval tv = {.tv_sec = epoch, .tv_usec = 0};
    settimeofday(&tv, NULL);
    Serial.println("[wifi] clock set from browser");
  }
  wifiServer->send(200, "application/json",
                   String("{\"time_ok\":") + (time(NULL) > 1750000000 ? "true" : "false") + "}");
}

// Device settings: static IP config, fallback-AP password, and web-login
// credentials. Each field is optional; all changes persist to NVS. The IP and
// AP-password changes take effect the next time the device joins the home
// network / raises the hotspot (changing them live would drop the link serving
// this request). Changing the web login invalidates the current session.
static void handleWifiCfg() {
  if (!wifiAuthWriteOk()) return;

  // Validate every present field FIRST, then apply - so a request that mixes a
  // valid field with an invalid one doesn't persist a partial update before the
  // 400. (The UI sends fields one at a time, but the endpoint accepts combined
  // args.) Parse into locals here; apply only once everything checks out.
  bool hasIp = wifiServer->hasArg("ipstatic");
  bool hasAp = wifiServer->hasArg("appass");
  bool hasHost = wifiServer->hasArg("host");
  bool hasCreds = wifiServer->hasArg("user") || wifiServer->hasArg("pass");
  bool authOff = wifiServer->hasArg("authoff"); // clear password -> passwordless
  bool hasVCreds = wifiServer->hasArg("vuser") || wifiServer->hasArg("vpass");
  bool ipOn = false;
  String ipA, ipM, ipG, ipD;
  String ap, host, user, pass, vuser, vpass;

  if (hasIp) {
    ipOn = wifiServer->arg("ipstatic") == "1" || wifiServer->arg("ipstatic") == "true";
    if (ipOn) {
      IPAddress t;
      ipA = wifiServer->arg("ip");  ipM = wifiServer->arg("mask");
      ipG = wifiServer->arg("gw");  ipD = wifiServer->arg("dns");
      if (!t.fromString(ipA)) { wifiServer->send(400, "text/plain", "static IP: invalid address"); return; }
      if (ipM.length() && !t.fromString(ipM)) { wifiServer->send(400, "text/plain", "static IP: invalid netmask"); return; }
      if (ipG.length() && !t.fromString(ipG)) { wifiServer->send(400, "text/plain", "static IP: invalid gateway"); return; }
      if (ipD.length() && !t.fromString(ipD)) { wifiServer->send(400, "text/plain", "static IP: invalid DNS"); return; }
    }
  }
  if (hasAp) {
    ap = wifiServer->arg("appass");
    if (ap.length() < 8 || ap.length() > 63) {
      wifiServer->send(400, "text/plain", "hotspot password must be 8..63 characters");
      return;
    }
  }
  if (hasHost) {
    // mDNS/DNS label rules: 1..32 of [a-z0-9-], no leading/trailing hyphen.
    host = wifiServer->arg("host");
    host.toLowerCase();
    bool ok = host.length() >= 1 && host.length() <= 32 &&
              host[0] != '-' && host[host.length() - 1] != '-';
    for (size_t i = 0; ok && i < host.length(); i++) {
      char c = host[i];
      if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-')) ok = false;
    }
    if (!ok) {
      wifiServer->send(400, "text/plain", "name must be 1..32 chars of a-z, 0-9, hyphen (no leading/trailing -)");
      return;
    }
  }
  if (hasCreds) {
    user = wifiServer->arg("user");
    pass = wifiServer->arg("pass");
    if (user.length() > 32 || pass.length() > 64) {
      wifiServer->send(400, "text/plain", "username <= 32, password <= 64 characters");
      return;
    }
  }
  if (hasVCreds) {
    // Read-only viewer login. Empty username disables it; otherwise a password
    // is required (an unauthenticated viewer would be meaningless).
    vuser = wifiServer->arg("vuser");
    vpass = wifiServer->arg("vpass");
    if (vuser.length() > 32 || vpass.length() > 64) {
      wifiServer->send(400, "text/plain", "viewer username <= 32, password <= 64 characters");
      return;
    }
    if (vuser.length() && !vpass.length()) {
      wifiServer->send(400, "text/plain", "read-only login needs a password (or clear the username to disable)");
      return;
    }
    if (vuser.length() && vuser == webAuthGetUser()) {
      wifiServer->send(400, "text/plain", "read-only username must differ from the admin username");
      return;
    }
  }

  // All present fields are valid: apply them.
  if (hasIp) {
    ipStatic = ipOn;
    if (ipOn) {
      strlcpy(ipAddr, ipA.c_str(), sizeof(ipAddr));
      strlcpy(ipMask, ipM.length() ? ipM.c_str() : "255.255.255.0", sizeof(ipMask));
      strlcpy(ipGw, ipG.c_str(), sizeof(ipGw));
      strlcpy(ipDns, ipD.c_str(), sizeof(ipDns));
    }
  }
  if (hasAp) strlcpy(apPass, ap.c_str(), sizeof(apPass));
  if (hasHost) strlcpy(deviceHost, host.c_str(), sizeof(deviceHost));
  if (hasIp || hasAp || hasHost) saveNets();
  if (hasHost && WiFi.status() == WL_CONNECTED) startMdns(); // re-announce live
  if (authOff) webAuthDisable();             // turn login off (passwordless)
  else if (hasCreds) webAuthSetCreds(user, pass); // empty fields leave that value unchanged
  if (hasVCreds) webAuthSetViewerCreds(vuser, vpass); // empty vuser disables the viewer

  wifiServer->send(200, "application/json", "{\"ok\":true}");
}

// Schedule a restart from the loop task once the response has gone out (doing
// ESP.restart() inside the HTTP handler can cut the reply mid-flight).
static void handleWifiReboot() {
  if (!wifiAuthWriteOk()) return;
  wifiServer->send(200, "application/json", "{\"rebooting\":true}");
  rebootAt = millis() + 600;
}

// ---------------- page ----------------

static const char WIFI_PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>)rawliteral" DEVICE_NAME R"rawliteral( - WiFi</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#14171c;color:#dde3ea;margin:0;padding:16px;max-width:760px;margin:auto}
a{color:#6fb3ff}
.card{background:#1d2229;border-radius:10px;padding:14px;margin-bottom:14px}
h3{font-size:.85em;color:#8a94a0;font-weight:500;margin:0 0 8px;text-transform:uppercase;letter-spacing:.05em}
#nav{display:flex;gap:6px;align-items:center;margin:4px 0 16px}
#nav b{margin-right:10px}
#nav a{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}
#nav a.cur{background:#1d2229;color:#dde3ea}
button{background:#3d9df0;color:#fff;border:0;border-radius:8px;padding:8px 14px;font-size:.9em;cursor:pointer}
button:disabled{opacity:.4}
button.sm{padding:4px 10px;font-size:.85em}
button.warn{background:#2a313a}
label{font-size:.9em;color:#aab4c0}
input[type=text],input[type=password]{background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:8px}
table{width:100%;border-collapse:collapse;font-size:.9em}
td,th{padding:8px 6px;text-align:left;border-bottom:1px solid #2a313a}
th{color:#8a94a0;font-weight:500;font-size:.85em}
.note{font-size:.85em;color:#8a94a0}
.ok{color:#5df0a0}.bad{color:#f05d5d}
.row{display:flex;flex-wrap:wrap;gap:10px;align-items:center}
@media(max-width:600px){
  body{padding:10px}
  .card{padding:12px;margin-bottom:10px}
  #nav{gap:4px;flex-wrap:wrap}#nav b{width:100%;margin:0 0 2px}
  input[type=text],input[type=password]{font-size:16px}
  /* Stack each settings row so every label, field and button is full-width and
     clearly visible on a narrow screen (the web-login and static-IP rows have
     several inputs that would otherwise get squeezed). */
  .row{flex-direction:column;align-items:stretch;gap:8px}
  .row>input[type=text],.row>input[type=password]{width:100%!important;min-width:0}
  .row>button{width:100%}
}
/* Read-only viewer: hide every admin card (this whole page is admin); the viewer
   just sees Status + the badge. .adm-hide toggled by body.viewer. */
body.viewer .adm-hide{display:none!important}
#vbadge{display:none;background:#2a313a;color:#7fd6a0;border-radius:8px;padding:3px 9px;font-size:.8em}
body.viewer #vbadge{display:inline-block}
</style></head><body>
<div id="nav"></div>
<div class="card"><h3>Status</h3>
  <div id="st">-</div>
  <div id="msg" class="note" style="margin-top:6px"></div>
</div>
<div class="card adm-hide"><h3>Standalone mode</h3>
  <label><input type="checkbox" id="sa" onchange="setSa()"> standalone (hotspot only, never joins WiFi)</label>
  <div class="note" style="margin-top:8px;line-height:1.6">
    The camera always hosts its own hotspot when it can't reach a saved network
    (after ~45s), so it is never unreachable. Standalone mode makes that
    permanent &mdash; useful in hotels, cars, or anywhere without usable WiFi.
    Connect your phone to <b id="apssid"></b> (password <b id="appass"></b>)
    and browse to <b>http://192.168.8.1</b>. Note: no internet on the camera
    means no NTP &mdash; the clock is set automatically from your browser when
    you open this page.
  </div>
</div>
<div class="card adm-hide"><h3>Saved networks</h3>
  <table><tbody id="nets"></tbody></table>
  <div class="note" style="margin-top:6px">Strongest saved network wins at boot. The home network keeps its static IP; everything else uses DHCP &mdash; when the IP is unknown, try the device's <b>&lt;name&gt;.local</b> (set in Device settings below).</div>
</div>
<div class="card adm-hide"><h3>Device settings</h3>
  <div class="row" style="margin-bottom:10px">
    <label style="min-width:120px">Device name (mDNS)</label>
    <input type="text" id="host" placeholder="cyclops-1" maxlength="32" style="width:160px">
    <button class="sm" onclick="saveHost()">save</button>
    <span class="note">reachable at &lt;name&gt;.local &mdash; give each device a unique name</span>
  </div>
  <div class="row" style="margin-bottom:6px">
    <label style="min-width:120px"><input type="checkbox" id="ipst" onchange="_ipDirty=true;ipFields()"> Static IP</label>
    <span class="note">off = DHCP. Applies next time it joins a network.</span>
  </div>
  <div class="row" id="iprow" style="margin-bottom:10px;display:none;flex-wrap:wrap;gap:6px">
    <input type="text" id="ipaddr" onfocus="_ipDirty=true" placeholder="address (e.g. 192.168.1.68)" style="width:190px">
    <input type="text" id="ipmask" onfocus="_ipDirty=true" placeholder="netmask" style="width:130px">
    <input type="text" id="ipgw" onfocus="_ipDirty=true" placeholder="gateway" style="width:150px">
    <input type="text" id="ipdns" onfocus="_ipDirty=true" placeholder="DNS (optional)" style="width:150px">
    <button class="sm" onclick="saveIp()">save</button>
  </div>
  <div class="row" style="margin-bottom:10px">
    <label style="min-width:120px">Hotspot password</label>
    <input type="text" id="appw" placeholder="8+ characters" style="flex:1;min-width:120px">
    <button class="sm" onclick="saveAp()">save</button>
  </div>
  <div class="row">
    <label style="min-width:120px">Web login</label>
    <input type="text" id="wu" placeholder="username" style="width:120px">
    <input type="password" id="wp" placeholder="new password" autocomplete="new-password" style="flex:1;min-width:120px">
    <button class="sm" onclick="saveLogin()">save</button>
    <button class="sm warn" id="woff" onclick="loginOff(this)" style="display:none">turn off</button>
  </div>
  <div class="note" id="wstate" style="margin-top:6px"></div>
  <div class="row" style="margin-top:10px">
    <label style="min-width:120px">Read-only login</label>
    <input type="text" id="vu" placeholder="viewer username" style="width:120px">
    <input type="password" id="vp" placeholder="password" autocomplete="new-password" style="flex:1;min-width:120px">
    <button class="sm" onclick="saveViewer()">save</button>
  </div>
  <div class="note" style="margin-top:6px">A second account that can view, watch, and listen but <b>cannot change anything</b>. Leave the username blank and save to disable it. <span id="vstate"></span></div>
  <div class="row" style="margin-top:12px;border-top:1px solid #2a313a;padding-top:12px">
    <button class="warn" id="rbbtn" onclick="reboot()">reboot device</button>
    <span class="note" id="rbnote">restarts the firmware; unreachable for ~15s</span>
  </div>
</div>
<div class="card adm-hide"><h3>Firmware</h3>
  <div class="note" id="fwver">version: &hellip;</div>
  <div class="row" style="margin-top:8px">
    <input type="file" id="fwfile" accept=".bin" style="flex:1;min-width:160px">
    <button class="warn" onclick="doUpdate(this)">upload &amp; flash</button>
  </div>
  <div class="row" id="fwprog" style="margin-top:8px;display:none">
    <progress id="fwbar" value="0" max="100" style="flex:1"></progress>
    <span id="fwpct" class="note">0%</span>
  </div>
  <div class="note" style="margin-top:6px">Upload a firmware.bin over the air; the device reboots into it (~15s). Don't close this tab mid&#8209;upload. A bad image only affects the inactive slot.</div>
</div>
<div class="card adm-hide"><h3>Join a network</h3>
  <div class="row"><button id="scanbtn" onclick="scan()">scan for networks</button></div>
  <table><tbody id="scanres"></tbody></table>
  <div class="row" style="margin-top:10px">
    <input type="text" id="mssid" placeholder="hidden network name" style="flex:1;min-width:140px">
    <input type="password" id="mpass" placeholder="password">
    <button onclick="addManual()">save &amp; join</button>
  </div>
</div>
<script>
const esc=s=>s.replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/"/g,"&quot;").replace(/'/g,"&#39;");
const jq=s=>esc(JSON.stringify(s));
// Inline feedback instead of alert()/confirm() popups, which several browsers
// suppress or auto-dismiss (that made buttons look like they did nothing). All
// status/errors go to the shared #msg line; errors are tinted red briefly.
let _msgUntil=0;
function _msg(t,err){const m=document.getElementById("msg");if(!m)return;m.textContent=t;m.style.color=err?"#f0776d":"";_msgUntil=Date.now()+8000;if(err)setTimeout(()=>{if(m.textContent===t)m.style.color="";},4000);}
// Generic two-step confirm ON THE BUTTON ITSELF (same popup-free idiom as reboot):
// first click arms it (label -> armLabel, auto-disarms after 4s), second click runs
// fn(). No dialog to be swallowed. Pass the button via `this` from onclick.
function arm(btn,armLabel,fn){
  if(!btn)return fn();
  if(btn.dataset.armed){clearTimeout(btn._armT);delete btn.dataset.armed;btn.textContent=btn.dataset.orig;fn();return;}
  btn.dataset.orig=btn.textContent;btn.dataset.armed="1";btn.textContent=armLabel;btn.classList.add("warn");
  btn._armT=setTimeout(()=>{if(btn.dataset.armed){delete btn.dataset.armed;btn.textContent=btn.dataset.orig;}},4000);
}
let _timeSent=false;
let _authon=false;
let _ipDirty=false;
async function st(){
  try{
    const s=await(await fetch("/wifi/status")).json();
    document.body.classList.toggle('viewer',!!s.viewer); // read-only: hide admin cards
    if(document.activeElement!==vu)vu.value=s.vuser||"";
    if(vstate)vstate.textContent=s.vuser?("Currently enabled for \""+s.vuser+"\"."):"Currently disabled.";
    let h="";
    h+=s.sta.up?"<span class='ok'>&#9679;</span> connected to <b>"+esc(s.sta.ssid)+"</b> &mdash; "
      +s.sta.ip+" ("+s.sta.rssi+" dBm), also <b>http://"+esc(s.host)+".local</b><br>"
      :(s.joining?"<span style='color:#f0c75d'>&#9679;</span> joining&hellip;<br>"
      :"<span class='bad'>&#9679;</span> not joined to any network<br>");
    if(s.ap.up)h+="<span class='ok'>&#9679;</span> hotspot <b>"+esc(s.ap.ssid)+"</b> up at http://"+s.ap.ip
      +" &mdash; "+s.ap.stations+" device"+(s.ap.stations==1?"":"s")+" connected";
    st_.innerHTML=h;
    // A server status (e.g. a join error) always wins; otherwise don't clobber a
    // recent client message (button feedback) with an empty poll result.
    if(s.msg)_msg(s.msg); else if(Date.now()>_msgUntil)msg.textContent="";
    if(document.activeElement!==sa)sa.checked=s.standalone;
    apssid.textContent=s.ap.ssid;appass.textContent=s.ap.pass;
    if(!_ipDirty){
      ipst.checked=!!s.ipstatic;
      ipaddr.value=s.ipaddr||"";ipmask.value=s.ipmask||"";ipgw.value=s.ipgw||"";ipdns.value=s.ipdns||"";
      iprow.style.display=ipst.checked?"":"none";
    }
    if(document.activeElement!==host)host.value=s.host;
    if(document.activeElement!==appw)appw.value=s.ap.pass;
    if(document.activeElement!==wu)wu.value=s.user;
    if(wstate){
      _authon=s.authon;
      if(s.authon){wstate.innerHTML="Login required (user <b>"+esc(s.user)+"</b>). Save a new password to change it; changing it signs you out.";if(woff)woff.style.display="";}
      else{wstate.innerHTML="<b>No password set</b> &mdash; anyone on the network has full control. Type a password and save to require login.";if(woff)woff.style.display="none";}
    }
    nets.innerHTML=s.nets.map(n=>"<tr><td>"+(n.cur?"<span class='ok'>&#9679;</span> ":"")+esc(n.ssid)
      +(n.home?" <span class='note'>(home, static IP)</span>":"")+"</td>"
      +"<td style='text-align:right'>"
      +(n.cur?"":"<button class='sm' onclick='joinSaved("+n.i+")'>connect</button> ")
      +"<button class='sm warn' onclick='forget("+n.i+",this)'>forget</button></td></tr>").join("")
      ||"<tr><td class='note'>none saved</td></tr>";
    if(!s.time_ok&&!_timeSent){_timeSent=true;fetch("/wifi/time?epoch="+Math.floor(Date.now()/1e3))}
  }catch(e){}
}
async function scan(){
  scanbtn.disabled=true;
  scanres.innerHTML="<tr><td class='note'>scanning&hellip;</td></tr>";
  try{await fetch("/wifi/scan?start=1")}catch(e){}
  const t=setInterval(async()=>{
    try{
      const s=await(await fetch("/wifi/scan")).json();
      if(s.scanning)return;
      clearInterval(t);scanbtn.disabled=false;
      scanres.innerHTML=s.nets.sort((a,b)=>b.rssi-a.rssi).map(n=>
        "<tr><td>"+esc(n.ssid)+(n.sec?" &#128274;":"")
        +(n.known?" <span class='note'>(saved)</span>":"")+"</td><td>"+n.rssi+" dBm</td>"
        +"<td style='text-align:right'><button class='sm' onclick='joinScan("+jq(n.ssid)+","+(n.sec?1:0)+")'>join</button></td></tr>")
        .join("")||"<tr><td class='note'>nothing found</td></tr>";
    }catch(e){}
  },1500);
}
function joinScan(ss,sec){
  let p="";
  if(sec){p=prompt('Password for "'+ss+'"');if(p===null)return}
  add(ss,p);
}
function addManual(){
  if(!mssid.value)return;
  add(mssid.value,mpass.value);
}
async function add(ss,pw){
  const r=await fetch("/wifi/add?join=1&ssid="+encodeURIComponent(ss)+"&pass="+encodeURIComponent(pw));
  if(!r.ok){_msg(await r.text(),true);return}
  _msg('joining "'+ss+'" - if you are on the camera’s hotspot it may drop for a few seconds');
  st();
}
async function joinSaved(i){await fetch("/wifi/join?i="+i);st()}
function forget(i,btn){
  arm(btn,"click to confirm",async()=>{await fetch("/wifi/del?i="+i);_msg("network forgotten");st();});
}
function ipFields(){iprow.style.display=ipst.checked?"":"none"}
async function saveIp(){
  const on=ipst.checked?1:0;
  let u="/wifi/cfg?ipstatic="+on;
  if(on){
    if(!ipaddr.value.trim()){_msg("Enter an IP address (or uncheck Static IP for DHCP).",true);return}
    u+="&ip="+encodeURIComponent(ipaddr.value.trim())
      +"&mask="+encodeURIComponent(ipmask.value.trim())
      +"&gw="+encodeURIComponent(ipgw.value.trim())
      +"&dns="+encodeURIComponent(ipdns.value.trim());
  }
  const r=await fetch(u);
  if(!r.ok){_msg(await r.text(),true);return}
  _ipDirty=false;
  _msg(on?("static IP saved ("+ipaddr.value.trim()+") - applies next time it joins a network")
        :"switched to DHCP - applies next time it joins a network");
}
async function saveHost(){
  const r=await fetch("/wifi/cfg?host="+encodeURIComponent(host.value.trim()));
  if(!r.ok){_msg(await r.text(),true);return}
  _msg("device name saved - now reachable at http://"+host.value.trim().toLowerCase()+".local");
}
async function saveAp(){
  const r=await fetch("/wifi/cfg?appass="+encodeURIComponent(appw.value));
  if(!r.ok){_msg(await r.text(),true);return}
  _msg("hotspot password saved - applies next time the hotspot starts");
}
async function saveLogin(){
  if(!_authon&&!wp.value){_msg("Enter a password to turn on the web login.",true);return}
  if(!wu.value&&!wp.value){_msg("Enter a username and password.",true);return}
  const r=await fetch("/wifi/cfg?user="+encodeURIComponent(wu.value)+"&pass="+encodeURIComponent(wp.value));
  if(!r.ok){_msg(await r.text(),true);return}
  wp.value="";
  _msg("login updated - you'll be asked to sign in with the new username and password");
  st();
}
async function loginOff(btn){
  arm(btn,"click again to turn off login",async()=>{
    const r=await fetch("/wifi/cfg?authoff=1");
    if(!r.ok){_msg(await r.text(),true);return}
    wp.value="";
    _msg("web login turned off - this device is now passwordless");
    st();
  });
}
async function saveViewer(){
  const u=vu.value.trim();
  if(u&&!vp.value){_msg("Set a password for the read-only login (or clear the username to disable it).",true);return}
  const r=await fetch("/wifi/cfg?vuser="+encodeURIComponent(u)+"&vpass="+encodeURIComponent(vp.value));
  if(!r.ok){_msg(await r.text(),true);return}
  vp.value="";
  _msg(u?("read-only login saved for \""+u+"\""):"read-only login disabled");
  st();
}
// Popup-free reboot: no confirm()/alert(), no overlay. The button arms on the
// first click (label changes to "click again to reboot"; auto-disarms after 4s),
// reboots on the second, then shows status inline in the row and reloads the
// moment the device answers /diag again.
let _rbArm=false;
function _rbReset(b,n){_rbArm=false;b.disabled=false;b.textContent="reboot device";n.textContent="restarts the firmware; unreachable for ~15s";}
async function reboot(){
  const b=document.getElementById("rbbtn"),n=document.getElementById("rbnote");
  if(!_rbArm){
    _rbArm=true;b.textContent="click again to reboot";n.textContent="confirm: this restarts the device (unreachable for ~15s)";
    setTimeout(()=>{if(_rbArm)_rbReset(b,n);},4000);
    return;
  }
  _rbArm=false;b.disabled=true;b.textContent="rebooting…";
  try{await fetch("/wifi/reboot")}catch(e){}
  let t=0;
  const tick=async()=>{
    t++;n.textContent="waiting for the device to come back… ("+t+"s)";
    try{const r=await fetch("/diag",{cache:"no-store"});if(r.ok){n.textContent="back online — reloading…";return location.reload();}}catch(e){}
    if(t>=45){n.textContent="not reachable — it may have a new IP; reloading…";return location.reload();}
    setTimeout(tick,1000);
  };
  setTimeout(tick,4000); // let it actually go down first, then start polling
}
async function setSa(){
  await fetch("/wifi/mode?standalone="+(sa.checked?1:0));
  _msg(sa.checked?("standalone ON - the camera now only hosts hotspot \""+apssid.textContent+"\"; reach it at http://192.168.8.1 (uncheck to rejoin WiFi)")
                 :"standalone OFF - the camera will join your saved WiFi again");
  st();
}
function doUpdate(btn){
  const f=fwfile.files[0];
  if(!f){_msg("Choose a firmware .bin first",true);return}
  // Two-step arm (popup-free) instead of confirm(): first click asks to confirm,
  // second click flashes. Guards against an accidental over-the-air reflash.
  if(btn&&!btn.dataset.armed){
    btn.dataset.orig=btn.textContent;btn.dataset.armed="1";
    btn.textContent="click again to flash "+((f.size/1024)|0)+"KB";
    btn._armT=setTimeout(()=>{if(btn.dataset.armed){delete btn.dataset.armed;btn.textContent=btn.dataset.orig;}},4000);
    return;
  }
  if(btn){clearTimeout(btn._armT);delete btn.dataset.armed;btn.textContent=btn.dataset.orig||"upload & flash";}
  const fd=new FormData();fd.append("firmware",f,f.name);
  const xhr=new XMLHttpRequest();
  xhr.open("POST","/update");
  fwprog.style.display="";
  xhr.upload.onprogress=e=>{if(e.lengthComputable){const p=Math.round(e.loaded/e.total*100);fwbar.value=p;fwpct.textContent=p+"%"}};
  xhr.onload=()=>{
    if(xhr.status==200){_msg("firmware uploaded - rebooting into it...");setTimeout(()=>location.reload(),16000)}
    else{_msg("update failed: "+(xhr.responseText||("HTTP "+xhr.status)),true);fwprog.style.display="none"}
  };
  // The device reboots the instant the write finishes, so the socket often drops
  // before a clean 200 arrives - treat that as success-in-progress, not an error.
  xhr.onerror=()=>{_msg("upload finished - if it flashed, the device is rebooting...");setTimeout(()=>location.reload(),16000)};
  xhr.send(fd);
}
function loadFw(){fetch("/diag").then(r=>r.json()).then(d=>{if(d.fw)fwver.textContent="version: "+d.fw}).catch(()=>{})}
const st_=document.getElementById("st");
st();setInterval(()=>{if(!document.hidden)st()},3000);loadFw();
document.addEventListener("visibilitychange",()=>{if(!document.hidden)st()});
</script><script src="/ui.js"></script>
<script>buildNav('/wifi')</script>
</body></html>
)rawliteral";

static void handleWifiPage() {
  if (!wifiAuthOk()) return;
  wifiServer->send_P(200, "text/html", WIFI_PAGE);
}

void wifiPortalRegisterEndpoints(WebServer &server) {
  wifiServer = &server;
  server.on("/wifi", HTTP_GET, handleWifiPage);
  server.on("/wifi/status", HTTP_GET, handleWifiStatus);
  server.on("/wifi/scan", HTTP_GET, handleWifiScan);
  server.on("/wifi/add", HTTP_GET, handleWifiAdd);
  server.on("/wifi/join", HTTP_GET, handleWifiJoin);
  server.on("/wifi/del", HTTP_GET, handleWifiDel);
  server.on("/wifi/mode", HTTP_GET, handleWifiMode);
  server.on("/wifi/time", HTTP_GET, handleWifiTime);
  server.on("/wifi/cfg", HTTP_GET, handleWifiCfg);
  server.on("/wifi/reboot", HTTP_GET, handleWifiReboot);
}

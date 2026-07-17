// wifi_portal.cpp - see wifi_portal.h

#include "wifi_portal.h"
#include "web_assets.gen.h"

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
#define STA_DOWN_LOG_PERIOD_MS 600000 // outage heartbeat into the dlog ring

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
static uint32_t dropCount = 0;      // station-link losses this boot (/diag)
static uint32_t lastDropMs = 0;     // millis() of the most recent loss
static uint32_t lastDownLogMs = 0;  // throttles the outage heartbeat
static int      lastScanNets = -1;  // latest scan result: -1 none yet, <0 scan error,
                                    // 0 radio ok but silent, >0 nets visible

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

// Close out an outage if one was in progress: one line with the total downtime.
// Every recovery path funnels through here so the marker can't stay stale.
static void staRecovered() {
  if (staLostMs)
    dlog(DLOG_INFO, "wifi recovered after %lus (drop #%lu)",
         (unsigned long)((millis() - staLostMs) / 1000), (unsigned long)dropCount);
  staLostMs = 0;
}

static void onStaUp() {
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // max TX power -> fewer retransmits
  // NTP (UTC): gives SD files real mtimes; browser renders local time
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  startMdns();
  staRecovered();
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
    dlog(DLOG_INFO, "AP \"%s\" up at %s", WIFI_AP_SSID,
         WiFi.softAPIP().toString().c_str());
  } else {
    // The AP needs nothing external, so this failing points at the radio
    // itself (on the P4: the C6 co-processor / esp-hosted link).
    dlog(DLOG_ERR, "softAP start FAILED - radio not responding?");
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
  lastScanNets = found;
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

uint32_t wifiPortalDrops() { return dropCount; }

uint32_t wifiPortalSecsSinceDrop() {
  return dropCount ? (millis() - lastDropMs) / 1000 : UINT32_MAX;
}

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
    staRecovered();
    // drop a leftover fallback AP once nobody is using it
    if (apUp && WiFi.softAPgetStationNum() == 0) stopAp();
    return;
  }

  uint32_t now = millis();
  if (staLostMs == 0) {
    staLostMs = now;
    lastDownLogMs = now;
    lastDropMs = now;
    dropCount++;
    dlog(DLOG_WARN, "wifi link lost (drop #%lu this boot) - reconnecting",
         (unsigned long)dropCount);
    WiFi.reconnect();
    return;
  }
  // Outage heartbeat: proves the retry loop is alive and shows what the radio
  // sees (scan <0 = scan itself failing, 0 = no networks, >0 = APs visible but
  // unjoinable). Mirrored to SD, so it survives the power cycle that usually
  // ends these incidents.
  if (now - lastDownLogMs >= STA_DOWN_LOG_PERIOD_MS) {
    lastDownLogMs = now;
    dlog(DLOG_WARN, "wifi still down %lumin (last scan: %d nets)",
         (unsigned long)((now - staLostMs) / 60000), lastScanNets);
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

// WEB_WIFI_HTML now lives in web/wifi.html (compiled in as WEB_WIFI_HTML via web/ codegen)

static void handleWifiPage() {
  if (!wifiAuthOk()) return;
  wifiServer->send_P(200, "text/html", WEB_WIFI_HTML);
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

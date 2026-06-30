// diag_log.cpp - see diag_log.h.

#include "diag_log.h"
#include <esp_system.h>
#include <esp_attr.h>
#include <stdarg.h>
#include <time.h>

#define DLOG_CAP       48   // ring entries (oldest overwritten)
#define DLOG_MSGLEN    80   // bytes per message, clamped
#define DLOG_MAGIC     0x43594332UL // "CYC2" - validates+versions the RTC ring
#define DLOG_CLOCK_MIN 1600000000L  // time(NULL) above this = real wall clock

struct DlogEntry {
  uint32_t seq;          // monotonic; used to track what's been mirrored to SD
  uint32_t up_s;         // uptime seconds at log time
  uint32_t epoch;        // wall-clock epoch (UTC) at log time, or 0 if unknown
  char     lvl;          // I / W / E
  char     msg[DLOG_MSGLEN];
};

// "2026-06-14 19:23:01Z" if the clock was set when this entry was logged,
// otherwise the boot-relative uptime "+1234s". Written into out[].
static void dlogStamp(const DlogEntry &e, char *out, size_t cap) {
  if (e.epoch >= (uint32_t)DLOG_CLOCK_MIN) {
    time_t t = (time_t)e.epoch;
    struct tm tm;
    gmtime_r(&t, &tm);
    strftime(out, cap, "%Y-%m-%d %H:%M:%SZ", &tm);
  } else {
    snprintf(out, cap, "+%lus", (unsigned long)e.up_s);
  }
}

// RTC_NOINIT survives watchdog/panic/software resets - the whole point: after
// the device reboots itself we can still read what happened just before.
struct DlogRing {
  uint32_t  magic;
  uint32_t  boots;       // power-on count (cold boots reset this only on garbage)
  uint32_t  seq;         // next sequence number
  uint16_t  head;        // next write slot
  uint16_t  count;       // valid entries (<= DLOG_CAP)
  uint32_t  lastReset;   // esp_reset_reason() captured at the last boot
  DlogEntry e[DLOG_CAP];
};
static RTC_NOINIT_ATTR DlogRing g_ring;

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t g_sdFlushed = 0; // RAM cursor: highest seq written to SD this boot

static const char *resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "ext-reset";
    case ESP_RST_SW:        return "sw-restart";   // ESP.restart() (reboot button, etc.)
    case ESP_RST_PANIC:     return "PANIC/crash";  // exception or abort()
    case ESP_RST_INT_WDT:   return "INT-watchdog";
    case ESP_RST_TASK_WDT:  return "TASK-watchdog"; // a task hung (the 45s WDT fired)
    case ESP_RST_WDT:       return "other-watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";      // power dip - undervolt/heat
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
  }
}

void dlogBegin() {
  esp_reset_reason_t reason = esp_reset_reason();
  bool fresh = (g_ring.magic != DLOG_MAGIC) ||
               (g_ring.head >= DLOG_CAP) || (g_ring.count > DLOG_CAP);
  if (fresh) {
    memset(&g_ring, 0, sizeof(g_ring));
    g_ring.magic = DLOG_MAGIC;
  }
  g_ring.boots++;
  g_ring.lastReset = (uint32_t)reason;
  // First line of the new session names how the previous one ended.
  dlog(reason == ESP_RST_POWERON || reason == ESP_RST_SW ? DLOG_INFO : DLOG_WARN,
       "boot #%lu, reset=%s, heap=%uK psram=%uK",
       (unsigned long)g_ring.boots, resetReasonStr(reason),
       (unsigned)(ESP.getFreeHeap() / 1024), (unsigned)(ESP.getFreePsram() / 1024));
}

void dlog(char level, const char *fmt, ...) {
  char buf[DLOG_MSGLEN];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  // A log entry is a single line. Strip control chars so attacker-influenced
  // fields (e.g. a WiFi SSID or device hostname containing a newline) can't
  // forge extra lines in /log or the /diag.log SD mirror (log injection).
  for (char *p = buf; *p; p++)
    if ((unsigned char)*p < 0x20 || (unsigned char)*p == 0x7f) *p = '.';

  Serial.printf("[log %c] %s\n", level, buf);

  time_t now = time(NULL);
  portENTER_CRITICAL(&g_mux);
  DlogEntry &e = g_ring.e[g_ring.head];
  e.seq = ++g_ring.seq;
  e.up_s = (uint32_t)(millis() / 1000);
  e.epoch = now > DLOG_CLOCK_MIN ? (uint32_t)now : 0;
  e.lvl = level;
  strlcpy(e.msg, buf, sizeof(e.msg));
  g_ring.head = (g_ring.head + 1) % DLOG_CAP;
  if (g_ring.count < DLOG_CAP) g_ring.count++;
  portEXIT_CRITICAL(&g_mux);
}

// Oldest slot index for the current ring contents.
static uint16_t tailIndex() {
  return (uint16_t)((g_ring.head + DLOG_CAP - g_ring.count) % DLOG_CAP);
}

// Append into out[], never overrunning cap. snprintf returns the would-have-
// written length, so clamp n to cap-1 after each call to keep cap-n sane.
static void appendClamped(char *out, size_t cap, size_t &n, const char *fmt, ...) {
  if (n >= cap - 1) return;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(out + n, cap - n, fmt, ap);
  va_end(ap);
  if (w < 0) return;
  n += (size_t)w;
  if (n > cap - 1) n = cap - 1; // truncated: leave room for the NUL
}

size_t dlogDump(char *out, size_t cap) {
  if (cap == 0) return 0;
  size_t n = 0;
  uint32_t up = (uint32_t)(millis() / 1000);
  time_t now = time(NULL);
  char nowStr[24] = "clock not set";
  if (now > DLOG_CLOCK_MIN) {
    struct tm tm;
    gmtime_r(&now, &tm);
    strftime(nowStr, sizeof(nowStr), "%Y-%m-%d %H:%M:%SZ", &tm);
  }
  appendClamped(out, cap, n,
                "Cyclops log - boot #%lu, last reset=%s, now=%s, uptime=%lus, %u entries\n"
                "(times are UTC; entries before the clock synced show +uptime)\n"
                "----------------------------------------\n",
                (unsigned long)g_ring.boots,
                resetReasonStr((esp_reset_reason_t)g_ring.lastReset),
                nowStr, (unsigned long)up, g_ring.count);

  portENTER_CRITICAL(&g_mux);
  uint16_t idx = tailIndex();
  uint16_t cnt = g_ring.count;
  portEXIT_CRITICAL(&g_mux);

  for (uint16_t i = 0; i < cnt && n < cap - 1; i++) {
    DlogEntry &e = g_ring.e[(idx + i) % DLOG_CAP];
    char ts[24];
    dlogStamp(e, ts, sizeof(ts));
    appendClamped(out, cap, n, "%-20s %c %s\n", ts, e.lvl, e.msg);
  }
  return n;
}

void dlogClear() {
  portENTER_CRITICAL(&g_mux);
  g_ring.count = 0;
  g_ring.head = 0;
  portEXIT_CRITICAL(&g_mux);
}

bool dlogSdPending() { return g_ring.seq > g_sdFlushed; }

void dlogFlushToFile(File &f) {
  // Write every still-resident entry newer than the SD cursor. Snapshot under
  // the lock, format/write outside it (SD I/O must not run in a critical
  // section). After a reboot g_sdFlushed starts at 0, so the pre-reboot tail
  // that survived in RTC gets mirrored once - which marks the reboot boundary.
  for (;;) {
    DlogEntry snap;
    bool have = false;
    portENTER_CRITICAL(&g_mux);
    uint16_t idx = tailIndex();
    for (uint16_t i = 0; i < g_ring.count; i++) {
      DlogEntry &e = g_ring.e[(idx + i) % DLOG_CAP];
      if (e.seq > g_sdFlushed) {
        snap = e;
        g_sdFlushed = e.seq;
        have = true;
        break;
      }
    }
    portEXIT_CRITICAL(&g_mux);
    if (!have) break;
    char ts[24];
    dlogStamp(snap, ts, sizeof(ts));
    f.printf("%-20s %c %s\n", ts, snap.lvl, snap.msg);
  }
}

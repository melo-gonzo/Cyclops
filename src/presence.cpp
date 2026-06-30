// presence.cpp - imperative shell for the connected-clients tracker.
// Pure table logic + tests live in presence.h / test/test_presence.

#include "presence.h"
#include <Arduino.h>

#define PRES_MAX        16     // max distinct clients tracked
#define PRES_WINDOW_MS  30000  // "connected" = a request within the last 30 s

static presence::Entry g_tbl[PRES_MAX];
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

void presenceTouch(uint32_t ip) {
  uint32_t now = millis();
  portENTER_CRITICAL(&g_mux);
  presence::touch(g_tbl, PRES_MAX, ip, now, PRES_WINDOW_MS);
  portEXIT_CRITICAL(&g_mux);
}

int presenceActive() {
  uint32_t now = millis();
  portENTER_CRITICAL(&g_mux);
  int c = presence::activeCount(g_tbl, PRES_MAX, now, PRES_WINDOW_MS);
  portEXIT_CRITICAL(&g_mux);
  return c;
}

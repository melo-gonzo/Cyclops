// diag_log.h
//
// Lightweight event log for diagnosing field problems (hangs, SD dropouts,
// WiFi/IP issues) without a serial cable. Two backings:
//
//   * RTC RAM ring  - a small fixed ring in RTC_NOINIT memory that SURVIVES
//     watchdog resets, panics, and software reboots (lost only on a full
//     power-cut). After the device reboots itself you can pull /log and see
//     the last events leading up to it, plus why it reset.
//   * SD mirror      - when an SD card is mounted, new entries are appended to
//     /diag.log (capped, oldest-truncated) so history survives power loss too.
//     Flushed only from the SD writer task, never an HTTP handler.
//
// dlog() is task-safe (guarded by a short critical section) and also echoes to
// Serial. Keep messages short - they are clamped to DLOG_MSGLEN.

#pragma once

#include <Arduino.h>
#include <FS.h> // for File (SD mirror)

#define DLOG_INFO 'I'
#define DLOG_WARN 'W'
#define DLOG_ERR  'E'

// Initialise the ring. Call once, very early in setup(): detects whether the
// RTC ring survived a reboot (preserve it) or is cold/garbage (clear it), then
// logs the reset reason and boot count.
void dlogBegin();

// Append a formatted line at the given level (DLOG_INFO/WARN/ERR).
void dlog(char level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// Render the whole ring (oldest->newest) plus a header (boot count, last reset
// reason, uptime) as plain text into the caller's buffer. Returns length used.
size_t dlogDump(char *out, size_t cap);

// Wipe the ring (keeps the boot counter). For /log?clear.
void dlogClear();

// True if there are entries logged since the last SD flush. The SD writer task
// polls this and calls dlogFlushToSd() when a card is available.
bool dlogSdPending();

// Append not-yet-mirrored entries to a File the caller has opened on SD (the
// caller owns the SD mutex and file lifetime). Updates the flushed cursor.
void dlogFlushToFile(File &f);

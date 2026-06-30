// metrics_svc.h
//
// Imperative shell for the diagnostics time-series. Owns the per-metric PSRAM
// rings, the sampler, and the HTTP endpoints behind the "/graphs" page:
//
//   GET /graphs          self-contained dashboard (canvas multi-line plot)
//   GET /metrics/meta    JSON: the metric table (keys, labels, colors, units)
//   GET /metrics/series  JSON: decimated history for every metric
//
// The pure encoding/metric set lives in metrics.h (unit-tested). Any module can
// feed values with the tiny set/event API below; the sampler snapshots them all
// onto one shared time base so they can be overlaid on a single normalized plot.

#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include "metrics.h"

// Allocate the PSRAM rings. Returns false if allocation fails (the rest of the
// firmware keeps running; /graphs just reports no data).
bool metricsInit();

// Register /graphs, /metrics/meta, /metrics/series. Call before server.begin().
void metricsRegisterEndpoints(WebServer &server);

// Report the latest value of a GAUGE metric (cheap; just stores into a register
// the sampler reads). Safe to call from any task.
void metricsSet(metrics::Id id, float value);

// Record one occurrence of a COUNT metric (trigger fired, SD dropped, ...).
// Accumulated per bucket, then reset by the next sample. Task-safe.
void metricsEvent(metrics::Id id);

// Snapshot one bucket of every metric onto the shared time base. The caller
// drives the cadence; call it every metricsBucketMs() so the retention window
// (set via metricsSetWindowMin) actually takes effect.
void metricsSample();

// Per-bucket duration in ms for the current retention window. The sampler's
// call cadence should track this so /graphs spans the configured window.
uint32_t metricsBucketMs();

// The current retention window in minutes, and a setter (clamped to
// [2h, 24h]) that restarts the ring and persists to NVS.
uint32_t metricsWindowMin();
void metricsSetWindowMin(uint32_t minutes);

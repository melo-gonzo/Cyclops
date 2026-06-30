// web_ui.h
//
// Shared, board-agnostic web UI assets served once and included by every page:
//   GET /ui.js   - shared components (the unified EventPlot timeline, helpers)
//   GET /caps    - board capability flags {has_audio, has_sd, has_video}
//
// This is the convergence point for the two boards' UIs: the same EventPlot is
// used by the XIAO audio dashboard (audio level + triggers) and the camera-only
// page (motion + triggers), against one data contract:
//   history(?secs&points&end[&thr=1]) -> binary Uint16Array + X-Bucket-Ms header
//   events(?secs) -> JSON [{age_ms,id,sd,rms,thr,hz,source}]

#pragma once

#include <WebServer.h>

void webUiRegisterEndpoints(WebServer &server);

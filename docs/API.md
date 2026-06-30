# Cyclops — HTTP API Reference

The device ships **passwordless** — auth only applies once a web password is set at `/wifi`.
**Auth levels:** `open` = none · `user` = any logged-in user (open when passwordless) · `admin` = write access (rejects the read-only viewer).

## Pages (HTML)

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/` | Home: audio dashboard (XIAO) or motion Event plot (camera-only) | user |
| GET | `/audio` | Audio dashboard (XIAO) | user |
| GET | `/live` | Live stream page | user |
| GET | `/graphs` | Metrics graphs page | user |
| GET | `/rec` | Continuous-record page (needs SD) | user |
| GET | `/camera` | Camera settings page | user |
| GET | `/wifi` | WiFi / network portal page | user |
| GET | `/docs` | Static settings reference | user |

## Shared web assets

One copy, served to every page (the convergence point for both boards' UIs).

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/ui.js` | Shared front-end: the unified `EventPlot` timeline component + `buildNav()` top nav | user |
| GET | `/caps` | Board capability flags `{name, has_audio, has_sd, has_video}` | user |
| GET | `/audio/player.js` | In-browser clip player | user |

## Core / Diagnostics

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/mjpeg/1` | Live MJPEG video stream | user |
| GET | `/jpg` | Single JPEG snapshot | user |
| GET | `/diag` | Diagnostics JSON | user |
| GET | `/diag.log` | Diagnostic log file | user |
| GET | `/camera/probe` | SCCB vs DVP fault isolation | user |
| GET | `/log` | Device log | admin |
| GET | `/camera/power` | Master live-stream on/off | admin |

## Audio

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/audio/stream` | Live audio stream | user |
| GET | `/audio/status` | Audio status JSON | user |
| GET | `/audio/clips` | List clips | user |
| GET | `/audio/clip` | Download one clip | user |
| GET | `/audio/clipmeta` | Clip metadata | user |
| GET | `/audio/history` | Level history | user |
| GET | `/audio/events` | Event log | user |
| GET | `/audio/spectrum` | Spectrum data | user |
| GET | `/audio/spectrogram` | Spectrogram data | user |
| GET | `/audio/trigger` | Manually trigger a clip | admin |
| GET | `/audio/config` | Update audio settings | admin |
| GET | `/audio/retune` | Re-tune noise floor | admin |
| GET | `/audio/clear` | Delete all audio clips | admin |
| GET | `/sd/list` | List SD files | user |
| GET | `/sd/file` | Download SD file | user |
| GET | `/sd/delete` | Delete SD file | admin |
| GET | `/sd/clear` | Wipe SD card | admin |
| GET | `/sd/remount` | Remount SD card | admin |

## Video clips

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/video/status` | Video status JSON | user |
| GET | `/video/list` | List video clips | user |
| GET | `/video/motion` | Live motion-detection data (overlay) | user |
| GET | `/video/motion/history` | Motion-level timeline (binary `Uint16` + `X-Bucket-Ms`, `?secs&points&end&thr=1`) — feeds the Event plot | user |
| GET | `/video/motion/events` | Motion trigger events (markers), `/audio/events` JSON shape | user |
| GET | `/video/trigger` | Manually trigger a clip | admin |
| GET | `/video/config` | Update video settings | admin |
| GET | `/video/clear` | Delete all video clips | admin |

## Continuous record

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/rec/status` | Recorder status JSON | user |
| GET | `/rec/config` | Update recorder settings | admin |
| GET | `/rec/clear` | Clear recordings | admin |

## Camera

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/camera/status` | Camera status JSON | user |
| GET | `/camera/config` | Update camera settings | admin |
| GET | `/camera/reset` | Reset camera settings | admin |

## Graphs / metrics

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/metrics/meta` | Series metadata | user |
| GET | `/metrics/series` | Time-series data | user |
| GET | `/metrics/config` | Update metrics config | admin |

## WiFi / network

| Method | Path | Purpose | Auth |
|---|---|---|---|
| GET | `/wifi/status` | Network status JSON | user |
| GET | `/wifi/scan` | Scan for networks | user |
| GET | `/wifi/add` | Add saved network | admin |
| GET | `/wifi/join` | Join a network | admin |
| GET | `/wifi/del` | Delete saved network | admin |
| GET | `/wifi/mode` | Set AP/STA / standalone mode | admin |
| GET | `/wifi/time` | Set time / NTP | admin |
| GET | `/wifi/cfg` | Update network/auth config | admin |
| GET | `/wifi/reboot` | Reboot device | admin |

## OTA

| Method | Path | Purpose | Auth |
|---|---|---|---|
| POST | `/update` | Upload firmware (OTA) | admin |

// audio_capture.cpp
// PDM mic capture + adaptive-threshold clip extraction. See audio_capture.h.
//
// Buffering model:
//   capture task --(i2s_read 16ms blocks)--> ring buffer (last ~11s, PSRAM)
//   trigger (RMS breach or HTTP) marks a position; once the post-roll has
//   been captured the [trigger - preroll, trigger + postroll] window is
//   copied out of the ring into one of CLIP_SLOTS WAV buffers in PSRAM.
//
// The adaptive threshold tracks ambient noise: the floor follows RMS with a
// slow attack / faster decay, and only learns from blocks below the trigger
// threshold so loud events don't drag the floor up.

#if defined(CAMERA_MODEL_XIAO_ESP32S3)

#include "audio_capture.h"
#include "video_record.h"
#include "web_auth.h"
#include "branding.h"
#include "diag_log.h"
#include "audio_dsp.h"  // pure trigger/threshold math (unit-tested in test/test_audio_dsp)
#include "biquad.h"     // pure low-pass IIR filter (unit-tested in test/test_biquad)
#include "fft.h"        // pure radix-2 FFT for the live spectrum (unit-tested in test/test_fft)
#include "wav.h"        // pure WAV header builder (unit-tested in test/test_wav)
#include "history.h"    // pure plot-history decimation (unit-tested in test/test_history)
#include "sd_health.h"  // pure SD mount/recovery policy (unit-tested in test/test_sd_health)
#include "path_safe.h"  // pure clip-name validation (unit-tested in test/test_path_safe)
#include "metrics_svc.h" // diagnostics time-series feed (/graphs)
#include "clip_index.h"  // in-RAM /clips index: lists + caps never scan the card
#include "ota_update.h"  // otaActive(): pause this task while firmware is flashing
#include <Preferences.h>
#include <WiFiClient.h>
#include <driver/i2s.h>
#include <math.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

// ---- SD card (XIAO Sense expansion board, SPI CS on GPIO21) ----
#define SD_CS_PIN        21
#define SD_CLIP_DIR      "/clips"
#define SD_LIST_MAX      200   // cap /sd/list entries
#define SD_STREAM_CHUNK  8192  // download read granularity: sdMutex is dropped
                               // between chunks so a slow transfer never stalls
                               // the recorders (see streamSdFileChunked). NB: a
                               // 32KB chunk was measured to give 0 throughput gain
                               // (the cap is Arduino-SD/FATFS overhead + CPU
                               // contention, not the bus), so it stays small.
// Per-chunk lock wait for downloads. Unlike the 600ms quick-endpoint timeout, a
// download has already committed a Content-Length, so it must NOT abort (and
// truncate) just because the bus is briefly busy - it waits patiently per chunk
// (still releasing between chunks so recorders interleave) and only gives up if
// the card is genuinely wedged this long. Tolerates a slow card's multi-second
// FAT ops without breaking the transfer.
#define SD_DL_LOCK_MS    12000

// Initial "open the file" lock wait for SD downloads. The quick-endpoint 600ms
// (SD_HTTP_LOCK_MS) fails fast so a wedged card can't wedge the server, but a
// clip read losing that 600ms race to a routine recorder write returned 503
// "sd busy, retry" - which the dashboard surfaced as a failed play (the warning
// sign + "click many times"). Triggered clips are written to SD almost
// constantly, so that race was common. A download is already a multi-second
// commitment, so it waits a few seconds here to ride through normal contention
// before giving up (the per-chunk SD_DL_LOCK_MS then keeps the transfer alive);
// the browser also auto-retries 503 as a backstop for longer stalls.
#define SD_DL_OPEN_LOCK_MS 3000

// ---- Hardware (XIAO ESP32S3 Sense onboard PDM mic) ----
#define AUDIO_I2S_PORT   I2S_NUM_0
#define PDM_CLK_PIN      42
#define PDM_DATA_PIN     41

// ---- Capture format ----
#define AUDIO_SAMPLE_RATE     16000  // Hz, 16-bit mono
#define SAMPLES_PER_MS        (AUDIO_SAMPLE_RATE / 1000)
#define CHUNK_SAMPLES         256    // 16ms analysis blocks

// ---- Clip sizing (runtime-tunable up to MAX_CLIP_MS via /audio/config) ----
#define MAX_CLIP_MS           10000
#define DEFAULT_CLIP_MS       5000
#define DEFAULT_PREROLL_MS    2000
#define DEFAULT_CLIP_SLOTS    4
#define MAX_CLIP_SLOTS        20      // hard cap; each slot is ~320KB of PSRAM
#define PSRAM_RESERVE_BYTES   (1024 * 1024) // never let slot growth eat the last MB
#define RING_SAMPLES          ((MAX_CLIP_MS + 1000) * SAMPLES_PER_MS)
#define MAX_CLIP_SAMPLES      (MAX_CLIP_MS * SAMPLES_PER_MS)
#define WAV_HEADER_BYTES      44
#define SLOT_BYTES            (WAV_HEADER_BYTES + MAX_CLIP_SAMPLES * 2)

// ---- Adaptive threshold ----
// Two selectable algorithms (trigAlgo): the classic floor x factor, and a
// statistical mean + k*sigma that adapts to how VARIABLE the room is (a busy room
// raises its own bar). See audio_dsp.h.
#define TRIG_ALGO_FACTOR          0
#define TRIG_ALGO_STAT            1
#define DEFAULT_TRIG_K            2.5f    // k: sigmas above the ambient mean (statistical)
#define DEFAULT_STAT_WIN_MIN      1.0f    // mean/sigma averaging window, MINUTES (tunable)
#define STAT_WIN_MIN_MIN          0.25f   // 15s floor
#define STAT_WIN_MIN_MAX          240.0f  // 4h ceiling (long ambient horizons)
#define STAT_WARMUP_WIN_MS        1500.0f  // faster window during warm-up so stats settle
#define DEFAULT_THRESHOLD_FACTOR  3.0f   // trigger when rms > floor * factor
#define MIN_TRIGGER_RMS_UNITY     500.0f // absolute minimum at unity mic gain; scaled by micGain
// The PDM mic's decimated output is very quiet (normal speech peaks around
// -27dBFS), so a digital gain is applied to every sample after DC removal.
// Trigger thresholds scale with it, keeping physical sensitivity unchanged.
#define DEFAULT_MIC_GAIN          16.0f
#define MIC_GAIN_MIN              1.0f
#define MIC_GAIN_MAX              64.0f
#define FLOOR_ALPHA_DOWN          0.02f  // adapt quickly when input is quieter
#define FLOOR_ALPHA_UP            0.005f // adapt slowly when louder (below threshold)
// Habituation: above threshold the floor leaks UP slowly so a sustained sound
// becomes the new background and stops re-triggering after ~this many minutes
// (slow-rise/fast-fall model). 0 = off (freeze the floor during loud sound).
#define DEFAULT_HABITUATE_MIN     5.0f
#define HABITUATE_MAX_MIN         120.0f
// alphaHab = update_period_ms / (minutes * 60000). Floors update per chunk
// (CHUNK_SAMPLES) and per FFT frame (FFT_N) respectively.
#define CHUNK_MS                  (1000.0f * CHUNK_SAMPLES / AUDIO_SAMPLE_RATE) // 16ms
#define FFT_FRAME_MS              (1000.0f * FFT_N / AUDIO_SAMPLE_RATE)         // 32ms
#define WARMUP_MS                 4000   // let the floor settle before auto triggers
#define RETRIGGER_HOLDOFF_MS      2000   // min gap between auto-triggered clips

// ---- Low-pass filter (rolls off broadband HF "static" on the PDM mic) ----
// Applied per-sample after DC removal + gain, so it cleans the live stream,
// the recorded clips, AND the RMS level metric in one place. Off by default;
// the cutoff is the -3dB corner. Q is Butterworth (maximally flat).
#define DEFAULT_LPF_CUTOFF_HZ     4000.0f
#define LPF_CUTOFF_MIN            200.0f
#define LPF_CUTOFF_MAX            7800.0f  // < Nyquist (8000)
#define LPF_Q                     0.70710678f
// High-pass: rolls off sub-bass rumble / HVAC / DC-thermal wander the ~13Hz
// one-pole DC blocker leaves behind. Cascaded before the low-pass.
#define DEFAULT_HPF_CUTOFF_HZ     120.0f
#define HPF_CUTOFF_MIN            30.0f
#define HPF_CUTOFF_MAX            2000.0f

// ---- Live spectrum (FFT) ----
// Computed on the RAW (pre-low-pass) signal so the plot shows where the static
// actually sits. Runs continuously while audio is enabled (not just while the
// plot is open): the always-on spectral trigger learns its per-band floors and
// detects events even when nobody is watching, and the heatmap history stays
// filled. It's ~1% of one core, cheap next to the camera/WiFi.
#define FFT_N                1024    // 15.6Hz bins over 0..8kHz; ~15.6 frames/s.
                                     // Fine enough that the low log bands
                                     // (60-190Hz) stop collapsing onto a single
                                     // 31.25Hz bin, at +8KB scratch and ~0.1% CPU.
// Band magnitudes are normalized to the 512-point scale (x 512/FFT_N): raw FFT
// magnitude grows with N for the same tone, so without this every magnitude
// consumer - learned band floors, a tuned sminmag, the dBFS ref below, heatmap
// history recorded at another N - would shift on an FFT_N change.
#define FFT_MAG_NORM         (512.0f / FFT_N)
// EMA on the DISPLAYED bars only (~0.5s, FFT runs every 64ms): the FFT produces
// ~15.6 frames/s but the plot polls ~1.5s apart, so publishing one raw frame
// strobed wildly. Smoothing the published bars makes them slow-moving without
// touching triggering (which still uses the raw per-frame magnitude).
// (0.22/frame at 15.6f/s ~= the old 0.12 at 31f/s: same wall-clock smoothing.)
#define SPECTRUM_DISPLAY_ALPHA   0.22f
// dB(magnitude) a full-scale sine produces in one bin at the normalized
// (512-point-equivalent) scale: 20*log10(32767*512/4). Subtracting it converts
// our stored "dB above magnitude 1" to ~dBFS (0=clip). FFT_MAG_NORM keeps this
// constant across FFT_N changes.
#define SPECTRO_DBFS_REF     132.4f
#define SPECTRUM_BANDS       32      // log-spaced display/analysis bands
#define SPECTRUM_FMIN        60.0f   // lowest band edge, Hz
// Per-band ("spectral") trigger: same adaptive model as the RMS path, applied
// to each band's FFT magnitude. Off by default; own factor + min-magnitude
// clamp (magnitude units, not RMS - watch the plot's threshold line to tune).
#define DEFAULT_SPECTRAL_FACTOR  4.0f
#define DEFAULT_SPECTRAL_MIN_MAG 1000.0f
// Per-band floor adaptation time (minutes): the band floor (hence the grey
// threshold line) is a per-band EMA over this window, so it tracks the ambient
// SLOWLY instead of jumping each frame. Used symmetrically for the below-
// threshold rate; above threshold, habituateMin governs the leak. Tunable.
#define DEFAULT_BAND_TC_MIN      2.0f
#define BAND_TC_MIN_MIN          0.05f   // 3s (fast, for tuning/testing)
#define BAND_TC_MAX_MIN          30.0f
// Band-trigger warm-up: when armed, learn the per-band floors (seeded from the
// running baseline) for this long before any trigger can fire, so enabling it
// doesn't unleash a burst while the floors are still settling.
#define BAND_WARMUP_MS           8000
#define BAND_BASELINE_ALPHA      0.04f   // running per-band baseline EMA (~1.6s at 15.6 frames/s)

// ---- Spectrogram history (RAM only; feeds the heatmap + rolling display) ----
// Fixed 240 columns; the window length is configurable in minutes by stretching
// each column's bucket. Memory stays 15KB for any window - longer windows just
// mean coarser time resolution (like the RMS plot's range selector). Peak per
// band per bucket.
#define SPECTRO_COLS         240
#define DEFAULT_SPECTRO_MIN  4       // 4 min -> 1s columns
#define SPECTRO_MIN_MINUTES  1       // 1 min -> 250ms columns
#define SPECTRO_MAX_MINUTES  120     // 120 min -> 30s columns
#define SPECTRO_FILE         "/clips/spectro.bin" // heatmap history survives reboots
#define SPECTRO_MAGIC        0x47433153u          // "1SCG" - distinguishes from HIST

#define TRIGGER_SOURCE_AUTO     0
#define TRIGGER_SOURCE_HTTP     1
#define TRIGGER_SOURCE_VIDEO    2  // cross-trigger from the video recorder
#define TRIGGER_SOURCE_SPECTRAL 3  // a frequency band crossed its own threshold

// ---- Amplitude history (dashboard plots) ----
// Fixed ring length (172KB PSRAM for both rings); the per-bucket duration is
// derived from a tunable retention window (histBucketMs() = window/HIST_BUCKETS),
// so memory stays constant - a longer window just means coarser buckets. This
// mirrors the spectrogram's "fixed columns, stretch the bucket" model.
#define HIST_BUCKETS         43200u            // 12h@1s .. 24h@2s .. 72h@6s, fixed
#define DEFAULT_HIST_WIN_MIN 1440u             // 24h default -> 2.0s buckets (unchanged)
#define HIST_WIN_MIN_MIN     360u              // 6h  -> 0.5s buckets (finest)
#define HIST_WIN_MAX_MIN     4320u             // 72h -> 6.0s buckets (coarsest)
#define HIST_SETTLE_MS   5000  // PDM mic spikes hard at boot; keep it out of the plot
#define HIST_FILE        "/clips/history.bin" // plot history survives reboots via SD
#define HIST_SAVE_MS     300000UL              // snapshot every 5 min (~170KB write)
#define HIST_MAGIC       0x48324743u           // "CG2H" (bumped: header now carries window)

// ---- Live audio streaming ----
#define MAX_AUDIO_STREAM_CLIENTS  4
// Send in 64ms blocks: fewer, larger TCP packets ride WiFi jitter much
// better than per-capture-chunk (16ms) writes.
#define STREAM_CHUNK_SAMPLES      1024

// Dashboard page: lists clips with inline playback + download, shows live
// mic levels, manual trigger, auto-trigger toggle. Served at / and /audio.
static const char AUDIO_PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>)rawliteral" DEVICE_NAME R"rawliteral( - Audio Clips</title>
<style>
body{font-family:-apple-system,system-ui,sans-serif;background:#14171c;color:#dde3ea;margin:0;padding:16px;max-width:760px;margin:auto}
h1{font-size:1.2em;margin:8px 0 16px}
a{color:#6fb3ff}
.card{background:#1d2229;border-radius:10px;padding:14px;margin-bottom:14px}
.row{display:flex;gap:16px;flex-wrap:wrap;align-items:center}
.stat b{display:block;font-size:1.15em}
.stat span{font-size:.75em;color:#8a94a0}
#meterwrap{flex:1;min-width:160px;height:14px;background:#0d0f12;border-radius:7px;overflow:hidden;position:relative}
#meter{height:100%;width:0;background:#3d9df0;transition:width .25s}
#meter.hot{background:#f05d5d}
#thr{position:absolute;top:0;bottom:0;width:2px;background:#ffd24d}
button{background:#3d9df0;color:#fff;border:0;border-radius:8px;padding:10px 16px;font-size:1em;cursor:pointer}
button:disabled{opacity:.5}
label{font-size:.9em;color:#aab4c0}
.tblwrap{overflow-x:auto;-webkit-overflow-scrolling:touch}
table{width:100%;border-collapse:collapse;font-size:.9em}
th,td{text-align:left;padding:8px 6px;border-bottom:1px solid #2a313a;white-space:nowrap}
th{color:#8a94a0;font-weight:500;font-size:.8em}
.play{width:38px;height:32px;padding:0;font-size:.9em}
.spin{display:inline-block;width:13px;height:13px;border:2px solid rgba(255,255,255,.3);border-top-color:#fff;border-radius:50%;animation:sp .7s linear infinite;vertical-align:-2px}
@keyframes sp{to{transform:rotate(360deg)}}
.dl{padding:4px 10px;font-size:.85em;background:#2a313a;border-radius:6px;text-decoration:none}
.src{font-size:.75em;padding:2px 8px;border-radius:10px}
.auto{background:#3a2f1d;color:#ffd24d}.http{background:#1d3a2a;color:#5df0a0}
.motion,.video{background:#2d1d3a;color:#c08bff}
.spectral{background:#3a1d22;color:#f0768a}
.thumb{width:64px;height:48px;object-fit:cover;border-radius:4px;cursor:pointer;background:#0d0f12;vertical-align:middle;display:block}
.thumb.big{width:280px;height:auto;position:absolute;z-index:20;box-shadow:0 6px 24px rgba(0,0,0,.6)}
.pnav{display:flex;align-items:center;gap:8px;margin-top:6px;font-size:.8em;color:#8a94a0}
.pnav>span:first-child{width:38px}
.pnav input[type=range]{flex:1;accent-color:#3d9df0;height:22px}
#plotinfo{margin-top:6px;font-size:.85em;color:#aab4c0;min-height:1.5em;display:flex;align-items:center;gap:8px;flex-wrap:wrap}
#empty,#sdempty,#vidempty{color:#8a94a0;padding:12px 0}
.pgnav{display:none;gap:10px;align-items:center;justify-content:flex-end;margin-top:8px;font-size:.85em;color:#8a94a0}
.pgnav button{padding:4px 12px;font-size:1em;background:#2a313a}
h3{display:flex;align-items:center;gap:8px;flex-wrap:wrap;font-size:.85em;color:#8a94a0;font-weight:500;margin:0 0 8px;text-transform:uppercase;letter-spacing:.05em}
h3>.dl,h3>span{margin-left:auto}
#nav{display:flex;gap:6px;align-items:center;margin:4px 0 16px}
#nav b{margin-right:10px}
#nav a{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}
#nav a.cur{background:#1d2229;color:#dde3ea}
.rng,.tog{color:#8a94a0;text-decoration:none;padding:2px 8px;border-radius:6px;font-size:.85em}
.rng.cur,.tog.cur{background:#2a313a;color:#dde3ea}
canvas{width:100%;display:block}
@media(max-width:600px){
  body{padding:10px}
  .card{padding:12px;margin-bottom:10px}
  .row{gap:8px 10px}
  #nav{gap:4px;flex-wrap:wrap}#nav b{width:100%;margin:0 0 2px}
  input,select{font-size:16px}
}
/* Read-only viewer: disable every control except those marked .vw (view-only:
   listen, clip play, plot zoom/scale). Fail-safe - anything unmarked is locked.
   .adm-hide removes admin-only sections entirely. Toggled by body.viewer. */
body.viewer input:not(.vw),body.viewer select:not(.vw),body.viewer button:not(.vw),body.viewer textarea:not(.vw){pointer-events:none;opacity:.4}
body.viewer .adm-hide{display:none!important}
#vbadge{display:none;background:#2a313a;color:#7fd6a0;border-radius:8px;padding:3px 9px;font-size:.8em}
body.viewer #vbadge{display:inline-block}
</style></head><body>
<div id="nav"></div>
<div class="card"><h3>Diagnostics</h3><div class="row" id="diag">-</div></div>
<div class="card"><div class="row">
  <div class="stat"><b id="rms">-</b><span>RMS</span></div>
  <div class="stat"><b id="floor">-</b><span>noise floor</span></div>
  <div class="stat"><b id="threshold">-</b><span id="thrmode">threshold</span></div>
  <div id="meterwrap"><div id="meter"></div><div id="thr"></div></div>
  <button id="listen" class="vw" onclick="toggleListen()" style="font-size:.85em;padding:6px 12px">&#128264; listen live</button>
  <span id="lvwitness" style="display:none;font-size:.8em;color:#7fd6a0">&#9654; playing <span id="lvbar" style="display:inline-block;width:64px;height:9px;background:#0d0f12;border-radius:4px;overflow:hidden;vertical-align:middle"><span id="lvfill" style="display:block;height:100%;width:0;background:#7fd6a0"></span></span></span>
  <button onclick="retune()" style="font-size:.85em;padding:6px 12px;background:#2a313a" title="forget the learned noise floor and re-learn from current ambient">re-tune</button>
  <label>manual thr <input type="number" id="mthrIn" min="0" max="32767" title="trigger threshold override; 0 = adaptive" style="width:70px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <button onclick="applyMthr()" style="font-size:.85em;padding:6px 12px">apply</button>
  <label>min thr <input type="number" id="minthrIn" min="0" max="32767" title="floor under the adaptive threshold so quiet rooms aren't hair-trigger; 0 = pure adaptive (floor x factor)" style="width:70px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <button onclick="applyMinThr()" style="font-size:.85em;padding:6px 12px">apply</button>
  <label>auto algo <select id="algoIn" onchange="applyAlgo()" title="how the adaptive threshold is computed: floor x factor (level only) or mean + k&middot;&sigma; (adapts to how VARIABLE the room is - a busy room auto-raises its own bar)" style="background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"><option value="0">floor &times; factor</option><option value="1">mean + k&middot;&sigma;</option></select></label>
  <span id="facWrap"><label>factor &times;<input type="number" id="factorIn" min="1.2" max="20" step="0.1" title="adaptive threshold = noise floor x this factor; HIGHER = less sensitive" style="width:60px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <button onclick="applyFactor()" style="font-size:.85em;padding:6px 12px">apply</button></span>
  <span id="kWrap" style="display:none"><label>k&times;<input type="number" id="kIn" min="0.5" max="12" step="0.5" title="k = how many standard deviations (sigma) above the average level to set the threshold. threshold = mean + k&middot;sigma. HIGHER k = FEWER triggers. sigma is MEASURED from the room (you can't set it); k is the knob you turn." style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <label>win min<input type="number" id="statWinIn" min="0.25" max="240" step="0.25" title="averaging window (minutes) the mean &amp; sigma are computed over. SHORTER = reacts faster to the room changing; LONGER = steadier, ignores brief events, good for long ambient horizons. Exponential, so this is the time-constant." style="width:60px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <button onclick="applyK()" style="font-size:.85em;padding:6px 12px">apply</button>
  <span id="statLive" style="font-size:.78em;color:#8a94a0"></span></span>
  <label>mic gain &times;<input type="number" id="gainIn" min="1" max="64" step="1" title="digital gain on the PDM mic; the raw output is very quiet. Trigger thresholds scale with it." style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <button onclick="applyGain()" style="font-size:.85em;padding:6px 12px">apply</button>
  <label title="high-pass filter: rolls off sub-bass rumble / HVAC / DC drift on the live stream, recorded clips, and the level metric"><input type="checkbox" id="hpfEn" onchange="applyHpf()"> high-pass</label>
  <input type="range" id="hpfHz" min="30" max="2000" step="10" value="120" oninput="hpfDrag()" onchange="applyHpf()" title="cutoff frequency; drag while listening to audition the effect locally before applying" style="vertical-align:middle;width:120px">
  <input type="number" id="hpfHzv" min="30" max="2000" step="10" value="120" onchange="hpfBox()" title="cutoff frequency in Hz (30-2000); type a value or use the slider" style="width:62px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:5px"><span class="note">Hz</span>
  <label title="low-pass filter: rolls off high-frequency hiss/static on the live stream, recorded clips, and the level metric"><input type="checkbox" id="lpfEn" onchange="applyLpf()"> low-pass</label>
  <input type="range" id="lpfHz" min="200" max="7800" step="50" value="4000" oninput="lpfDrag()" onchange="applyLpf()" title="cutoff frequency; drag while listening to audition the effect locally before applying" style="vertical-align:middle;width:120px">
  <input type="number" id="lpfHzv" min="200" max="7800" step="50" value="4000" onchange="lpfBox()" title="cutoff frequency in Hz (200-7800); type a value or use the slider" style="width:62px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:5px"><span class="note">Hz</span>
</div></div>
<div class="card">
  <h3>Event plot
    <span style="float:right;font-weight:400">
      <select id="histwin" onchange="setHistWin(this.value)" title="device retention: how far back level/threshold history is kept. Sets the per-bucket resolution at fixed memory; existing history is re-bucketed to the new resolution."
        style="background:#222a33;color:#9aa4b0;border:1px solid #2a313a;border-radius:6px;font-size:.85em;padding:1px 4px">
        <option value="360">keep 6h</option>
        <option value="720">keep 12h</option>
        <option value="1440">keep 24h</option>
        <option value="2880">keep 48h</option>
        <option value="4320">keep 72h</option>
      </select>
    </span></h3>
  <div id="ep"></div>
</div>
<div class="card">
  <h3>Live spectrum
    <span style="float:right;font-weight:400;font-size:.85em">
      <a href="#" id="scaletog" class="tog" title="amplitude axis: 'dB rel' = dB below the loudest current band (0 at top) · 'dBFS' = true level in dBFS, auto-scaled so the loudest band is at top · 'linear' = raw amplitude vs peak. Display only - triggering always uses raw magnitude.">dB rel</a>
      <a href="#" id="filttog" class="tog" title="Off: raw mic signal BEFORE the low-pass (where the noise really is). On: overlay the low-pass effect, i.e. what the stream &amp; clips actually contain.">show filtered</a>
    </span></h3>
  <canvas id="spec" height="150" style="cursor:crosshair"></canvas>
  <div id="spectip" style="position:fixed;display:none;pointer-events:none;z-index:20;background:#0d0f12;border:1px solid #2a313a;border-radius:6px;padding:6px 9px;font-size:.78em;line-height:1.45;color:#dde3ea;white-space:nowrap;box-shadow:0 2px 8px #000a"></div>
  <div id="specpin" style="display:none;font-size:.8em;color:#cdd6df;margin-top:4px"></div>
  <canvas id="spech" height="120" title="spectrogram history: time runs left→right, frequency bottom→top, brighter = louder" style="margin-top:6px"></canvas>
  <div id="specnote" style="font-size:.8em;color:#8a94a0;margin-top:4px"></div>
  <div class="row" style="margin-top:8px">
    <label title="record a clip when any frequency band rises above its own adaptive threshold (the grey step line) - catches tonal events the overall RMS trigger misses"><input type="checkbox" id="strig" onchange="applySpec()"> band trigger</label>
    <label>sensitivity &times;<input type="number" id="sfac" min="1.5" max="100" step="0.5" onchange="applySpec()" title="per-band threshold = that band's learned floor x this factor; HIGHER = less sensitive (needs a bigger jump over the floor to fire)" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
    <label title="advanced: absolute floor (FFT magnitude units, not RMS) a band's threshold can never drop below, so a dead-quiet band can't become a hair-trigger. Leave at the default unless tuning against the plot.">min mag <input type="number" id="sminmag" min="0" max="10000000" step="500" onchange="applySpec()" style="width:80px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
    <label title="how slowly each band's threshold line (grey) adapts to the ambient: a per-band EMA over this many minutes. Larger = slower/steadier line; smaller = quicker to follow level changes.">band avg <input type="number" id="bandtc" min="0.05" max="30" step="0.25" onchange="applySpec()" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"> min</label>
    <label title="habituation: a SUSTAINED sound is slowly learned as the new background and stops re-triggering after roughly this many minutes (slow-rise noise floor). Applies to BOTH the band and overall-level triggers. 0 = off: steady sounds keep firing until you Retune.">habituate <input type="number" id="habmin" min="0" max="120" step="0.5" onchange="applySpec()" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"> min</label>
    <label title="how much time the spectrogram heatmap spans; longer = coarser per-column. Changing it restarts the heatmap.">history <input type="number" id="shist" min="1" max="120" step="1" onchange="applySpec()" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"> min</label>
    <button onclick="applySpec()" style="font-size:.85em;padding:6px 12px">apply</button>
  </div>
</div>
<div class="card"><div class="row">
  <label><input type="checkbox" id="auden" onchange="setAudEn()" title="master switch: mic capture, all audio triggers and live listening; video keeps running"> enable audio</label>
  <button id="rec" onclick="trigger()">&#9210; Record clip now</button>
  <label><input type="checkbox" id="auto" onchange="setAuto()"> auto-trigger on noise</label>
  <label>clip <input type="number" id="clipIn" min="0.5" max="10" step="0.5" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px">s</label>
  <label>pre-roll <input type="number" id="preIn" min="0" max="9.5" step="0.5" title="seconds kept from before the trigger" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px">s</label>
  <button onclick="applyLen()" style="font-size:.85em;padding:6px 12px">apply</button>
  <span id="recState" style="color:#f05d5d"></span>
</div></div>
<div class="card"><div class="row">
  <label>clip slots <input type="number" id="slotsIn" min="1" style="width:60px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <button onclick="applySlots()">apply</button>
  <span id="mem" style="font-size:.85em;color:#8a94a0"></span>
</div>
<div class="row" style="margin-top:10px">
  <label><input type="checkbox" id="sdsave" onchange="setSd()"> save clips to SD card</label>
  <label>max SD clips <input type="number" id="sdmaxIn" min="0" title="0 = unlimited" style="width:70px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
  <button onclick="applySdMax()">apply</button>
  <span id="sdinfo" style="font-size:.85em;color:#8a94a0"></span>
  <button id="sdretry" onclick="remountSd()" style="display:none;font-size:.85em;padding:6px 12px">retry SD</button>
</div></div>
<div class="card">
  <h3>Recent clips (RAM &mdash; cleared on reboot)
    <a class="dl" href="#" onclick="clearRam();return false" style="float:right;color:#f05d5d">clear all</a></h3>
  <div class="tblwrap"><table><thead><tr><th>ID</th><th>Captured</th><th>Trigger</th><th>Over thr</th><th>Listen</th><th></th></tr></thead>
  <tbody id="clips"></tbody></table></div>
  <div id="clipsnav" class="pgnav"></div>
  <div id="empty">No clips yet.</div>
</div>
<div class="card" id="sdcard" style="display:none">
  <h3>SD card clips
    <a class="dl" href="#" onclick="clearSd();return false" style="float:right;color:#f05d5d">clear all</a></h3>
  <div class="tblwrap"><table><thead><tr><th>Key</th><th>File</th><th>Time</th><th>Trigger</th><th>Over thr</th><th>Size</th><th>Listen</th><th></th><th></th></tr></thead>
  <tbody id="sdclips"></tbody></table></div>
  <div id="sdnav" class="pgnav"></div>
  <div id="sdempty">No files on SD.</div>
</div>
<div class="card">
  <h3>Video clips
    <a class="dl" href="#" onclick="clearVid();return false" style="float:right;color:#f05d5d">clear all</a></h3>
  <div class="row" style="margin-bottom:8px">
    <label><input type="checkbox" id="viden" onchange="setVidEn()"> enable video recording</label>
    <label title="Master switch: off = audio-only, camera stays parked (cooler)"><input type="checkbox" id="strmen" onchange="setStrmEn()"> live stream</label>
    <button id="vrec" onclick="vidTrigger()" style="font-size:.85em;padding:6px 12px">&#9210; record now</button>
    <span id="vidState" style="color:#f05d5d"></span>
    <span id="vidinfo" style="font-size:.85em;color:#8a94a0"></span>
  </div>
  <div class="row" style="margin-bottom:8px">
    <label>clip <input type="number" id="vclipIn" min="2" max="60" step="1" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px">s</label>
    <label>pre-roll <input type="number" id="vpreIn" min="0" max="8" step="0.5" title="seconds kept from before the trigger" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px">s</label>
    <label>fps <input type="number" id="vfpsIn" min="2" max="10" style="width:50px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
    <label>max files <input type="number" id="vmaxIn" min="0" title="0 = unlimited" style="width:60px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
    <button onclick="applyVidCfg()" style="font-size:.85em;padding:6px 12px">apply</button>
  </div>
  <div class="row" style="margin-bottom:8px">
    <label><input type="checkbox" id="vonaud" onchange="fetch('/video/config?on_audio='+(vonaud.checked?1:0)).catch(()=>{})"> video on audio trigger</label>
    <label><input type="checkbox" id="vtrigaud" onchange="fetch('/video/config?trig_audio='+(vtrigaud.checked?1:0)).catch(()=>{})"> audio on video trigger</label>
  </div>
  <div class="row" style="margin-bottom:8px">
    <label title="Minimum gap after an automatic clip before motion/audio can trigger another. 0 = no limit. Caps how many clips record per period; manual 'record now' always works."> min gap <input type="number" id="vgapIn" min="0" max="3600" step="1" style="width:60px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px">s</label>
    <label title="Separate cap on how often an audio trigger may also start a video clip (0 = off)">audio&rarr;video cooldown <input type="number" id="vxtrigIn" min="0" max="3600" step="1" style="width:60px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px">s</label>
    <button onclick="applyRateCfg()" style="font-size:.85em;padding:6px 12px">apply</button>
  </div>
  <div class="row" style="margin-bottom:8px">
    <label title="Auto-pause video recording when the die runs too hot (protects the over-temp PSRAM). Leave on unless you have active cooling."><input type="checkbox" id="vthen" onchange="setThEn()"> thermal throttle</label>
    <label>cutoff <input type="number" id="vthIn" min="60" max="125" step="1" title="pause video above this die temp (&deg;C); resumes 8&deg;C below" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px">&deg;C</label>
    <button onclick="applyThCfg()" style="font-size:.85em;padding:6px 12px">apply</button>
    <span id="thState" style="font-size:.85em;color:#ffb84d"></span>
  </div>
  <div class="row" style="margin-bottom:8px">
    <label><input type="checkbox" id="vmot" onchange="setVidMot()"> on-device motion detect</label>
    <label>blocks <input type="number" id="vmotIn" min="1" max="3072" title="changed blocks needed to trigger; lower = more sensitive" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
    <label>pixel diff <input type="number" id="vdiffIn" min="3" max="60" title="avg luma delta for a block to count as changed; lower = more sensitive" style="width:55px;background:#0d0f12;color:#dde3ea;border:1px solid #2a313a;border-radius:6px;padding:6px"></label>
    <button onclick="applyVidMot()" style="font-size:.85em;padding:6px 12px">apply</button>
    <span id="motlvl" style="font-size:.85em;color:#8a94a0"></span>
  </div>
  <div class="tblwrap"><table><thead><tr><th>Key</th><th>File</th><th>Time</th><th>Trigger</th><th>Size</th><th>Watch</th><th>Audio</th><th></th><th></th></tr></thead>
  <tbody id="vidclips"></tbody></table></div>
  <div id="vidnav" class="pgnav"></div>
  <div id="vidempty">No video clips.</div>
</div>
<div id="vplayer" style="display:none;position:fixed;inset:0;background:rgba(0,0,0,.85);z-index:10;padding:16px;box-sizing:border-box" onclick="if(event.target===this)closeVid()">
  <div style="max-width:900px;margin:24px auto;background:#1d2229;border-radius:10px;padding:14px">
    <canvas id="vcv"></canvas>
    <div class="row" style="margin-top:10px">
      <button id="vpp" onclick="vidPlayPause()" style="width:44px">&#9208;</button>
      <input type="range" id="vseek" min="0" value="0" style="flex:1;min-width:120px">
      <span id="vfr" style="font-size:.85em;color:#8a94a0"></span>
      <button id="vaud" onclick="vidAudioToggle()" style="display:none;background:#2a313a" title="play the paired audio clip">&#128264;</button>
      <button onclick="closeVid()" style="background:#2a313a">close</button>
    </div>
  </div>
</div>
<script src="/ui.js"></script>
<script>
// Escape device strings before they hit innerHTML (firmware-generated, but a stray quote must not corrupt a row).
function esc(s){return String(s).replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
function age(ms){if(ms<60000)return Math.round(ms/1000)+"s ago";if(ms<3600000)return Math.round(ms/60000)+"m ago";return (ms/3600000).toFixed(1)+"h ago"}
const _p2=n=>String(n).padStart(2,"0");
function fmtTs(t){const d=new Date(t);
  return d.getFullYear()+"-"+_p2(d.getMonth()+1)+"-"+_p2(d.getDate())+" "+_p2(d.getHours())+":"+_p2(d.getMinutes())+":"+_p2(d.getSeconds())}
function fmtHM(t){const d=new Date(t);return _p2(d.getHours())+":"+_p2(d.getMinutes())}
let _sdCount=-1,_wasRec=false,_sdAudio=[];
// Client-side pagination: 10 rows per table, page state kept across polls.
const PG=10;let _pg={},_pgD={};
function page(k,items,render,tb,nav,emptyEl){
  _pgD[k]=[items,render,tb,nav,emptyEl];
  const n=items.length,maxP=Math.max(0,Math.ceil(n/PG)-1);
  if(!(k in _pg))_pg[k]=0;
  if(_pg[k]>maxP)_pg[k]=maxP;
  const p=_pg[k],s=p*PG;
  emptyEl.style.display=n?"none":"block";
  // Only touch the DOM when the rendered rows actually changed: assigning even an
  // identical innerHTML recreates every element, which refetches thumbnails and
  // restarts their lazy-load (visible as flicker on each 10s refresh).
  const html=items.slice(s,s+PG).map(render).join("");
  if(tb._h!==html){tb._h=html;tb.innerHTML=html}
  nav.style.display=n>PG?"flex":"none";
  nav.innerHTML=n>PG?
    "<button onclick=\"pgMove('"+k+"',-1)\""+(p?"":" disabled")+">&#8249; prev</button>"+
    "<span>"+(s+1)+"&ndash;"+Math.min(s+PG,n)+" of "+n+"</span>"+
    "<button onclick=\"pgMove('"+k+"',1)\""+(p<maxP?"":" disabled")+">next &#8250;</button>":"";
}
function pgMove(k,d){_pg[k]+=d;page(k,..._pgD[k])}
async function applySdMax(){
  await fetch("/audio/config?sd_max="+sdmaxIn.value);
  refreshSd();refreshStatus();
}
// The status poll runs every 2s and writes the config inputs from the device.
// Don't clobber a control the user is actively setting: hold a field for ~12s
// after its last edit (the focus check alone isn't enough - on mobile the
// keyboard-dismiss blurs the field before you reach 'apply', and the next poll
// would snap it back). The apply functions read the field, so once the device
// value matches, the hold expiring is a no-op.
var _lastEdit={};
document.addEventListener('input',e=>{if(e.target&&e.target.id)_lastEdit[e.target.id]=Date.now()},true);
function _hold(el){return !el||document.activeElement===el||(_lastEdit[el.id]&&Date.now()-_lastEdit[el.id]<12000);}
async function refreshStatus(){
  try{
    const s=await(await fetch("/audio/status")).json();
    document.body.classList.toggle('viewer',!!s.viewer); // read-only: lock controls
    rms.textContent=Math.round(s.rms);
    floor.textContent=Math.round(s.noise_floor);
    threshold.textContent=Math.round(s.threshold);
    thrmode.textContent=s.manual_thr>0?"threshold (manual)":"threshold (auto)";
    if(!_hold(mthrIn))mthrIn.value=Math.round(s.manual_thr);
    if(!_hold(factorIn))factorIn.value=s.factor;
    if(s.algo!=null){if(!_hold(algoIn))algoIn.value=s.algo;
      if(!_hold(kIn)&&s.trig_k!=null)kIn.value=s.trig_k;
      if(!_hold(statWinIn)&&s.stat_win!=null)statWinIn.value=s.stat_win;
      facWrap.style.display=s.algo==1?"none":"";kWrap.style.display=s.algo==1?"":"none";
      if(s.algo==1&&s.sigma!=null){
        // Show the MEASURED sigma and the raw mean+k*sigma so it's obvious when the
        // min-thr floor is the thing actually setting the threshold (k looks "dead").
        const raw=s.noise_floor+s.trig_k*s.sigma;
        statLive.textContent="σ="+s.sigma.toFixed(0)+" · mean+k·σ="+Math.round(raw)+
          (raw<s.min_thr?" → clamped to min thr "+Math.round(s.min_thr):"");
      }}
    if(!_hold(gainIn))gainIn.value=Math.round(s.gain);
    if(!_hold(minthrIn))minthrIn.value=Math.round(s.min_thr);
    if(!_hold(lpfEn))lpfEn.checked=s.lpf;
    if(!_hold(lpfHz)&&!_hold(lpfHzv)){lpfHz.value=Math.round(s.lpf_hz);lpfHzv.value=Math.round(s.lpf_hz);}
    if(!_hold(hpfEn))hpfEn.checked=s.hpf;
    if(!_hold(hpfHz)&&!_hold(hpfHzv)){hpfHz.value=Math.round(s.hpf_hz);hpfHzv.value=Math.round(s.hpf_hz);}
    if(!_hold(clipIn))clipIn.value=s.clip_ms/1000;
    if(!_hold(preIn))preIn.value=s.preroll_ms/1000;
    auto.checked=s.auto;
    auden.checked=s.enabled;
    rec.disabled=!s.enabled;
    auto.disabled=!s.enabled;
    listen.disabled=!s.enabled&&!_ac;
    recState.textContent=s.recording?"● recording":s.enabled?"":"audio off";
    if(_wasRec&&!s.recording)setTimeout(()=>{refreshClips();refreshSd();window.ep&&ep.reload()},1500);
    _wasRec=s.recording;
    const span=Math.max(s.threshold*2,1);
    meter.style.width=Math.min(100,s.rms/span*100)+"%";
    meter.className=s.rms>s.threshold?"hot":"";
    thr.style.left=Math.min(100,s.threshold/span*100)+"%";
    slotsIn.max=s.max_slots;
    if(!_hold(slotsIn))slotsIn.value=s.slots;
    const mb=(s.slot_bytes/1048576).toFixed(2);
    mem.textContent="recommended max "+s.recommended_slots+" ("+mb+"MB each, "
      +(s.free_psram/1048576).toFixed(1)+"MB PSRAM free)";
    sdsave.checked=s.save_to_sd;
    sdsave.disabled=!s.sd;
    if(!_hold(sdmaxIn))sdmaxIn.value=s.sd_max_clips;
    sdinfo.textContent=s.sd?("SD: "+s.sd_used_mb+" / "+s.sd_total_mb+" MB used"
      +(_sdCount>=0?" · "+_sdCount+" clips":"")
      +(s.sd_max_clips>0?" (cap "+s.sd_max_clips+")":" (no cap)"))
      :"no SD card detected — auto-retrying…";
    sdcard.style.display=s.sd?"block":"none";
    sdretry.style.display=s.sd?"none":"inline-block";
  }catch(e){}
}
async function remountSd(){
  sdretry.disabled=true;
  try{await fetch("/sd/remount")}catch(e){}
  sdretry.disabled=false;
  refreshStatus();refreshSd();
}
// "1.9x" over the trigger threshold, with the raw post-gain RMS/threshold.
function ovr(rms,thr){
  if(rms==null||!thr||thr<=0)return "<span class='note'>&mdash;</span>";
  rms=+rms;thr=+thr; // coerce numeric before interpolating
  return "<b>"+(rms/thr).toFixed(1)+"&times;</b> <span class='note'>"+rms+"/"+thr+"</span>";
}
// Per-clip key-frame thumbnail cell. The .jpg sidecar shares the clip's name;
// lazy-loaded so a whole table doesn't fetch at once, and clicking toggles an
// enlarged overlay. A load error retries a few times with backoff (so a transient
// "sd busy" 503 doesn't blank it) before giving up - a genuinely missing sidecar
// (older clips, or the camera had no frame) then just removes itself.
const _noThumb=new Set(); // clips confirmed to have no .jpg sidecar (pre-feature
                          // files): never re-request them, or every table rebuild
                          // would restart the fetch-fail-retry cycle (UI flicker).
function thumbCell(name){
  const j=encodeURIComponent(name.replace(/\.(wav|avi)$/,".jpg"));
  if(_noThumb.has(j))return "";
  return "<img class=thumb loading=lazy alt=key data-n='"+j+"' src='/sd/file?name="+j+
    "' onerror='thumbErr(this)' onclick='this.classList.toggle(\"big\")'>";
}
function thumbErr(img){
  const t=(+img.dataset.t||0)+1;
  if(t>3){_noThumb.add(img.dataset.n);img.remove();return} // no sidecar: give up for good
  img.dataset.t=t;
  setTimeout(()=>{img.src="/sd/file?name="+img.dataset.n+"&r="+Date.now()},500*t);
}
async function refreshSd(){
  if(window._clipAudio)return; // don't rebuild rows mid-playback
  try{
    // Over-threshold data per clip number, from the SD-persisted store (survives
    // reboots, covers every saved clip) - {"<num>":[rms,thr,hz]}. Replaces the old
    // RAM-event-log source, which was wiped on reboot (only newest clip showed).
    try{window._sdMeta=await(await fetch("/audio/clipmeta")).json();}catch(e){window._sdMeta=window._sdMeta||{}}
    const r=await fetch("/sd/list");
    if(!r.ok)return;
    const c=await r.json();
    _sdCount=c.length;
    _sdAudio=c; // kept for pairing audio with video clips by mtime
    c.sort((a,b)=>b.name.localeCompare(a.name));
    page("sd",c,x=>{
      const m=x.name.match(/_(a|h|v|s)\.wav$/);
      const src=m?{a:"auto",h:"http",v:"video",s:"spectral"}[m[1]]:null;
      // Over-threshold [rms,thr,hz] keyed by SD file number, from clipmeta (above).
      // Clips written by older firmware (before this was persisted) show "—".
      // Coerce to a number so the zero-padded filename digits ("00863") match the
      // clipmeta keys, which are the bare integer ("863", from String(num)).
      const idm=x.name.match(/clip_(\d+)_/);
      const e=(idm&&window._sdMeta)?window._sdMeta[+idm[1]]:null;
      const en=encodeURIComponent(x.name);
      return "<tr><td style='position:relative'>"+thumbCell(x.name)+"</td><td>"+esc(x.name)+"</td>"+
      "<td>"+(x.mtime>1600000000?fmtTs(x.mtime*1000):"&mdash;")+"</td>"+
      "<td>"+(src?"<span class='src "+src+"'>"+src+"</span>"+(src=='spectral'&&e&&e[2]?" <span class='note'>"+(+e[2])+" Hz</span>":""):"<span class='src'>&mdash;</span>")+"</td>"+
      "<td>"+ovr(e?e[0]:null,e?e[1]:null)+"</td>"+
      "<td>"+Math.round(x.size/1024)+"KB</td>"+
      "<td><button class='play vw' aria-label='play clip' onclick=\"playClip('/sd/file?name="+en+"',this)\">&#9654;</button></td>"+
      "<td><a class='dl' href='/sd/file?name="+en+"&dl=1'>&#8595; wav</a></td>"+
      "<td><a class='dl' href='#' onclick=\"delSd('"+en+"');return false\" style='color:#f05d5d'>&#10005;</a></td></tr>"},sdclips,sdnav,sdempty);
  }catch(e){}
}
async function delSd(n){await fetch("/sd/delete?name="+n);refreshSd();refreshStatus()}
async function clearRam(){
  if(!confirm("Delete all clips from RAM?"))return;
  await fetch("/audio/clear");refreshClips();
}
async function clearSd(){
  if(!confirm("Delete ALL clips from the SD card? This cannot be undone."))return;
  await fetch("/sd/clear");refreshSd();refreshStatus();
}
function setSd(){fetch("/audio/config?sd="+(sdsave.checked?1:0))}
async function applySlots(){
  await fetch("/audio/config?slots="+slotsIn.value);
  refreshStatus();refreshClips();
}
async function refreshClips(){
  if(window._clipAudio)return; // don't rebuild rows mid-playback
  try{
    const c=await(await fetch("/audio/clips")).json();
    c.sort((a,b)=>b.id-a.id);
    page("ram",c,x=>
      "<tr><td>#"+(+x.id)+"</td><td title='"+age(x.age_ms)+"'>"+fmtTs(Date.now()-x.age_ms)+"</td>"+
      "<td><span class='src "+esc(x.source)+"'>"+esc(x.source)+"</span>"+(x.source=='spectral'&&x.hz?" <span class='note'>"+(+x.hz)+" Hz</span>":"")+"</td>"+
      "<td>"+ovr(x.rms,x.thr)+"</td>"+
      "<td><button class='play vw' aria-label='play clip' onclick=\"playClip('/audio/clip?id="+(+x.id)+"',this)\">&#9654;</button></td>"+
      "<td><a class='dl' href='/audio/clip?id="+(+x.id)+"&dl=1'>&#8595; wav</a></td></tr>",clips,clipsnav,empty);
  }catch(e){}
}
async function trigger(){
  rec.disabled=true;
  try{
    const r=await(await fetch("/audio/trigger")).json();
    setTimeout(()=>{refreshClips();rec.disabled=false},r.ready_in_ms+400);
  }catch(e){rec.disabled=false}
}
function setAuto(){fetch("/audio/config?auto="+(auto.checked?1:0))}
async function setAudEn(){
  if(!auden.checked&&_ac)stopListen();
  await fetch("/audio/config?en="+(auden.checked?1:0));
  refreshStatus();
}
async function retune(){await fetch("/audio/retune");refreshStatus()}
async function applyMthr(){
  await fetch("/audio/config?manual_thr="+(mthrIn.value||0));
  refreshStatus();
}
async function applyFactor(){
  await fetch("/audio/config?factor="+(factorIn.value||3));
  refreshStatus();
}
async function applyAlgo(){
  await fetch("/audio/config?algo="+(algoIn.value||0));
  refreshStatus();
}
async function applyK(){
  await fetch("/audio/config?trig_k="+(kIn.value||2.5)+"&stat_win="+(statWinIn.value||1));
  refreshStatus();
}
async function applyGain(){
  await fetch("/audio/config?gain="+(gainIn.value||16));
  refreshStatus();
}
async function applyLpf(){
  await fetch("/audio/config?lpf="+(lpfEn.checked?1:0)+"&lpf_hz="+(lpfHz.value||4000));
  // Now committed server-side (stream + clips + level metric). Drop the local
  // audition filter so you hear the REAL device-filtered stream, not it stacked
  // on top.
  if(window.clientLpf)clientLpf(false,0);
  refreshStatus();drawSpectrum();drawSpectrogram();
}
// Live audition: while DRAGGING, only the local monitor is filtered (instant,
// before the device round-trip) so you can pick a value by ear - the device
// stream/clips are unchanged until you release (applyLpf commits them).
function lpfDrag(){
  lpfHzv.value=lpfHz.value;
  if(window.clientLpf)clientLpf(true,+lpfHz.value);
}
// Typed cutoff: clamp to the slider's range, sync the slider, then commit.
function lpfBox(){
  let v=Math.round(+lpfHzv.value||4000);
  v=Math.max(+lpfHz.min,Math.min(+lpfHz.max,v));
  lpfHzv.value=v;lpfHz.value=v;applyLpf();
}
async function applyHpf(){
  await fetch("/audio/config?hpf="+(hpfEn.checked?1:0)+"&hpf_hz="+(hpfHz.value||120));
  if(window.clientHpf)clientHpf(false,0); // committed server-side; drop local preview
  refreshStatus();drawSpectrum();drawSpectrogram();
}
function hpfDrag(){
  hpfHzv.value=hpfHz.value;
  if(window.clientHpf)clientHpf(true,+hpfHz.value);
}
function hpfBox(){
  let v=Math.round(+hpfHzv.value||120);
  v=Math.max(+hpfHz.min,Math.min(+hpfHz.max,v));
  hpfHzv.value=v;hpfHz.value=v;applyHpf();
}
async function applyMinThr(){
  await fetch("/audio/config?min_thr="+(minthrIn.value||0));
  refreshStatus();
}
async function applyLen(){
  const r=await fetch("/audio/config?clip_ms="+Math.round((clipIn.value||5)*1000)
    +"&preroll_ms="+Math.round((preIn.value||0)*1000));
  if(!r.ok)alert(await r.text());
  refreshStatus();
}
// Event plot: the shared component (/ui.js, same one the camera board uses) bound
// to the audio level history + trigger events. SD clips are merged as extra
// markers (extraMarks); the clips table's over-threshold column is fed via
// onEvents (window._evBySd); clicking a marker plays that clip (onMarker).
buildNav('/'); // shared top nav (/ui.js)
window.ep=EventPlot(document.getElementById('ep'),{
  historyUrl:q=>'/audio/history'+q,
  eventsUrl:s=>'/audio/events?secs='+s,
  hasLog:true,
  onEvents:base=>{window._evBySd={};window._seenSd=new Set();for(const e of base)if(e.sd>=0){_evBySd[e.sd]=e;_seenSd.add(e.sd)}},
  extraMarks:async secs=>{try{
    if(Date.now()-(window._sdFilesAt||0)>15000){const sr=await fetch('/sd/list');if(sr.ok){const list=await sr.json(),sf=[];for(const f of list){const nm=f.name.match(/clip_(\d+)_(a|h|v|s)\.wav$/);if(nm&&f.mtime>1600000000)sf.push({num:+nm[1],mtime:f.mtime,source:{a:'auto',h:'http',v:'video',s:'spectral'}[nm[2]]})}window._sdFiles=sf;window._sdFilesAt=Date.now()}}
    const out=[],nowS=Date.now()/1000,seen=window._seenSd||new Set();
    for(const f of (window._sdFiles||[])){if(seen.has(f.num))continue;const age=(nowS-f.mtime)*1000;if(age<0||age>secs*1000)continue;out.push({age_ms:age,source:f.source,id:null,sd:f.num,rms:null,thr:null})}
    return out;
  }catch(e){return[]}},
  // Prefer the PERSISTENT SD file (same source the SD-clips table plays) so older
  // events load reliably; the RAM ring only keeps the newest few clips, so playing
  // by RAM id 404s for almost every marker. Fall back to RAM id only when the event
  // has no SD clip (e.g. save-to-SD off). playClip(url,btn) gives the same spinner/
  // retry/stop behavior as the tables.
  onMarker:(ev,btn)=>{const L={auto:'a',http:'h',video:'v',spectral:'s'};const u=ev.sd>=0?'/sd/file?name=clip_'+String(ev.sd).padStart(5,'0')+'_'+(L[ev.source]||'a')+'.wav':(ev.id!=null?'/audio/clip?id='+ev.id:null);if(u&&window.playClip)playClip(u,btn||null)}
});
filttog.onclick=e=>{
  e.preventDefault();
  filttog.classList.toggle("cur");
  try{localStorage.setItem("filttog",filttog.classList.contains("cur")?"1":"0")}catch(e){}
  drawSpectrum();drawSpectrogram(); // redraw in the new view immediately
};
// Restore the persisted "show filtered" choice; the regular spectrum poll then
// draws in this view (class set before the first draw, so no extra redraw needed).
if(window.localStorage&&localStorage.getItem("filttog")=="1")filttog.classList.add("cur");
// Amplitude-axis mode for the bar spectrum: 'rel' = dB vs the rolling peak
// (0dB=peak, shape always visible), 'dbfs' = absolute dBFS (0dB=full-scale, same
// ref as the heatmap), 'lin' = raw linear amplitude vs peak. Display only -
// triggering always uses raw magnitude regardless.
let _specScale=(window.localStorage&&localStorage.getItem("specscale"))||"rel",_specDbRef=132.4;
const _SCALES=[["rel","dB rel"],["dbfs","dBFS"],["lin","linear"]];
function _scaleLbl(){scaletog.textContent=(_SCALES.find(s=>s[0]==_specScale)||_SCALES[0])[1];}
_scaleLbl(); // reflect a persisted choice on load
scaletog.onclick=e=>{
  e.preventDefault();
  const i=_SCALES.findIndex(s=>s[0]==_specScale);
  _specScale=_SCALES[(i+1)%_SCALES.length][0];
  try{localStorage.setItem("specscale",_specScale)}catch(e){}
  _scaleLbl();
  drawSpectrum(); // bar plot only; heatmap keeps its own absolute dBFS scale
};
// (the audio level/event plot is now the shared EventPlot - see its init above)
// Slow-rolling display reference so the dB scale is STABLE frame-to-frame -
// you see a band actually move instead of the plot re-normalising every frame.
// Rises fairly quick (0.3), decays slow (0.03). Shared with the heatmap so both
// use one scale. specPAD is the left gutter for the dB axis labels.
let _specRef=0,_specFmin=60,_specFmax=8000,_specLpf=false,_specLpfHz=4000,_specHpf=false,_specHpfHz=120,_specData=null;
let _specPin=null; // clicked band index (dashed-line pin), or null
const SPEC_DB=72,specPAD=34,specAX=16;   // specAX: bottom strip reserved for the freq axis
// Heatmap colours map a FIXED dBFS window (0dB = digital full-scale, from the
// device's X-DbRef), so a given loudness is always the same colour and history
// never re-tints. Quiet rooms sit low (dark); events climb toward 0dB (red).
const SPEC_DBLO=-60,SPEC_DBHI=0;
// 2nd-order Butterworth magnitudes (match the device biquads' Q) - used to overlay
// the filters' effect on the spectrum/heatmap when "show filtered" is on.
function specFiltOn(){return filttog.classList.contains("cur")&&(_specLpf||_specHpf)}
function filtGain(f){let g=1;if(_specHpf){const r=f/_specHpfHz;g*=r*r/Math.sqrt(1+r*r*r*r);}if(_specLpf){const r=f/_specLpfHz;g*=1/Math.sqrt(1+r*r*r*r);}return g}
// Top of the plot in absolute dBFS = the rolling peak, so dBFS auto-fits the
// vertical range (bars fill the plot like 'rel') while the labels stay true
// dBFS. 'rel' anchors the top at 0 (i.e. "dB below peak").
function _specTopDb(){return _specScale=="dbfs"?20*Math.log10(_specRef+1)-_specDbRef:0;}
function specY(v,B){
  let u;
  if(_specScale=="lin")u=v/(_specRef||1);
  else{const db=_specScale=="dbfs"?20*Math.log10(v+1)-_specDbRef:20*Math.log10((v+1)/(_specRef+1));
       u=(db-_specTopDb())/SPEC_DB+1;}
  u=Math.max(0,Math.min(1,u));return 2+(1-u)*(B-4);}
// Live FFT spectrum of the RAW (pre-low-pass) signal. Bands are log-spaced on
// the device, so equal-width bars give a log frequency axis; the dashed line
// marks the low-pass cutoff and the grey steps the per-band trigger threshold.
// The frequency labels sit in their own strip below the bars (B..H).
async function drawSpectrum(){
  const cv=spec;if(!cv)return;
  let s;try{s=await(await fetch("/audio/spectrum")).json();}catch(e){return}
  _specData=s; // stash for the hover readout (see the spec mousemove handler)
  cv.width=cv.clientWidth||700;
  const ctx=cv.getContext("2d"),W=cv.width,H=cv.height,B=H-specAX,m=s.m,n=m.length,t=s.t||[];
  ctx.clearRect(0,0,W,H);
  if(!s.on){ctx.fillStyle="#8a94a0";ctx.font="12px sans-serif";ctx.fillText("audio off",specPAD+4,20);specnote.textContent="";specpin.style.display="none";return}
  _specFmin=s.f[0];_specFmax=s.fs/2;_specLpf=s.lpf;_specLpfHz=s.lpf_hz;_specHpf=s.hpf;_specHpfHz=s.hpf_hz;
  if(s.dbref)_specDbRef=s.dbref;
  const filt=specFiltOn();
  let fm=1;for(const v of m)if(v>fm)fm=v;
  _specRef=_specRef===0?fm:_specRef+(fm>_specRef?0.3:0.03)*(fm-_specRef);
  const X0=specPAD,PW=W-X0;
  const lx=f=>X0+Math.max(0,Math.min(1,Math.log(f/_specFmin)/Math.log(_specFmax/_specFmin)))*PW;
  // gridlines + axis labels for the current amplitude mode (see _specScale).
  ctx.font="10px sans-serif";ctx.lineWidth=1;
  if(_specScale=="lin"){
    for(let k=0;k<=4;k++){const u=k/4,y=2+(1-u)*(B-4);ctx.strokeStyle="#20262e";ctx.beginPath();ctx.moveTo(X0,y);ctx.lineTo(W,y);ctx.stroke();ctx.fillStyle="#6a7480";ctx.fillText(Math.round(_specRef*u),2,Math.min(B,y+3));}
  }else{
    const top=_specTopDb(),unit=_specScale=="dbfs"?"dBFS":"dB";
    for(let k=0;k<=SPEC_DB;k+=24){const u=(SPEC_DB-k)/SPEC_DB,y=2+(1-u)*(B-4);ctx.strokeStyle="#20262e";ctx.beginPath();ctx.moveTo(X0,y);ctx.lineTo(W,y);ctx.stroke();ctx.fillStyle="#6a7480";ctx.fillText(Math.round(top-k)+unit,2,Math.min(B,y+3));}
  }
  const bw=PW/n;
  // Bars. "show filtered" scales each band by the low-pass/high-pass response
  // (what the stream/clips contain). The band trigger works on the RAW signal,
  // so the threshold (t[]) is in raw units; in the filtered view we scale BOTH
  // the bar and the threshold line by the same filter gain, which preserves the
  // over/under relationship (filtered bar crosses filtered line exactly when raw
  // mag crosses raw threshold), so red = "would trigger" stays correct.
  for(let i=0;i<n;i++){const v=filt?m[i]*filtGain(s.f[i]):m[i],x=X0+i*bw,y=specY(v,B);const hot=s.strig&&t[i]!=null&&m[i]>t[i];ctx.fillStyle=hot?"#f05d5d":filt?"#3a7fd0":"#4f9dff";ctx.fillRect(x+1,y,Math.max(1,bw-2),B-y);}
  if(s.strig&&t.length){ctx.strokeStyle="#aeb6c0";ctx.lineWidth=1;ctx.beginPath();for(let i=0;i<n;i++){const y=specY(filt?t[i]*filtGain(s.f[i]):t[i],B),x0=X0+i*bw,x1=x0+bw;if(i===0)ctx.moveTo(x0,y);else ctx.lineTo(x0,y);ctx.lineTo(x1,y);}ctx.stroke();}
  // Pinned band: a dashed vertical indicator line + a persistent readout below
  // (mirrors clicking the level plot). Cleared if the band count changed.
  if(_specPin!=null&&_specPin<n){
    const px=X0+(_specPin+0.5)*bw;
    ctx.save();ctx.setLineDash([3,3]);ctx.strokeStyle="#ffd24d";ctx.lineWidth=1.5;
    ctx.beginPath();ctx.moveTo(px,0);ctx.lineTo(px,B);ctx.stroke();ctx.restore();
    specpin.innerHTML="<b style='color:#ffd24d'>pinned</b> "+_specBandText(_specPin," · ");specpin.style.display="";
  }else{_specPin=null;specpin.style.display="none";}
  // cutoff markers: low-pass orange, high-pass teal (dashed).
  ctx.setLineDash([4,3]);
  if(s.lpf){const x=lx(s.lpf_hz);ctx.strokeStyle="#f0a85d";ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,B);ctx.stroke();}
  if(s.hpf){const x=lx(s.hpf_hz);ctx.strokeStyle="#5fd0b0";ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,B);ctx.stroke();}
  ctx.setLineDash([]);
  // Frequency axis in its own strip below the bars (was overlapping them, hence
  // hard to read). Tick marks + centred labels on a slightly inset background.
  ctx.fillStyle="#161a1f";ctx.fillRect(0,B+1,W,specAX-1);
  ctx.strokeStyle="#3a424d";ctx.fillStyle="#aab2bc";ctx.textAlign="center";
  for(const f of [100,300,1000,3000,6000]){if(f<=_specFmin||f>=_specFmax)continue;const x=lx(f);ctx.beginPath();ctx.moveTo(x,B+1);ctx.lineTo(x,B+4);ctx.stroke();ctx.fillText(f>=1000?(f/1000)+"k":""+f,Math.max(10,Math.min(W-10,x)),H-4);}
  ctx.textAlign="start";
  if(!_hold(strig))strig.checked=s.strig;
  if(!_hold(sfac))sfac.value=s.sfactor;
  if(!_hold(sminmag))sminmag.value=s.sminmag;
  {const hm=document.getElementById("habmin");if(hm&&!_hold(hm)&&s.hab_min!=null)hm.value=s.hab_min;}
  {const bt=document.getElementById("bandtc");if(bt&&!_hold(bt)&&s.band_tc!=null)bt.value=s.band_tc;}
  if(!_hold(shist))shist.value=s.spectro_min;
  {const hw=document.getElementById("histwin");if(hw&&!_hold(hw))hw.value=s.hist_win;}
  const warm=s.bandwarm_ms>0?" · warming up "+Math.ceil(s.bandwarm_ms/1000)+"s":"";
  const view=filt?"showing FILTERED spectrum (what the stream & clips contain)":
    "showing RAW spectrum (pre-filter)";
  const fl=(s.hpf?"HP "+Math.round(s.hpf_hz)+"Hz":"")+(s.hpf&&s.lpf?" + ":"")+(s.lpf?"LP "+Math.round(s.lpf_hz)+"Hz":"");
  // Legend moved out of the canvas title (which clashed with the hover tooltip)
  // into this caption below the plot. Hover any band for its range/level/threshold.
  specnote.innerHTML="FFT of the raw (pre-filter) mic signal · hover a band for details · "+view+" · "+(fl||"no filters")+
    ((s.lpf||s.hpf)?" · cutoff "+(s.lpf?"<span style='color:#f0a85d'>LP</span>":"")+(s.lpf&&s.hpf?" + ":"")+(s.hpf?"<span style='color:#5fd0b0'>HP</span>":""):"")+
    (s.strig?" · band trigger ON: <span style='color:#aeb6c0'>grey line</span> = per-band threshold"+(filt?" (filter-scaled)":"")+", bars red when over"+warm:"");
}
// Hover readout: the band under the cursor, its log-spaced frequency range, the
// current level and the per-band trigger threshold (red OVER when it would fire).
// Bars are evenly spaced by band index (bw=PW/n from X0=specPAD), so map the
// cursor x straight to a band; f[] are band centres, edges are center*/sqrt(step).
// Map a mouse event's x to the band index under it (or -1). Bars are evenly
// spaced by index (bw=(W-X0)/n from X0=specPAD), shared by hover + click-pin.
function _specBandAt(e){
  const s=_specData;if(!s||!s.m||!s.on)return -1;
  const r=spec.getBoundingClientRect(),W=spec.width,n=s.m.length,X0=specPAD;
  const cx=(e.clientX-r.left)*(W/(r.width||W)); // client px -> canvas px (CSS scaling)
  const i=Math.floor((cx-X0)/((W-X0)/n));
  return (i<0||i>=n)?-1:i;
}
// One-line readout for band i (separator " · " inline, or "<br>" stacked).
function _specBandText(i,sep){
  const s=_specData,f=s.f,n=s.m.length,hs=Math.sqrt(n>1?f[1]/f[0]:1.1),lo=f[i]/hs,hi=f[i]*hs;
  const fmt=hz=>hz>=1000?(hz/1000).toFixed(hz>=10000?0:1)+"k":Math.round(hz);
  const mag=s.m[i],thr=(s.t&&s.t[i]!=null)?s.t[i]:null,over=s.strig&&thr!=null&&mag>thr;
  let h="<b>"+fmt(lo)+"–"+fmt(hi)+" Hz</b>"+sep+"level "+Math.round(mag);
  if(thr!=null)h+=sep+"threshold "+Math.round(thr)+(s.strig?(over?" <span style='color:#f05d5d'>· OVER</span>":""):" <span style='color:#6a7480'>(band trig off)</span>");
  return h;
}
function _specHover(e){
  const i=_specBandAt(e);
  if(i<0){spectip.style.display="none";return}
  spectip.innerHTML=_specBandText(i,"<br>");spectip.style.display="block";
  spectip.style.left=Math.min(e.clientX+14,innerWidth-spectip.offsetWidth-8)+"px";
  spectip.style.top=(e.clientY+14)+"px";
}
spec.addEventListener("mousemove",_specHover);
spec.addEventListener("mouseleave",()=>{spectip.style.display="none"});
// Click to PIN a band: a dashed vertical line + a persistent readout that survive
// the periodic redraw (like clicking the level plot). Click the same band or
// outside the bars to clear.
spec.addEventListener("click",e=>{
  const i=_specBandAt(e);
  _specPin=(i<0||i===_specPin)?null:i;
  drawSpectrum();
});
// Spectrogram heatmap: time → (oldest left), frequency ↑ (low at bottom),
// brightness = magnitude on the same rolling reference as the bar plot.
async function drawSpectrogram(){
  const cv=spech;if(!cv)return;
  let cols=0,bands=0,bms=1000,db256=0,dbRef=0,buf;
  try{const r=await fetch("/audio/spectrogram");if(!r.ok)return;
    cols=+r.headers.get("X-Cols")||0;bands=+r.headers.get("X-Bands")||0;bms=+r.headers.get("X-Bucket-Ms")||1000;
    db256=(+r.headers.get("X-Db256")||0)?256:0;dbRef=+r.headers.get("X-DbRef")||0;
    buf=new Uint16Array(await r.arrayBuffer());}catch(e){return}
  cv.width=cv.clientWidth||700;
  const RG=36;                                   // right gutter for the colour scale
  const ctx=cv.getContext("2d"),W=cv.width,H=cv.height,X0=specPAD,PW=W-X0-RG;
  ctx.clearRect(0,0,W,H);
  if(!cols||!bands){ctx.fillStyle="#8a94a0";ctx.font="12px sans-serif";ctx.fillText("collecting spectrogram…",X0+4,20);return}
  // Stored values are dB*256 (relative loudness). FIXED dB window -> colours
  // never remap as the live level drifts; only the data moves.
  const filt=specFiltOn();
  const bf=b=>_specFmin*Math.pow(_specFmax/_specFmin,(b+0.5)/bands);
  const g=filt?Array.from({length:bands},(_,b)=>20*Math.log10(Math.max(1e-6,filtGain(bf(b))))):null; // filter atten in dB
  const toDb=v=>(db256?v/db256:20*Math.log10(v+1))-dbRef; // dBFS of the cell (0=full-scale)
  const cw=PW/cols,chh=H/bands;
  for(let c=0;c<cols;c++)for(let b=0;b<bands;b++){
    let db=toDb(buf[c*bands+b]); if(filt)db+=g[b];
    const u=(db-SPEC_DBLO)/(SPEC_DBHI-SPEC_DBLO);
    ctx.fillStyle=heat(Math.max(0,Math.min(1,u)));
    ctx.fillRect(X0+c*cw,H-(b+1)*chh,Math.ceil(cw),Math.ceil(chh)); // band 0 = lowest freq, at bottom
  }
  // Colour scale legend (right edge): fixed dB window.
  const cbx=W-RG+8,cbw=9,cbtop=12,cbbot=H-4;
  for(let y=cbtop;y<=cbbot;y++){const u=1-(y-cbtop)/(cbbot-cbtop);ctx.fillStyle=heat(u);ctx.fillRect(cbx,y,cbw,1);}
  ctx.fillStyle="#8a94a0";ctx.font="9px sans-serif";ctx.textAlign="left";
  for(let k=0;k<=4;k++){const db=SPEC_DBLO+(SPEC_DBHI-SPEC_DBLO)*k/4,y=cbbot-(cbbot-cbtop)*k/4;ctx.fillText(Math.round(db)+(k===4?"dB":""),cbx+cbw+2,Math.min(H-1,y+3));}
  ctx.textAlign="start";
  // frequency ticks (left)
  ctx.fillStyle="#cdd3da";ctx.font="10px sans-serif";
  for(const f of [100,1000,6000]){if(f<=_specFmin||f>=_specFmax)continue;const fr=Math.log(f/_specFmin)/Math.log(_specFmax/_specFmin);ctx.fillText(f>=1000?(f/1000)+"k":f,2,Math.max(9,H-fr*H));}
  // span labels: oldest (left) .. now (right of the plot area)
  const spanMin=cols*bms/60000;
  ctx.fillStyle="#8a94a0";ctx.fillText("-"+(spanMin>=1?spanMin.toFixed(spanMin<10?1:0)+"m":Math.round(cols*bms/1000)+"s"),X0+2,11);
  ctx.fillText("now",X0+PW-22,11);
}
// dark → blue → cyan → yellow → red colormap for u in [0,1].
function heat(u){const s=[[10,12,18],[30,60,140],[40,170,180],[230,200,60],[240,70,60]];const x=u*(s.length-1),i=Math.min(s.length-2,Math.floor(x)),f=x-i,a=s[i],b=s[i+1];return"rgb("+Math.round(a[0]+f*(b[0]-a[0]))+","+Math.round(a[1]+f*(b[1]-a[1]))+","+Math.round(a[2]+f*(b[2]-a[2]))+")";}
async function setHistWin(v){await fetch("/audio/config?hist_win="+v);} // ring re-buckets server-side; poll repaints
async function applySpec(){
  // Optional chaining: don't throw if habmin/bandtc are absent (partial DOM /
  // future edits) - the sibling reads are already null-safe id-globals.
  await fetch("/audio/config?strig="+(strig.checked?1:0)+"&sfactor="+(sfac.value||4)+"&sminmag="+(sminmag.value||0)+"&spectro_min="+(shist.value||4)+"&hab_min="+(document.getElementById("habmin")?.value||0)+"&band_tc="+(document.getElementById("bandtc")?.value||2));
  drawSpectrum();drawSpectrogram();
}
// (plot rendering + interaction now live in the shared EventPlot, /ui.js)
function fmtUp(ms){const m=Math.floor(ms/60000);return Math.floor(m/60)+"h "+(m%60)+"m"}
async function refreshDiag(){
  try{
    const d=await(await fetch("/diag")).json();
    if(d.stream_on!==undefined&&!_hold(strmen))strmen.checked=d.stream_on;
    const q=d.rssi>-55?"excellent":d.rssi>-67?"good":d.rssi>-75?"fair":"weak";
    diag.innerHTML=
      "<div class='stat'><b>"+d.rssi+" dBm</b><span>WiFi ("+q+")</span></div>"+
      "<div class='stat'><b style='color:"+(d.camera?"#5df0a0":"#f05d5d")+"'>"+(d.camera?"ok":"down")+"</b><span>camera</span></div>"+
      (d.sd===undefined?"":"<div class='stat'><b style='color:"+(d.sd?"#5df0a0":"#f05d5d")+"'>"+(d.sd?"connected":"no card")+"</b><span>SD card"+(d.sd_drops>0?" ("+d.sd_drops+" drop"+(d.sd_drops>1?"s":"")+")":"")+"</span></div>")+
      "<div class='stat'><b>"+d.res+"</b><span>q"+d.quality+" @ "+d.xclk+"MHz</span></div>"+
      "<div class='stat'><b>"+(d.fps>0?d.fps.toFixed(1)+(d.net_fps>0?" / "+d.net_fps.toFixed(1):""):"idle")+"</b><span>fps cam/net</span></div>"+
      "<div class='stat'><b>"+d.clients+"</b><span>clients</span></div>"+
      "<div class='stat'><b>"+Math.round(d.free_heap/1024)+"KB</b><span>heap free</span></div>"+
      "<div class='stat'><b>"+(d.free_psram/1048576).toFixed(1)+"MB</b><span>PSRAM free</span></div>"+
      "<div class='stat'><b style='color:"+(d.temp_c>80?"#f05d5d":"#dde3ea")+"'>"+d.temp_c.toFixed(1)+"&deg;C</b><span>chip temp</span></div>"+
      "<div class='stat'><b>"+fmtUp(d.uptime_ms)+"</b><span>uptime</span></div>";
  }catch(e){}
}
// ---- Video clips: status/config, table, and in-browser AVI playback ----
let _wasVRec=false;
async function refreshVidStatus(){
  try{
    const s=await(await fetch("/video/status")).json();
    viden.checked=s.enabled;
    if(!_hold(vclipIn))vclipIn.value=s.clip_ms/1000;
    if(!_hold(vpreIn))vpreIn.value=s.preroll_ms/1000;
    if(!_hold(vfpsIn))vfpsIn.value=s.fps;
    if(!_hold(vmaxIn))vmaxIn.value=s.vmax;
    vonaud.checked=s.on_audio;vtrigaud.checked=s.trig_audio;
    if(!_hold(vgapIn))vgapIn.value=Math.round(s.min_gap_ms/1000);
    if(!_hold(vxtrigIn))vxtrigIn.value=Math.round(s.xtrig_cd_ms/1000);
    vthen.checked=s.thermal_en;
    if(!_hold(vthIn))vthIn.value=s.temp_max;
    thState.textContent=s.thermal_paused?("⏸ paused — die ≥ "+s.temp_max+"°C"):(s.thermal_en?"":"off");
    vmot.checked=s.motion;
    if(!_hold(vmotIn))vmotIn.value=s.motion_blocks;
    if(!_hold(vdiffIn))vdiffIn.value=s.motion_diff;
    motlvl.textContent=s.motion?("level "+s.motion_level+" / "+s.motion_blocks
      +(s.motion_global?" (lighting change)":"")):"";
    vidState.textContent=s.recording?"● recording":"";
    vrec.disabled=!s.enabled||!s.sd;
    vidinfo.textContent=!s.sd?"needs an SD card":s.enabled
      ?("pre-roll buffer "+s.ring_kb+"KB · "+s.ring_frames+" frames"
        +(s.drops?" · "+s.drops+" dropped":"")):"";
    if(_wasVRec&&!s.recording)setTimeout(refreshVid,800);
    _wasVRec=s.recording;
  }catch(e){}
}
async function setVidEn(){
  const r=await fetch("/video/config?en="+(viden.checked?1:0));
  if(!r.ok)alert(await r.text());
  refreshVidStatus();refreshDiag();
}
async function setStrmEn(){
  // master live-stream switch; off keeps the camera parked (audio-only, cooler)
  await fetch("/camera/power?on="+(strmen.checked?1:0));
  refreshDiag();
}
async function applyVidCfg(){
  const r=await fetch("/video/config?clip_ms="+Math.round((vclipIn.value||10)*1000)
    +"&preroll_ms="+Math.round((vpreIn.value||0)*1000)
    +"&fps="+(vfpsIn.value||5)+"&vmax="+(vmaxIn.value||0));
  if(!r.ok)alert(await r.text());
  refreshVidStatus();
}
async function applyRateCfg(){
  const r=await fetch("/video/config?min_gap="+(vgapIn.value||0)+"&xtrig_cd="+(vxtrigIn.value||0));
  if(!r.ok)alert(await r.text());
  refreshVidStatus();
}
async function setThEn(){
  await fetch("/video/config?temp_en="+(vthen.checked?1:0));
  refreshVidStatus();refreshDiag();
}
async function applyThCfg(){
  const r=await fetch("/video/config?temp_max="+(vthIn.value||100));
  if(!r.ok)alert(await r.text());
  refreshVidStatus();
}
async function vidTrigger(){
  vrec.disabled=true;
  try{await fetch("/video/trigger")}catch(e){}
  refreshVidStatus();
}
async function setVidMot(){
  const r=await fetch("/video/config?motion="+(vmot.checked?1:0));
  if(!r.ok)alert(await r.text());
  refreshVidStatus();
}
async function applyVidMot(){
  await fetch("/video/config?motion_blocks="+(vmotIn.value||12)
    +"&motion_diff="+(vdiffIn.value||12));
  refreshVidStatus();
}
function pairAudio(mt){
  if(mt<1600000000)return null;
  for(const a of _sdAudio)if(Math.abs(a.mtime-mt)<=8)return a.name;
  return null;
}
async function refreshVid(){
  try{
    const r=await fetch("/video/list");
    if(!r.ok)return;
    const c=await r.json();
    c.sort((a,b)=>b.name.localeCompare(a.name));
    page("vid",c,x=>{
      const m=x.name.match(/_(h|m|a)\.avi$/);
      const src=m?{h:"http",m:"motion",a:"audio"}[m[1]]:null;
      const cls=src=="http"?"http":src=="motion"?"motion":"auto";
      const aud=pairAudio(x.mtime);
      const en=encodeURIComponent(x.name),ena=aud?encodeURIComponent(aud):null;
      return "<tr><td style='position:relative'>"+thumbCell(x.name)+"</td><td>"+esc(x.name)+"</td>"+
      "<td>"+(x.mtime>1600000000?fmtTs(x.mtime*1000):"&mdash;")+"</td>"+
      "<td>"+(src?"<span class='src "+cls+"'>"+src+"</span>":"&mdash;")+"</td>"+
      "<td>"+(x.size/1048576).toFixed(1)+"MB</td>"+
      "<td><button class='play vw' aria-label='play clip' onclick=\"playVid('"+en+"',"+(ena?"'"+ena+"'":"null")+",this)\">&#9654;</button></td>"+
      "<td>"+(aud?"&#128266;":"&mdash;")+"</td>"+
      "<td><a class='dl' href='/sd/file?name="+en+"&dl=1'>&#8595; avi</a></td>"+
      "<td><a class='dl' href='#' onclick=\"delVid('"+en+"');return false\" style='color:#f05d5d'>&#10005;</a></td></tr>"},vidclips,vidnav,vidempty);
  }catch(e){}
}
async function delVid(n){await fetch("/sd/delete?name="+n);refreshVid()}
async function clearVid(){
  if(!confirm("Delete ALL video clips from the SD card?"))return;
  await fetch("/video/clear");refreshVid();
}
// Minimal AVI demuxer: walk the RIFF tree, collect the '00dc' MJPEG chunks
// and the frame rate, then paint JPEGs to a canvas at the recorded pacing.
// <video> can't play MJPEG-AVI, but the same file opens in VLC if downloaded.
function _cc(dv,o){return String.fromCharCode(dv.getUint8(o),dv.getUint8(o+1),dv.getUint8(o+2),dv.getUint8(o+3))}
function parseAvi(buf){
  const dv=new DataView(buf);
  if(dv.byteLength<224||_cc(dv,0)!="RIFF"||_cc(dv,8)!="AVI ")return null;
  let fps=5,frames=[],p=12;
  while(p+8<=dv.byteLength){
    const cc=_cc(dv,p),sz=dv.getUint32(p+4,true);
    if(cc=="LIST"){
      const lt=_cc(dv,p+8);
      if(lt=="movi"){
        let q=p+12;const end=Math.min(p+8+sz,dv.byteLength);
        while(q+8<=end){
          const fcc=_cc(dv,q),fsz=dv.getUint32(q+4,true);
          if(fcc=="00dc"&&fsz&&q+8+fsz<=dv.byteLength)frames.push([q+8,fsz]);
          q+=8+fsz+(fsz&1);
        }
      }else if(lt=="hdrl"){
        const us=dv.getUint32(p+20,true); // avih dwMicroSecPerFrame
        if(us>0)fps=1e6/us;
      }
    }
    p+=8+sz+(sz&1);
  }
  return{fps:fps,frames:frames};
}
let _vbuf=null,_vfrm=null,_vfps=5,_vi=0,_vtimer=null,_vaudUrl=null,_vmot=null;
// Translucent boxes from the clip's .mot sidecar: the motion detector's
// changed-block map nearest this frame's timestamp (maps are ~2/s).
function drawMotMap(ctx,w,h,i){
  if(!_vmot||!_vmot.maps||!_vmot.maps.length)return;
  const t=i*1000/_vfps;
  let m=null;
  for(const e of _vmot.maps){if(e[0]<=t)m=e;else break}
  if(!m||t-m[0]>1500)return;
  const bw=w/_vmot.bx,bh=h/_vmot.by;
  ctx.fillStyle="rgba(240,93,93,.22)";ctx.strokeStyle="rgba(240,93,93,.85)";ctx.lineWidth=2;
  for(let b=0;b<_vmot.bx*_vmot.by;b++){
    if(parseInt(m[2].substr((b>>3)*2,2),16)>>(b&7)&1){
      const x=(b%_vmot.bx)*bw,y=((b/_vmot.bx)|0)*bh;
      ctx.fillRect(x,y,bw,bh);ctx.strokeRect(x,y,bw,bh);
    }
  }
  ctx.fillStyle="#f05d5d";ctx.font=Math.round(h/40)+"px monospace";
  ctx.fillText("motion "+m[1],10,Math.round(h/40)+8);
}
async function showFrame(i){
  if(!_vbuf)return; // closeVid() may have nulled the buffer between scheduled frames
  _vi=i;
  const f=_vfrm[i];
  const bmp=await createImageBitmap(new Blob([new Uint8Array(_vbuf,f[0],f[1])],{type:"image/jpeg"}));
  vcv.width=bmp.width;vcv.height=bmp.height;
  const ctx=vcv.getContext("2d");
  ctx.drawImage(bmp,0,0);
  drawMotMap(ctx,bmp.width,bmp.height,i);
  bmp.close();
  vseek.value=i;
  vfr.textContent=(i+1)+"/"+_vfrm.length;
}
async function playVid(name,audName,btn){
  if(btn){btn.disabled=true;btn.innerHTML="<span class='spin'></span>"}
  try{
    const r=await fetch("/sd/file?name="+name);
    if(!r.ok)throw 0;
    // stream the body so the button can show download progress (AVIs are
    // a few MB and take seconds from the device)
    const total=+r.headers.get("Content-Length")||0;
    const rd=r.body.getReader();
    const parts=[];let got=0;
    for(;;){
      const{done,value}=await rd.read();
      if(done)break;
      parts.push(value);got+=value.length;
      if(btn&&total)btn.textContent=Math.round(got/total*100)+"%";
    }
    const all=new Uint8Array(got);
    let o=0;for(const p of parts){all.set(p,o);o+=p.length}
    _vbuf=all.buffer;
    const av=parseAvi(_vbuf);
    if(!av||!av.frames.length){alert("could not parse "+name+" (empty or truncated clip)");return}
    _vfrm=av.frames;_vfps=av.fps||5;
    _vmot=null;
    try{ // motion-map sidecar (only written when the detector was running)
      const mr=await fetch("/sd/file?name="+name.replace(/\.avi$/,".mot"));
      if(mr.ok)_vmot=await mr.json();
    }catch(e){}
    vseek.max=_vfrm.length-1;
    _vaudUrl=audName?"/sd/file?name="+audName:null;
    vaud.style.display=_vaudUrl?"inline-block":"none";
    vplayer.style.display="block";
    await showFrame(0);
    vidPlay();
  }catch(e){}
  if(btn){btn.disabled=false;btn.innerHTML="&#9654;"}
}
function vidPlay(){
  if(_vtimer||!_vfrm)return;
  vpp.innerHTML="&#9208;";
  _vtimer=setInterval(()=>{
    if(_vi+1>=_vfrm.length){vidPause();return}
    showFrame(_vi+1);
  },1000/_vfps);
}
function vidPause(){
  if(_vtimer){clearInterval(_vtimer);_vtimer=null}
  vpp.innerHTML="&#9654;";
}
function vidPlayPause(){
  if(_vtimer)vidPause();
  else{if(_vi+1>=_vfrm.length)showFrame(0);vidPlay()}
}
vseek.oninput=()=>{vidPause();showFrame(+vseek.value)};
function vidAudioToggle(){
  if(window._clipAudio)stopClip();
  else if(_vaudUrl)playClip(_vaudUrl,null);
}
function closeVid(){
  vidPause();
  vplayer.style.display="none";
  _vbuf=null;_vfrm=null;_vmot=null;
  if(window._clipAudio)stopClip();
}
// Poll gently: tables/diag refresh slowly (manual actions and clip-end events
// trigger instant refreshes), and everything pauses while the tab is hidden.
const _vis=fn=>()=>{if(!document.hidden)fn()};
// In-flight guard: skip a periodic tick if the previous one is still pending, so
// slow/large polls can't stack up on the device's tiny TCP window (each returns a
// stable closure with its own busy flag, created once at setup).
const _once=fn=>{let busy=false;return async()=>{if(busy)return;busy=true;try{await fn()}finally{busy=false}}};
refreshStatus();refreshClips();refreshSd();refreshDiag();refreshVidStatus();refreshVid();
setTimeout(drawSpectrum,700);setTimeout(drawSpectrogram,1000); // Event plot self-inits (/ui.js)
setInterval(_vis(_once(refreshStatus)),2000);setInterval(_vis(_once(refreshClips)),10000);
setInterval(_vis(_once(refreshSd)),10000);setInterval(_vis(_once(refreshDiag)),10000);
// Spectrum is live: poll quickly while visible. The device FFT runs continuously
// while audio is enabled (for the always-on spectral trigger), so polling only
// affects how fresh the plot is. The 15KB spectrogram polls slower.
setInterval(_vis(_once(drawSpectrum)),1500);
setInterval(_vis(_once(drawSpectrogram)),2500);
setInterval(_vis(_once(refreshVidStatus)),5000);setInterval(_vis(_once(refreshVid)),30000);
document.addEventListener("visibilitychange",()=>{
  if(!document.hidden){refreshStatus();refreshClips();refreshSd();refreshDiag();window.ep&&ep.reload();drawSpectrum();drawSpectrogram();refreshVidStatus();refreshVid()}
});
</script>
<script src="/audio/player.js"></script>
</body></html>)rawliteral";

// Shared live-PCM player: fetches /audio/stream, skips the WAV header,
// converts s16le to float and schedules it into a WebAudio context.
// Exposes toggleListen()/stopListen(); expects a #listen button.
static const char PLAYER_JS[] PROGMEM = R"rawliteral(
let _ac=null,_abort=null,_hpfNode=null,_lpfNode=null,_chain=null,_srcs=[];
let _hpfOn=false,_hpfHz=120,_lpfOn=false,_lpfHz=4000;
let _an=null,_raf=0; // analyser tap + rAF handle for the live "playing" meter
// Client-side filter preview: lets the audio page audition cutoffs by ear
// (hpfDrag/lpfDrag) before committing to the device. A fixed high-pass→low-pass
// chain sits ahead of the output (built once in flush); "off" just parks a node
// at a transparent cutoff (10Hz HP / 20kHz LP) so the graph never reconnects.
// The DEVICE filters are the real ones (clean stream+clips+metric) - these only
// color what this browser hears, and clear on stop.
function clientHpf(on,hz){_hpfOn=on;_hpfHz=hz;if(_hpfNode)_hpfNode.frequency.value=on?hz:10;}
function clientLpf(on,hz){_lpfOn=on;_lpfHz=hz;if(_lpfNode)_lpfNode.frequency.value=on?hz:20000;}
async function toggleListen(){
  if(_ac){stopListen();return}
  // "connecting" until the first audio is actually scheduled - otherwise the
  // ~0.4s buffer fill + scheduling lead looks like a working button playing
  // silence. Flips to "stop" the instant sound starts.
  listen.innerHTML="&#9203; connecting&hellip;";
  // iOS Safari throws if you force a 16kHz context rate, so use the hardware
  // rate (each createBuffer below declares 16000 and WebAudio resamples). iOS
  // also needs the context unlocked inside the tap gesture: resume() plus a
  // one-sample silent buffer, both synchronous here before any await.
  try{
    _ac=new (window.AudioContext||window.webkitAudioContext)();
  }catch(e){listen.innerHTML="&#128264; listen live";_ac=null;return}
  _ac.resume();
  try{
    const u=_ac.createBuffer(1,1,_ac.sampleRate),us=_ac.createBufferSource();
    us.buffer=u;us.connect(_ac.destination);us.start(0);
  }catch(e){}
  _abort=new AbortController();
  // Jitter buffer: schedule LEAD seconds ahead of the playhead and feed
  // WebAudio in AGG-sample pieces; WiFi hiccups shorter than LEAD are
  // inaudible (and bursty delivery banks extra cushion on top, since a burst
  // that lands faster than real-time pushes the schedule further ahead). Total
  // added latency ~= LEAD. LEAD is sized for *remote* streaming: the device's
  // lwIP TCP window only holds ~180ms of 256kbps PCM, so over a high-RTT
  // WAN/cellular link TCP delivers in multi-second bursts with stalls between
  // - a small buffer underruns into silence there. 2.5s covers those stalls;
  // on LAN it just adds harmless latency. The first buffer flushes eagerly
  // (after EAGER samples) so playback starts quickly; later ones use AGG for
  // resilience.
  // LEAD: initial cushion (covers WAN bursts). REARM: the cushion re-armed AFTER
  // an underrun - small, so a mid-stream stall costs a brief gap, NOT a fresh 2.5s
  // of silence (the old code re-armed the full LEAD on every underrun, which was
  // the periodic "blackout"). On LAN underruns are rare, so the short re-arm just
  // recovers fast.
  const LEAD=2.5,REARM=0.6,AGG=2048,EAGER=512;
  let nextT=0,pend=new Int16Array(AGG),fill=0,started=false,_uruns=0;
  const flush=()=>{
    if(!fill||!_ac)return;
    const ab=_ac.createBuffer(1,fill,16000);
    const ch=ab.getChannelData(0);
    for(let i=0;i<fill;i++)ch[i]=pend[i]/32768;
    const src=_ac.createBufferSource();
    src.buffer=ab;
    // Build the high-pass→low-pass chain once; "off" nodes sit at transparent
    // cutoffs so the graph is stable (no reconnects when toggling).
    if(!_chain){
      _hpfNode=_ac.createBiquadFilter();_hpfNode.type="highpass";_hpfNode.Q.value=0.707;_hpfNode.frequency.value=_hpfOn?_hpfHz:10;
      _lpfNode=_ac.createBiquadFilter();_lpfNode.type="lowpass";_lpfNode.Q.value=0.707;_lpfNode.frequency.value=_lpfOn?_lpfHz:20000;
      // AnalyserNode sits as a pass-through just before the output, so the meter
      // reflects exactly what you hear (post client filters). Audio still plays.
      _an=_ac.createAnalyser();_an.fftSize=1024;
      _hpfNode.connect(_lpfNode);_lpfNode.connect(_an);_an.connect(_ac.destination);_chain=_hpfNode;
    }
    src.connect(_chain);
    if(nextT<_ac.currentTime+0.05){
      nextT=_ac.currentTime+(started?REARM:LEAD); // small re-arm after an underrun
      if(started)console.warn("audio underrun #"+(++_uruns)+" - re-buffering "+REARM+"s");
    }
    src.start(nextT);nextT+=ab.duration;
    // Track scheduled sources so stopListen() can stop the ones queued ahead
    // (up to LEAD seconds out); drop each from the list when it finishes so the
    // array doesn't grow unbounded over a long listen.
    _srcs.push(src);
    src.onended=()=>{const i=_srcs.indexOf(src);if(i>=0)_srcs.splice(i,1)};
    fill=0;
    if(!started){started=true;listen.innerHTML="&#128266; stop";lvwitness.style.display="";meterTick();}
  };
  try{
    const r=await fetch("/audio/stream",{signal:_abort.signal});
    if(!r.ok)throw 0;
    const rd=r.body.getReader();
    let skip=44,carry=new Uint8Array(0);
    while(_ac){
      const{done,value}=await rd.read();
      if(done)break;
      let d=value;
      if(skip>0){const s=Math.min(skip,d.length);d=d.subarray(s);skip-=s;if(!d.length)continue}
      const b=new Uint8Array(carry.length+d.length);
      b.set(carry);b.set(d,carry.length);
      const n=b.length>>1;
      if(!n){carry=b;continue}
      carry=b.slice(n*2);
      const i16=new Int16Array(b.buffer,0,n);
      for(let i=0;i<n;i++){
        pend[fill++]=i16[i];
        if(fill===AGG||(!started&&fill>=EAGER))flush();
      }
    }
  }catch(e){}
  stopListen();
}
// Drives the "playing" level bar from the actual playback (proof audio is live,
// not just connected). Peak of the time-domain samples -> bar width, ~per frame.
function meterTick(){
  if(!_ac||!_an){_raf=0;return}
  const buf=new Uint8Array(_an.fftSize);
  _an.getByteTimeDomainData(buf);
  let peak=0;
  for(let i=0;i<buf.length;i++){const v=Math.abs(buf[i]-128);if(v>peak)peak=v}
  lvfill.style.width=Math.min(100,peak/128*140)+"%"; // 140 = a little headroom boost
  _raf=requestAnimationFrame(meterTick);
}
function stopListen(){
  if(_raf){cancelAnimationFrame(_raf);_raf=0}
  lvwitness.style.display="none";lvfill.style.width="0";_an=null;
  if(_abort){_abort.abort();_abort=null}
  // Stop sources scheduled into the future BEFORE closing the context, so iOS
  // WebKit (where close() can lag) doesn't keep the old context alive on rapid
  // toggle and stack contexts.
  for(const s of _srcs){try{s.stop()}catch(e){}}
  _srcs=[];
  if(_ac){_ac.close();_ac=null}
  _chain=null;_hpfNode=null;_lpfNode=null;_hpfOn=false;_lpfOn=false;
  listen.innerHTML="&#128264; listen live";
}
// Clip playback: download the whole WAV, decode it, and play through a WebAudio
// AudioContext. WebAudio (not <audio>) because the context is unlocked
// SYNCHRONOUSLY in the click gesture below, so play() isn't blocked by the loss
// of user-activation that happens after the download await - which made the
// first click silently fail (you had to click again). The device doesn't do
// HTTP Range anyway, so we fetch the whole clip. window._clipAudio is a truthy
// "playing" sentinel so table re-renders pause while a clip plays.
var _clipAc=null,_clipSrc=null,_clipBtn=null;
function _clipBtnIcon(b,html){if(b){b.innerHTML=html;b.disabled=false}}
// Download a clip's bytes, retrying TRANSIENT failures with backoff. The device
// serves clips off the shared SD bus and returns 503 "sd busy, retry" when a
// recorder holds the lock (common - clips are written constantly); a brief stall
// can also truncate the body, which makes fetch/arrayBuffer reject. Both are
// transient, so we retry a handful of times (~6s total) instead of failing the
// click - which is why playback used to need many manual clicks + showed the
// warning sign. A genuine hard error (404) stops immediately. `live()` lets us
// bail the moment the user clicks a different clip.
async function fetchClipBuf(url,live){
  let err,delay=300;
  for(let i=0;i<6;i++){
    if(!live())throw 0;                       // superseded by another click
    let r;
    try{r=await fetch(url)}catch(e){err=e}    // network error: retry
    if(r){
      if(r.ok){
        try{return await r.arrayBuffer()}     // success
        catch(e){err=e}                       // truncated/aborted body: retry
      }else if(r.status==503){err=new Error("sd busy")} // transient: retry
      else throw new Error("HTTP "+r.status); // 404 etc: hard fail, stop now
    }
    if(!live())throw 0;
    await new Promise(s=>setTimeout(s,delay));
    delay=Math.min(delay*1.6,1500);
  }
  throw err||new Error("clip unavailable");
}
async function playClip(url,btn){
  if(btn&&_clipBtn===btn){stopClip();return} // same button: toggle off (incl. mid-load)
  stopClip();
  _clipBtn=btn;
  if(btn){btn.innerHTML="<span class='spin'></span>";btn.disabled=true}
  try{
    // Unlock the context in the gesture, BEFORE any await. Re-create it if a
    // prior _clipRelease() (tab hidden) closed it, else decode would throw.
    if(!_clipAc||_clipAc.state=="closed")_clipAc=new (window.AudioContext||window.webkitAudioContext)();
    _clipAc.resume();
    const data=await fetchClipBuf(url,()=>_clipBtn===btn);
    if(_clipBtn!==btn)return;                 // cancelled while downloading
    const audio=await _clipAc.decodeAudioData(data);
    if(_clipBtn!==btn)return;                 // cancelled while decoding
    const src=_clipAc.createBufferSource();
    src.buffer=audio;src.connect(_clipAc.destination);
    src.onended=()=>{if(_clipSrc===src)stopClip()};
    src.start(0);
    _clipSrc=src;window._clipAudio=src;       // sentinel for the table-rebuild guards
    _clipBtnIcon(btn,"&#9209;");              // ■ stop
  }catch(e){
    if(e===0)return;                          // superseded by another click: leave its state alone
    // Never fail silently. A 404 means the clip was evicted out from under us:
    // the OLDEST clips churn constantly under the SD cap, so a row in a list that
    // was fetched seconds ago can already be gone. Drop the dead row (refresh)
    // and show ✖; any other failure flashes ⚠.
    if(_clipSrc){try{_clipSrc.stop()}catch(_){}}
    _clipSrc=null;window._clipAudio=null;
    const gone=/\b404\b/.test(e&&e.message||"");
    const b=_clipBtn;_clipBtn=null;
    if(b){b.disabled=false;b.innerHTML=gone?"&#10006;":"&#9888;";setTimeout(()=>{if(_clipBtn!==b)b.innerHTML="&#9654;"},1500)}
    if(gone){if(typeof refreshSd=="function")refreshSd();if(typeof refreshClips=="function")refreshClips()}
  }
}
function stopClip(){
  if(_clipSrc){try{_clipSrc.onended=null;_clipSrc.stop()}catch(e){}_clipSrc=null}
  window._clipAudio=null;
  if(_clipBtn){_clipBtnIcon(_clipBtn,"&#9654;");_clipBtn=null}
}
// Release the audio HW when the page goes away / is hidden: stop any clip and
// fully close the long-lived context (it's lazily re-created on next playClip).
function _clipRelease(){stopClip();if(_clipAc){try{_clipAc.close()}catch(e){}_clipAc=null}}
window.addEventListener("pagehide",_clipRelease);
document.addEventListener("visibilitychange",()=>{if(document.hidden)_clipRelease()});
)rawliteral";

// Live A/V page: MJPEG stream + live audio side by side. True muxing isn't
// feasible on this hardware; played together they sit within a few hundred
// ms of each other.
static const char LIVE_PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>)rawliteral" DEVICE_NAME R"rawliteral( - Live</title>
<style>
body{margin:0 auto;padding:16px;max-width:760px;background:#14171c;color:#dde3ea;font-family:-apple-system,system-ui,sans-serif}
#bar{display:flex;gap:6px;align-items:center;margin:4px 0 16px;flex-wrap:wrap}
#bar b{margin-right:10px}
#bar a,#bar span{color:#8a94a0;text-decoration:none;padding:6px 12px;border-radius:8px;font-size:.9em}
#bar span.cur{background:#1d2229;color:#dde3ea}
#bar button{margin-left:auto}
button{background:#3d9df0;color:#fff;border:0;border-radius:8px;padding:8px 14px;font-size:.95em;cursor:pointer}
img{width:100%;display:block;border-radius:10px;background:#000;min-height:120px}
@media(max-width:600px){
  body{padding:10px}
  #bar{gap:4px}#bar b{width:100%;margin:0 0 2px}
}
</style></head><body>
<div id="bar"><b>)rawliteral" DEVICE_NAME R"rawliteral(</b>
  <a href="/">Clips</a><span class="cur">Live</span><a href="/camera">Camera</a><a href="/rec">Record</a><a href="/wifi">WiFi</a><a href="/docs">Docs</a><a href="/jpg">Snapshot</a>
  <span id="fps" style="color:#5df0a0"></span>
  <label id="motwrap" style="padding:0;font-size:.85em;color:#8a94a0"><input type="checkbox" id="moton" onchange="motPoll()"> motion</label>
  <button id="listen" onclick="toggleListen()">&#128264; listen live</button>
</div>
<div style="position:relative">
  <img id="cam" alt="live stream">
  <canvas id="mot" style="position:absolute;inset:0;width:100%;height:100%;pointer-events:none;display:none"></canvas>
</div>
<script src="/ui.js"></script>
<script>
// Robust MJPEG: reads the stream via fetch() and auto-reconnects on stall/drop
// (a bare <img> would freeze on a half-decoded frame forever). Self-pauses while
// the tab is hidden; audio keeps playing.
const camStream=mjpegStream(cam,"/mjpeg/1");camStream.start();
setInterval(()=>{
  if(document.hidden)return;
  fetch("/camera/status").then(r=>r.json()).then(s=>{
    fps.textContent=s.fps>0?"cam "+s.fps.toFixed(1)+(s.net_fps>0?" · net "+s.net_fps.toFixed(1):"")+" fps":"";
  }).catch(()=>{});
},2000);
// Motion debug overlay: poll the detector's changed-block map and paint it
// over the stream. Polling itself keeps analysis alive on the device, so
// this works even with auto-trigger off. Hidden if the board lacks video.
let _motMiss=0,_motMap=null,_motMapAt=0;
function drawMot(m){
  // The device's analysis task can briefly miss its slot under streaming load,
  // returning an empty map for a poll or two. Reuse the last good map for a
  // short grace period so the overlay doesn't flicker to "warming up".
  if(m.map){_motMap=m.map;_motMapAt=Date.now()}
  else if(_motMap&&Date.now()-_motMapAt<3000)m.map=_motMap;
  mot.style.display="block";
  mot.width=cam.clientWidth;mot.height=cam.clientHeight;
  const ctx=mot.getContext("2d");
  ctx.clearRect(0,0,mot.width,mot.height);
  ctx.font="13px monospace";
  const bit=(hex,b)=>(parseInt(hex.substr((b>>3)*2,2),16)>>(b&7))&1;
  if(!m.bx){ctx.fillStyle="#8a94a0";ctx.fillText("motion: warming up",10,20);return}
  const bw=mot.width/m.bx,bh=mot.height/m.by;
  if(m.mask){ // user-masked blocks (edited on the camera page)
    ctx.fillStyle="rgba(61,157,240,.3)";
    for(let b=0;b<m.bx*m.by;b++)
      if(bit(m.mask,b))ctx.fillRect((b%m.bx)*bw,((b/m.bx)|0)*bh,bw,bh);
  }
  if(!m.map){ctx.fillStyle="#8a94a0";ctx.fillText("motion: warming up",10,20);return}
  ctx.fillStyle="rgba(240,93,93,.22)";ctx.strokeStyle="rgba(240,93,93,.85)";ctx.lineWidth=2;
  for(let b=0;b<m.bx*m.by;b++){
    if(bit(m.map,b)){
      const x=(b%m.bx)*bw,y=((b/m.bx)|0)*bh;
      ctx.fillRect(x,y,bw,bh);ctx.strokeRect(x,y,bw,bh);
    }
  }
  ctx.fillStyle=m.global?"#f0c95d":m.level>=m.thresh?"#f05d5d":"#5df0a0";
  ctx.fillText("motion "+m.level+"/"+m.thresh+(m.global?" lighting change":"")
    +(m.enabled?"":" (trigger off)"),10,20);
}
async function motPoll(){
  if(document.hidden)return;
  if(!moton.checked){mot.style.display="none";return}
  try{
    const r=await fetch("/video/motion");
    if(!r.ok)throw 0;
    drawMot(await r.json());_motMiss=0;
  }catch(e){if(++_motMiss>2){motwrap.style.display="none";mot.style.display="none"}}
}
setInterval(motPoll,500);
</script>
<script src="/audio/player.js"></script>
</body></html>)rawliteral";

struct ClipSlot {
  uint8_t *buf;       // WAV header + PCM, PSRAM
  size_t   len;       // valid bytes in buf
  uint32_t id;        // monotonic clip id (0 = never used)
  uint32_t tsMs;      // millis() at trigger time, for correlating with motion events
  uint32_t sdIdx;     // reserved SD file number, UINT32_MAX if not saved
  uint8_t  source;
  uint16_t trigRms;   // RMS at trigger, post-gain (how loud the event was)
  uint16_t trigThr;   // effective trigger threshold at that moment
  uint16_t trigHz;    // band that fired, Hz (spectral trigger only; 0 otherwise)
  volatile bool ready;
  volatile int  busy; // HTTP readers currently streaming this slot
};

// Trigger event log: timestamps + clip/SD identity for plot markers.
// RAM-only (no RTC), so it covers events since boot.
#define EVENT_LOG_SIZE 2048
struct ClipEvent {
  uint32_t tsMs;
  uint32_t id;
  uint32_t sdIdx; // UINT32_MAX = RAM only
  uint8_t  source;
  uint16_t trigRms; // RMS at trigger, post-gain
  uint16_t trigThr; // effective threshold at trigger
  uint16_t trigHz;  // band that fired, Hz (spectral trigger only; 0 otherwise)
};
static ClipEvent *eventLog = NULL;
static volatile uint32_t eventWrite = 0;
static volatile uint32_t eventCount = 0;

static portMUX_TYPE audioMux = portMUX_INITIALIZER_UNLOCKED;

static int16_t *ring = NULL;
static size_t   ringWrite = 0;     // next sample index to write

static ClipSlot slots[MAX_CLIP_SLOTS];   // buf == NULL beyond clipSlotCount
static volatile int clipSlotCount = 0;

// In-flight recording state (guarded by audioMux)
static volatile bool recording = false;
static size_t   triggerPos = 0;
static long     postRemaining = 0; // samples of post-roll still to capture
static uint32_t clipCounter = 0;
static uint32_t activeId = 0;
static uint32_t activeTs = 0;
static uint8_t  activeSource = TRIGGER_SOURCE_AUTO;
static uint16_t activeTrigRms = 0; // RMS + effective threshold captured at the
static uint16_t activeTrigThr = 0; // moment the active clip was triggered
static uint16_t activeTrigHz = 0;  // triggering band freq, Hz (spectral only)

// Runtime config
static volatile uint32_t clipMs = DEFAULT_CLIP_MS;
static volatile uint32_t prerollMs = DEFAULT_PREROLL_MS;
static volatile float    thresholdFactor = DEFAULT_THRESHOLD_FACTOR;
static volatile int      trigAlgo = TRIG_ALGO_FACTOR; // 0=floor*factor, 1=mean+k*sigma
static volatile float    trigK = DEFAULT_TRIG_K;      // k: sigma multiplier (statistical algo)
static volatile float    statWinMin = DEFAULT_STAT_WIN_MIN; // mean/sigma averaging window (min)
static volatile float    manualThresholdRms = 0.0f; // 0 = adaptive (floor * factor)
static volatile float    micGain = DEFAULT_MIC_GAIN;
// Low-pass filter (HF "static" rejection). The audio task owns the actual
// biquad State/Coeffs and recomputes them when it sees lpfCutoffHz change;
// these two scalars are the only thing the config handler writes (each an
// aligned single-word store, so no lock is needed across the task boundary).
static volatile bool     lpfEnabled = false;
static volatile float    lpfCutoffHz = DEFAULT_LPF_CUTOFF_HZ;
static volatile bool     hpfEnabled = false;
static volatile float    hpfCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
// Per-band ("spectral") trigger settings; band state lives near the FFT code.
static volatile bool     spectralTrigEnabled = false;
static volatile float    spectralFactor = DEFAULT_SPECTRAL_FACTOR;
static volatile float    spectralMinMag = DEFAULT_SPECTRAL_MIN_MAG;
static volatile float    bandFloorTcMin = DEFAULT_BAND_TC_MIN; // band floor EMA window (min)
static volatile uint32_t spectroMinutes = DEFAULT_SPECTRO_MIN; // heatmap window length
// Absolute floor under the adaptive threshold, in post-gain RMS units like
// every other level in the UI. Keeps a quiet room's floor*factor from
// producing a hair-trigger threshold; 0 disables the clamp (pure adaptive).
static volatile float    minTrigRms = MIN_TRIGGER_RMS_UNITY * DEFAULT_MIC_GAIN;
static volatile float    habituateMin = DEFAULT_HABITUATE_MIN; // sustained-sound fade window (min; 0=off)
// Master toggle: when off the mic is still drained (so re-enable is
// seamless) but nothing is processed - no levels, no triggers from any
// source, no live stream. Lets the device run video-only, matching boards
// that ship without a mic.
static volatile bool     audioEnabled = true;
static volatile bool     autoTriggerEnabled = true;

// Telemetry
static volatile float currentRms = 0.0f;
static volatile float noiseFloor = 0.0f;
static float noiseVar = 0.0f; // variance for the statistical algo (its mean = noiseFloor)

// The single trigger threshold every site must agree on (audioTask gate, the
// history plot, /audio/status, clip-event snapshots): the selected algorithm's
// adaptive value, with a manual override winning when set. noiseFloor is the
// floor (factor algo) or the running mean (statistical algo).
static float currentThreshold() {
  float adaptive = (trigAlgo == TRIG_ALGO_STAT)
      ? audsp::statisticalThreshold(noiseFloor, noiseVar, trigK, minTrigRms)
      : audsp::adaptiveThreshold(noiseFloor, thresholdFactor, minTrigRms);
  return audsp::effectiveThreshold(adaptive, manualThresholdRms);
}
static volatile uint32_t lastClipEndMs = 0;
static volatile uint32_t warmupUntilMs = WARMUP_MS;

// Retention window for the amplitude/threshold history, in minutes (NVS-backed).
// The bucket duration spreads this window over the fixed HIST_BUCKETS columns,
// so a longer window trades resolution for span at constant memory. Changing it
// re-buckets the ring to the new cadence (histRebucketReq) so collected history
// is preserved - never wiped - across a window change.
static volatile uint32_t histWindowMin = DEFAULT_HIST_WIN_MIN;
static volatile bool     histRebucketReq = false;   // window changed: capture task re-buckets the ring
static volatile uint32_t histRebucketOldMs = 0;     // cadence the ring was recorded at

static inline uint32_t histBucketMsFor(uint32_t winMin) {
  uint32_t b = (uint32_t)((uint64_t)winMin * 60000ULL / HIST_BUCKETS);
  return b < 250 ? 250 : b; // floor so cosf/sinf-free integer math stays sane
}
static inline uint32_t histBucketMs() { return histBucketMsFor(histWindowMin); }

// Amplitude history ring (peak RMS per bucket, newest at histWrite-1).
// thrHistBuf records the effective trigger threshold at each bucket close
// so the plot can show how the adaptive level moved over time.
static uint16_t *histBuf = NULL;
static uint16_t *thrHistBuf = NULL;
static volatile uint32_t histWrite = 0;
static volatile uint32_t histCount = 0;
// Buckets closed since boot, bumped atomically with histWrite. This is the
// pooling-grid index for /audio/history (planAlignedPool): it must tick at the
// same instant as the write index or polls landing between a wall-clock tick
// and the ring's own close regroup every column (see history.h). Restore and
// re-bucket jump histWrite without touching this; a one-time regroup then is
// fine - the data itself changed.
static volatile uint32_t histSeq = 0;
static volatile bool histRestoring = false; // audioTask skips bucket stores during rebuild
static float bucketPeak = 0.0f;
static uint32_t bucketStartMs = 0;

// Retention window changed: re-bucket the amplitude + threshold rings from the
// recorded cadence to the current one (history.h resampleMaxPool) so the plot
// keeps its history. Runs in the capture task (the rings' only writer); HTTP
// readers snapshot indices under audioMux and read cells unlocked, so at worst
// one poll draws a torn line - same accepted exposure as sdRestoreHistory.
// Scratch alloc failure falls back to the old restart-the-ring behavior (n=0).
static void histRebucketRings() {
  uint32_t oldMs = histRebucketOldMs, newMs = histBucketMs();
  uint32_t cnt, w;
  portENTER_CRITICAL(&audioMux);
  cnt = histCount;
  w = histWrite;
  portEXIT_CRITICAL(&audioMux);
  if (oldMs == newMs) return; // e.g. both windows floor to the 250ms minimum
  uint32_t n = 0;
  uint16_t *tmp = (cnt && oldMs && histBuf) ? (uint16_t *)ps_malloc(HIST_BUCKETS * 2) : NULL;
  if (tmp) {
    n = history::resampleMaxPool(histBuf, HIST_BUCKETS, w, cnt, oldMs, newMs, tmp, HIST_BUCKETS);
    memcpy(histBuf, tmp, n * 2);
    if (thrHistBuf) {
      history::resampleMaxPool(thrHistBuf, HIST_BUCKETS, w, cnt, oldMs, newMs, tmp, HIST_BUCKETS);
      memcpy(thrHistBuf, tmp, n * 2);
    }
    free(tmp);
  }
  portENTER_CRITICAL(&audioMux);
  histWrite = n % HIST_BUCKETS;
  histCount = n;
  portEXIT_CRITICAL(&audioMux);
}

static WebServer *audioServer = NULL;
static TaskHandle_t tAudio = NULL;

// SD persistence: finalized PSRAM clips are queued (by slot index) to a
// low-priority writer task so capture and HTTP never block on the card.
// All SD/SPI access is serialized through sdMutex.
// Live PCM fan-out: /audio/stream subscribers, fed from the same ring the
// clip extractor uses. A dedicated task writes the sockets so a slow
// listener can never stall capture.
static QueueHandle_t audioStreamClients = NULL;
static TaskHandle_t tAudioStream = NULL;

static volatile bool sdAvailable = false;
// Set via sdRequestBootSkip() when the previous boot hung inside init: a wedged
// card can block an SPI transaction forever (no timeout in the SD layer), so
// this boot runs without SD and the auto-remount tick stays off - only an
// explicit /sd/remount (or a reboot) retries the card.
static volatile bool sdBootSkip = false;
static volatile bool saveToSd = true;
static volatile uint32_t sdMaxClips = 100; // delete-oldest cap; 0 = unlimited
static uint32_t sdFileIndex = 0;       // next clip_NNNNN.wav number
static SemaphoreHandle_t sdMutex = NULL;

// ---- SD robustness (auto-mount / auto-recover) ----
// The card sometimes loses the power-up race at cold boot and has been seen to
// drop out mid-run (heat - the die runs hot and cards are only rated ~85C).
// sdInit() retries the mount a few times with settle delays, and a health tick
// in the writer task probes the card and silently re-mounts it after a dropout
// so no physical re-seat is needed.
#define SD_MOUNT_ATTEMPTS 4      // cold-boot mount tries before giving up
#define SD_SCAN_MAX_ENTRIES 20000 // boot-scan bound: a corrupt FAT directory
                                  // chain can loop, feeding entries forever
#define SD_PROBE_MS       5000   // liveness-probe cadence while mounted
#define SD_RECOVER_MIN_MS 2000   // first auto-remount delay after a dropout
#define SD_RECOVER_MAX_MS 30000  // backoff ceiling between remount attempts
#define SD_CAP_REFRESH_MS 60000  // usedBytes()/totalBytes() scan the FAT (slow on
                                 // big cards) - refresh the /audio/status capacity
                                 // at most this often, NOT on every liveness probe
// Combined-budget backstop: per-category caps (audio/video clip counts, the
// continuous MB cap) don't see each other, so on a small/misconfigured card the
// categories can collectively fill it -> write failures -> remount thrash. Keep
// at least SD_FREE_FLOOR_MB free; below it, continuous recording pauses NEW
// segments (triggered event clips still save). Hysteresis avoids oscillating at
// the boundary as the continuous pruner frees space.
#define SD_FREE_FLOOR_MB  256
#define SD_FREE_HYST_MB   128
static uint32_t sdNextProbeMs = 0;                 // when the next tick acts
static uint32_t sdRecoverDelayMs = SD_RECOVER_MIN_MS; // current backoff
static volatile uint32_t sdDropCount = 0;          // times a mounted card dropped
static volatile uint32_t sdLastDropMs = 0;         // millis() of the last drop
static volatile uint8_t  sdMountMhz = 0;           // SPI clock the card mounted at (20 or 4)
// Capacity cached by the writer task (under sdMutex) so /audio/status never
// touches the SD/SPI bus from the single-threaded server thread - doing so
// raced the writer/recorder, which could wedge the bus and hang the server.
static volatile uint64_t sdUsedMbCache = 0, sdTotalMbCache = 0;
static volatile bool sdLowSpaceFlag = false; // free space below the safety floor

// The recovery timing/backoff is a pure policy (sd_health.h, unit-tested); these
// adapters view the three globals as its State so the policy stays the single
// source of truth and sdAvailable remains the volatile other tasks read.
static const sdhealth::Config sdHCfg = {SD_PROBE_MS, SD_RECOVER_MIN_MS, SD_RECOVER_MAX_MS};
static sdhealth::State sdHLoad() {
  return {(bool)sdAvailable, sdNextProbeMs, sdRecoverDelayMs};
}
static void sdHStore(const sdhealth::State &s) {
  sdAvailable = s.available;
  sdNextProbeMs = s.nextProbeMs;
  sdRecoverDelayMs = s.recoverDelayMs;
}

// Flag a mounted card as gone and schedule an immediate remount attempt.
static void sdMarkDown() {
  if (sdAvailable) {
    sdDropCount++;
    metricsEvent(metrics::M_SDDROP);
    sdLastDropMs = millis();
    dlog(DLOG_ERR, "SD DROPPED (#%lu this session)", (unsigned long)sdDropCount);
  }
  sdhealth::State st = sdHLoad();
  sdhealth::markDown(st, sdHCfg, millis());
  sdHStore(st);
}
static QueueHandle_t sdWriteQueue = NULL;
static TaskHandle_t tSdWriter = NULL;

void sdRequestBootSkip() { sdBootSkip = true; }

// ---- Settings persistence (NVS) ----
// Loaded once at init; saved on every /audio/config change. NVS skips
// writes when the value is unchanged, so wear is a non-issue here.
static int nvsSlots = DEFAULT_CLIP_SLOTS;

static void loadSettings() {
  Preferences p;
  if (!p.begin("audio", true)) return; // first boot: namespace doesn't exist yet
  // Clamp the same bounds /audio/config enforces: a stored clip_ms > MAX_CLIP_MS
  // would overrun a SLOT_BYTES slot in finalizeClip's memcpy. Re-apply the
  // preroll guard AFTER clamping clipMs.
  clipMs = audsp::clampClipMs(p.getUInt("clip_ms", clipMs), MAX_CLIP_MS);
  prerollMs = p.getUInt("pre_ms", prerollMs);
  if (prerollMs >= clipMs) prerollMs = clipMs / 2;
  thresholdFactor = p.getFloat("factor", thresholdFactor);
  if (thresholdFactor < 1.2f || thresholdFactor > 20.0f) thresholdFactor = DEFAULT_THRESHOLD_FACTOR;
  trigAlgo = p.getInt("algo", trigAlgo);
  if (trigAlgo != TRIG_ALGO_FACTOR && trigAlgo != TRIG_ALGO_STAT) trigAlgo = TRIG_ALGO_FACTOR;
  trigK = p.getFloat("trigk", trigK);
  if (trigK < 0.5f || trigK > 12.0f) trigK = DEFAULT_TRIG_K;
  // New key "statwinm" (minutes); the old "statwin" stored seconds, so ignore it.
  statWinMin = p.getFloat("statwinm", statWinMin);
  if (statWinMin < STAT_WIN_MIN_MIN || statWinMin > STAT_WIN_MIN_MAX) statWinMin = DEFAULT_STAT_WIN_MIN;
  manualThresholdRms = p.getFloat("mthr", manualThresholdRms);
  if (manualThresholdRms < 0.0f || manualThresholdRms > 32767.0f) manualThresholdRms = 0.0f;
  micGain = p.getFloat("gain", micGain);
  if (micGain < MIC_GAIN_MIN || micGain > MIC_GAIN_MAX) micGain = DEFAULT_MIC_GAIN;
  minTrigRms = p.getFloat("minthr", minTrigRms);
  if (minTrigRms < 0.0f || minTrigRms > 32767.0f) minTrigRms = MIN_TRIGGER_RMS_UNITY * DEFAULT_MIC_GAIN;
  habituateMin = p.getFloat("habmin", habituateMin);
  if (habituateMin < 0.0f || habituateMin > HABITUATE_MAX_MIN) habituateMin = DEFAULT_HABITUATE_MIN;
  lpfEnabled = p.getBool("lpf", lpfEnabled);
  lpfCutoffHz = p.getFloat("lpf_hz", lpfCutoffHz);
  if (lpfCutoffHz < LPF_CUTOFF_MIN || lpfCutoffHz > LPF_CUTOFF_MAX) lpfCutoffHz = DEFAULT_LPF_CUTOFF_HZ;
  hpfEnabled = p.getBool("hpf", hpfEnabled);
  hpfCutoffHz = p.getFloat("hpf_hz", hpfCutoffHz);
  if (hpfCutoffHz < HPF_CUTOFF_MIN || hpfCutoffHz > HPF_CUTOFF_MAX) hpfCutoffHz = DEFAULT_HPF_CUTOFF_HZ;
  spectralTrigEnabled = p.getBool("strig", spectralTrigEnabled);
  spectralFactor = p.getFloat("sfactor", spectralFactor);
  if (spectralFactor < 1.5f || spectralFactor > 100.0f) spectralFactor = DEFAULT_SPECTRAL_FACTOR;
  spectralMinMag = p.getFloat("sminmag", spectralMinMag);
  if (spectralMinMag < 0.0f || spectralMinMag > 1e7f) spectralMinMag = DEFAULT_SPECTRAL_MIN_MAG;
  bandFloorTcMin = p.getFloat("bandtc", bandFloorTcMin);
  if (bandFloorTcMin < BAND_TC_MIN_MIN || bandFloorTcMin > BAND_TC_MAX_MIN) bandFloorTcMin = DEFAULT_BAND_TC_MIN;
  spectroMinutes = p.getUInt("spectromin", spectroMinutes);
  if (spectroMinutes < SPECTRO_MIN_MINUTES || spectroMinutes > SPECTRO_MAX_MINUTES) spectroMinutes = DEFAULT_SPECTRO_MIN;
  histWindowMin = p.getUInt("histwin", histWindowMin);
  if (histWindowMin < HIST_WIN_MIN_MIN || histWindowMin > HIST_WIN_MAX_MIN) histWindowMin = DEFAULT_HIST_WIN_MIN;
  audioEnabled = p.getBool("en", audioEnabled);
  autoTriggerEnabled = p.getBool("auto", autoTriggerEnabled);
  saveToSd = p.getBool("sd", saveToSd);
  sdMaxClips = p.getUInt("sdmax", sdMaxClips);
  nvsSlots = p.getInt("slots", nvsSlots);
  p.end();
}

static void saveSettings() {
  Preferences p;
  if (!p.begin("audio", false)) return;
  p.putUInt("clip_ms", clipMs);
  p.putUInt("pre_ms", prerollMs);
  p.putFloat("factor", thresholdFactor);
  p.putInt("algo", trigAlgo);
  p.putFloat("trigk", trigK);
  p.putFloat("statwinm", statWinMin);
  p.putFloat("mthr", manualThresholdRms);
  p.putFloat("gain", micGain);
  p.putFloat("minthr", minTrigRms);
  p.putFloat("habmin", habituateMin);
  p.putBool("lpf", lpfEnabled);
  p.putFloat("lpf_hz", lpfCutoffHz);
  p.putBool("hpf", hpfEnabled);
  p.putFloat("hpf_hz", hpfCutoffHz);
  p.putBool("strig", spectralTrigEnabled);
  p.putFloat("sfactor", spectralFactor);
  p.putFloat("sminmag", spectralMinMag);
  p.putFloat("bandtc", bandFloorTcMin);
  p.putUInt("spectromin", spectroMinutes);
  p.putUInt("histwin", histWindowMin);
  p.putBool("en", audioEnabled);
  p.putBool("auto", autoTriggerEnabled);
  p.putBool("sd", saveToSd);
  p.putUInt("sdmax", sdMaxClips);
  p.putInt("slots", clipSlotCount);
  p.end();
}

// busy must be 0 on both sides (caller checks before sorting)
static void swapSlots(ClipSlot &a, ClipSlot &b) {
  uint8_t *buf = a.buf;   a.buf = b.buf;       b.buf = buf;
  size_t len = a.len;     a.len = b.len;       b.len = len;
  uint32_t id = a.id;     a.id = b.id;         b.id = id;
  uint32_t ts = a.tsMs;   a.tsMs = b.tsMs;     b.tsMs = ts;
  uint32_t si = a.sdIdx;  a.sdIdx = b.sdIdx;   b.sdIdx = si;
  uint8_t src = a.source; a.source = b.source; b.source = src;
  bool r = a.ready;       a.ready = b.ready;   b.ready = r;
}

/**
 * Grow/shrink the clip store. Growth stops if it would eat into the PSRAM
 * reserve; shrink keeps the newest clips (they're compacted into the
 * surviving low slots first) and stops at a slot that's mid-download.
 * SD files are never affected. Returns actual count.
 */
static int setClipSlotCount(int want) {
  if (want < 1) want = 1;
  if (want > MAX_CLIP_SLOTS) want = MAX_CLIP_SLOTS;

  if (clipSlotCount > want) {
    // Sort newest-first so the slots about to be freed hold the oldest
    // clips. Skipped if anything is pinned (downloads finish in <1s).
    portENTER_CRITICAL(&audioMux);
    bool anyBusy = false;
    for (int i = 0; i < clipSlotCount; i++) {
      if (slots[i].busy > 0) { anyBusy = true; break; }
    }
    if (!anyBusy) {
      for (int i = 0; i < clipSlotCount - 1; i++) {
        int best = i;
        for (int j = i + 1; j < clipSlotCount; j++) {
          uint32_t kj = slots[j].ready ? slots[j].id : 0;
          uint32_t kb = slots[best].ready ? slots[best].id : 0;
          if (kj > kb) best = j;
        }
        if (best != i) swapSlots(slots[i], slots[best]);
      }
    }
    portEXIT_CRITICAL(&audioMux);
  }

  while (clipSlotCount < want) {
    if (ESP.getFreePsram() < PSRAM_RESERVE_BYTES + SLOT_BYTES) break;
    uint8_t *buf = (uint8_t *)ps_malloc(SLOT_BYTES);
    if (buf == NULL) break;
    int i = clipSlotCount;
    slots[i].buf = buf;
    slots[i].ready = false;
    slots[i].busy = 0;
    slots[i].id = 0;
    portENTER_CRITICAL(&audioMux);
    clipSlotCount = i + 1;
    portEXIT_CRITICAL(&audioMux);
  }

  while (clipSlotCount > want) {
    int i = clipSlotCount - 1;
    uint8_t *buf = NULL;
    portENTER_CRITICAL(&audioMux);
    if (slots[i].busy == 0) {
      buf = slots[i].buf;
      slots[i].buf = NULL;
      slots[i].ready = false;
      clipSlotCount = i;
    }
    portEXIT_CRITICAL(&audioMux);
    if (buf == NULL) break; // slot is streaming to a client right now
    free(buf);
  }
  return clipSlotCount;
}

/** How many slots could exist while keeping the PSRAM reserve intact. */
static int recommendedSlotCount() {
  uint32_t freePs = ESP.getFreePsram();
  int rec = clipSlotCount;
  if (freePs > PSRAM_RESERVE_BYTES) rec += (freePs - PSRAM_RESERVE_BYTES) / SLOT_BYTES;
  return rec > MAX_CLIP_SLOTS ? MAX_CLIP_SLOTS : rec;
}

/**
 * Mount the SD card and resume the persistent file numbering from whatever
 * is already in /clips. Failure is non-fatal: clips just stay PSRAM-only.
 */
// One mount attempt: fully reset the SPI bus, let the card settle, then try a
// fast clock and a conservative fallback for fussy cards. Returns true only
// with a real card present.
static bool sdMountOnce(uint32_t settleMs) {
  SD.end();  // safe no-op on first call; required for remount
  SPI.end(); // drop the bus so a wedged transaction can't poison the retry
  delay(settleMs);
  SPI.begin(SCK, MISO, MOSI, SD_CS_PIN);
  // Try a fast clock first, then a conservative fallback for fussy/slow cards.
  // Record which won: 4 MHz is ~5x slower and a prime suspect for slow clip
  // loads, so it's surfaced in /diag. (40 MHz was tried and negotiates cleanly,
  // but gave 0 throughput gain - the cap is the Arduino-SD/FATFS layer + CPU
  // contention, not the bus - so we stay at the proven 20 MHz tier.)
  if (SD.begin(SD_CS_PIN, SPI, 20000000)) {
    sdMountMhz = 20;
  } else if (SD.begin(SD_CS_PIN, SPI, 4000000)) {
    sdMountMhz = 4;
  } else {
    sdMountMhz = 0;
    return false;
  }
  return SD.cardType() != CARD_NONE;
}

// ---- Persisted per-clip over-threshold metadata --------------------------
// The trigger rms/threshold/Hz live in the RAM event log (lost on reboot), so
// the SD clips table's "Over thr" column would only cover clips since boot. We
// also append one line per saved clip to /clips/clipmeta.csv and mirror it in a
// PSRAM ring, so the column shows for EVERY clip across reboots. Served as JSON
// at /audio/clipmeta (no SD hit on the request - read from the ring). Only clips
// captured by firmware that has this feature get an entry; older ones show "-".
#define CLIPMETA_PATH SD_CLIP_DIR "/clipmeta.csv"
#define CLIPMETA_CAP  1024  // most-recent clips kept in RAM (and after compaction)
struct ClipMeta { uint32_t num; uint16_t rms, thr, hz; };
static ClipMeta *clipMeta = NULL;                 // PSRAM ring
static ClipMeta *clipMetaSnap = NULL;             // PSRAM scratch for lock-free /clips/meta serialize
static volatile uint16_t clipMetaCount = 0, clipMetaHead = 0;
static SemaphoreHandle_t clipMetaMutex = NULL;

static void clipMetaInit() {
  if (!clipMetaMutex) clipMetaMutex = xSemaphoreCreateMutex();
  if (!clipMeta) clipMeta = (ClipMeta *)ps_calloc(CLIPMETA_CAP, sizeof(ClipMeta));
  if (!clipMetaSnap) clipMetaSnap = (ClipMeta *)ps_malloc(CLIPMETA_CAP * sizeof(ClipMeta));
}
static void clipMetaPut(uint32_t num, uint16_t rms, uint16_t thr, uint16_t hz) {
  if (!clipMeta || !clipMetaMutex) return;
  xSemaphoreTake(clipMetaMutex, portMAX_DELAY);
  clipMeta[clipMetaHead] = {num, rms, thr, hz};
  clipMetaHead = (clipMetaHead + 1) % CLIPMETA_CAP;
  if (clipMetaCount < CLIPMETA_CAP) clipMetaCount++;
  xSemaphoreGive(clipMetaMutex);
}
// Add to the ring AND persist. Takes sdMutex itself (callers don't hold it here).
static void clipMetaPersist(uint32_t num, uint16_t rms, uint16_t thr, uint16_t hz) {
  clipMetaPut(num, rms, thr, hz);
  if (!sdAvailable || sdMutex == NULL) return;
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return;
  File f = SD.open(CLIPMETA_PATH, FILE_APPEND);
  if (f) {
    char line[40];
    int n = snprintf(line, sizeof(line), "%u,%u,%u,%u\n", num, rms, thr, hz);
    f.write((const uint8_t *)line, n);
    f.close();
  }
  xSemaphoreGive(sdMutex);
}
// Load the CSV into the ring (keeping the last CLIPMETA_CAP). Compacts the file
// back down if it has grown past the cap. Caller owns SD access (boot/remount).
static void clipMetaLoad() {
  if (!clipMeta || !clipMetaMutex) return;
  xSemaphoreTake(clipMetaMutex, portMAX_DELAY);
  clipMetaCount = 0; clipMetaHead = 0;
  uint32_t lines = 0;
  File f = SD.open(CLIPMETA_PATH, FILE_READ);
  if (f) {
    char line[48];
    while (f.available()) {
      int n = f.readBytesUntil('\n', line, sizeof(line) - 1);
      // If the buffer filled without hitting a newline, this was an over-long /
      // corrupt line: drain to the next '\n' so the following read doesn't start
      // mid-line and feed garbage to sscanf below.
      if (n == (int)sizeof(line) - 1) {
        while (f.available() && f.read() != '\n') {}
      }
      if (n <= 0) continue;
      line[n] = 0;
      unsigned num, rms, thr, hz;
      if (sscanf(line, "%u,%u,%u,%u", &num, &rms, &thr, &hz) == 4) {
        // Range-validate before storing: the three level fields are uint16_t and
        // num is a 0..9999-ish file number; reject anything that won't fit.
        if (rms > 65535 || thr > 65535 || hz > 65535 || num > 0xFFFFFF) continue;
        clipMeta[clipMetaHead] = {num, (uint16_t)rms, (uint16_t)thr, (uint16_t)hz};
        clipMetaHead = (clipMetaHead + 1) % CLIPMETA_CAP;
        if (clipMetaCount < CLIPMETA_CAP) clipMetaCount++;
        lines++;
      }
    }
    f.close();
  }
  // The file accumulated more than we keep: rewrite it compactly (oldest first)
  // so it can't grow without bound over the device's life.
  if (lines > CLIPMETA_CAP) {
    File w = SD.open(CLIPMETA_PATH, FILE_WRITE); // truncate
    if (w) {
      uint16_t start = (clipMetaHead + CLIPMETA_CAP - clipMetaCount) % CLIPMETA_CAP;
      for (uint16_t i = 0; i < clipMetaCount; i++) {
        const ClipMeta &m = clipMeta[(start + i) % CLIPMETA_CAP];
        char line[40];
        int ln = snprintf(line, sizeof(line), "%u,%u,%u,%u\n", m.num, m.rms, m.thr, m.hz);
        w.write((const uint8_t *)line, ln);
      }
      w.close();
    }
  }
  xSemaphoreGive(clipMetaMutex);
}
static void clipMetaClear() {
  if (clipMetaMutex) {
    xSemaphoreTake(clipMetaMutex, portMAX_DELAY);
    clipMetaCount = 0; clipMetaHead = 0;
    xSemaphoreGive(clipMetaMutex);
  }
  if (sdAvailable && sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    SD.remove(CLIPMETA_PATH);
    xSemaphoreGive(sdMutex);
  }
}

// attempts: how many mount tries before giving up. Boot uses SD_MOUNT_ATTEMPTS
// (blocking is fine when single-threaded); the auto-recovery tick passes 1 so
// it never holds sdMutex across a long doomed retry and stalls the web server.
static bool sdInit(int attempts = SD_MOUNT_ATTEMPTS) {
  bool mounted = false;
  for (int attempt = 0; attempt < attempts && !mounted; attempt++) {
    // Give the card longer to power up / recover on each successive try.
    mounted = sdMountOnce(attempt == 0 ? 20 : 120 + attempt * 80);
  }
  if (!mounted) {
    Serial.println("[audio] SD mount failed - check seating and FAT32 format "
                   "(exFAT cards >32GB are not supported); clips will be PSRAM-only");
    return false;
  }
  uint8_t type = SD.cardType();
  Serial.printf("[audio] SD card type %s\n",
                type == CARD_MMC ? "MMC" : type == CARD_SD ? "SDSC" : "SDHC/SDXC");
  if (!SD.exists(SD_CLIP_DIR)) SD.mkdir(SD_CLIP_DIR);

  // The one full scan we accept: seed the next file number AND (re)populate the
  // in-RAM audio clip index so the lists + cap eviction never scan again. Runs at
  // boot and on every remount, so clear the audio kind first to stay idempotent.
  clipIndexClear(CLIP_AUDIO);
  File dir = SD.open(SD_CLIP_DIR);
  if (dir) {
    File entry;
    uint32_t scanned = 0;
    while (scanned < SD_SCAN_MAX_ENTRIES && (entry = dir.openNextFile())) {
      scanned++;
      unsigned idx = 0;
      const char *base = strrchr(entry.name(), '/');
      base = base ? base + 1 : entry.name();
      // .wav only: the .jpg key-frame sidecars share the clip_ prefix and would
      // otherwise be indexed as (1KB) audio clips - the video scan filters on
      // .avi for the same reason.
      if (sscanf(base, "clip_%u", &idx) == 1 && pathsafe::endsWith(base, ".wav")) {
        clipIndexAdd(CLIP_AUDIO, base, entry.size(), (uint32_t)entry.getLastWrite());
      }
      entry.close();
    }
    if (scanned >= SD_SCAN_MAX_ENTRIES)
      dlog(DLOG_ERR, "SD boot scan stopped at %u entries - FAT directory chain "
           "suspect, reformat advised", (unsigned)SD_SCAN_MAX_ENTRIES);
    dir.close();
  }
  // Resume numbering just past the newest existing clip - wrap-aware (the number
  // wraps at CLIP_NUM_WRAP), so this is correct even across the wrap boundary.
  sdFileIndex = clipIndexNextNum(CLIP_AUDIO);
  clipMetaLoad(); // restore the over-threshold ring from /clips/clipmeta.csv

  sdUsedMbCache = SD.usedBytes() / (1024 * 1024);
  sdTotalMbCache = SD.totalBytes() / (1024 * 1024);
  Serial.printf("[audio] SD ready: %lluMB used / %lluMB, next file index %u\n",
                sdUsedMbCache, sdTotalMbCache, sdFileIndex);
  return true;
}

static void contAudSeed(); // defined with the continuous-recording task below

/**
 * Periodic SD watchdog, run from the writer task's idle path. While mounted it
 * does a cheap dir-open every SD_PROBE_MS (real card I/O, so a silent dropout
 * is caught even with nothing being written); once down it keeps retrying the
 * mount with exponential backoff so the card recovers without a power cycle.
 * Continuous audio/video tasks gate on sdIsAvailable(), so they pause on a drop
 * and resume on their own once this re-mounts. Must run holding no locks.
 */
static void sdHealthTick() {
  // Boot-skip mode: no auto-remount. A card that wedged a previous boot can
  // hang SD.begin() on the bus, which would kill this task while it holds
  // sdMutex; retrying is reserved for an explicit /sd/remount.
  if (sdBootSkip) return;
  uint32_t now = millis();
  sdhealth::State st = sdHLoad();
  sdhealth::Action act = sdhealth::nextAction(st, now);
  if (act == sdhealth::NONE) return;

  if (act == sdhealth::PROBE) {
    static uint32_t lastCapMs = 0;
    // The liveness check is a cheap dir-open every SD_PROBE_MS; the capacity
    // figures (usedBytes/totalBytes) scan the whole FAT and are far more
    // expensive, so refresh them only every SD_CAP_REFRESH_MS.
    bool refreshCap = (lastCapMs == 0) || (now - lastCapMs >= SD_CAP_REFRESH_MS);
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File d = SD.open(SD_CLIP_DIR);
    bool alive = (bool)d;
    if (d) d.close();
    if (alive && refreshCap) {
      sdUsedMbCache = SD.usedBytes() / (1024 * 1024); // refresh for /audio/status
      sdTotalMbCache = SD.totalBytes() / (1024 * 1024);
      lastCapMs = now;
    }
    xSemaphoreGive(sdMutex);
    sdhealth::onProbeResult(st, sdHCfg, now, alive);
    sdHStore(st);
    if (!alive) {
      sdDropCount++;
      metricsEvent(metrics::M_SDDROP);
      sdLastDropMs = now;
      dlog(DLOG_ERR, "SD stopped responding (#%lu) - auto-remounting",
           (unsigned long)sdDropCount);
    }
    return;
  }

  // REMOUNT: one quick try. A single attempt holds sdMutex only ~1-2s (vs many
  // seconds for the full retry), so HTTP/recorder tasks aren't stalled for long;
  // the policy's backoff provides the repeated tries over time instead.
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  bool ok = sdInit(1);
  xSemaphoreGive(sdMutex);
  sdhealth::onRemountResult(st, sdHCfg, now, ok);
  sdHStore(st);
  if (ok) {
    contAudSeed(); // re-sync continuous counter past existing files (takes mutex)
    dlog(DLOG_INFO, "SD remounted (%llu/%lluMB) - recording resumed",
         sdUsedMbCache, sdTotalMbCache);
  }
}

#define DIAG_LOG_PATH "/diag.log"
#define DIAG_LOG_MAX  (256 * 1024) // rotate past this so the card can't fill up

/**
 * Mirror any new diag-log entries to /diag.log so history survives a full
 * power-cut (the RTC ring only survives soft resets). Runs only from the
 * writer task's idle path, holding sdMutex - never from an HTTP handler.
 */
static void sdMirrorLog() {
  if (!sdAvailable || !dlogSdPending()) return;
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  if (SD.exists(DIAG_LOG_PATH)) {
    File chk = SD.open(DIAG_LOG_PATH, FILE_READ);
    bool big = chk && chk.size() > DIAG_LOG_MAX;
    if (chk) chk.close();
    if (big) SD.remove(DIAG_LOG_PATH); // simple rotation: start a fresh file
  }
  File f = SD.open(DIAG_LOG_PATH, FILE_APPEND);
  if (f) {
    dlogFlushToFile(f);
    f.close();
  }
  xSemaphoreGive(sdMutex);
}

/**
 * Delete oldest clips (lowest file number) until at most sdMaxClips remain.
 * Runs after every SD write and whenever the cap is lowered.
 *
 * Index-based, NO directory scan: the in-RAM clip index names the oldest file,
 * we SD.remove it and drop it from the index. A full /clips scan can take many
 * seconds on a slow card and holds the SD mutex the whole time, so doing it after
 * every write was the dominant source of "SD busy" stalls.
 */
static void sdEnforceCap() {
  if (!sdAvailable || sdMaxClips == 0) return;
  char name[28];
  while (clipIndexCount(CLIP_AUDIO) > sdMaxClips &&
         clipIndexOldest(CLIP_AUDIO, name, sizeof(name))) {
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    bool removed = SD.remove(String(SD_CLIP_DIR) + "/" + name);
    String thumb = String(SD_CLIP_DIR) + "/" + name;
    thumb.replace(".wav", ".jpg");
    SD.remove(thumb); // key-frame thumbnail sidecar rides along
    xSemaphoreGive(sdMutex);
    clipIndexRemove(name); // drop from the index whether or not the file existed
    if (removed) Serial.printf("[audio] SD cap (%u): removed %s\n", sdMaxClips, name);
  }
}

/**
 * Snapshot both history rings to SD (oldest->newest, with the wall-clock
 * epoch of the newest bucket) so plots survive reboots. Needs NTP: without
 * real time the snapshot couldn't be re-anchored on the next boot.
 * A bucket closing mid-write can tear at most one 2s sample - acceptable.
 */
static void sdSaveHistory() {
  if (!sdAvailable || histBuf == NULL || thrHistBuf == NULL) return;
  if (time(NULL) < 1600000000) return;
  uint32_t cnt, w;
  portENTER_CRITICAL(&audioMux);
  cnt = histCount;
  w = histWrite;
  portEXIT_CRITICAL(&audioMux);
  if (cnt == 0) return;

  uint32_t start = (w + HIST_BUCKETS - cnt) % HIST_BUCKETS;
  uint32_t run1 = cnt < HIST_BUCKETS - start ? cnt : HIST_BUCKETS - start;

  // The whole snapshot is ~170KB; holding sdMutex across it stalls any video
  // clip frame write (and SD downloads) for a few hundred ms every 5 min.
  // Write the amplitude buffer, drop the mutex, then append the threshold
  // buffer - the file is closed before each release so no handle spans the
  // gap, and a video-frame write waiting on the bus gets serviced in between.
  bool ok;
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File f = SD.open(HIST_FILE, FILE_WRITE); // truncates
  ok = (bool)f;
  if (ok) {
    uint32_t hdr[4] = {HIST_MAGIC, (uint32_t)time(NULL), cnt, histWindowMin};
    f.write((uint8_t *)hdr, sizeof(hdr));
    f.write((uint8_t *)(histBuf + start), run1 * 2);
    if (run1 < cnt) f.write((uint8_t *)histBuf, (cnt - run1) * 2);
    f.close();
  }
  xSemaphoreGive(sdMutex);
  if (!ok) return;

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  f = SD.open(HIST_FILE, FILE_APPEND);
  if (f) {
    f.write((uint8_t *)(thrHistBuf + start), run1 * 2);
    if (run1 < cnt) f.write((uint8_t *)thrHistBuf, (cnt - run1) * 2);
    f.close();
  }
  xSemaphoreGive(sdMutex);
}

/**
 * Rebuild the history rings from the last snapshot: saved buckets, then a
 * zero gap for the time the device was down, then whatever live buckets
 * accumulated since boot. Runs once, after NTP sync.
 */
static void sdRestoreHistory() {
  if (!sdAvailable || histBuf == NULL || thrHistBuf == NULL) return;

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File f = SD.open(HIST_FILE, FILE_READ);
  uint32_t hdr[4] = {0, 0, 0, 0};
  bool ok = f && f.read((uint8_t *)hdr, sizeof(hdr)) == (int)sizeof(hdr) && hdr[0] == HIST_MAGIC;
  uint32_t cnt = ok && hdr[2] <= HIST_BUCKETS ? hdr[2] : 0;
  // hdr[3] is the retention window the snapshot was recorded at; only trust it
  // in the valid range (older/corrupt files discard rather than misresample).
  if (ok && (hdr[3] < HIST_WIN_MIN_MIN || hdr[3] > HIST_WIN_MAX_MIN)) cnt = 0;
  uint16_t *amp = NULL, *thr = NULL;
  if (cnt) {
    amp = (uint16_t *)ps_malloc(cnt * 2);
    thr = (uint16_t *)ps_malloc(cnt * 2);
    if (!amp || !thr ||
        f.read((uint8_t *)amp, cnt * 2) != cnt * 2 ||
        f.read((uint8_t *)thr, cnt * 2) != cnt * 2) {
      cnt = 0;
    }
  }
  if (f) f.close();
  xSemaphoreGive(sdMutex);

  // Window changed since the snapshot: re-bucket it to the current cadence
  // instead of discarding (the old behavior deleted a day of history for a
  // settings tweak). Dense arrays: cap == write == count.
  uint32_t savedMs = histBucketMsFor(hdr[3]);
  if (cnt && savedMs != histBucketMs()) {
    uint16_t *amp2 = (uint16_t *)ps_malloc(HIST_BUCKETS * 2);
    uint16_t *thr2 = (uint16_t *)ps_malloc(HIST_BUCKETS * 2);
    if (amp2 && thr2) {
      uint32_t n = history::resampleMaxPool(amp, cnt, cnt, cnt, savedMs,
                                            histBucketMs(), amp2, HIST_BUCKETS);
      history::resampleMaxPool(thr, cnt, cnt, cnt, savedMs, histBucketMs(),
                               thr2, HIST_BUCKETS);
      free(amp); free(thr);
      amp = amp2; thr = thr2;
      cnt = n;
    } else { // scratch alloc failed: keep the old discard behavior
      if (amp2) free(amp2);
      if (thr2) free(thr2);
      cnt = 0;
    }
  }

  uint32_t gap = ((uint32_t)time(NULL) - hdr[1]) * 1000ULL / histBucketMs();
  if (cnt && gap < HIST_BUCKETS) {
    histRestoring = true;
    uint32_t liveCnt;
    portENTER_CRITICAL(&audioMux);
    liveCnt = histCount; // boot-fresh ring: live data sits at [0..liveCnt), unwrapped
    portEXIT_CRITICAL(&audioMux);

    uint32_t zeros = gap > liveCnt ? gap - liveCnt : 0; // downtime not covered by live
    uint32_t keep = cnt;
    if (keep + zeros + liveCnt > HIST_BUCKETS) {
      uint32_t drop = keep + zeros + liveCnt - HIST_BUCKETS;
      keep = drop < keep ? keep - drop : 0;
    }
    if (keep) {
      uint32_t total = keep + zeros + liveCnt;
      memmove(histBuf + keep + zeros, histBuf, liveCnt * 2);
      memset(histBuf + keep, 0, zeros * 2);
      memcpy(histBuf, amp + (cnt - keep), keep * 2);
      memmove(thrHistBuf + keep + zeros, thrHistBuf, liveCnt * 2);
      memset(thrHistBuf + keep, 0, zeros * 2);
      memcpy(thrHistBuf, thr + (cnt - keep), keep * 2);
      portENTER_CRITICAL(&audioMux);
      histWrite = total % HIST_BUCKETS;
      histCount = total;
      portEXIT_CRITICAL(&audioMux);
      Serial.printf("[audio] plot history restored: %u buckets (+%u gap)\n", keep, zeros);
    }
    histRestoring = false;
  }
  if (amp) free(amp);
  if (thr) free(thr);
}

/**
 * SD writer task: drains finalized clips to the card. The slot stays pinned
 * (busy refcount, taken in finalizeClip) until the write completes.
 * Also owns plot-history persistence: restores once NTP comes up, then
 * snapshots every HIST_SAVE_MS.
 */
static void sdSaveSpectro();    // defined with the spectrogram code below
static void sdRestoreSpectro();
static void sdUpdateLowSpace(); // free-space backstop, defined with the public SD API

static void sdWriterTask(void *pvParameters) {
  static bool histRestored = false; // shared so an SD remount doesn't re-restore
  uint32_t lastHistSave = millis();
  int slot;
  for (;;) {
    if (xQueueReceive(sdWriteQueue, &slot, pdMS_TO_TICKS(1000)) != pdTRUE) {
      sdHealthTick(); // probe / auto-remount the card when nothing is queued
      sdUpdateLowSpace(); // refresh the free-space backstop the cont tasks gate on
      if (sdAvailable) {
        sdMirrorLog(); // append new diag-log entries to /diag.log
        if (!histRestored && time(NULL) >= 1600000000) {
          histRestored = true;
          sdRestoreHistory();
          sdRestoreSpectro();
          lastHistSave = millis();
        } else if (millis() - lastHistSave >= HIST_SAVE_MS) {
          lastHistSave = millis();
          sdSaveHistory();
          sdSaveSpectro();
        }
      }
      continue;
    }
    ClipSlot &s = slots[slot];

    // _a/_h/_v suffix encodes the trigger source so files stay
    // self-describing on the card and across reboots.
    char path[48];
    snprintf(path, sizeof(path), SD_CLIP_DIR "/clip_%05u_%c.wav", s.sdIdx,
             s.source == TRIGGER_SOURCE_AUTO ? 'a'
             : s.source == TRIGGER_SOURCE_HTTP ? 'h'
             : s.source == TRIGGER_SOURCE_SPECTRAL ? 's' : 'v');

    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File f = SD.open(path, FILE_WRITE);
    size_t written = 0;
    if (f) {
      written = f.write(s.buf, s.len);
      f.close();
    }
    xSemaphoreGive(sdMutex);

    if (written == s.len) {
      Serial.printf("[audio] clip %u -> %s (%u bytes)\n", s.id, path, (unsigned)written);
      // Index the new file (path is SD_CLIP_DIR "/clip_..."; pass the bare name).
      time_t nowt = time(NULL);
      clipIndexAdd(CLIP_AUDIO, path + sizeof(SD_CLIP_DIR), (uint32_t)written,
                   nowt > 1600000000 ? (uint32_t)nowt : 0);
      // Persist its over-threshold data so the SD table shows it across reboots.
      clipMetaPersist(s.sdIdx, s.trigRms, s.trigThr, s.trigHz);
      // Key-frame thumbnail: give this sound event a "what did the camera see"
      // image (clip_NNNNN_x.jpg). No-op if the camera has no frame; non-fatal.
      char tpath[48];
      snprintf(tpath, sizeof(tpath), SD_CLIP_DIR "/clip_%05u_%c.jpg", s.sdIdx,
               s.source == TRIGGER_SOURCE_AUTO ? 'a'
               : s.source == TRIGGER_SOURCE_HTTP ? 'h'
               : s.source == TRIGGER_SOURCE_SPECTRAL ? 's' : 'v');
      videoSaveKeyThumb(tpath);
    } else {
      Serial.printf("[audio] SD write failed for clip %u (%s) - flagging for remount\n",
                    s.id, path);
      sdMarkDown(); // a failed write means the card is gone/wedged
    }

    portENTER_CRITICAL(&audioMux);
    s.busy--;
    portEXIT_CRITICAL(&audioMux);

    sdEnforceCap();
  }
}

// Thin shell over the pure builder (wav.h, unit-tested): 16 kHz mono 16-bit.
static void writeWavHeader(uint8_t *p, uint32_t pcmBytes) {
  wav::writeHeader(p, pcmBytes, AUDIO_SAMPLE_RATE, 1, 16);
}

/**
 * Begin a clip capture. Safe to call from any task. If a capture is already
 * in flight, returns its id (the window overlaps the new request anyway).
 */
static const char *srcName(uint8_t source); // defined just below

static uint32_t startClip(uint8_t source, uint16_t hz = 0) {
  if (!audioEnabled) return 0; // master toggle off: refuse every trigger source
  uint32_t id;
  bool started = false;
  portENTER_CRITICAL(&audioMux);
  if (recording) {
    id = activeId;
  } else {
    recording = true;
    triggerPos = ringWrite;
    postRemaining = (long)(clipMs - prerollMs) * SAMPLES_PER_MS;
    activeId = ++clipCounter;
    activeTs = millis();
    activeSource = source;
    activeTrigHz = hz;
    // Snapshot how loud this event was vs the trigger threshold, using the same
    // shared math as audioTask. For HTTP/video triggers there's no real
    // crossing, but the ratio is still informative (often < 1x for a manual
    // capture).
    float thr = currentThreshold();
    activeTrigRms = (uint16_t)(currentRms > 65535.0f ? 65535 : currentRms);
    activeTrigThr = (uint16_t)(thr > 65535.0f ? 65535 : thr);
    id = activeId;
    started = true;
  }
  uint16_t evRms = activeTrigRms, evThr = activeTrigThr;
  portEXIT_CRITICAL(&audioMux);

  // Persist every trigger to the diag log so the algorithm's behaviour can be
  // reviewed across reboots (rms vs threshold, source). Only on a real start,
  // not when a trigger arrives mid-clip.
  if (started) {
    dlog(DLOG_INFO, "trig %s clip %u: rms %u / thr %u (%.2fx) floor %.0f",
         srcName(source), id, evRms, evThr,
         evThr ? (float)evRms / evThr : 0.0f, noiseFloor);
    metricsEvent(metrics::M_ATRIG);
  }

  // Cross-trigger video (gated inside videoNotifyTrigger by its own
  // settings). The source guard makes a trigger loop structurally impossible.
  if (started && source != TRIGGER_SOURCE_VIDEO) {
    videoNotifyTrigger(VID_TRIG_AUDIO);
  }
  return id;
}

static const char *srcName(uint8_t source) {
  return source == TRIGGER_SOURCE_AUTO ? "auto"
       : source == TRIGGER_SOURCE_HTTP ? "http"
       : source == TRIGGER_SOURCE_SPECTRAL ? "spectral" : "video";
}

/**
 * Copy the finished clip window out of the ring into a free slot.
 * Runs in the capture task; the ring keeps ~1s of slack beyond MAX_CLIP_MS
 * so the pre-roll is still intact when this runs.
 */
static void finalizeClip() {
  int chosen = -1;
  uint32_t oldest = UINT32_MAX;
  portENTER_CRITICAL(&audioMux);
  for (int i = 0; i < clipSlotCount; i++) {
    if (slots[i].busy > 0) continue;
    uint32_t key = slots[i].ready ? slots[i].id : 0;
    if (key < oldest) { oldest = key; chosen = i; }
  }
  if (chosen >= 0) {
    // busy also pins the slot against setClipSlotCount() freeing it mid-copy
    slots[chosen].ready = false;
    slots[chosen].busy++;
  }
  portEXIT_CRITICAL(&audioMux);

  if (chosen < 0) {
    // every slot is mid-download; drop this clip rather than stall capture
    portENTER_CRITICAL(&audioMux);
    recording = false;
    portEXIT_CRITICAL(&audioMux);
    Serial.println("[audio] all clip slots busy, clip dropped");
    return;
  }

  size_t clipSamples = (size_t)clipMs * SAMPLES_PER_MS;
  size_t prerollSamples = (size_t)prerollMs * SAMPLES_PER_MS;
  size_t start = (triggerPos + RING_SAMPLES - prerollSamples) % RING_SAMPLES;

  ClipSlot &s = slots[chosen];
  uint8_t *pcm = s.buf + WAV_HEADER_BYTES;
  size_t firstRun = RING_SAMPLES - start;
  if (firstRun > clipSamples) firstRun = clipSamples;
  memcpy(pcm, ring + start, firstRun * 2);
  if (firstRun < clipSamples) {
    memcpy(pcm + firstRun * 2, ring, (clipSamples - firstRun) * 2);
  }
  writeWavHeader(s.buf, clipSamples * 2);
  s.len = WAV_HEADER_BYTES + clipSamples * 2;
  s.id = activeId;
  s.tsMs = activeTs;
  s.source = activeSource;
  s.trigRms = activeTrigRms;
  s.trigThr = activeTrigThr;
  s.trigHz = activeTrigHz;

  // Hand our busy ref to the SD writer if it takes the job; it decrements
  // after the card write completes. The file number is reserved here so the
  // event log can reference it immediately.
  bool sdQueued = false;
  s.sdIdx = UINT32_MAX;
  if (sdAvailable && saveToSd) {
    s.sdIdx = sdFileIndex;
    sdFileIndex = (sdFileIndex + 1) % CLIP_NUM_WRAP; // 5-digit names, wrap-aware eviction
    sdQueued = (xQueueSend(sdWriteQueue, &chosen, 0) == pdTRUE);
    if (!sdQueued) s.sdIdx = UINT32_MAX; // burned number; harmless gap
  }

  if (eventLog != NULL) {
    // Store the struct INSIDE the critical section, before bumping the index, so
    // a reader (/audio/events) can never see the bumped count with a torn entry.
    // The store is plain words (no heap), safe inside a portMUX section.
    portENTER_CRITICAL(&audioMux);
    eventLog[eventWrite] = {s.tsMs, s.id, s.sdIdx, s.source, s.trigRms, s.trigThr, s.trigHz};
    eventWrite = (eventWrite + 1) % EVENT_LOG_SIZE;
    if (eventCount < EVENT_LOG_SIZE) eventCount++;
    portEXIT_CRITICAL(&audioMux);
  }

  portENTER_CRITICAL(&audioMux);
  s.ready = true;
  if (!sdQueued) s.busy--;
  recording = false;
  portEXIT_CRITICAL(&audioMux);
  lastClipEndMs = millis();

  Serial.printf("[audio] clip %u saved (%ums, %s trigger, slot %d)\n",
                s.id, clipMs, srcName(s.source), chosen);
}

/**
 * Live-stream fan-out task: follows the ring with its own read cursor and
 * writes fresh PCM to every /audio/stream subscriber. Clients whose TCP
 * buffer stays full are skipped (brief glitch) and dropped if it persists.
 */
static void audioStreamTask(void *pvParameters) {
  static int16_t out[STREAM_CHUNK_SAMPLES];
  size_t readPos = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    portENTER_CRITICAL(&audioMux);
    size_t w = ringWrite;
    portEXIT_CRITICAL(&audioMux);

    if (uxQueueMessagesWaiting(audioStreamClients) == 0) {
      readPos = w; // stay caught up while idle
      continue;
    }

    size_t avail = (w + RING_SAMPLES - readPos) % RING_SAMPLES;
    if (avail > AUDIO_SAMPLE_RATE / 2) {
      // fell badly behind; jump to near-live rather than replaying backlog
      readPos = (w + RING_SAMPLES - STREAM_CHUNK_SAMPLES) % RING_SAMPLES;
      avail = STREAM_CHUNK_SAMPLES;
    }

    while (avail >= STREAM_CHUNK_SAMPLES) {
      size_t first = RING_SAMPLES - readPos;
      if (first > STREAM_CHUNK_SAMPLES) first = STREAM_CHUNK_SAMPLES;
      memcpy(out, ring + readPos, first * 2);
      if (first < STREAM_CHUNK_SAMPLES) {
        memcpy(out + first, ring, (STREAM_CHUNK_SAMPLES - first) * 2);
      }
      readPos = (readPos + STREAM_CHUNK_SAMPLES) % RING_SAMPLES;
      avail -= STREAM_CHUNK_SAMPLES;

      int n = uxQueueMessagesWaiting(audioStreamClients);
      for (int i = 0; i < n; i++) {
        WiFiClient *client;
        if (xQueueReceive(audioStreamClients, &client, 0) != pdTRUE) break;

        // Same convention as the MJPEG fan-out: a short write means the
        // client can't keep up (or vanished), so it gets dropped. The write
        // may block briefly, but only this task waits on it.
        if (!client->connected() ||
            client->write((uint8_t *)out, sizeof(out)) != sizeof(out)) {
          client->stop();
          delete client;
          Serial.println("[audio] live stream client dropped");
        } else {
          xQueueSend(audioStreamClients, &client, 0);
        }
      }
    }
  }
}

/**
 * Capture task: drain I2S, maintain the adaptive floor, feed the ring,
 * and run the trigger/post-roll state machine.
 */
// ---- Live spectrum state ----
// The capture task fills specAccum with raw (pre-filter) samples; once FFT_N
// are collected it windows + transforms them and reduces the result to
// SPECTRUM_BANDS log-spaced band magnitudes, published under audioMux for the
// /audio/spectrum handler. Scratch buffers are file-scope to keep them off the
// 16KB task stack.
static float specBands[SPECTRUM_BANDS];            // latest per-band magnitude (guarded)
static float specThr[SPECTRUM_BANDS];              // latest per-band threshold (guarded)
static float specFreq[SPECTRUM_BANDS];             // band geometric-center freq, Hz (set once)
static int   specLo[SPECTRUM_BANDS];               // first FFT bin in band
static int   specHi[SPECTRUM_BANDS];               // one-past-last FFT bin in band
static float specAccum[FFT_N];                     // raw-sample accumulator
static int   specFill = 0;
static float specRe[FFT_N], specIm[FFT_N];         // FFT scratch
static float bandFloor[SPECTRUM_BANDS];            // per-band EMA noise floor (capture task only)
static float bandBaseline[SPECTRUM_BANDS];         // running per-band baseline (seeds the floors)
static volatile uint16_t spectralHotHz = 0;        // freq of the last triggering band
static volatile uint32_t bandWarmupUntil = 0;      // no band triggers until past this
static volatile bool     bandSeedReq = false;      // capture task: seed floors from baseline

// Spectrogram ring: peak magnitude per band per SPECTRO_BUCKET_MS. uint16 cells
// are written by the capture task and read (chronological, unlocked - a torn
// column is cosmetic) by the /audio/spectrogram handler; only the ring indices
// are guarded.
static uint16_t spectro[SPECTRO_COLS * SPECTRUM_BANDS];
static int      spectroWrite = 0;
static int      spectroCount = 0;
static float    spectroAccum[SPECTRUM_BANDS];
static uint32_t spectroBucketStart = 0;
static volatile bool spectroRebucketReq = false;   // window changed: capture task re-buckets the ring
static volatile uint32_t spectroRebucketOldMs = 0; // cadence the ring was recorded at
static volatile bool spectroRestoring = false;    // capture task skips column stores during rebuild
static float    specWin[FFT_N];                    // precomputed Hann window (constant)

// Per-column bucket duration for the current window: minutes spread over the
// fixed 240 columns.
static inline uint32_t spectroBucketMsFor(uint32_t minutes) {
  uint32_t b = minutes * 60000UL / SPECTRO_COLS;
  return b < 50 ? 50 : b;
}
static inline uint32_t spectroBucketMs() { return spectroBucketMsFor(spectroMinutes); }

// Heatmap window changed: re-bucket the ring column-wise to the new cadence
// (per-band strided resampleMaxPool) so the collected heatmap survives the
// change. Capture task only (the ring's sole writer); readers snapshot indices
// under audioMux and stream cells unlocked - a torn column is cosmetic, same
// as the /audio/spectrogram contract. Alloc failure falls back to a clear.
static void spectroRebucketRing() {
  uint32_t oldMs = spectroRebucketOldMs, newMs = spectroBucketMs();
  uint32_t cnt, w;
  portENTER_CRITICAL(&audioMux);
  cnt = (uint32_t)spectroCount;
  w = (uint32_t)spectroWrite;
  portEXIT_CRITICAL(&audioMux);
  if (oldMs == newMs) return;
  uint32_t n = 0;
  uint16_t *tmp = (cnt && oldMs)
                      ? (uint16_t *)ps_malloc(SPECTRO_COLS * SPECTRUM_BANDS * 2)
                      : NULL;
  if (tmp) {
    for (uint32_t b = 0; b < SPECTRUM_BANDS; b++)
      n = history::resampleMaxPool(spectro, SPECTRO_COLS, w, cnt, oldMs, newMs,
                                   tmp, SPECTRO_COLS, SPECTRUM_BANDS, b);
    memcpy(spectro, tmp, n * SPECTRUM_BANDS * 2);
    free(tmp);
  }
  portENTER_CRITICAL(&audioMux);
  spectroWrite = (int)(n % SPECTRO_COLS);
  spectroCount = (int)n;
  portEXIT_CRITICAL(&audioMux);
}

// Snapshot the heatmap ring to SD so the frequency history survives a reboot,
// mirroring sdSaveHistory. Far smaller (~15KB) so one mutex hold is fine. The
// header carries spectroMinutes so a restore under a different window can
// re-bucket the columns to the current cadence. Shares the HIST_SAVE_MS
// cadence from sdWriterTask.
static void sdSaveSpectro() {
  if (!sdAvailable) return;
  if (time(NULL) < 1600000000) return;
  uint32_t cnt; int w;
  portENTER_CRITICAL(&audioMux);
  cnt = (uint32_t)spectroCount;
  w = spectroWrite;
  portEXIT_CRITICAL(&audioMux);
  if (cnt == 0) return;

  const uint32_t colBytes = SPECTRUM_BANDS * 2;
  uint32_t start = (uint32_t)((w - (int)cnt + SPECTRO_COLS) % SPECTRO_COLS); // oldest column
  uint32_t run1 = cnt < SPECTRO_COLS - start ? cnt : SPECTRO_COLS - start;

  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File f = SD.open(SPECTRO_FILE, FILE_WRITE); // truncates
  if (f) {
    uint32_t hdr[4] = {SPECTRO_MAGIC, (uint32_t)time(NULL), cnt, spectroMinutes};
    f.write((uint8_t *)hdr, sizeof(hdr));
    f.write((uint8_t *)&spectro[start * SPECTRUM_BANDS], run1 * colBytes);
    if (run1 < cnt) f.write((uint8_t *)&spectro[0], (cnt - run1) * colBytes);
    f.close();
  }
  xSemaphoreGive(sdMutex);
}

// Rebuild the heatmap ring from the last snapshot: saved columns, then a zero
// (silence) gap for the downtime, then the live columns captured since boot -
// the same age-by-gap reconstruction as sdRestoreHistory, with a 32-band
// column stride. A snapshot saved at a different window is re-bucketed to the
// current cadence (per-band resampleMaxPool) instead of discarded.
static void sdRestoreSpectro() {
  if (!sdAvailable) return;

  const uint32_t colBytes = SPECTRUM_BANDS * 2;
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File f = SD.open(SPECTRO_FILE, FILE_READ);
  uint32_t hdr[4] = {0, 0, 0, 0};
  bool ok = f && f.read((uint8_t *)hdr, sizeof(hdr)) == (int)sizeof(hdr) && hdr[0] == SPECTRO_MAGIC;
  uint32_t cnt = ok && hdr[2] <= SPECTRO_COLS ? hdr[2] : 0;
  // hdr[3] = window (minutes) the snapshot was recorded at; discard only if
  // it's outside the valid range (older/corrupt file).
  if (ok && (hdr[3] < SPECTRO_MIN_MINUTES || hdr[3] > SPECTRO_MAX_MINUTES)) cnt = 0;
  uint16_t *cols = NULL;
  if (cnt) {
    cols = (uint16_t *)ps_malloc(cnt * colBytes);
    if (!cols || f.read((uint8_t *)cols, cnt * colBytes) != (int)(cnt * colBytes)) cnt = 0;
  }
  if (f) f.close();
  xSemaphoreGive(sdMutex);

  uint32_t bucketMs = spectroBucketMs();
  // Window changed since the snapshot: re-bucket to the current cadence
  // (dense arrays: cap == write == count) instead of losing the heatmap.
  uint32_t savedMs = spectroBucketMsFor(hdr[3]);
  if (cnt && savedMs != bucketMs) {
    uint16_t *cols2 = (uint16_t *)ps_malloc(SPECTRO_COLS * colBytes);
    if (cols2) {
      uint32_t n = 0;
      for (uint32_t b = 0; b < SPECTRUM_BANDS; b++)
        n = history::resampleMaxPool(cols, cnt, cnt, cnt, savedMs, bucketMs,
                                     cols2, SPECTRO_COLS, SPECTRUM_BANDS, b);
      free(cols);
      cols = cols2;
      cnt = n;
    } else {
      cnt = 0; // scratch alloc failed: keep the old discard behavior
    }
  }
  uint32_t gap = ((uint32_t)time(NULL) - hdr[1]) * 1000UL / bucketMs; // downtime in columns
  if (cnt && gap < SPECTRO_COLS) {
    spectroRestoring = true;
    uint32_t liveCnt;
    portENTER_CRITICAL(&audioMux);
    liveCnt = (uint32_t)spectroCount; // boot-fresh ring: live data at [0..liveCnt), unwrapped
    portEXIT_CRITICAL(&audioMux);

    uint32_t zeros = gap > liveCnt ? gap - liveCnt : 0; // downtime not covered by live cols
    uint32_t keep = cnt;
    if (keep + zeros + liveCnt > SPECTRO_COLS) {
      uint32_t drop = keep + zeros + liveCnt - SPECTRO_COLS;
      keep = drop < keep ? keep - drop : 0;
    }
    if (keep) {
      uint32_t total = keep + zeros + liveCnt;
      memmove(&spectro[(keep + zeros) * SPECTRUM_BANDS], &spectro[0], liveCnt * colBytes);
      memset(&spectro[keep * SPECTRUM_BANDS], 0, zeros * colBytes);
      memcpy(&spectro[0], cols + (cnt - keep) * SPECTRUM_BANDS, keep * colBytes);
      portENTER_CRITICAL(&audioMux);
      spectroWrite = (int)(total % SPECTRO_COLS);
      spectroCount = (int)total;
      portEXIT_CRITICAL(&audioMux);
      Serial.printf("[audio] spectrogram restored: %u cols (+%u gap)\n", keep, zeros);
    }
    spectroRestoring = false;
  }
  if (cols) free(cols);
}

// Map log-spaced band edges to FFT bin ranges; called once at init.
static void spectrumInit() {
  // Reuse the unit-tested window (fft::hann multiplies in place) instead of a
  // duplicate Hann formula: seed to 1.0, then apply the window.
  for (int i = 0; i < FFT_N; i++) specWin[i] = 1.0f;
  fft::hann(specWin, FFT_N);
  const float binHz = (float)AUDIO_SAMPLE_RATE / FFT_N;  // 15.625 Hz
  const float fmax = 0.5f * AUDIO_SAMPLE_RATE;           // Nyquist 8000
  const float ratio = logf(fmax / SPECTRUM_FMIN);
  for (int b = 0; b < SPECTRUM_BANDS; b++) {
    float f0 = SPECTRUM_FMIN * expf(ratio * b / SPECTRUM_BANDS);
    float f1 = SPECTRUM_FMIN * expf(ratio * (b + 1) / SPECTRUM_BANDS);
    int lo = (int)(f0 / binHz + 0.5f);
    int hi = (int)(f1 / binHz + 0.5f);
    if (lo < 1) lo = 1;                  // skip the DC bin
    if (hi <= lo) hi = lo + 1;
    if (hi > FFT_N / 2) hi = FFT_N / 2;
    specLo[b] = lo; specHi[b] = hi;
    specFreq[b] = sqrtf(f0 * f1);
    specBands[b] = 0.0f;
    specThr[b] = 0.0f;
    bandFloor[b] = 0.0f;
    bandBaseline[b] = 0.0f;
    spectroAccum[b] = 0.0f;
  }
}

// Window + transform the accumulated frame, reduce to width-normalized per-band
// RMS magnitude, learn each band's adaptive floor/threshold (same pure math as
// the scalar path), publish for the plot, and - if the spectral trigger is on -
// start a clip when a band crosses. Runs in the capture task between chunks.
static void spectrumCompute() {
  for (int i = 0; i < FFT_N; i++) { specRe[i] = specAccum[i] * specWin[i]; specIm[i] = 0.0f; }
  fft::forward(specRe, specIm, FFT_N);

  float mag[SPECTRUM_BANDS], thr[SPECTRUM_BANDS];
  const float factor = spectralFactor, minMag = spectralMinMag;

  // Running per-band baseline (always learns while the FFT is up). When the
  // band trigger is armed, this seeds the floors so they start representative
  // instead of snapping from zero and firing a burst.
  for (int b = 0; b < SPECTRUM_BANDS; b++) {
    float e = 0.0f; int cnt = 0;
    for (int k = specLo[b]; k < specHi[b]; k++) {
      float m = fft::magnitude(specRe, specIm, k);
      e += m * m; cnt++;
    }
    mag[b] = cnt ? sqrtf(e / cnt) * FFT_MAG_NORM : 0.0f;
    bandBaseline[b] = (bandBaseline[b] == 0.0f)
                          ? mag[b]
                          : bandBaseline[b] + BAND_BASELINE_ALPHA * (mag[b] - bandBaseline[b]);
  }
  if (bandSeedReq) {
    for (int b = 0; b < SPECTRUM_BANDS; b++) bandFloor[b] = bandBaseline[b];
    bandSeedReq = false;
  }
  // Band floor is a slow per-band EMA over bandFloorTcMin minutes (symmetric, so
  // the grey threshold line tracks ambient gradually rather than snapping each
  // 32ms frame). Above threshold, habituateMin governs the leak. DURING the
  // warm-up window the floor adapts FAST in every direction so the threshold
  // settles to the current ambient within a few seconds before triggers arm -
  // otherwise a multi-minute EMA would take minutes to stabilize from a cold seed.
  // Correct the slow EMA rates for the interval that actually elapsed since the
  // last FFT frame (frames don't arrive on an exact FFT_FRAME_MS cadence under
  // contention), so the band-floor time-constant and habituation hold to their
  // wall-clock minutes. The fast warm-up phase stays at the fixed rate.
  static uint32_t lastBandMs = 0;
  uint32_t bnow = millis();
  uint32_t bdt = lastBandMs ? (bnow - lastBandMs) : (uint32_t)FFT_FRAME_MS;
  lastBandMs = bnow;
  bool bandWarm = (int32_t)(millis() - bandWarmupUntil) < 0;
  float bandA = bandWarm ? FLOOR_ALPHA_DOWN
              : (bandFloorTcMin > 0.0f
                   ? audsp::scaleAlphaForDt(FFT_FRAME_MS / (bandFloorTcMin * 60000.0f), bdt, FFT_FRAME_MS)
                   : audsp::scaleAlphaForDt(FLOOR_ALPHA_UP, bdt, FFT_FRAME_MS));
  float habA = bandWarm ? bandA
             : (habituateMin > 0.0f
                   ? audsp::scaleAlphaForDt(FFT_FRAME_MS / (habituateMin * 60000.0f), bdt, FFT_FRAME_MS)
                   : 0.0f);
  for (int b = 0; b < SPECTRUM_BANDS; b++) {
    // Per-band adaptive threshold + floor, reusing the unit-tested scalar core.
    thr[b] = audsp::adaptiveThreshold(bandFloor[b], factor, minMag);
    bandFloor[b] = audsp::adaptFloor(bandFloor[b], mag[b], thr[b], bandA, bandA, habA);
  }

  portENTER_CRITICAL(&audioMux);
  for (int b = 0; b < SPECTRUM_BANDS; b++) {
    // Smooth the DISPLAYED bar (EMA); snap on the first frame after a (re)start.
    // specThr follows the slow floor EMA already, so it's left as-is.
    specBands[b] = specBands[b] > 0.0f
                       ? specBands[b] + SPECTRUM_DISPLAY_ALPHA * (mag[b] - specBands[b])
                       : mag[b];
    specThr[b] = thr[b];
  }
  portEXIT_CRITICAL(&audioMux);

  // Fold into the spectrogram ring (peak per band per bucket). A window change
  // re-buckets the ring to the new cadence, preserving the collected heatmap.
  uint32_t now = millis();
  if (spectroRebucketReq) {
    spectroRebucketRing();
    for (int b = 0; b < SPECTRUM_BANDS; b++) spectroAccum[b] = 0.0f;
    spectroBucketStart = now;
    spectroRebucketReq = false;
  }
  if (spectroBucketStart == 0) spectroBucketStart = now;
  for (int b = 0; b < SPECTRUM_BANDS; b++)
    if (mag[b] > spectroAccum[b]) spectroAccum[b] = mag[b];
  if (!spectroRestoring && now - spectroBucketStart >= spectroBucketMs()) {
    int base = spectroWrite * SPECTRUM_BANDS;
    for (int b = 0; b < SPECTRUM_BANDS; b++) {
      // Store dB*256, not raw magnitude: FFT magnitudes (gain x16, 512-pt) run
      // tens of thousands and overflow uint16, pinning the heatmap full-scale.
      // Log scale never clamps and gives real dynamic range. (Arbitrary ref - a
      // relative loudness, not SPL.)
      float e = 20.0f * log10f(spectroAccum[b] + 1.0f) * 256.0f;
      spectro[base + b] = (uint16_t)(e < 0.0f ? 0.0f : (e > 65535.0f ? 65535.0f : e));
      spectroAccum[b] = 0.0f;
    }
    portENTER_CRITICAL(&audioMux);
    spectroWrite = (spectroWrite + 1) % SPECTRO_COLS;
    if (spectroCount < SPECTRO_COLS) spectroCount++;
    portEXIT_CRITICAL(&audioMux);
    spectroBucketStart = now;
  }

  // Spectral trigger - evaluated AFTER releasing the lock, since startClip()
  // takes audioMux itself (portMUX is not recursive). The band warm-up holds
  // off triggers while the seeded floors settle.
  if (spectralTrigEnabled && !recording &&
      millis() > warmupUntilMs && millis() > bandWarmupUntil &&
      millis() - lastClipEndMs > RETRIGGER_HOLDOFF_MS) {
    int hot = audsp::hottestBandOverThreshold(mag, thr, SPECTRUM_BANDS);
    if (hot >= 0) {
      spectralHotHz = (uint16_t)(specFreq[hot] + 0.5f);
      uint32_t id = startClip(TRIGGER_SOURCE_SPECTRAL, spectralHotHz);
      if (id) dlog(DLOG_INFO, "spectral band %uHz: mag %.0f / thr %.0f",
                   spectralHotHz, mag[hot], thr[hot]);
    }
  }
}

static void audioTask(void *pvParameters) {
  static int16_t chunk[CHUNK_SAMPLES];
  // PDM DC-offset estimate (one-pole DC blocker). Function-scoped so the resume
  // path can clear it: a stale estimate from before a disable would saturate the
  // first chunk after re-enable (see the DC blocker below).
  static float dcEst = 0.0f;
  // millis() of the last noise-floor update, so the EMA rates can be corrected
  // for the interval that actually elapsed (vs the nominal CHUNK_MS) - see D1.
  static uint32_t lastFloorMs = 0;

  for (;;) {
    if (otaActive()) { vTaskDelay(pdMS_TO_TICKS(200)); continue; } // idle during OTA flash
    // Full disable: when audio is off (and no clip is finishing), suspend the
    // task entirely - zero CPU, mic idle - the audio parallel to the camera
    // parking. Resumed by handleAudioConfig on /audio/config?en=1. (At this
    // point we hold no locks, so self-suspend is safe.)
    if (!audioEnabled && !recording) {
      currentRms = 0.0f;
      // Block until re-enabled. Notification (not vTaskSuspend) is race-free:
      // a give that arrives before we block is latched, so no lost wakeup.
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      noiseFloor = 0.0f; noiseVar = 0.0f;    // re-learn ambient after resume (floor + stats)
      for (int b = 0; b < SPECTRUM_BANDS; b++) bandFloor[b] = 0.0f; // re-learn per band
      dcEst = 0.0f;                          // forget the stale DC offset (mic was idle)
      lastFloorMs = millis();                // fresh dt baseline so resume doesn't over-step
      warmupUntilMs = millis() + WARMUP_MS;  // no triggers until it settles
      continue;
    }

    size_t bytesRead = 0;
    if (i2s_read(AUDIO_I2S_PORT, chunk, sizeof(chunk), &bytesRead, portMAX_DELAY) != ESP_OK ||
        bytesRead == 0) {
      continue;
    }
    int n = bytesRead / 2;

    // One-pole DC blocker (~13Hz cutoff). The PDM mic carries a large,
    // temperature-dependent DC offset (observed >6000 counts) that would
    // otherwise dominate the RMS, pin it above the threshold, and freeze
    // the adaptive floor (which only learns while rms < threshold).
    // Samples are rewritten in place so clips and the live stream are
    // centered too. (dcEst is declared at function scope so resume can reset it.)
    // Low-pass filter (after DC removal + gain): recompute coefficients only
    // when the cutoff actually changes - not every 16ms chunk - and clear the
    // filter memory on each off->on so a stale tail can't pop. Both controls
    // are sampled once here so the whole chunk sees a consistent setting.
    static biquad::State  lpState  = {0.0f, 0.0f};
    static biquad::Coeffs lpCoeffs = biquad::lowpass(DEFAULT_LPF_CUTOFF_HZ, AUDIO_SAMPLE_RATE, LPF_Q);
    static float lpApplied = DEFAULT_LPF_CUTOFF_HZ;
    static bool  lpWasOn   = false;
    const bool  lpOn = lpfEnabled;
    const float lpHz = lpfCutoffHz;
    if (lpOn) {
      if (lpHz != lpApplied) { lpCoeffs = biquad::lowpass(lpHz, AUDIO_SAMPLE_RATE, LPF_Q); lpApplied = lpHz; }
      if (!lpWasOn) biquad::reset(lpState);
    }
    lpWasOn = lpOn;
    // High-pass (cascaded before the low-pass), same recompute-on-change pattern.
    static biquad::State  hpState  = {0.0f, 0.0f};
    static biquad::Coeffs hpCoeffs = biquad::highpass(DEFAULT_HPF_CUTOFF_HZ, AUDIO_SAMPLE_RATE, LPF_Q);
    static float hpApplied = DEFAULT_HPF_CUTOFF_HZ;
    static bool  hpWasOn   = false;
    const bool  hpOn = hpfEnabled;
    const float hpHz = hpfCutoffHz;
    if (hpOn) {
      if (hpHz != hpApplied) { hpCoeffs = biquad::highpass(hpHz, AUDIO_SAMPLE_RATE, LPF_Q); hpApplied = hpHz; }
      if (!hpWasOn) biquad::reset(hpState);
    }
    hpWasOn = hpOn;

    // Spectrum/spectrogram: run the FFT continuously while audio is enabled, so
    // the heatmap history and band-trigger floors are always populated (the FFT
    // is ~1% of one core; cheap vs the camera/WiFi). We're only here when audio
    // is enabled or a clip is finishing; skip the latter.
    const bool specOn = audioEnabled;
    if (!specOn) specFill = 0;

    float sum = 0.0f;
    const float g = micGain; // applied after DC removal so the offset isn't amplified
    for (int i = 0; i < n; i++) {
      float x = chunk[i];
      dcEst += 0.005f * (x - dcEst);
      float v = (x - dcEst) * g;
      if (specOn) {
        specAccum[specFill++] = v;            // RAW tap, before both filters
        if (specFill >= FFT_N) { spectrumCompute(); specFill = 0; }
      }
      if (hpOn) v = biquad::process(hpState, hpCoeffs, v);
      if (lpOn) v = biquad::process(lpState, lpCoeffs, v);
      if (v > 32767.0f) v = 32767.0f;
      else if (v < -32768.0f) v = -32768.0f;
      chunk[i] = (int16_t)v;
      sum += v * v;
    }
    float rms = sqrtf(sum / n);
    // Paused: publish silence but keep the history advancing (real zeros
    // keep the plot timeline honest). An in-flight clip finishes first.
    const bool paused = !audioEnabled && !recording;
    if (paused) rms = 0.0f;
    currentRms = rms;
    metricsSet(metrics::M_AUDIO, rms);

    // fold into the amplitude history (peak per HIST_BUCKET_MS)
    if (rms > bucketPeak) bucketPeak = rms;
    uint32_t nowMs = millis();
    // Discard the PDM mic's DC-settling spike at boot AND after every resume.
    // When audio is toggled off->on (or the task restarts) the stale dcEst
    // clamps the whole first chunk full-scale, saturating the bucket peak to
    // 32767 - a fake spike that then persists to SD and survives reboots. The
    // boot-only HIST_SETTLE_MS guard let those resume spikes through; gate on
    // warmupUntilMs too, which is re-armed on every (re)start (lines ~1987,
    // ~2606) right alongside the noise-floor reset.
    if (histRebucketReq) { // retention window changed: re-bucket, don't wipe
      histRebucketRings();
      bucketPeak = 0.0f;
      bucketStartMs = nowMs;
      histRebucketReq = false;
    }
    if (nowMs < HIST_SETTLE_MS || nowMs < warmupUntilMs) {
      bucketPeak = 0.0f; // discard boot/resume DC-settling transient
      bucketStartMs = nowMs;
    } else if (histBuf != NULL && !histRestoring && nowMs - bucketStartMs >= histBucketMs()) {
      histBuf[histWrite] = (uint16_t)(bucketPeak > 65535.0f ? 65535 : bucketPeak);
      if (thrHistBuf != NULL) {
        float t = currentThreshold();
        thrHistBuf[histWrite] = (uint16_t)(t > 65535.0f ? 65535 : t);
      }
      portENTER_CRITICAL(&audioMux);
      histWrite = (histWrite + 1) % HIST_BUCKETS;
      if (histCount < HIST_BUCKETS) histCount++;
      histSeq++;
      portEXIT_CRITICAL(&audioMux);
      bucketPeak = 0.0f;
      bucketStartMs = nowMs;
    }

    // Skip floor learning (zeros would drag it down), the ring, the live
    // stream and triggers while paused; the DC estimate stays warm above.
    if (paused) continue;

    // The adaptive estimate always learns, so it stays current even while a manual
    // trigger threshold is in force. EMA rates are corrected for the interval that
    // actually elapsed (chunks don't arrive on an exact CHUNK_MS cadence under
    // camera/WiFi contention) so all time-constants hold their wall-clock meaning.
    uint32_t dtMs = lastFloorMs ? (uint32_t)(nowMs - lastFloorMs) : (uint32_t)CHUNK_MS;
    lastFloorMs = nowMs;
    float threshold;
    if (trigAlgo == TRIG_ALGO_STAT) {
      // Statistical: threshold = mean + k*sigma of the ambient distribution. Use
      // the current estimate for THIS chunk, then fold this sample in. A fast
      // window during warm-up settles the stats before triggers arm; the slow
      // window afterward keeps rare loud events from inflating the estimate.
      float mean = noiseFloor, var = noiseVar; // locals (noiseFloor is volatile)
      threshold = audsp::statisticalThreshold(mean, var, trigK, minTrigRms);
      float winMs = ((int32_t)(nowMs - warmupUntilMs) < 0) ? STAT_WARMUP_WIN_MS : (statWinMin * 60000.0f);
      audsp::adaptMeanVar(mean, var, rms, audsp::scaleAlphaForDt(CHUNK_MS / winMs, dtMs, CHUNK_MS));
      noiseFloor = mean; noiseVar = var;
    } else {
      // Floor x factor: the threshold tracks the learned noise floor.
      float floorNow = noiseFloor;
      threshold = audsp::adaptiveThreshold(floorNow, thresholdFactor, minTrigRms);
      float aDown = audsp::scaleAlphaForDt(FLOOR_ALPHA_DOWN, dtMs, CHUNK_MS);
      float aUp   = audsp::scaleAlphaForDt(FLOOR_ALPHA_UP, dtMs, CHUNK_MS);
      float habA  = habituateMin > 0.0f
                  ? audsp::scaleAlphaForDt(CHUNK_MS / (habituateMin * 60000.0f), dtMs, CHUNK_MS)
                  : 0.0f;
      noiseFloor = audsp::adaptFloor(floorNow, rms, threshold, aDown, aUp, habA);
    }
    float triggerThr = audsp::effectiveThreshold(threshold, manualThresholdRms);

    size_t w = ringWrite;
    size_t firstRun = RING_SAMPLES - w;
    if (firstRun > (size_t)n) firstRun = n;
    memcpy(ring + w, chunk, firstRun * 2);
    if (firstRun < (size_t)n) {
      memcpy(ring, chunk + firstRun, (n - firstRun) * 2);
    }

    bool rec;
    portENTER_CRITICAL(&audioMux);
    ringWrite = (w + n) % RING_SAMPLES;
    rec = recording;
    if (rec) postRemaining -= n;
    portEXIT_CRITICAL(&audioMux);

    if (tAudioStream != NULL) xTaskNotifyGive(tAudioStream);

    if (rec) {
      if (postRemaining <= 0) finalizeClip();
    } else if (audsp::shouldAutoTrigger(
                   rms, triggerThr, autoTriggerEnabled,
                   millis() > warmupUntilMs,
                   millis() - lastClipEndMs > RETRIGGER_HOLDOFF_MS)) {
      uint32_t id = startClip(TRIGGER_SOURCE_AUTO);
      Serial.printf("[audio] auto trigger: clip %u (rms %.0f > thr %.0f%s, floor %.0f)\n",
                    id, rms, triggerThr,
                    manualThresholdRms > 0.0f ? " manual" : "", noiseFloor);
    }
  }
}

// ---------------- HTTP handlers ----------------
// These run in the WebServer task. Clip downloads write the raw socket
// directly (same pattern as /jpg) and pin the slot with a busy refcount so
// finalizeClip() won't recycle it mid-transfer.

static void handleAudioTrigger() {
  uint32_t id = startClip(TRIGGER_SOURCE_HTTP);
  if (id == 0) {
    audioServer->send(409, "text/plain", "audio is disabled (/audio/config?en=1)");
    return;
  }

  long remaining;
  portENTER_CRITICAL(&audioMux);
  remaining = postRemaining;
  portEXIT_CRITICAL(&audioMux);
  uint32_t readyInMs = remaining > 0 ? (uint32_t)(remaining / SAMPLES_PER_MS) : 0;

  char json[96];
  snprintf(json, sizeof(json), "{\"id\":%u,\"ready_in_ms\":%u}", id, readyInMs);
  audioServer->send(200, "application/json", json);
}

static void handleAudioClip() {
  uint32_t wantId = audioServer->hasArg("id")
                      ? (uint32_t)strtoul(audioServer->arg("id").c_str(), NULL, 10)
                      : 0;

  int chosen = -1;
  uint32_t newest = 0;
  portENTER_CRITICAL(&audioMux);
  for (int i = 0; i < clipSlotCount; i++) {
    if (!slots[i].ready) continue;
    if (wantId != 0) {
      if (slots[i].id == wantId) { chosen = i; break; }
    } else if (slots[i].id > newest) {
      newest = slots[i].id;
      chosen = i;
    }
  }
  if (chosen >= 0) slots[chosen].busy++;
  portEXIT_CRITICAL(&audioMux);

  if (chosen < 0) {
    audioServer->send(404, "text/plain",
                      wantId ? "clip not found (still recording, or evicted)"
                             : "no clips stored yet");
    return;
  }

  ClipSlot &s = slots[chosen];
  WiFiClient client = audioServer->client();
  client.setNoDelay(true);

  // inline so the dashboard <audio> player works; ?dl=1 forces a download
  bool asDownload = audioServer->hasArg("dl") && audioServer->arg("dl") == "1";
  char header[224];
  int headerLen = snprintf(header, sizeof(header),
                           "HTTP/1.1 200 OK\r\n"
                           "Content-Type: audio/wav\r\n"
                           "Content-Length: %u\r\n"
                           "Content-Disposition: %s; filename=clip_%u.wav\r\n"
                           "Access-Control-Allow-Origin: *\r\n"
                           "Connection: close\r\n\r\n",
                           (unsigned)s.len, asDownload ? "attachment" : "inline", s.id);
  client.write(header, headerLen);

  size_t offset = 0;
  while (offset < s.len && client.connected()) {
    size_t toWrite = s.len - offset;
    if (toWrite > 8192) toWrite = 8192;
    size_t written = client.write(s.buf + offset, toWrite);
    if (written == 0) break;
    offset += written;
  }
  client.stop();

  portENTER_CRITICAL(&audioMux);
  s.busy--;
  portEXIT_CRITICAL(&audioMux);
}

static void handleAudioClips() {
  String json = "[";
  bool firstEntry = true;
  uint32_t now = millis();
  for (int i = 0; i < clipSlotCount; i++) {
    if (!slots[i].ready) continue;
    if (!firstEntry) json += ",";
    firstEntry = false;
    json += "{\"id\":" + String(slots[i].id);
    json += ",\"age_ms\":" + String(now - slots[i].tsMs);
    json += ",\"size\":" + String(slots[i].len);
    json += ",\"rms\":" + String(slots[i].trigRms);
    json += ",\"thr\":" + String(slots[i].trigThr);
    json += ",\"hz\":" + String(slots[i].trigHz);
    json += ",\"source\":\"";
    json += srcName(slots[i].source);
    json += "\"}";
  }
  json += "]";
  audioServer->send(200, "application/json", json);
}

static void handleAudioStatus() {
  float floorNow = noiseFloor;
  float threshold = currentThreshold();

  float sigma = noiseVar > 0.0f ? sqrtf(noiseVar) : 0.0f;

  char json[896];
  snprintf(json, sizeof(json),
           "{\"rms\":%.1f,\"noise_floor\":%.1f,\"threshold\":%.1f,\"gain\":%.1f,\"min_thr\":%.1f,"
           "\"enabled\":%s,\"recording\":%s,\"auto\":%s,\"clip_ms\":%u,\"preroll_ms\":%u,"
           "\"factor\":%.2f,\"algo\":%d,\"trig_k\":%.2f,\"sigma\":%.1f,\"stat_win\":%.2f,"
           "\"manual_thr\":%.1f,\"lpf\":%s,\"lpf_hz\":%.0f,"
           "\"hpf\":%s,\"hpf_hz\":%.0f,"
           "\"slots\":%d,\"max_slots\":%d,"
           "\"recommended_slots\":%d,\"slot_bytes\":%u,\"free_psram\":%u,"
           "\"sd\":%s,\"save_to_sd\":%s,\"sd_used_mb\":%llu,\"sd_total_mb\":%llu,"
           "\"sd_max_clips\":%u,\"viewer\":%s}",
           currentRms, floorNow, threshold, micGain, minTrigRms,
           audioEnabled ? "true" : "false",
           recording ? "true" : "false",
           autoTriggerEnabled ? "true" : "false",
           clipMs, prerollMs, thresholdFactor, trigAlgo, trigK, sigma, statWinMin,
           manualThresholdRms,
           lpfEnabled ? "true" : "false", lpfCutoffHz,
           hpfEnabled ? "true" : "false", hpfCutoffHz,
           clipSlotCount, MAX_CLIP_SLOTS, recommendedSlotCount(),
           (unsigned)SLOT_BYTES, ESP.getFreePsram(),
           sdAvailable ? "true" : "false",
           (sdAvailable && saveToSd) ? "true" : "false",
           sdAvailable ? sdUsedMbCache : 0,  // cached by the writer task; never
           sdAvailable ? sdTotalMbCache : 0, // touch SD from the server thread
           sdMaxClips,
           webAuthIsViewer() ? "true" : "false");
  audioServer->send(200, "application/json", json);
}

// Live FFT spectrum: log-spaced band magnitudes of the RAW (pre-low-pass)
// signal. The FFT runs continuously in the capture task while audio is enabled
// (for the always-on spectral trigger), so this just snapshots the latest bands.
static void handleAudioSpectrum() {
  float bands[SPECTRUM_BANDS], thr[SPECTRUM_BANDS];
  portENTER_CRITICAL(&audioMux);
  for (int b = 0; b < SPECTRUM_BANDS; b++) { bands[b] = specBands[b]; thr[b] = specThr[b]; }
  portEXIT_CRITICAL(&audioMux);

  String json;
  json.reserve(128 + SPECTRUM_BANDS * 24);  // scalar fields + 3 arrays of ~7 chars/band
  json = "{\"fs\":";
  json += AUDIO_SAMPLE_RATE;
  json += ",\"dbref\":"; json += String(SPECTRO_DBFS_REF, 1); // full-scale dB ref for absolute dBFS view
  json += ",\"n\":";   json += FFT_N;
  json += ",\"on\":";  json += (audioEnabled ? "true" : "false");
  json += ",\"lpf\":"; json += (lpfEnabled ? "true" : "false");
  json += ",\"lpf_hz\":"; json += String(lpfCutoffHz, 0);
  json += ",\"hpf\":"; json += (hpfEnabled ? "true" : "false");
  json += ",\"hpf_hz\":"; json += String(hpfCutoffHz, 0);
  json += ",\"strig\":"; json += (spectralTrigEnabled ? "true" : "false");
  json += ",\"sfactor\":"; json += String(spectralFactor, 1);
  json += ",\"sminmag\":"; json += String(spectralMinMag, 0);
  json += ",\"hab_min\":"; json += String(habituateMin, 1);
  json += ",\"band_tc\":"; json += String(bandFloorTcMin, 2);
  { uint32_t now = millis();
    json += ",\"bandwarm_ms\":";
    json += (spectralTrigEnabled && bandWarmupUntil > now) ? (bandWarmupUntil - now) : 0; }
  json += ",\"spectro_min\":"; json += spectroMinutes;
  json += ",\"hist_win\":"; json += histWindowMin;
  json += ",\"hist_bucket_ms\":"; json += histBucketMs();
  json += ",\"f\":[";
  for (int b = 0; b < SPECTRUM_BANDS; b++) { if (b) json += ','; json += (int)(specFreq[b] + 0.5f); }
  json += "],\"m\":[";
  for (int b = 0; b < SPECTRUM_BANDS; b++) { if (b) json += ','; json += String(bands[b], 1); }
  json += "],\"t\":[";
  for (int b = 0; b < SPECTRUM_BANDS; b++) { if (b) json += ','; json += String(thr[b], 1); }
  json += "]}";
  audioServer->send(200, "application/json", json);
}

// Spectrogram history: the per-band ring as raw LE uint16, oldest column first,
// SPECTRUM_BANDS values per column. Indices are read under the lock; the cells
// are streamed unlocked (a torn column is cosmetic on a heatmap).
static void handleAudioSpectrogram() {
  uint32_t cnt, w;
  portENTER_CRITICAL(&audioMux);
  cnt = spectroCount;
  w = spectroWrite;
  portEXIT_CRITICAL(&audioMux);

  int o = (int)((w - cnt + SPECTRO_COLS) % SPECTRO_COLS); // oldest column
  uint32_t bytes = cnt * SPECTRUM_BANDS * 2;

  WiFiClient client = audioServer->client();
  char hdr[224];
  int hl = snprintf(hdr, sizeof(hdr),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/octet-stream\r\n"
                    "Content-Length: %u\r\n"
                    "X-Cols: %u\r\nX-Bands: %u\r\nX-Bucket-Ms: %u\r\nX-Db256: 1\r\n"
                    "X-DbRef: %.1f\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Connection: close\r\n\r\n",
                    (unsigned)bytes, (unsigned)cnt, (unsigned)SPECTRUM_BANDS,
                    (unsigned)spectroBucketMs(), (double)SPECTRO_DBFS_REF);
  client.write(hdr, hl);
  if (cnt > 0) {
    // Chronological order in at most two contiguous runs (the ring may wrap).
    int run1 = SPECTRO_COLS - o; if ((uint32_t)run1 > cnt) run1 = cnt;
    client.write((uint8_t *)&spectro[o * SPECTRUM_BANDS], run1 * SPECTRUM_BANDS * 2);
    if ((uint32_t)run1 < cnt)
      client.write((uint8_t *)&spectro[0], (cnt - run1) * SPECTRUM_BANDS * 2);
  }
  client.stop();
}

static void handleAudioConfig() {
  if (recording && (audioServer->hasArg("clip_ms") || audioServer->hasArg("preroll_ms"))) {
    audioServer->send(409, "text/plain", "busy: clip capture in progress, retry shortly");
    return;
  }

  if (audioServer->hasArg("clip_ms")) {
    uint32_t v = strtoul(audioServer->arg("clip_ms").c_str(), NULL, 10);
    if (v < 500 || v > MAX_CLIP_MS) {
      audioServer->send(400, "text/plain", "clip_ms must be 500.." + String(MAX_CLIP_MS));
      return;
    }
    clipMs = v;
    if (prerollMs >= clipMs) prerollMs = clipMs / 2;
  }
  if (audioServer->hasArg("preroll_ms")) {
    uint32_t v = strtoul(audioServer->arg("preroll_ms").c_str(), NULL, 10);
    if (v >= clipMs) {
      audioServer->send(400, "text/plain", "preroll_ms must be < clip_ms");
      return;
    }
    prerollMs = v;
  }
  if (audioServer->hasArg("factor")) {
    float v = atof(audioServer->arg("factor").c_str());
    if (v >= 1.2f && v <= 20.0f) thresholdFactor = v;
  }
  if (audioServer->hasArg("algo")) {
    int a = atoi(audioServer->arg("algo").c_str());
    if ((a == TRIG_ALGO_FACTOR || a == TRIG_ALGO_STAT) && a != trigAlgo) {
      trigAlgo = a;
      // noiseFloor doubles as the floor (factor) and the mean (statistical), so
      // re-seed the estimator and warm up when switching so it relearns cleanly.
      noiseFloor = 0.0f; noiseVar = 0.0f;
      warmupUntilMs = millis() + WARMUP_MS;
    }
  }
  if (audioServer->hasArg("trig_k")) {
    float v = atof(audioServer->arg("trig_k").c_str());
    if (v >= 0.5f && v <= 12.0f) trigK = v;
  }
  if (audioServer->hasArg("stat_win")) {
    float v = atof(audioServer->arg("stat_win").c_str());
    if (v >= STAT_WIN_MIN_MIN && v <= STAT_WIN_MIN_MAX) statWinMin = v;
  }
  if (audioServer->hasArg("manual_thr")) {
    float v = atof(audioServer->arg("manual_thr").c_str());
    if (v >= 0.0f && v <= 32767.0f) manualThresholdRms = v;
  }
  if (audioServer->hasArg("gain")) {
    float v = atof(audioServer->arg("gain").c_str());
    if (v >= MIC_GAIN_MIN && v <= MIC_GAIN_MAX) {
      // minTrigRms / manualThresholdRms are stored as post-gain absolute RMS, so
      // rescale them by the gain ratio to keep physical sensitivity constant (the
      // invariant promised in the header). Re-clamp to their valid ranges after.
      float oldGain = micGain;
      micGain = v;
      if (oldGain > 0.0f && v != oldGain) {
        float r = v / oldGain;
        minTrigRms *= r;
        if (minTrigRms < 0.0f || minTrigRms > 32767.0f) minTrigRms = MIN_TRIGGER_RMS_UNITY * micGain;
        manualThresholdRms *= r;
        if (manualThresholdRms < 0.0f) manualThresholdRms = 0.0f;
        if (manualThresholdRms > 32767.0f) manualThresholdRms = 32767.0f;
      }
    }
  }
  if (audioServer->hasArg("min_thr")) {
    float v = atof(audioServer->arg("min_thr").c_str());
    if (v >= 0.0f && v <= 32767.0f) minTrigRms = v;
  }
  if (audioServer->hasArg("hab_min")) {
    float v = atof(audioServer->arg("hab_min").c_str());
    if (v >= 0.0f && v <= HABITUATE_MAX_MIN) habituateMin = v;
  }
  if (audioServer->hasArg("lpf")) {
    lpfEnabled = (audioServer->arg("lpf") == "1");
  }
  if (audioServer->hasArg("lpf_hz")) {
    float v = atof(audioServer->arg("lpf_hz").c_str());
    if (v >= LPF_CUTOFF_MIN && v <= LPF_CUTOFF_MAX) lpfCutoffHz = v;
  }
  if (audioServer->hasArg("hpf")) {
    hpfEnabled = (audioServer->arg("hpf") == "1");
  }
  if (audioServer->hasArg("hpf_hz")) {
    float v = atof(audioServer->arg("hpf_hz").c_str());
    if (v >= HPF_CUTOFF_MIN && v <= HPF_CUTOFF_MAX) hpfCutoffHz = v;
  }
  if (audioServer->hasArg("strig")) {
    bool was = spectralTrigEnabled;
    spectralTrigEnabled = (audioServer->arg("strig") == "1");
    // Arming it: seed the per-band floors from the running baseline and hold
    // off triggers through a warm-up, so it doesn't fire a burst while the
    // floors settle (capture task does the seed at its next frame).
    if (spectralTrigEnabled && !was) {
      bandSeedReq = true;
      bandWarmupUntil = millis() + BAND_WARMUP_MS;
    }
  }
  if (audioServer->hasArg("sfactor")) {
    float v = atof(audioServer->arg("sfactor").c_str());
    if (v >= 1.5f && v <= 100.0f) spectralFactor = v;
  }
  if (audioServer->hasArg("sminmag")) {
    float v = atof(audioServer->arg("sminmag").c_str());
    if (v >= 0.0f && v <= 1e7f) spectralMinMag = v;
  }
  if (audioServer->hasArg("band_tc")) {
    float v = atof(audioServer->arg("band_tc").c_str());
    if (v >= BAND_TC_MIN_MIN && v <= BAND_TC_MAX_MIN && v != bandFloorTcMin) {
      bandFloorTcMin = v;
      // Changing the EMA window changes the dynamics: re-seed the floors from the
      // running baseline and hold off triggers until they re-stabilize.
      if (spectralTrigEnabled) { bandSeedReq = true; bandWarmupUntil = millis() + BAND_WARMUP_MS; }
    }
  }
  if (audioServer->hasArg("spectro_min")) {
    uint32_t v = strtoul(audioServer->arg("spectro_min").c_str(), NULL, 10);
    if (v >= SPECTRO_MIN_MINUTES && v <= SPECTRO_MAX_MINUTES && v != spectroMinutes) {
      // Same pattern as hist_win: note the cadence the ring holds now, then let
      // the capture task re-bucket to the new one - history is preserved.
      if (!spectroRebucketReq) spectroRebucketOldMs = spectroBucketMs();
      spectroMinutes = v;
      spectroRebucketReq = true;
    }
  }
  if (audioServer->hasArg("hist_win")) {
    uint32_t v = strtoul(audioServer->arg("hist_win").c_str(), NULL, 10);
    if (v >= HIST_WIN_MIN_MIN && v <= HIST_WIN_MAX_MIN && v != histWindowMin) {
      // Record the cadence the ring holds NOW (unless an earlier change is
      // still pending - the ring is then still at that older cadence), then
      // let the capture task re-bucket to the new one, preserving history.
      if (!histRebucketReq) histRebucketOldMs = histBucketMs();
      histWindowMin = v;
      histRebucketReq = true;
    }
  }
  if (audioServer->hasArg("en")) {
    bool was = audioEnabled;
    audioEnabled = (audioServer->arg("en") == "1");
    // Wake the capture task if it had self-suspended (full-disable mode).
    if (audioEnabled && !was && tAudio != NULL) xTaskNotifyGive(tAudio);
    // Re-seed + warm up the band floors on resume so a stale floor can't fire.
    if (audioEnabled && !was && spectralTrigEnabled) {
      bandSeedReq = true; bandWarmupUntil = millis() + BAND_WARMUP_MS;
    }
  }
  if (audioServer->hasArg("auto")) {
    autoTriggerEnabled = (audioServer->arg("auto") == "1");
  }
  if (audioServer->hasArg("sd")) {
    saveToSd = (audioServer->arg("sd") == "1");
  }
  if (audioServer->hasArg("sd_max")) {
    uint32_t v = strtoul(audioServer->arg("sd_max").c_str(), NULL, 10);
    // Clamp like the other fields: 0 = unlimited, upper bound is the file-number
    // space (0..9999) since a card realistically holds at most a few thousand clips.
    if (v > 9999) v = 9999;
    sdMaxClips = v;
    sdEnforceCap(); // lowering the cap prunes immediately
  }
  if (audioServer->hasArg("slots")) {
    int want = (int)strtol(audioServer->arg("slots").c_str(), NULL, 10);
    int got = setClipSlotCount(want);
    if (got != want) {
      Serial.printf("[audio] slot count clamped: wanted %d, got %d\n", want, got);
    }
  }
  saveSettings();
  handleAudioStatus();
}

// ---------------- Public API ----------------

bool audioInit() {
  loadSettings();
  spectrumInit();
  ring = (int16_t *)ps_calloc(RING_SAMPLES, sizeof(int16_t));
  if (ring == NULL) {
    Serial.println("[audio] ring buffer allocation failed");
    return false;
  }
  if (setClipSlotCount(nvsSlots) < 1) {
    Serial.println("[audio] clip slot allocation failed");
    return false;
  }

  clipIndexInit(); // RAM /clips index (before sdInit's scan populates it)
  clipMetaInit();  // RAM over-threshold ring (loaded from SD by sdInit's scan)
  sdMutex = xSemaphoreCreateMutex();
  sdWriteQueue = xQueueCreate(MAX_CLIP_SLOTS, sizeof(int));
  if (sdBootSkip) {
    // Previous boot hung inside init: the card is the suspect and a mount can
    // block the SPI bus forever, so don't touch it. The device stays reachable;
    // /sd/remount (or a plain reboot) retries once the card is fixed/replaced.
    sdAvailable = false;
    dlog(DLOG_ERR, "SD skipped: last boot hung in init (card suspect) - "
                   "fix/replace card, then /sd/remount or reboot");
  } else {
    sdAvailable = sdInit();
    if (sdAvailable)
      dlog(DLOG_INFO, "SD mounted at boot (%llu/%lluMB)", sdUsedMbCache, sdTotalMbCache);
    else
      dlog(DLOG_WARN, "SD NOT mounted at boot - clips PSRAM-only, auto-retrying");
  }

  audioStreamClients = xQueueCreate(MAX_AUDIO_STREAM_CLIENTS, sizeof(WiFiClient *));

  histBuf = (uint16_t *)ps_calloc(HIST_BUCKETS, sizeof(uint16_t));
  thrHistBuf = (uint16_t *)ps_calloc(HIST_BUCKETS, sizeof(uint16_t));
  if (histBuf == NULL) Serial.println("[audio] history buffer alloc failed - plots disabled");
  bucketStartMs = millis();

  eventLog = (ClipEvent *)ps_calloc(EVENT_LOG_SIZE, sizeof(ClipEvent));
  if (eventLog == NULL) Serial.println("[audio] event log alloc failed - plot markers disabled");

  i2s_config_t i2sConfig = {};
  i2sConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
  i2sConfig.sample_rate = AUDIO_SAMPLE_RATE;
  i2sConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2sConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  i2sConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2sConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2sConfig.dma_buf_count = 8;
  i2sConfig.dma_buf_len = CHUNK_SAMPLES;
  i2sConfig.use_apll = false;

  if (i2s_driver_install(AUDIO_I2S_PORT, &i2sConfig, 0, NULL) != ESP_OK) {
    Serial.println("[audio] i2s_driver_install failed");
    return false;
  }

  i2s_pin_config_t pinConfig = {};
  pinConfig.mck_io_num = I2S_PIN_NO_CHANGE;
  pinConfig.bck_io_num = I2S_PIN_NO_CHANGE;
  pinConfig.ws_io_num = PDM_CLK_PIN;   // PDM clock rides the WS line in RX mode
  pinConfig.data_out_num = I2S_PIN_NO_CHANGE;
  pinConfig.data_in_num = PDM_DATA_PIN;

  if (i2s_set_pin(AUDIO_I2S_PORT, &pinConfig) != ESP_OK) {
    Serial.println("[audio] i2s_set_pin failed");
    i2s_driver_uninstall(AUDIO_I2S_PORT);
    return false;
  }

  Serial.printf("[audio] PDM mic ready: %dHz mono, ring %us, %d clip slots x %us max\n",
                AUDIO_SAMPLE_RATE, (MAX_CLIP_MS + 1000) / 1000, clipSlotCount,
                MAX_CLIP_MS / 1000);
  return true;
}

// ---------------- continuous (loop) recording ----------------
//
// Writes back-to-back WAV segments straight to SD, following the capture ring
// with its own cursor like audioStreamTask. Bypasses the PSRAM clip slots
// entirely (a 60s segment is ~1.9MB - far over a slot). Retention (minutes +
// MB caps) is enforced separately by the pruner in cont_record.cpp; this task
// only produces cont_NNNNN_a.wav files. SD-write failure pauses the stream and
// is retried once the card is back.

static volatile bool     contAudOn = false;
static volatile uint32_t contAudSegMs = 60000;
static volatile bool     contAudRec = false;    // mid-segment right now
static volatile bool     contAudSdFail = false; // last write/open failed
static volatile uint32_t contAudFile = 0;       // current/last segment number
static uint32_t contAudCounter = 0;             // next number (seeded from SD)
static TaskHandle_t      tContAud = NULL;

// Seed the segment counter past whatever cont_*_a.wav files already exist.
static void contAudSeed() {
  if (!sdAvailable) return;
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File dir = SD.open(SD_CLIP_DIR);
  if (dir) {
    File e;
    while ((e = dir.openNextFile())) {
      const char *base = strrchr(e.name(), '/');
      base = base ? base + 1 : e.name();
      unsigned idx;
      if (sscanf(base, "cont_%u_a.wav", &idx) == 1 && idx + 1 > contAudCounter)
        contAudCounter = idx + 1;
      e.close();
    }
    dir.close();
  }
  xSemaphoreGive(sdMutex);
}

static void contAudCloseSeg(File &f, uint32_t pcmBytes) {
  uint8_t hdr[WAV_HEADER_BYTES];
  writeWavHeader(hdr, pcmBytes);
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  f.seek(0);
  f.write(hdr, WAV_HEADER_BYTES);
  f.close();
  xSemaphoreGive(sdMutex);
}

static void contAudTask(void *pv) {
  static int16_t buf[2048]; // 4KB SD write chunk
  size_t readPos = 0;
  File f;
  bool open = false;
  uint32_t pcmBytes = 0, wantSamples = 0;
  char path[48];

  for (;;) {
    // Idle / blocked: capture off, continuous off, no card, or the card is low
    // on space (the combined-budget backstop pauses NEW continuous segments so
    // triggered event clips keep their headroom; see sdLowSpace()).
    if (!contAudOn || !audioEnabled || !sdAvailable || sdLowSpace()) {
      if (open) { contAudCloseSeg(f, pcmBytes); open = false; }
      contAudRec = false;
      portENTER_CRITICAL(&audioMux);
      readPos = ringWrite; // stay live so we don't dump stale backlog on resume
      portEXIT_CRITICAL(&audioMux);
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (!open) {
      uint32_t num = contAudCounter;
      snprintf(path, sizeof(path), SD_CLIP_DIR "/cont_%05u_a.wav", num);
      uint8_t hdr[WAV_HEADER_BYTES];
      writeWavHeader(hdr, 0); // placeholder; patched at close
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      f = SD.open(path, FILE_WRITE);
      bool okh = f && f.write(hdr, WAV_HEADER_BYTES) == WAV_HEADER_BYTES;
      xSemaphoreGive(sdMutex);
      if (!okh) {
        // Open/create failure: could be a full card (ENOSPC) rather than a
        // removed one, so DON'T mark the card down here (that would thrash
        // unmount/remount on a full card). The retry + liveness probe handle it.
        if (f) f.close();
        contAudSdFail = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      contAudSdFail = false;
      contAudFile = num;
      contAudRec = true;
      pcmBytes = 0;
      wantSamples = contAudSegMs * SAMPLES_PER_MS;
      open = true;
      portENTER_CRITICAL(&audioMux);
      readPos = ringWrite;
      portEXIT_CRITICAL(&audioMux);
    }

    portENTER_CRITICAL(&audioMux);
    size_t w = ringWrite;
    portEXIT_CRITICAL(&audioMux);
    size_t avail = (w + RING_SAMPLES - readPos) % RING_SAMPLES;
    size_t remain = wantSamples - pcmBytes / 2;
    if (avail > remain) avail = remain;
    if (avail == 0) { vTaskDelay(pdMS_TO_TICKS(40)); }

    while (avail > 0) {
      size_t n = avail > 2048 ? 2048 : avail;
      size_t first = RING_SAMPLES - readPos;
      if (first > n) first = n;
      memcpy(buf, ring + readPos, first * 2);
      if (first < n) memcpy(buf + first, ring, (n - first) * 2);
      xSemaphoreTake(sdMutex, portMAX_DELAY);
      size_t wr = f.write((uint8_t *)buf, n * 2);
      xSemaphoreGive(sdMutex);
      if (wr != n * 2) { // card pulled / full
        xSemaphoreTake(sdMutex, portMAX_DELAY);
        f.close();
        xSemaphoreGive(sdMutex);
        contAudSdFail = true;
        contAudRec = false;
        open = false;
        sdMarkDown(); // failed write = card gone/wedged; let health recover it
        break;
      }
      readPos = (readPos + n) % RING_SAMPLES;
      pcmBytes += n * 2;
      avail -= n;
    }

    if (open && pcmBytes / 2 >= wantSamples) {
      contAudCloseSeg(f, pcmBytes);
      open = false;
      contAudRec = false;
      contAudCounter++;
    }
  }
}

void audioContSet(bool on, uint32_t segMs) {
  if (segMs) contAudSegMs = segMs;
  contAudOn = on;
}

String audioContJson() {
  String s;
  s.reserve(80);
  s += "\"a_on\":";   s += contAudOn ? "true" : "false";
  s += ",\"a_rec\":"; s += contAudRec ? "true" : "false";
  s += ",\"a_file\":"; s += String(contAudFile);
  s += ",\"a_sd\":";  s += contAudSdFail ? "false" : "true";
  return s;
}

void audioStart() {
  // Seed + warm up the band trigger at boot too (it may load enabled from NVS),
  // so the per-band thresholds settle before the first triggers can fire.
  if (spectralTrigEnabled) { bandSeedReq = true; bandWarmupUntil = millis() + BAND_WARMUP_MS; }
  // PRO_CPU (core 0), alongside streamCB; camera capture owns core 1.
  xTaskCreatePinnedToCore(audioTask, "audio", 4096, NULL, 4, &tAudio, 0);
  xTaskCreatePinnedToCore(audioStreamTask, "audio_stream", 4096, NULL, 3, &tAudioStream, 0);
  // Always run the writer task: even if the card didn't mount at boot, its idle
  // health tick keeps trying so a card inserted/recovered later comes up on its
  // own. It also handles history save/restore once a card is present.
  xTaskCreatePinnedToCore(sdWriterTask, "sd_writer", 4096, NULL, 2, &tSdWriter, 0);
  if (sdAvailable) contAudSeed();
  xTaskCreatePinnedToCore(contAudTask, "cont_aud", 4096, NULL, 2, &tContAud, 0);
}

bool sdIsAvailable() { return sdAvailable; }

SemaphoreHandle_t sdGetMutex() { return sdMutex; }

void sdNotifyWriteFailed() { sdMarkDown(); } // external drop signal (video recorder)

uint32_t sdFreeMb() {
  if (!sdAvailable || sdTotalMbCache == 0) return 0;
  return sdTotalMbCache > sdUsedMbCache ? (uint32_t)(sdTotalMbCache - sdUsedMbCache) : 0;
}

bool sdLowSpace() { return sdLowSpaceFlag; }

// Re-evaluate the sticky low-space backstop from the cached capacity figures.
// Owned by the writer task (single writer); the continuous tasks and HTTP status
// only read sdLowSpaceFlag. Hysteresis: trips below the floor, clears only once
// free recovers past floor + margin (the continuous pruner frees space in
// segment-sized chunks, so a bare threshold would oscillate).
static void sdUpdateLowSpace() {
  if (!sdAvailable || sdTotalMbCache == 0) { sdLowSpaceFlag = false; return; }
  // Pure hysteresis (sd_health.h, unit-tested in test/test_sd_health).
  sdLowSpaceFlag = sdhealth::lowSpaceNext(sdLowSpaceFlag, sdFreeMb(),
                                          SD_FREE_FLOOR_MB, SD_FREE_HYST_MB);
}

bool sdTryLock(uint32_t ms) {
  return sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(ms)) == pdTRUE;
}
void sdUnlock() { xSemaphoreGive(sdMutex); }

uint32_t sdDrops() { return sdDropCount; }
uint32_t sdSecsSinceDrop() {
  return sdLastDropMs == 0 ? UINT32_MAX : (millis() - sdLastDropMs) / 1000;
}
uint8_t sdMountClockMhz() { return sdAvailable ? sdMountMhz : 0; }

uint32_t audioTriggerFromVideo() { return startClip(TRIGGER_SOURCE_VIDEO); }

/** Reject anything that isn't a bare clip_*.wav, vclip_*.avi, or its
 *  vclip_*.mot motion-map sidecar. */
// Thin shell over the pure guard (path_safe.h, unit-tested).
static bool validClipName(const String &name) {
  return pathsafe::validClipName(name.c_str());
}

/**
 * Stream an already-open SD file to the HTTP client in chunks, releasing
 * sdMutex between each read so the recorders, history snapshot, and other SD
 * users aren't blocked for the whole (possibly multi-second, slow-link)
 * transfer. The old streamFile() held the bus across the entire download,
 * which stalled every SD writer and made concurrent /sd/list 503. The caller
 * opens `f` under the lock and reads its size; this function reads each chunk
 * under a fresh short lock, writes it to the socket unlocked, then closes `f`.
 * `dlName` non-NULL emits a download Content-Disposition.
 */
static void streamSdFileChunked(File &f, uint32_t size, const char *ctype,
                                const char *dlName) {
  String hdr = "HTTP/1.1 200 OK\r\nContent-Type: ";
  hdr += ctype;
  hdr += "\r\nContent-Length: ";
  hdr += String(size);
  hdr += "\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n";
  // Key-frame thumbnails are immutable (a clip's .jpg sidecar is written once,
  // and a reused clip number gets a different name only after 10k wraps): let the
  // browser cache them so the tables' periodic re-render doesn't refetch every
  // thumbnail from SD (which read as constant flicker/reload in the UI).
  if (strcmp(ctype, "image/jpeg") == 0)
    hdr += "Cache-Control: public, max-age=604800, immutable\r\n";
  if (dlName) {
    hdr += "Content-Disposition: attachment; filename=";
    hdr += dlName;
    hdr += "\r\n";
  }
  hdr += "\r\n";

  WiFiClient client = audioServer->client();
  client.write(hdr.c_str(), hdr.length());

  static uint8_t buf[SD_STREAM_CHUNK]; // single-threaded server task only
  uint32_t sent = 0;
  while (sent < size && client.connected()) {
    // Wait patiently per chunk (Content-Length is committed; aborting would
    // truncate). Still releases between chunks so recorders interleave; only a
    // genuinely wedged card exhausts SD_DL_LOCK_MS.
    if (!sdTryLock(SD_DL_LOCK_MS)) break;
    int n = f.read(buf, sizeof(buf));
    sdUnlock();
    if (n <= 0) break; // EOF or read error (e.g. card dropped mid-download)
    if (client.write(buf, n) != (size_t)n) break; // client slow/gone
    sent += n;
  }
  if (sdTryLock(SD_DL_LOCK_MS)) { f.close(); sdUnlock(); }
  else f.close(); // best effort; never leak the handle
  client.stop();
}

static void handleSdList() {
  if (!sdAvailable) {
    audioServer->send(503, "application/json", "{\"error\":\"no sd card\"}");
    return;
  }
  // Served entirely from the in-RAM index - NO SD access, so it can't 503 on bus
  // contention and is near-instant even on a slow card with hundreds of clips.
  String json;
  json.reserve(8192);
  clipIndexListJson(CLIP_AUDIO, SD_LIST_MAX, json);
  audioServer->send(200, "application/json", json);
}

static void handleSdFile() {
  if (!sdAvailable || !audioServer->hasArg("name")) {
    audioServer->send(404, "text/plain", "missing sd card or name");
    return;
  }
  String name = audioServer->arg("name");
  if (!validClipName(name)) {
    audioServer->send(400, "text/plain", "bad name");
    return;
  }
  String path = String(SD_CLIP_DIR) + "/" + name;

  // Wait a few seconds (not the 600ms quick timeout) for the bus so a clip read
  // losing the race to a routine recorder write doesn't bounce the play with a
  // 503; see SD_DL_OPEN_LOCK_MS. The browser still retries 503 for longer stalls.
  if (!sdTryLock(SD_DL_OPEN_LOCK_MS)) {
    audioServer->send(503, "text/plain", "sd busy, retry");
    return;
  }
  File f = SD.open(path, FILE_READ);
  if (!f) {
    sdUnlock();
    audioServer->send(404, "text/plain", "not found");
    return;
  }
  uint32_t size = f.size();
  sdUnlock(); // chunked streamer re-locks per read so the download can't stall writers

  bool dl = audioServer->hasArg("dl") && audioServer->arg("dl") == "1";
  const char *ctype = name.endsWith(".avi")   ? "video/x-msvideo"
                    : name.endsWith(".mot")   ? "application/json"
                    : name.endsWith(".jpg")   ? "image/jpeg"
                                              : "audio/wav";
  streamSdFileChunked(f, size, ctype, dl ? name.c_str() : nullptr);
}

/** Forget the learned noise floor and re-learn from current ambient. */
static void handleAudioRetune() {
  noiseFloor = 0.0f; noiseVar = 0.0f;   // forget the floor + running stats
  warmupUntilMs = millis() + WARMUP_MS; // no auto triggers while settling
  if (spectralTrigEnabled) {            // same for the per-band floors
    bandSeedReq = true; bandWarmupUntil = millis() + BAND_WARMUP_MS;
  }
  handleAudioStatus();
}

/**
 * Amplitude history as packed uint16 LE peaks, oldest first. Decimated
 * server-side (max-pooling) to ?points, covering the last ?secs seconds.
 * Returns fewer points when less history exists.
 */
static void handleAudioHistory() {
  static uint16_t out[2000]; // server task only; 4KB
  uint32_t secs = audioServer->hasArg("secs")
                      ? strtoul(audioServer->arg("secs").c_str(), NULL, 10)
                      : 3600;
  if (secs < 60) secs = 60;
  if (secs > 86400) secs = 86400;
  uint32_t points = audioServer->hasArg("points")
                        ? strtoul(audioServer->arg("points").c_str(), NULL, 10)
                        : 720;
  if (points < 16) points = 16;
  if (points > 2000) points = 2000;
  // ?end=N shifts the window's right edge N ms into the past (0 = now), so the
  // dashboard can zoom into an arbitrary inner range at full bucket detail.
  uint32_t endMs = audioServer->hasArg("end")
                       ? strtoul(audioServer->arg("end").c_str(), NULL, 10)
                       : 0;

  uint32_t cnt, w, seq;
  portENTER_CRITICAL(&audioMux);
  cnt = histCount;
  w = histWrite;
  seq = histSeq;
  portEXIT_CRITICAL(&audioMux);

  // ?thr=1 serves the trigger-threshold series instead of amplitude peaks
  const uint16_t *src = (audioServer->hasArg("thr") && audioServer->arg("thr") == "1")
                            ? thrHistBuf : histBuf;

  uint32_t bucketMs = histBucketMs();
  uint32_t endBuckets = endMs / bucketMs;                 // skip from the newest
  uint32_t avail = history::availableBuckets(cnt, endBuckets);
  uint32_t want = secs * 1000UL / bucketMs;
  if (want > avail) want = avail;

  // Grid-aligned, integer-factor pooling so each column max-pools a fixed set of
  // source buckets: the historical part of the line is byte-identical every
  // refresh (no shimmer); only the newest column turns over. The grid index is
  // the ring's own bucket-close counter (snapshotted with w/cnt above) so it
  // ticks at the exact instant the write index does - a wall-clock index here
  // ticks at a different moment, and a poll landing between the two ticks
  // regrouped every column (whole-line shimmer on ~half the refreshes).
  history::PoolPlan pl =
      history::planAlignedPool(want, points, cnt, endBuckets, seq);
  want = pl.want;
  points = pl.columns;
  endBuckets += pl.extraEnd;

  // Pure decimation (history.h, unit-tested); returns the effective point count.
  points = history::decimateMaxPool(src, HIST_BUCKETS, w, endBuckets, want, points, out);

  WiFiClient client = audioServer->client();
  char hdr[192];
  int hl = snprintf(hdr, sizeof(hdr),
                    "HTTP/1.1 200 OK\r\n"
                    "Content-Type: application/octet-stream\r\n"
                    "Content-Length: %u\r\n"
                    "X-Bucket-Ms: %u\r\n"
                    "Access-Control-Allow-Origin: *\r\n"
                    "Connection: close\r\n\r\n",
                    (unsigned)(points * 2),
                    points ? (unsigned)(want * bucketMs / points) : 0);
  client.write(hdr, hl);
  if (points > 0) client.write((uint8_t *)out, points * 2);
  client.stop();
}

/** Trigger events within the last ?secs, newest first, capped at 200. */
static void handleAudioEvents() {
  uint32_t secs = audioServer->hasArg("secs")
                      ? strtoul(audioServer->arg("secs").c_str(), NULL, 10)
                      : 3600;
  if (secs < 60) secs = 60;
  if (secs > 86400) secs = 86400;

  uint32_t cnt, w;
  portENTER_CRITICAL(&audioMux);
  cnt = eventCount;
  w = eventWrite;
  portEXIT_CRITICAL(&audioMux);

  uint32_t now = millis();
  String json = "[";
  json.reserve(4096); // up to 200 events; pre-size to avoid realloc churn
  int emitted = 0;
  for (uint32_t k = 0; k < cnt && emitted < 200 && eventLog != NULL; k++) {
    const ClipEvent &ev = eventLog[(w + EVENT_LOG_SIZE - 1 - k) % EVENT_LOG_SIZE];
    uint32_t age = now - ev.tsMs;
    if (age > secs * 1000UL) break;
    if (emitted > 0) json += ",";
    json += "{\"age_ms\":" + String(age);
    json += ",\"id\":" + String(ev.id);
    json += ",\"sd\":" + (ev.sdIdx == UINT32_MAX ? String(-1) : String(ev.sdIdx));
    json += ",\"rms\":" + String(ev.trigRms);
    json += ",\"thr\":" + String(ev.trigThr);
    json += ",\"hz\":" + String(ev.trigHz);
    json += ",\"source\":\"";
    json += srcName(ev.source);
    json += "\"}";
    emitted++;
  }
  json += "]";
  audioServer->send(200, "application/json", json);
}

// Persisted over-threshold metadata for the SD clips table: {"<num>":[rms,thr,hz]}.
// Served from the RAM ring (no SD hit), so it covers clips across reboots.
static void handleClipMeta() {
  String json = "{";
  json.reserve((uint32_t)clipMetaCount * 26 + 16);
  if (clipMeta && clipMetaMutex && clipMetaSnap) {
    // Snapshot the ring (oldest-first) under the lock, then GIVE it before
    // building the multi-KB String, so the capture task isn't stalled across
    // the serialize.
    uint16_t count;
    xSemaphoreTake(clipMetaMutex, portMAX_DELAY);
    count = clipMetaCount;
    uint16_t start = (clipMetaHead + CLIPMETA_CAP - clipMetaCount) % CLIPMETA_CAP;
    for (uint16_t i = 0; i < count; i++) clipMetaSnap[i] = clipMeta[(start + i) % CLIPMETA_CAP];
    xSemaphoreGive(clipMetaMutex);
    for (uint16_t i = 0; i < count; i++) {
      const ClipMeta &m = clipMetaSnap[i];
      if (i) json += ",";
      json += "\"" + String(m.num) + "\":[" + String(m.rms) + "," +
              String(m.thr) + "," + String(m.hz) + "]";
    }
  } else if (clipMeta && clipMetaMutex) {
    // Snapshot buffer alloc failed: fall back to building under the lock.
    xSemaphoreTake(clipMetaMutex, portMAX_DELAY);
    uint16_t start = (clipMetaHead + CLIPMETA_CAP - clipMetaCount) % CLIPMETA_CAP;
    for (uint16_t i = 0; i < clipMetaCount; i++) {
      const ClipMeta &m = clipMeta[(start + i) % CLIPMETA_CAP];
      if (i) json += ",";
      json += "\"" + String(m.num) + "\":[" + String(m.rms) + "," +
              String(m.thr) + "," + String(m.hz) + "]";
    }
    xSemaphoreGive(clipMetaMutex);
  }
  json += "}";
  audioServer->send(200, "application/json", json);
}

static void handleAudioClear() {
  int cleared = 0;
  portENTER_CRITICAL(&audioMux);
  for (int i = 0; i < clipSlotCount; i++) {
    if (slots[i].ready && slots[i].busy == 0) {
      slots[i].ready = false;
      slots[i].id = 0;
      cleared++;
    }
  }
  portEXIT_CRITICAL(&audioMux);
  char json[48];
  snprintf(json, sizeof(json), "{\"cleared\":%d}", cleared);
  audioServer->send(200, "application/json", json);
}

static void handleSdClear() {
  if (!sdAvailable) {
    audioServer->send(503, "text/plain", "no sd card");
    return;
  }
  // Collect-then-delete in batches: removing entries while iterating a FAT
  // directory is unreliable.
  int deleted = 0;
  for (;;) {
    char names[16][32];
    int n = 0;
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File dir = SD.open(SD_CLIP_DIR);
    if (dir) {
      File entry;
      while (n < 16 && (entry = dir.openNextFile())) {
        const char *base = strrchr(entry.name(), '/');
        base = base ? base + 1 : entry.name();
        if (strncmp(base, "clip_", 5) == 0) { // never delete history.bin
          strlcpy(names[n++], base, sizeof(names[0]));
        }
        entry.close();
      }
      dir.close();
    }
    for (int i = 0; i < n; i++) {
      if (SD.remove(String(SD_CLIP_DIR) + "/" + names[i])) deleted++;
    }
    xSemaphoreGive(sdMutex);
    if (n == 0) break;
  }
  clipIndexClear(CLIP_AUDIO); // all audio clips gone
  clipMetaClear();            // and their persisted over-threshold metadata
  char json[48];
  snprintf(json, sizeof(json), "{\"deleted\":%d}", deleted);
  audioServer->send(200, "application/json", json);
}

// Factory-reset helper: like handleSdClear but removes ALL files (including
// vclip_* and history.bin), not just clip_*. Called from the BOOT-button
// recovery path in main.cpp, not over HTTP.
void audioSdWipeAll() {
  if (!sdAvailable) return;
  int deleted = 0;
  for (;;) {
    char names[16][32];
    int n = 0;
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File dir = SD.open(SD_CLIP_DIR);
    if (dir) {
      File entry;
      while (n < 16 && (entry = dir.openNextFile())) {
        const char *base = strrchr(entry.name(), '/');
        base = base ? base + 1 : entry.name();
        strlcpy(names[n++], base, sizeof(names[0]));
        entry.close();
      }
      dir.close();
    }
    for (int i = 0; i < n; i++) {
      if (SD.remove(String(SD_CLIP_DIR) + "/" + names[i])) deleted++;
    }
    xSemaphoreGive(sdMutex);
    if (n == 0) break;
  }
  clipIndexClear(-1); // factory wipe: drop every indexed clip (audio + video)
  clipMetaClear();    // and the over-threshold ring (file already removed above)
  Serial.printf("[reset] wiped %d files from %s\n", deleted, SD_CLIP_DIR);
}

static void handleSdRemount() {
  // A deliberate remount lifts boot-skip mode: the user is telling us the card
  // has been dealt with, so mounting and the health tick's auto-retry are fair
  // game again.
  sdBootSkip = false;
  // One attempt only: the full SD_MOUNT_ATTEMPTS retry holds sdMutex (and this
  // single-threaded server task) for seconds on a flaky card. The health tick's
  // backoff keeps retrying afterward if this one misses, same as sdHealthTick.
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  sdAvailable = sdInit(1);
  xSemaphoreGive(sdMutex);
  // Re-sync the monitor's backoff so it doesn't immediately re-probe/contend,
  // and re-seed the continuous counter past any files already on the card.
  sdRecoverDelayMs = SD_RECOVER_MIN_MS;
  sdNextProbeMs = millis() + SD_PROBE_MS;
  if (sdAvailable) contAudSeed();
  char json[64];
  snprintf(json, sizeof(json), "{\"sd\":%s}", sdAvailable ? "true" : "false");
  audioServer->send(200, "application/json", json);
}

static void handleSdDelete() {
  if (!sdAvailable || !audioServer->hasArg("name")) {
    audioServer->send(404, "text/plain", "missing sd card or name");
    return;
  }
  String name = audioServer->arg("name");
  if (!validClipName(name)) {
    audioServer->send(400, "text/plain", "bad name");
    return;
  }
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  bool ok = SD.remove(String(SD_CLIP_DIR) + "/" + name);
  if (ok && name.endsWith(".avi")) {
    String mot = name; // motion-map sidecar goes with its clip
    mot.replace(".avi", ".mot");
    SD.remove(String(SD_CLIP_DIR) + "/" + mot);
  }
  if (ok) { // key-frame thumbnail sidecar (audio clip_*.jpg or video vclip_*.jpg)
    String thumb = name;
    thumb.replace(name.endsWith(".avi") ? ".avi" : ".wav", ".jpg");
    SD.remove(String(SD_CLIP_DIR) + "/" + thumb);
  }
  xSemaphoreGive(sdMutex);
  if (ok) clipIndexRemove(name.c_str()); // drop from the index (audio clip_ or video vclip_)
  audioServer->send(ok ? 200 : 404, "text/plain", ok ? "deleted" : "not found");
}

/**
 * Live audio: endless WAV over HTTP, same subscribe-and-return pattern as
 * the MJPEG stream so the server task is never tied up.
 */
static void handleAudioStream() {
  if (!audioEnabled) {
    audioServer->send(503, "text/plain", "audio is disabled");
    return;
  }
  if (audioStreamClients == NULL || !uxQueueSpacesAvailable(audioStreamClients)) {
    audioServer->send(503, "text/plain", "live stream slots full");
    return;
  }

  static const char STREAM_HTTP[] = "HTTP/1.1 200 OK\r\n"
                                    "Content-Type: audio/wav\r\n"
                                    "Access-Control-Allow-Origin: *\r\n"
                                    "Cache-Control: no-store\r\n"
                                    "Connection: close\r\n\r\n";

  WiFiClient *client = new WiFiClient();
  *client = audioServer->client();
  client->setNoDelay(true);
  client->write(STREAM_HTTP, sizeof(STREAM_HTTP) - 1);
  uint8_t wav[WAV_HEADER_BYTES];
  writeWavHeader(wav, 0xF0000000u); // ~37h of 16kHz PCM: effectively endless
  client->write(wav, WAV_HEADER_BYTES);

  // The spaces-available check above makes this succeed on the single-threaded
  // server today, but never leak the heap WiFiClient + socket if the enqueue
  // ever fails (matches the MJPEG stream path).
  if (xQueueSend(audioStreamClients, &client, 0) != pdTRUE) {
    client->stop();
    delete client;
  }
}

static void handleAudioPage() {
  audioServer->send_P(200, "text/html", AUDIO_PAGE);
}

static void handleLivePage() {
  audioServer->send_P(200, "text/html", LIVE_PAGE);
}

static void handlePlayerJs() {
  audioServer->send_P(200, "application/javascript", PLAYER_JS);
}

// Stream the full diag log mirrored on SD (the RTC ring at /log only holds the
// most recent ~48 entries; this is the complete history, surviving power loss).
static void handleDiagLogFile() {
  if (!sdAvailable) {
    audioServer->send(503, "text/plain", "no sd card");
    return;
  }
  if (!sdTryLock(SD_HTTP_LOCK_MS)) {
    audioServer->send(503, "text/plain", "sd busy, retry");
    return;
  }
  File f = SD.open(DIAG_LOG_PATH, FILE_READ);
  if (!f) {
    sdUnlock();
    audioServer->send(404, "text/plain", "no diag.log yet");
    return;
  }
  uint32_t size = f.size();
  sdUnlock();
  streamSdFileChunked(f, size, "text/plain", nullptr); // chunked: never lock the bus across the transfer
}

// Digest auth on every route (NVS-backed credentials; see web_auth.cpp).
// Wrapping at registration keeps the handlers themselves auth-free.
static WebServer::THandlerFunction guarded(void (*h)()) {
  return [h]() {
    if (!webAuthCheck(*audioServer)) return;
    h();
  };
}

// Same, but for MUTATING routes: the read-only viewer is rejected with 403.
static WebServer::THandlerFunction guardedW(void (*h)()) {
  return [h]() {
    if (!webAuthRequireWrite(*audioServer)) return;
    h();
  };
}

void audioRegisterEndpoints(WebServer &server) {
  audioServer = &server;
  server.on("/", HTTP_GET, guarded(handleAudioPage));
  server.on("/audio", HTTP_GET, guarded(handleAudioPage));
  server.on("/live", HTTP_GET, guarded(handleLivePage));
  server.on("/audio/player.js", HTTP_GET, guarded(handlePlayerJs));
  server.on("/diag.log", HTTP_GET, guarded(handleDiagLogFile));
  server.on("/audio/stream", HTTP_GET, guarded(handleAudioStream));
  server.on("/audio/trigger", HTTP_GET, guardedW(handleAudioTrigger));
  server.on("/audio/clip", HTTP_GET, guarded(handleAudioClip));
  server.on("/audio/clips", HTTP_GET, guarded(handleAudioClips));
  server.on("/audio/status", HTTP_GET, guarded(handleAudioStatus));
  server.on("/audio/config", HTTP_GET, guardedW(handleAudioConfig));
  server.on("/audio/retune", HTTP_GET, guardedW(handleAudioRetune));
  server.on("/audio/history", HTTP_GET, guarded(handleAudioHistory));
  server.on("/audio/events", HTTP_GET, guarded(handleAudioEvents));
  server.on("/audio/clipmeta", HTTP_GET, guarded(handleClipMeta));
  server.on("/audio/spectrum", HTTP_GET, guarded(handleAudioSpectrum));
  server.on("/audio/spectrogram", HTTP_GET, guarded(handleAudioSpectrogram));
  server.on("/audio/clear", HTTP_GET, guardedW(handleAudioClear));
  server.on("/sd/clear", HTTP_GET, guardedW(handleSdClear));
  server.on("/sd/remount", HTTP_GET, guardedW(handleSdRemount));
  server.on("/sd/list", HTTP_GET, guarded(handleSdList));
  server.on("/sd/file", HTTP_GET, guarded(handleSdFile));
  server.on("/sd/delete", HTTP_GET, guardedW(handleSdDelete));
}

#endif // CAMERA_MODEL_XIAO_ESP32S3

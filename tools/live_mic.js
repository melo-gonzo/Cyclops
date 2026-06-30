// Live-mic emulator for the Cyclops audio dashboard. Injected before the page's
// own script; overrides fetch() so /audio/* endpoints are driven by the laptop
// microphone via the Web Audio API, reproducing the device's DSP behaviour
// (32 log bands 60-8kHz, adaptive per-band + scalar thresholds, dBFS heatmap,
// level/threshold history, auto + spectral triggers). Display-faithful, not
// sample-accurate - it's for exercising the UI with real sound.
(function () {
  "use strict";
  const NB = 32, FMIN = 60, FMAX = 8000, DBREF = 132.4, HBUCKETS = 43200, SCOLS = 240;
  // Config mirrors /audio/status + /audio/spectrum; mutated by /audio/config.
  const cfg = { enabled: 1, auto: 1, gain: 16, factor: 3, min_thr: 8000, manual_thr: 0, hab_min: 5,
    lpf: 0, lpf_hz: 4000, hpf: 1, hpf_hz: 120, strig: 1, sfactor: 4, sminmag: 1000, band_tc: 2,
    spectro_min: 4, hist_win: 1440, clip_ms: 5000, preroll_ms: 1000 };

  let mic = false, ac = null, an = null, freq = null, tim = null, sr = 48000;
  const bandHz = []; for (let b = 0; b < NB; b++) bandHz.push(Math.round(FMIN * Math.pow(FMAX / FMIN, (b + 0.5) / NB)));
  const bandMag = new Float32Array(NB), bandFloor = new Float32Array(NB), bandThr = new Float32Array(NB);
  const bandDisp = new Float32Array(NB); // EMA of bandMag for the plot only (trigger uses raw)
  let level = 0, floorLvl = 0, thrLvl = cfg.min_thr;
  const levelBuf = [], thrBuf = []; let lastHistT = 0;
  const spectro = [], spAccum = new Float32Array(NB); let lastSpT = 0;
  let events = [], evId = 0, lastTrigT = 0, warmUntil = 0;
  const WARMUP_MS = 8000;
  function armWarm() { warmUntil = Date.now() + WARMUP_MS; } // hold off + fast-settle the floors

  const histBucketMs = () => Math.max(250, Math.round(cfg.hist_win * 60000 / HBUCKETS));
  const spBucketMs = () => Math.max(50, Math.round(cfg.spectro_min * 60000 / SCOLS));

  async function startMic() {
    ac = new (window.AudioContext || window.webkitAudioContext)();
    const stream = await navigator.mediaDevices.getUserMedia(
      { audio: { echoCancellation: false, noiseSuppression: false, autoGainControl: false } });
    sr = ac.sampleRate;
    an = ac.createAnalyser(); an.fftSize = 2048; an.smoothingTimeConstant = 0.3;
    ac.createMediaStreamSource(stream).connect(an);
    freq = new Float32Array(an.frequencyBinCount); tim = new Float32Array(an.fftSize);
    mic = true; armWarm();
    requestAnimationFrame(tick);
  }

  function readBands() {
    an.getFloatFrequencyData(freq);
    const binHz = sr / an.fftSize;
    for (let b = 0; b < NB; b++) {
      const f0 = FMIN * Math.pow(FMAX / FMIN, b / NB), f1 = FMIN * Math.pow(FMAX / FMIN, (b + 1) / NB);
      let lo = Math.max(1, Math.floor(f0 / binHz)), hi = Math.min(freq.length - 1, Math.ceil(f1 / binHz));
      if (hi <= lo) hi = lo + 1;
      let p = 0; for (let i = lo; i < hi; i++) { const d = freq[i]; if (isFinite(d)) p += Math.pow(10, d / 10); }
      const bandDb = p > 0 ? 10 * Math.log10(p) : -140;       // band energy, dBFS
      bandMag[b] = Math.max(0, Math.pow(10, (bandDb + DBREF) / 20) - 1); // device-style magnitude
      bandDisp[b] = bandDisp[b] > 0 ? bandDisp[b] + 0.06 * (bandMag[b] - bandDisp[b]) : bandMag[b]; // display EMA
    }
  }
  function readLevel() {
    an.getFloatTimeDomainData(tim);
    let s = 0; for (let i = 0; i < tim.length; i++) s += tim[i] * tim[i];
    level = Math.min(32767, Math.sqrt(s / tim.length) * 32767 * cfg.gain);
  }
  function adapt() {
    // per ~16ms tick: alpha = tick_ms / (minutes*60000). During warm-up the floors
    // adapt FAST (~0.4s) so the threshold settles before triggers arm.
    const inWarm = Date.now() < warmUntil;
    const habA = cfg.hab_min > 0 ? 16 / (cfg.hab_min * 60000) : 0;
    const bandA = cfg.band_tc > 0 ? 16 / (cfg.band_tc * 60000) : 0.005; // slow band-floor EMA
    thrLvl = cfg.manual_thr > 0 ? cfg.manual_thr : Math.max(cfg.min_thr, floorLvl * cfg.factor);
    if (floorLvl === 0) floorLvl = level;
    else if (inWarm) floorLvl += 0.04 * (level - floorLvl);          // fast settle
    else if (level < thrLvl) floorLvl += (level < floorLvl ? 0.05 : 0.005) * (level - floorLvl);
    else if (habA > 0) floorLvl += habA * (level - floorLvl);        // slow habituation above thr
    for (let b = 0; b < NB; b++) {
      bandThr[b] = Math.max(cfg.sminmag, bandFloor[b] * cfg.sfactor);
      if (bandFloor[b] === 0) bandFloor[b] = bandMag[b];
      else if (inWarm) bandFloor[b] += 0.04 * (bandMag[b] - bandFloor[b]);   // fast settle
      else if (bandMag[b] < bandThr[b]) bandFloor[b] += bandA * (bandMag[b] - bandFloor[b]); // slow EMA
      else if (habA > 0) bandFloor[b] += habA * (bandMag[b] - bandFloor[b]); // habituation
    }
  }
  function fire(src, hz) { lastTrigT = Date.now();
    events.unshift({ t: Date.now(), id: ++evId, source: src, hz: hz, rms: Math.round(level), thr: Math.round(thrLvl), sd: -1 });
    if (events.length > 200) events.pop(); }
  function triggers() {
    const now = Date.now();
    if (now < warmUntil || now - lastTrigT < 2000 || !cfg.enabled) return;
    if (cfg.auto && level > thrLvl) return fire("auto", 0);
    if (cfg.strig) { let hot = -1, best = 0; for (let b = 0; b < NB; b++) { const o = bandMag[b] - bandThr[b]; if (o > best) { best = o; hot = b; } } if (hot >= 0) fire("spectral", bandHz[hot]); }
  }
  function rings() {
    const now = Date.now();
    if (now - lastHistT >= histBucketMs()) { lastHistT = now;
      levelBuf.push(Math.round(level)); thrBuf.push(Math.round(thrLvl));
      if (levelBuf.length > HBUCKETS) { levelBuf.shift(); thrBuf.shift(); } }
    for (let b = 0; b < NB; b++) if (bandMag[b] > spAccum[b]) spAccum[b] = bandMag[b];
    if (now - lastSpT >= spBucketMs()) { lastSpT = now;
      const col = new Uint16Array(NB);
      for (let b = 0; b < NB; b++) { col[b] = Math.max(0, Math.min(65535, Math.round(20 * Math.log10(spAccum[b] + 1) * 256))); spAccum[b] = 0; }
      spectro.push(col); if (spectro.length > SCOLS) spectro.shift(); }
  }
  function tick() { if (!mic) return; readBands(); readLevel(); adapt(); triggers(); rings(); requestAnimationFrame(tick); }

  // ---- responses ----
  const J = o => ({ ok: true, status: 200, headers: { get: () => null }, json: async () => o, text: async () => JSON.stringify(o) });
  const BIN = (u16, h) => ({ ok: true, status: 200, headers: { get: k => h[k.toLowerCase()] ?? null }, arrayBuffer: async () => u16.buffer, json: async () => ({}), text: async () => "" });
  function setArgs(q) {
    const pStrig = cfg.strig, pTc = cfg.band_tc, pEn = cfg.enabled;
    for (const k of ["gain", "factor", "min_thr", "manual_thr", "hab_min", "band_tc", "lpf_hz", "hpf_hz", "sfactor", "sminmag", "spectro_min", "hist_win", "clip_ms", "preroll_ms"]) if (q.has(k)) cfg[k] = +q.get(k);
    for (const k of ["lpf", "hpf", "strig", "auto"]) if (q.has(k)) cfg[k] = +q.get(k);
    if (q.has("en")) cfg.enabled = +q.get("en");
    // re-warm the band trigger when it's (re)armed or its EMA window changes
    if ((cfg.strig && !pStrig) || cfg.band_tc !== pTc || (cfg.enabled && !pEn && cfg.strig)) armWarm();
  }
  function spectrum() {
    return J({ on: cfg.enabled, fs: 16000, dbref: DBREF, n: 512, strig: cfg.strig, sfactor: cfg.sfactor,
      sminmag: cfg.sminmag, hab_min: cfg.hab_min, band_tc: cfg.band_tc, bandwarm_ms: Math.max(0, warmUntil - Date.now()), spectro_min: cfg.spectro_min,
      hist_win: cfg.hist_win, hist_bucket_ms: histBucketMs(), lpf: !!cfg.lpf, lpf_hz: cfg.lpf_hz, hpf: !!cfg.hpf, hpf_hz: cfg.hpf_hz,
      f: bandHz, m: Array.from(bandDisp, v => +v.toFixed(1)), t: Array.from(bandThr, v => +v.toFixed(1)) });
  }
  function spectrogramResp() {
    const cols = spectro.length, a = new Uint16Array(cols * NB);
    for (let c = 0; c < cols; c++) a.set(spectro[c], c * NB);
    return BIN(a, { "x-cols": String(cols), "x-bands": String(NB), "x-bucket-ms": String(spBucketMs()), "x-db256": "1", "x-dbref": String(DBREF) });
  }
  function historyResp(q) {
    const thr = q.get("thr") === "1", src = thr ? thrBuf : levelBuf;
    const bucket = histBucketMs(), end = +q.get("end") || 0, endB = Math.floor(end / bucket);
    let pts = Math.min(+q.get("points") || 240, 2000);
    const avail = Math.max(0, src.length - endB);
    let want = Math.min(Math.floor((+q.get("secs") || 3600) * 1000 / bucket), avail);
    pts = Math.max(1, Math.min(pts, want || 1));
    const out = new Uint16Array(pts), newest = src.length - 1 - endB;
    for (let p = 0; p < pts; p++) {
      const lo = newest - want + 1 + Math.floor(p * want / pts), hi = newest - want + 1 + Math.floor((p + 1) * want / pts);
      let mx = 0; for (let i = Math.max(0, lo); i < Math.min(src.length, Math.max(lo + 1, hi)); i++) if (src[i] > mx) mx = src[i];
      out[p] = mx;
    }
    return BIN(out, { "x-bucket-ms": String(bucket) });
  }
  function status() {
    // Keys MUST match the device /audio/status (handleAudioStatus in audio_capture.cpp).
    return J({ enabled: !!cfg.enabled, recording: false, auto: !!cfg.auto, clip_ms: cfg.clip_ms, preroll_ms: cfg.preroll_ms,
      rms: Math.round(level), noise_floor: Math.round(floorLvl), threshold: Math.round(thrLvl), factor: cfg.factor, algo: 0, trig_k: 2.5, sigma: 8, stat_win: 1, viewer: false, gain: cfg.gain,
      min_thr: cfg.min_thr, manual_thr: cfg.manual_thr, lpf: !!cfg.lpf, lpf_hz: cfg.lpf_hz, hpf: !!cfg.hpf, hpf_hz: cfg.hpf_hz,
      slots: 0, max_slots: 20, recommended_slots: 20, slot_bytes: 0, free_psram: 3900000,
      sd: false, save_to_sd: false, sd_used_mb: 0, sd_total_mb: 0, sd_max_clips: 200 });
  }
  function eventsResp(q) {
    const now = Date.now(), secs = +q.get("secs") || 3600;
    return J(events.filter(e => now - e.t <= secs * 1000 + 5000)
      .map(e => ({ age_ms: now - e.t, id: e.id, sd: e.sd, rms: e.rms, thr: e.thr, hz: e.hz, source: e.source })));
  }

  const _fetch = window.fetch.bind(window);
  window.fetch = function (u, opt) {
    u = String(u); const q = new URLSearchParams((u.split("?")[1] || ""));
    if (u.startsWith("/audio/config")) { setArgs(q); return Promise.resolve(J({ ok: true })); }
    if (u.startsWith("/audio/spectrogram")) return Promise.resolve(spectrogramResp());
    if (u.startsWith("/audio/spectrum")) return Promise.resolve(spectrum());
    if (u.startsWith("/audio/history")) return Promise.resolve(historyResp(q));
    if (u.startsWith("/audio/status")) return Promise.resolve(status());
    if (u.startsWith("/audio/events")) return Promise.resolve(eventsResp(q));
    if (u.startsWith("/audio/trigger")) { fire("http", 0); return Promise.resolve(J({ ready_in_ms: cfg.clip_ms - cfg.preroll_ms })); }
    if (u.startsWith("/audio/retune")) { floorLvl = 0; bandFloor.fill(0); return Promise.resolve(J({ ok: true })); }
    if (u.startsWith("/audio/clips") || u.startsWith("/sd/list") || u.startsWith("/video/list")) return Promise.resolve(J([]));
    if (u.startsWith("/video/status")) return Promise.resolve(J({ enabled: false, sd: false, recording: false, clip_ms: 10000, preroll_ms: 2000, fps: 5, vmax: 50, on_audio: true, trig_audio: true, min_gap_ms: 3000, xtrig_cd_ms: 5000, thermal_en: true, temp_max: 95, thermal_paused: false, motion: false, motion_blocks: 20, motion_diff: 12, motion_level: 0, motion_global: false, ring_kb: 0, ring_frames: 0 }));
    if (u.startsWith("/diag")) return Promise.resolve(J({ rssi: 0, ip: "localhost", uptime_ms: Date.now() - PAGE_T0, free_heap: 0, free_psram: 0, camera: false, clients: 1, temp_c: 0, fps: 0, net_fps: 0, res: "-", quality: 0, xclk: 0, sd: false, sd_drops: 0, stream_on: false, thermal_paused: false }));
    return Promise.resolve(J({}));
  };
  const PAGE_T0 = Date.now();

  // Floating "start mic" button (getUserMedia needs a user gesture).
  function ui() {
    const b = document.createElement("button");
    b.textContent = "🎤  Start mic";
    b.style.cssText = "position:fixed;right:14px;bottom:14px;z-index:9999;background:#3d9df0;color:#fff;border:0;border-radius:10px;padding:12px 18px;font-size:1em;box-shadow:0 4px 14px rgba(0,0,0,.4);cursor:pointer";
    b.onclick = async () => { b.disabled = true; b.textContent = "starting…";
      try { await startMic(); b.textContent = "🔴 mic live"; b.style.background = "#1d3a2a"; b.style.color = "#5df0a0"; }
      catch (e) { b.disabled = false; b.textContent = "🎤  Start mic (denied?)"; alert("Mic error: " + e); } };
    document.body.appendChild(b);
  }
  if (document.readyState !== "loading") ui(); else document.addEventListener("DOMContentLoaded", ui);
})();

// Headless runtime test for the dashboard JS (tools/_page.js): runs the REAL
// page code in a stubbed DOM with mocked fetch, then drives the new plot
// features and asserts no exceptions + expected side effects. Catches runtime
// errors that `node --check` (syntax only) cannot.
//
// Run: node tools/headless_test.mjs   (after make_ui_mock.py)
import fs from "node:fs";
import vm from "node:vm";

const CODE = fs.readFileSync(new URL("./_page.js", import.meta.url), "utf8");
let fail = 0;
const ok = (c, m) => { console.log((c ? "  ok  " : " FAIL ") + m); if (!c) fail++; };

// ---- tiny DOM stub ---------------------------------------------------------
const ctxCalls = {};
const ctx2d = new Proxy({}, {
  get(t, k) {
    if (k === "measureText") return s => ({ width: String(s).length * 6 });
    if (typeof k === "string" && !(k in t)) { return (...a) => { ctxCalls[k] = (ctxCalls[k] || 0) + 1; }; }
    return t[k];
  },
  set(t, k, v) { t[k] = v; return true; },
});
const els = {};
function makeEl(id) {
  const o = { id, value: "1000", checked: false, disabled: false, textContent: "",
    innerHTML: "", clientWidth: 700, width: 700, height: 130, style: {},
    dataset: {}, className: "", offsetWidth: 700 };
  o.getContext = () => ctx2d;
  o.addEventListener = () => {};
  o.removeEventListener = () => {};
  o.getBoundingClientRect = () => ({ left: 0, top: 0, width: 700, height: 130 });
  o.classList = { add() {}, remove() {}, toggle() {}, contains: () => false };
  o.querySelectorAll = () => [];
  o.appendChild = () => {};
  return new Proxy(o, { get(t, k) { return k in t ? t[k] : (typeof k === "string" ? () => {} : t[k]); } });
}
const getEl = id => (els[id] ||= makeEl(id));
const document = {
  getElementById: getEl,
  querySelectorAll: () => [],
  querySelector: () => null,
  addEventListener: () => {},
  createElement: () => makeEl("new"),
  body: makeEl("body"), head: makeEl("head"),
  hidden: false, activeElement: null,
};

// ---- mock fetch (compact mirror of the browser harness) --------------------
const NOW = Date.now();
const EV_AGES = [40000, 300000, 1200000, 1800000, 2600000, 3200000];
// Keys MUST match the device /audio/status (handleAudioStatus in audio_capture.cpp);
// the drift guard at the end asserts the page's `s.<key>` reads are all provided.
const STATUS = { rms: 240, noise_floor: 190, threshold: 10000, gain: 16, min_thr: 0,
  enabled: true, recording: false, auto: true, clip_ms: 5000, preroll_ms: 1000,
  factor: 3, algo: 0, trig_k: 2.5, sigma: 8, stat_win: 1, manual_thr: 0, lpf: false, lpf_hz: 4000, hpf: true, hpf_hz: 120,
  slots: 6, max_slots: 20, recommended_slots: 20, slot_bytes: 332844, free_psram: 3900000,
  sd: true, save_to_sd: true, sd_used_mb: 71, sd_total_mb: 29838, sd_max_clips: 200, viewer: false };
const J = o => ({ ok: true, status: 200, headers: { get: () => null }, json: async () => o, text: async () => "" });
function ab(u16, hdrs) { return { ok: true, status: 200, headers: { get: k => hdrs[k.toLowerCase()] ?? null }, arrayBuffer: async () => u16.buffer, json: async () => ({}), text: async () => "" }; }
function fetchMock(u) {
  u = String(u); const q = new URLSearchParams((u.split("?")[1] || ""));
  const secs = +q.get("secs") || 3600, pts = Math.min(+q.get("points") || 240, 2000);
  if (u.startsWith("/audio/history")) {
    const end = +q.get("end") || 0, bms = Math.round(secs * 1000 / pts);
    const a = new Uint16Array(pts);                 // TIME-based, matches make_ui_mock.py
    for (let i = 0; i < pts; i++) { const age = end + (pts - 1 - i) * bms;
      let v = 210; for (const ea of EV_AGES) if (Math.abs(age - ea) < bms * 1.5) { v = 12000; break; } a[i] = v; }
    return Promise.resolve(ab(a, { "x-bucket-ms": String(bms) }));
  }
  if (u.startsWith("/audio/events")) return Promise.resolve(J([
    { age_ms: 40000, id: 42, sd: 42, rms: 17717, thr: 10000, hz: 0, source: "auto" },
    { age_ms: 300000, id: 41, sd: 41, rms: 557, thr: 10000, hz: 551, source: "spectral" },
    { age_ms: 3500000, id: 36, sd: 36, rms: 9100, thr: 10000, hz: 0, source: "auto" }, // near left edge
  ]));
  if (u.startsWith("/audio/clipmeta")) return Promise.resolve(J({ "41": [557, 10000, 551] }));
  if (u.startsWith("/audio/clips")) return Promise.resolve(J([
    { id: 41, age_ms: 300000, size: 332844, rms: 557, thr: 10000, hz: 551, source: "spectral" }]));
  if (u.startsWith("/audio/status")) return Promise.resolve(J(STATUS));
  if (u.startsWith("/audio/spectrum")) return Promise.resolve(J({ enabled: true, on: true, strig: true, sfactor: 8, sminmag: 10000, bandwarm_ms: 0, spectro_min: 15, hist_win: 1440, hist_bucket_ms: 2000, fs: 16000, dbref: 132.4, hab_min: 5, band_tc: 2, f: Array.from({ length: 32 }, (_, i) => Math.round(60 * Math.pow(133, i / 31))), m: Array.from({ length: 32 }, (_, i) => 200 + 8000 * Math.exp(-((i - 9) ** 2) / 8)), t: Array(32).fill(9000) }));
  if (u.startsWith("/audio/spectrogram")) { const a = new Uint16Array(200 * 32); return Promise.resolve(ab(a, { "x-cols": "200", "x-bands": "32", "x-bucket-ms": "3750", "x-db256": "1", "x-dbref": "132.4" })); }
  if (u.startsWith("/sd/list")) return Promise.resolve(J([{ name: "clip_00041_s.wav", size: 332844, mtime: Math.round((NOW - 300000) / 1000) }]));
  if (u.startsWith("/video/status")) return Promise.resolve(J({ enabled: false, sd: true, recording: false, clip_ms: 10000, preroll_ms: 2000, fps: 5, vmax: 50, on_audio: true, trig_audio: true, min_gap_ms: 3000, xtrig_cd_ms: 5000, thermal_en: true, temp_max: 95, thermal_paused: false, motion: false, motion_blocks: 20, motion_diff: 12, motion_level: 0, motion_global: false, ring_kb: 1500, ring_frames: 34 }));
  if (u.startsWith("/video/list")) return Promise.resolve(J([]));
  if (u.startsWith("/diag")) return Promise.resolve(J({ rssi: -72, ip: "x", uptime_ms: 5400000, free_heap: 124000, free_psram: 3900000, camera: true, clients: 1, temp_c: 78.5, fps: 25, net_fps: 0, res: "800x600", quality: 30, xclk: 20, sd: true, sd_drops: 0, stream_on: true, thermal_paused: false }));
  return Promise.resolve(J({}));
}

const sandbox = {
  document, console, Math, Date, JSON, Uint16Array, Uint8Array, Float32Array, URLSearchParams,
  Promise, Array, Object, String, Number, isFinite, isNaN, parseInt, parseFloat, Set, Map,
  fetch: fetchMock, setTimeout, clearTimeout, setInterval: () => 0, clearInterval: () => {},
  requestAnimationFrame: () => 0, alert: () => {}, confirm: () => true, location: { href: "", reload() {} },
  navigator: { userAgent: "node" }, performance: { now: () => Date.now() },
  localStorage: { getItem: () => null, setItem() {}, removeItem() {} },
};
sandbox.window = sandbox; sandbox.self = sandbox; sandbox.globalThis = sandbox;
// Browsers expose every element with an id as a global (window.<id>); emulate
// that so the page's bare `plot`, `logtog`, `zoomSld`... references resolve.
const html = fs.readFileSync(new URL("./ui_mock.html", import.meta.url), "utf8");
for (const m of html.matchAll(/\bid=["']([A-Za-z_][\w-]*)["']/g)) {
  if (!(m[1] in sandbox)) sandbox[m[1]] = getEl(m[1]);
}
vm.createContext(sandbox);

let evalErr = null;
try { vm.runInContext(CODE, sandbox, { filename: "_page.js" }); }
catch (e) { evalErr = e; }
ok(!evalErr, "page script evaluates without throwing" + (evalErr ? " -> " + evalErr : ""));

// let the on-load async IIFE + fetches settle, then drive the new features
await new Promise(r => setTimeout(r, 400));

async function drive() {
  // 1) drawPlot pulls history+events and renders (exercises the new tick axis)
  ok(typeof sandbox.drawPlot === "function", "drawPlot() is defined");
  await sandbox.drawPlot();
  ok((ctxCalls.fillText || 0) > 0, "renderPlot drew axis tick labels (fillText called " + (ctxCalls.fillText || 0) + "x)");
  ok((ctxCalls.stroke || 0) > 0, "renderPlot drew gridlines/strokes");

  // 2) zoom slider narrows the window
  const z = getEl("zoomSld"); z.value = "500";
  ok(typeof z.oninput === "function", "zoomSld.oninput wired");
  z.oninput();
  ok(/[smh]$/.test(getEl("zoomLbl").textContent), "zoom label shows a duration: '" + getEl("zoomLbl").textContent + "'");
  await new Promise(r => setTimeout(r, 200)); // let navRedraw fire

  // 3) pan slider moves the window (only meaningful while zoomed)
  const p = getEl("panSld"); p.value = "300";
  ok(typeof p.oninput === "function", "panSld.oninput wired");
  let panErr = null; try { p.oninput(); } catch (e) { panErr = e; }
  ok(!panErr, "panSld.oninput runs without throwing" + (panErr ? " -> " + panErr : ""));

  // reset to full range so the spectral marker (age 300s) is on-screen
  getEl("zoomSld").value = "1000"; getEl("zoomSld").oninput();
  await new Promise(r => setTimeout(r, 200));
  await sandbox.drawPlot();

  // 4) tapping the SPECTRAL marker pins a persistent info box WITH the frequency
  ok(typeof sandbox.plotTap === "function", "plotTap() is defined");
  const info = getEl("plotinfo");
  let foundHz = false, tapErr = null; const seen = new Set();
  try {
    for (let x = 0; x <= 700; x += 2) {       // sweep; capture the spectral hit
      sandbox.plotTap(x);
      if (info.innerHTML) seen.add(info.innerHTML.replace(/<button.*$/, "").trim());
      if (/551\s*Hz/.test(info.innerHTML)) { foundHz = true; break; }
    }
  } catch (e) { tapErr = e; }
  if (!foundHz) console.log("   [debug] marker infos seen:", [...seen]);
  ok(!tapErr, "plotTap never throws" + (tapErr ? " -> " + tapErr : ""));
  ok(foundHz, "tapping the spectral marker pins info incl. '551 Hz' + Play button");
  ok(/playClip/.test(info.innerHTML), "pinned marker info includes a Play action");
  // empty-area tap shows a persistent level/time readout (the old flashing fix)
  sandbox.plotTap(5);
  ok(/level/.test(info.textContent) || /Hz/.test(info.innerHTML), "empty tap pins level/time readout");

  // 4b) marker x must sit exactly on its waveform sample (the off-by-one fix):
  // evX(age) should equal the sample-index x mapping within sub-pixel, esp. near
  // the LEFT edge where the old d.length-vs-(d.length-1) bug diverged ~1px.
  const W = 700, pts = 700, bms = Math.round(3600 * 1000 / pts);
  const waveX = age => (pts - 1 - age / bms) * W / (pts - 1);
  for (const age of [300000, 3500000]) {
    const mx = sandbox.evX({ age_ms: age, source: "auto" }), wx = waveX(age);
    ok(Math.abs(mx - wx) < 0.6, `marker@${age}ms aligns with its sample (evX=${mx.toFixed(2)} wave=${wx.toFixed(2)})`);
  }

  // 4c) amplitude-scale toggle cycles rel -> dBFS -> linear and each renders
  const st = getEl("scaletog");
  ok(typeof st.onclick === "function", "scaletog.onclick wired");
  const labels = []; let scErr = null;
  try {
    for (let k = 0; k < 3; k++) { st.onclick({ preventDefault() {} }); labels.push(st.textContent); await sandbox.drawSpectrum(); }
  } catch (e) { scErr = e; }
  ok(!scErr, "drawSpectrum renders in every scale mode" + (scErr ? " -> " + scErr : ""));
  ok(labels.join(",") === "dBFS,linear,dB rel", "toggle cycles dBFS -> linear -> dB rel (saw: " + labels.join(",") + ")");

  // 5) the clips & SD tables render the triggering frequency for spectral clips
  if (typeof sandbox.refreshClips === "function") await sandbox.refreshClips();
  if (typeof sandbox.refreshSd === "function") await sandbox.refreshSd();
  const tables = Object.values(els).map(e => e.innerHTML || "").join("\n");
  ok(/551\s*Hz/.test(tables), "clips/SD table shows '551 Hz' for the spectral trigger");
}
let driveErr = null;
try { await drive(); } catch (e) { driveErr = e; }
ok(!driveErr, "feature drive completed" + (driveErr ? " -> " + driveErr : ""));

// ---- drift guard: every status field the page reads must exist in the mock ----
// The page reads /audio/status as `s.<key>` inside refreshStatus(). Extract that
// function body and assert each key it reads is provided by STATUS, so a future
// device-side rename (or a mock that drifts) fails the test instead of silently
// rendering NaN. STATUS here mirrors make_ui_mock.py / live_mic.js.
{
  const i = CODE.indexOf("function refreshStatus");
  let body = "";
  if (i >= 0) {
    const open = CODE.indexOf("{", i);
    let depth = 0;
    for (let j = open; j < CODE.length; j++) {
      const c = CODE[j];
      if (c === "{") depth++;
      else if (c === "}") { if (--depth === 0) { body = CODE.slice(open, j + 1); break; } }
    }
  }
  const read = new Set([...body.matchAll(/\bs\.([A-Za-z_]\w*)/g)].map(m => m[1]));
  ok(read.size > 0, "drift guard located refreshStatus status reads");
  const missing = [...read].filter(k => !(k in STATUS));
  ok(missing.length === 0, "mock /audio/status provides every key the page reads"
    + (missing.length ? " -> missing: " + missing.join(", ") : ""));
}

console.log(fail ? `\n${fail} check(s) FAILED` : "\nALL CHECKS PASSED");
process.exit(fail ? 1 : 0);

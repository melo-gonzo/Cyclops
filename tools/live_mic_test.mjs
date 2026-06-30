// Runtime test for tools/live_mic.js: loads it in a stubbed environment (mic
// off) and exercises the fetch() override - verifies the /audio/* responses
// have the right shape and that /audio/config + /audio/trigger mutate state.
// Web Audio (the live tick) can't run headlessly; this covers everything else.
import fs from "node:fs";
import vm from "node:vm";

const CODE = fs.readFileSync(new URL("./live_mic.js", import.meta.url), "utf8");
let fail = 0; const ok = (c, m) => { console.log((c ? "  ok  " : " FAIL ") + m); if (!c) fail++; };

const btn = { style: {}, onclick: null, set textContent(v) {}, get textContent() { return ""; } };
const document = { readyState: "complete", createElement: () => btn, body: { appendChild() {} }, addEventListener() {} };
const sandbox = {
  document, console, Math, Date, JSON, Float32Array, Uint16Array, URLSearchParams, Promise, Array, isFinite,
  requestAnimationFrame: () => 0, navigator: { mediaDevices: {} }, alert: () => {},
  fetch: () => Promise.resolve({ ok: false }), // browser provides this; stub so .bind() works
};
sandbox.window = sandbox; sandbox.self = sandbox;
vm.createContext(sandbox);
let evalErr = null; try { vm.runInContext(CODE, sandbox, { filename: "live_mic.js" }); } catch (e) { evalErr = e; }
ok(!evalErr, "live_mic.js evaluates + installs fetch override" + (evalErr ? " -> " + evalErr : ""));
ok(sandbox.fetch && sandbox.fetch.name !== "fetch", "window.fetch was overridden");

const get = async (u) => sandbox.fetch(u);
let err = null;
try {
  const spec = await (await get("/audio/spectrum")).json();
  ok(spec.f.length === 32 && spec.m.length === 32 && spec.t.length === 32, "spectrum returns 32 bands (f/m/t)");
  ok(spec.dbref === 132.4 && spec.fs === 16000, "spectrum carries dbref=132.4 and fs=16000");

  const st = await (await get("/audio/status")).json();
  ok(typeof st.rms === "number" && typeof st.threshold === "number", "status has rms + threshold");
  ok(st.factor === 3 && st.gain === 16, "status defaults factor=3 gain=16");

  const h = await get("/audio/history?secs=600&points=120");
  const buf = new Uint16Array(await h.arrayBuffer());
  ok(h.headers.get("X-Bucket-Ms") !== null, "history sends X-Bucket-Ms header");
  ok(buf.length >= 1, "history returns a Uint16 buffer (" + buf.length + " pts)");

  const sg = await get("/audio/spectrogram");
  ok(sg.headers.get("X-Cols") !== null && sg.headers.get("X-DbRef") === "132.4", "spectrogram sends X-Cols + X-DbRef");

  // config mutates state
  await get("/audio/config?factor=5&sfactor=9&min_thr=1234");
  const st2 = await (await get("/audio/status")).json();
  const spec2 = await (await get("/audio/spectrum")).json();
  ok(st2.factor === 5 && st2.min_thr === 1234, "/audio/config updates factor + min_thr");
  ok(spec2.sfactor === 9, "/audio/config updates sfactor");
  await get("/audio/config?hab_min=2.5&band_tc=3");
  const spec3 = await (await get("/audio/spectrum")).json();
  ok(spec3.hab_min === 2.5, "/audio/config updates hab_min (habituation window)");
  ok(spec3.band_tc === 3, "/audio/config updates band_tc (band-floor EMA window)");
  ok(spec3.sminmag === 1000, "default min mag is 1000 (was 10000)");
  ok(spec3.bandwarm_ms > 1000, "changing band_tc re-arms the band warmup (" + spec3.bandwarm_ms + "ms)");

  // manual trigger logs an event with a frequency field
  await get("/audio/trigger");
  const ev = await (await get("/audio/events?secs=3600")).json();
  ok(ev.length === 1 && ev[0].source === "http" && "hz" in ev[0], "/audio/trigger logs an event with hz field");
} catch (e) { err = e; }
ok(!err, "endpoint exercise completed" + (err ? " -> " + err : ""));

console.log(fail ? `\n${fail} check(s) FAILED` : "\nALL CHECKS PASSED");
process.exit(fail ? 1 : 0);

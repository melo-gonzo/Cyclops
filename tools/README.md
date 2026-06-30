# UI test harness (no device required)

The dashboard HTML/JS is embedded in `src/audio_capture.cpp` as a PROGMEM string.
These tools let you exercise it without flashing hardware.

## Interactive browser mock
```sh
python3 tools/make_ui_mock.py      # -> tools/ui_mock.html
open tools/ui_mock.html            # or serve it / load on your phone
```
Extracts the real `AUDIO_PAGE` and wraps it with a mock `fetch()` that returns
synthetic data (a level history, events incl. a **spectral** trigger at 551 Hz,
clips, spectrum, spectrogram, SD list, status). The actual UI runs — useful for
eyeballing layout and the mobile zoom/pan/tap interactions. Re-run after editing
the page; it always reflects the current source.

## Live microphone mode (real audio, no device)
```sh
python3 tools/serve_live.py        # serves on http://localhost:8765 + opens it
```
Serves the real dashboard with `tools/live_mic.js` injected, which captures your
**laptop microphone** via the Web Audio API and runs the device's DSP in the
browser (32 log bands 60–8 kHz, adaptive scalar + per-band thresholds, dBFS
heatmap, level/threshold history, auto + spectral triggers). Click **Start mic**,
grant permission, and the live spectrum / heatmap / level plot / triggers all
react to real sound — useful for exercising the spectral trigger, the scale
toggle, zoom/pan, etc. with no hardware. `getUserMedia` needs a secure context,
hence localhost (not `file://`). The amplitudes are display-faithful, not
sample-accurate to the PDM mic. `node tools/live_mic_test.mjs` covers the
endpoint layer.

## Headless runtime test (CI-friendly)
```sh
python3 tools/make_ui_mock.py && node tools/headless_test.mjs
```
Runs the page's JS in a stubbed DOM with mocked fetch and asserts the plot/table
features work (axis ticks render, zoom/pan sliders, tap pins a persistent
readout, spectral frequency shows in the marker info and the clips/SD tables).
`node --check tools/_page.js` additionally lints syntax. This catches runtime
errors that a plain syntax check can't.

Generated files (`ui_mock.html`, `_page.js`) are git-ignored.

# Cyclops — Configuration & Operation

Terse field guide. All controls live on the device's web dashboard: networking/identity/auth on `/wifi`, capture tuning on `/audio` and the dashboard video card. No app, no cloud.

## First boot & networking
- **Fallback AP:** if no saved network is reachable (or standalone is on), the device raises Wi-Fi AP **`Cyclops`** (pass `cyclops1234`) at **`http://192.168.8.1`**.
- **Portal `/wifi`:** scan + join, save **multiple** networks (joins strongest saved on boot), forget, manual join. Optional pre-seed via git-ignored `src/wifikeys.h` before flashing.
- **Static IP:** fully configurable at `/wifi` — toggle **Static IP** on and set address / netmask / gateway / DNS. When enabled it applies to whatever network the device joins (off = DHCP); takes effect on the next join.
- **mDNS host:** reachable at `<host>.local` (default `cyclops.local`); rename per device at `/wifi`, re-announces live, no reboot. Give each unit a unique name.
- **Standalone (AP-only) mode:** toggle at `/wifi` to stay on its own AP and never join a network.
- The fallback-AP password is also editable at `/wifi`.

## Authentication (`web_auth.cpp`)
- **Ships passwordless** — empty admin password = open access, every request treated as admin.
- Set **Web login** (username + password) at `/wifi` to enable **HTTP Digest auth** on every endpoint; persists in NVS. Turn it back off there to return to passwordless.
- Successful login also sets a signed 30-day remember-me cookie (`cyc_auth`); changing the password invalidates all sessions.
- **Read-only viewer** account (optional, at `/wifi`): can watch/listen/download but is blocked (403) from changing settings. Blank viewer username disables it.

## Audio (`audio_capture.cpp`, XIAO only)
- **Adaptive threshold:** `floor × factor` or **`mean + k·σ`** algorithm; optional manual override and floor clamp; mic gain; a *habituate* window leaks sustained sound into the floor.
- **Per-band spectral trigger:** fires when any FFT band crosses its own learned floor (sensitivity, min magnitude, band-average window).
- **Filters:** high-pass and low-pass applied to stream, clips, and the level metric; drag to audition before committing.
- **Live monitoring:** in-browser WebAudio listen + FFT spectrum/spectrogram + zoomable level history.
- **Clips:** length (default 5 s) and pre-roll (default 2 s); captured to a PSRAM RAM ring and/or SD.

## Motion detection (`motion.h` / `video_record.cpp`)
- Block/pixel-diff: counts blocks whose average per-pixel luma delta exceeds a threshold; triggers when changed blocks ≥ a block count. Off by default; tune visually against the live overlay.
- **Trigger cooldown** (default 5 s, 0–60 s, on every board incl. the ESP32-CAM): after a trigger fires, new triggers are suppressed for this long — motion is still graphed on the Event plot, only repeat triggers are limited. Set on the Camera page; `0` disables the limit.

## Thermal governor (`thermal.h`)
- 30 s **moving average** of die temp vs a single hard cutoff **`temp_max`** (default 100 °C). Over cutoff → **pauses video recording only** (triggered + continuous); **audio keeps running**. No hysteresis — the average prevents flapping. Can be disabled.

## Video clips (`video_record.cpp`, XIAO only)
- Enable (default on), **fps** (default 5), **clip length** (default 10 s), **pre-roll** (default 4 s, from the PSRAM ring).
- **Cross-triggers:** *video on audio* and *audio on video* (both default on).
- **Cooldowns:** an audio→video cooldown (default 30 s) and a global min gap between auto clips (default 3 s). Manual/HTTP triggers bypass.
- **Max files** (default 40, delete-oldest).

## Storage & retention
- **Save-to-SD:** audio (default on, cap default 100, 0 = unlimited); video clips always to SD (capped).
- The audio RAM ring is volatile; **SD clips, all saved settings/networks, mDNS host, and static-IP config survive reboot** — and OTA re-flash (the `nvs` offset is preserved).

## OTA updates (`ota_update.cpp`)
- **HTTP `POST /update`:** upload `firmware.bin` from the Firmware card on `/wifi`; digest-gated once a password is set; reboots into the new image (~15 s).
- **ArduinoOTA / espota:** push from PlatformIO, protected by a separate `OTA_PASSWORD` (defaults to `DEVICE_AP_PASS`; espota reads the `OTA_PASS` env var). Requires the one-time USB flash of the dual-OTA partition table first.

## Factory reset (XIAO)
- **Hold BOOT ~5 s** on a running device: wipes all saved networks, AP/web credentials, and the static-IP config, **erases SD clips**, then reboots onto the `Cyclops` AP **passwordless**. Camera/audio/video tuning is left intact. (Holding BOOT *at power-on* enters the bootloader instead.)

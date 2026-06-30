# Cyclops — Hardware & Build

Firmware for an ESP32 MJPEG camera with motion/audio-triggered clip capture.

## Supported boards

| Board | Default | Camera | Audio (PDM mic) | microSD | Flash |
|-------|---------|--------|-----------------|---------|-------|
| **Seeed XIAO ESP32S3 Sense** | ✅ | OV2640 | ✅ onboard | ✅ on Sense expansion | 8MB |
| **AI-Thinker ESP32-CAM** | — | OV2640 | ❌ | ❌ | 4MB |

- Feature gating is by `HAS_AUDIO`/`HAS_SD` in `src/capabilities.h`. **Audio** (clips, live audio) and **SD** (saved clips, continuous recording) are **XIAO Sense only**. The ESP32-CAM still runs live video (Live tab), **motion detection** (off the PSRAM frame ring, no SD — logged to the Graphs Motion timeline), the camera page, metrics/graphs, diagnostics, WiFi, and OTA. Only the SD-backed **Record** tab shows an honest "needs an SD card" page.
- Note: with motion enabled on a camera-only board the camera stays continuously awake (it can't park between viewers), so it runs a little warmer / draws more.
- Camera model selected via `-DCAMERA_MODEL_XIAO_ESP32S3` (XIAO env). The ESP32-CAM env defines no model flag; `src/main.cpp` defaults to `CAMERA_MODEL_AI_THINKER`.

## Key peripherals & pins (verified)

XIAO ESP32S3 Sense (`src/camera_pins.h` XIAO block, `src/audio_capture.cpp`):
- **Camera:** XCLK=GPIO10, SIOD=GPIO40, RESET=none; full pinout in `camera_pins.h`.
- **PDM microphone:** I2S_NUM_0 — CLK=GPIO42, DATA=GPIO41.
- **microSD:** SPI, CS=**GPIO21** (on the Sense expansion board).
- **BOOT button:** GPIO0, active-low; hold 5s = factory reset.

AI-Thinker pinout: `camera_pins.h` (`CAMERA_MODEL_AI_THINKER`, XCLK=GPIO0, SIOD=GPIO26, RESET=15).

## PlatformIO environments (`platformio.ini`)

| Env | Purpose |
|-----|---------|
| `native` | Host unit tests of pure logic modules (`pio test -e native`), Unity, no board. |
| `seeed_xiao_esp32s3` | **Default.** XIAO USB build/flash (PSRAM, 8MB QIO, 240MHz). |
| `seeed_xiao_esp32s3_ota` | Extends XIAO env; OTA upload over WiFi (espota). |
| `esp32cam` | AI-Thinker ESP32-CAM USB build/flash (4MB). |
| `esp32cam_ota` | Extends esp32cam env; OTA upload over WiFi (espota). |

OTA envs set `upload_protocol = espota`, `upload_port = cyclops.local` (override per device with `--upload-port <host/ip>`), `upload_flags = --auth=${sysenv.OTA_PASS}`. The OTA password must match the device's compiled `OTA_PASSWORD` (defaults to `DEVICE_AP_PASS`, see `src/branding.h`).

## Partition tables (dual-OTA)

Both are dual-OTA layouts (app0/app1) with `nvs` at the stock offset, so WiFi creds, mDNS hostname, and saved settings survive the one-time USB re-flash. SPIFFS dropped.

- XIAO → `partitions_xiao_8mb_ota.csv` — two **3MB** app slots (8MB flash).
- ESP32-CAM → `partitions_cam_4mb_ota.csv` — two **~2.0MB** app slots (4MB flash).

OTA requires a one-time USB flash to install this table first.

## Build & flash

```bash
# Host unit tests (no hardware)
pio test -e native

# USB build + flash
pio run -e seeed_xiao_esp32s3 -t upload      # XIAO (default)
pio run -e esp32cam -t upload                # AI-Thinker ESP32-CAM

# OTA upload over WiFi (export password once per shell)
export OTA_PASS=...
pio run -e seeed_xiao_esp32s3_ota -t upload --upload-port cyclops.local
pio run -e esp32cam_ota -t upload --upload-port <host-or-ip>
```

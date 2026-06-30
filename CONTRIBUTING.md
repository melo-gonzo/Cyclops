# Contributing to Cyclops

Thanks for your interest! Cyclops is an ESP32 MJPEG camera firmware with
audio/video clip capture. This is a small project — issues and pull requests are
welcome.

## Getting set up

1. Install [PlatformIO](https://platformio.org/) (the VS Code extension or the
   `pio` CLI).
2. Clone the repo.
3. (Optional) Seed a home WiFi network for the first boot:
   ```sh
   cp src/wifikeys.h.example src/wifikeys.h   # then edit in your SSID/PSK
   ```
   Skip this and the device just boots to its `Cyclops` setup AP instead.
4. Build:
   ```sh
   pio run -e seeed_xiao_esp32s3      # XIAO ESP32-S3 Sense (default target)
   pio run -e esp32cam                # AI-Thinker ESP32-CAM
   pio run -e seeed_xiao_esp32s3 -t upload   # flash over USB
   ```

See the [README](README.md) for hardware targets, OTA updates, endpoints, and
the security model.

## Tests

The hardware-independent logic (DSP, ring buffers, parsers, retention, motion,
FFT, thermal governor, …) is unit-tested on the host with Unity:

```sh
pio test -e native              # run the whole suite
pio test -e native -f test_fft  # run one suite
```

Please keep these green and add coverage when you change pure logic. The point
of the `native` env is that this logic stays free of Arduino/hardware includes
so it can be tested without a device.

## Pull requests

- Keep changes focused; one concern per PR.
- Match the surrounding code style (the codebase favors small, well-commented
  functions and explains the *why*).
- Run `pio test -e native` and build at least the `seeed_xiao_esp32s3` env
  before opening the PR.
- Describe what you changed and how you verified it (host tests, on-device, …).

## Security

Found a vulnerability? Please open an issue describing the impact. Note that
Cyclops is designed as a **LAN-only device** with a documented plain-HTTP threat
model — see the "Security model" section of the README before reporting, as some
behaviors are intentional trade-offs for this hardware class.

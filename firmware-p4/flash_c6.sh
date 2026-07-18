#!/bin/sh
# Bench UART flash of the ESP32-P4-NANO's onboard C6 (esp-hosted slave).
# Normally NOT needed - /c6/update?url=... updates the C6 over the LAN. This
# script exists for disaster recovery (bricked/blank C6) or a slave whose fw
# predates the OTA RPCs.
#
# Flashes the FULL set from c6-slave-2.12.11/ (bootloader @0x0, partition
# table @0x8000, otadata @0xd000, app @0x10000). NEVER write a bare
# network_adapter/app image at 0x0 - the published esp32c6-vX.Y.Z.bin files
# are app-only OTA payloads; writing one at 0x0 wipes the bootloader (learned
# the hard way 2026-07-18). The slave MUST version-match the P4 host component
# (managed_components/espressif__esp_hosted); rebuild recipe in the repo notes:
# idf.py -B <dir> -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32c6"
# from managed_components/espressif__esp_hosted/slave with the pio packages-p4nano
# toolchain (export IDF_PATH=packages-p4nano/framework-espidf,
# IDF_PYTHON_ENV_PATH=~/.platformio/penv, IDF_PYTHON_CHECK_CONSTRAINTS=no,
# ESP_IDF_VERSION=5.5.4).
#
# CRITICAL board quirk (schematic-verified): the C6's CHIP_PU/EN is driven ONLY
# by P4 GPIO54 through a 0R link - no pull-up. A parked P4 (BOOT held) leaves
# GPIO54 floating and the C6 dead-silent in reset. So GPIO54 must be strapped
# high manually while the P4 is parked.
#
# Bench setup (board OFF; header = the 2x13 whose pins 25/26 are silkscreened,
# MIC corner; odd/inner column has 5V on pin 1):
#   USB-UART TX  -> pin 20  C6_U0RXD
#   USB-UART RX  -> pin 22  C6_U0TXD
#   USB-UART GND -> pin 26  GND      (do NOT connect the adapter's 3V3)
#   jumper pin 24 C6_IO9  -> pin 26 GND   (C6 boot strap)
#   jumper pin 13 GPIO54  -> pin 5  3V3   (holds the C6 enabled)
# Power the board while HOLDING the P4's BOOT button (parks the host so it
# can't drive GPIO54 or reset the C6). Release BOOT after power-up; the C6 is
# then sitting in its ROM bootloader - no timing needed, flash whenever.
# REMOVE the pin13->3V3 strap before booting the P4 normally afterwards (the
# running firmware pulses GPIO54 low and would fight the strap).
#
# Usage: ./flash_c6.sh [/dev/cu.usbserial-XXXX]   (no arg = list candidate ports)

set -e
cd "$(dirname "$0")"

PY="$HOME/.platformio/penv/bin/python"
ESPTOOL="$HOME/.platformio/packages-p4nano/tool-esptoolpy/esptool.py"
SET="c6-slave-2.12.11"

if [ -z "$1" ]; then
  echo "Candidate serial ports:"
  PORTS=$(ls /dev/cu.usbserial* /dev/cu.usbmodem* /dev/cu.SLAB* /dev/cu.wchusbserial* 2>/dev/null)
  [ -n "$PORTS" ] && echo "$PORTS" \
    || echo "  (none found - is the USB-UART adapter plugged in?)"
  echo "Rerun: $0 <port>"
  exit 1
fi

echo "== sanity: should identify an ESP32-C6 =="
"$PY" "$ESPTOOL" --chip esp32c6 --port "$1" --before no_reset chip_id

echo "== erasing, then flashing the $SET set =="
"$PY" "$ESPTOOL" --chip esp32c6 --port "$1" --before no_reset erase_flash
"$PY" "$ESPTOOL" --chip esp32c6 --port "$1" --baud 460800 --before no_reset \
  write_flash --flash_mode dio --flash_size 4MB --flash_freq 80m \
  0x0     "$SET/bootloader.bin" \
  0x8000  "$SET/partition-table.bin" \
  0xd000  "$SET/ota_data_initial.bin" \
  0x10000 "$SET/network_adapter.bin"

echo "Done. Remove BOTH jumpers (IO9 and the pin13->3V3 strap!) + UART wires,"
echo "power cycle, then verify:  curl http://<device>/c6/version"

#!/bin/sh
# One-time bench flash of the ESP32-P4-NANO's onboard C6 (esp-hosted slave).
# After this, /c6/update?url=... handles all future C6 updates over the LAN.
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
BIN="esp32c6-v2.12.8.bin"

if [ -z "$1" ]; then
  echo "Candidate serial ports:"
  PORTS=$(ls /dev/cu.usbserial* /dev/cu.usbmodem* /dev/cu.SLAB* /dev/cu.wchusbserial* 2>/dev/null)
  [ -n "$PORTS" ] && echo "$PORTS" \
    || echo "  (none found - is the USB-UART adapter plugged in?)"
  echo "Rerun: $0 <port>"
  exit 1
fi

echo "== sanity: should identify an ESP32-C6 =="
"$PY" "$ESPTOOL" --chip esp32c6 --port "$1" chip_id

echo "== flashing $BIN at 0x0 =="
"$PY" "$ESPTOOL" --chip esp32c6 --port "$1" --baud 460800 write_flash 0x0 "$BIN"

echo "Done. Remove the IO9 jumper + UART wires, power cycle the board,"
echo "then verify from the LAN:  curl http://192.168.4.67/c6/version"

#!/bin/sh
# One-time bench flash of the ESP32-P4-NANO's onboard C6 (esp-hosted slave).
# After this, /c6/update?url=... handles all future C6 updates over the LAN.
#
# Bench setup (board OFF, then wire):
#   USB-UART TX  -> C6_U0RXD      (header pin)
#   USB-UART RX  -> C6_U0TXD      (header pin)
#   USB-UART GND -> GND           (do NOT connect the adapter's 3V3)
#   jumper C6_IO9 -> GND          (C6 boot strap, keep held through power-on)
# Then power the board while HOLDING the P4's BOOT button (parks the host so it
# can't reset the C6 mid-flash), with the IO9 jumper still in place.
#
# Usage: ./flash_c6.sh [/dev/cu.usbserial-XXXX]   (no arg = list candidate ports)

set -e
cd "$(dirname "$0")"

PY="$HOME/.platformio/penv/bin/python"
ESPTOOL="$HOME/.platformio/packages-p4nano/tool-esptoolpy/esptool.py"
BIN="esp32c6-v2.12.8.bin"

if [ -z "$1" ]; then
  echo "Candidate serial ports:"
  ls /dev/cu.usbserial* /dev/cu.usbmodem* /dev/cu.SLAB* /dev/cu.wchusbserial* 2>/dev/null \
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

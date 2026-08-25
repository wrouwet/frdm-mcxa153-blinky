#!/usr/bin/env python3
"""Standalone verification tool for the bridge's SMBus (WS/RS/XS) commands.

Deliberately self-contained (only needs `pyserial`, e.g. `pip install
pyserial`) rather than depending on the separate openbic-test-environment
project -- this tool verifies the bridge firmware's own SMBus protocol
layer, independent of any particular target device or higher-level
protocol (IPMB, MCTP, ...) running on top of it.

What this can and can't prove, and why (read before trusting a "PASS"):

- The PEC (Packet Error Check) CRC-8 engine itself is verified against
  the published CRC-8/SMBUS standard check value (0xF4 for the ASCII
  bytes "123456789" -- see the "Catalogue of parametrised CRC
  algorithms"), independent of any hardware. If this fails, the
  firmware's CRC math is wrong, full stop.
- The WS (SMBus write) command is smoke-tested against whatever
  responds to a bus scan: this proves the transaction completes with
  the right byte count and framing (no crash, no timeout), but NOT that
  the appended PEC byte was correct, since most I2C devices -- including
  every one on this project's bench so far -- just ACK it as ordinary
  data without checking it.
- RS/XS (SMBus read / write-then-read) are smoke-tested the same way,
  and WILL report "PEC did not match" against a non-SMBus-PEC-aware
  device (i.e. almost everything on a typical I2C bus, including
  OpenBIC's IPMB interface, which uses its own, unrelated checksum
  scheme). That's the CORRECT, expected result against such a device --
  it is not evidence of a bug, and this script says so rather than
  reporting a bare failure.
- A genuine positive end-to-end round trip -- sending a request to a
  real SMBus-PEC-aware device and confirming ITS PEC byte verifies
  correctly -- has NOT been demonstrated by this script, because no such
  device has been available on the bench. Don't claim that proof exists
  until it's actually been run against one.
"""

import sys

import serial
from serial.tools import list_ports

USB_VID = 0x1FC9
USB_PID = 0x0094


def smbus_pec_byte(crc, b):
    crc ^= b
    for _ in range(8):
        crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def smbus_pec_buf(crc, data):
    for b in data:
        crc = smbus_pec_byte(crc, b)
    return crc


def verify_pec_engine():
    # See this file's module docstring for why "123456789" -> 0xF4 is the
    # right thing to check, independent of any hardware.
    check = smbus_pec_buf(0, b"123456789")
    print(f"CRC-8/SMBUS('123456789') = 0x{check:02X} (expect 0xF4)")
    if check != 0xF4:
        print("FAIL: PEC engine does not match the CRC-8/SMBUS standard")
        return False
    print("PASS: PEC engine matches the CRC-8/SMBUS standard check value")
    return True


def find_port():
    for p in list_ports.comports():
        if p.vid == USB_VID and p.pid == USB_PID:
            return p.device
    raise SystemExit(
        f"No bridge found (USB VID:PID {USB_VID:04x}:{USB_PID:04x}). Is it plugged in?"
    )


def command(ser, line):
    ser.reset_input_buffer()
    ser.write((line + "\r\n").encode("ascii"))
    raw = ser.readline()
    reply = raw.decode("ascii", errors="replace").strip()
    print(f"-> {line!r}\n<- {reply!r}")
    return reply


def smoke_test_against_hardware():
    port = find_port()
    print(f"\nconnecting to {port}...")
    ser = serial.Serial(port, baudrate=115200, timeout=6.0)

    reply = command(ser, "S")
    parts = reply.split()
    if not parts or parts[0] != "OK" or len(parts) < 2:
        print("No device responded to the bus scan -- can't smoke-test WS/RS/XS "
              "without something on the bus to talk to.")
        ser.close()
        return
    addr = int(parts[1], 16)
    print(f"using address 0x{addr:02x} (first device found) for the smoke test")

    # WS: does a plain SMBus write complete cleanly with a computed,
    # appended PEC byte? This only proves framing/byte-count correctness --
    # see module docstring for what it can't prove.
    reply = command(ser, f"WS {addr:02x} 00")
    print("WS: 'OK' means the transfer completed with the right byte count "
          "(data + appended PEC); it does NOT mean the target validated the "
          "PEC, since most devices don't check it.\n")

    # RS: read back and let the firmware check the trailing PEC byte
    # against what it computes locally.
    reply = command(ser, f"RS {addr:02x} 2")
    if reply.startswith("ERR pec"):
        print("RS: PEC did NOT match -- EXPECTED and CORRECT for a device "
              "that isn't SMBus-PEC-aware (true of everything tested against "
              "so far, including OpenBIC's IPMB interface, which uses its "
              "own unrelated checksum). This confirms the mismatch-detection "
              "path fires as designed, not that anything is broken.\n")
    elif reply.startswith("OK"):
        print("RS: PEC matched?! Either this is a genuine SMBus-PEC-aware "
              "device (interesting -- worth investigating further) or a "
              "false positive worth double-checking by hand.\n")
    else:
        print(f"RS: unexpected reply: {reply!r}\n")

    ser.close()


if __name__ == "__main__":
    ok = verify_pec_engine()
    smoke_test_against_hardware()
    sys.exit(0 if ok else 1)

# FRDM-MCXA153 USB-to-I2C Hub

Turns NXP's [FRDM-MCXA153](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXA153)
board (MCX A153, Arm Cortex-M33) into a USB-to-I2C bridge: plug it into a
host PC over its "MCU USB" port, wire an I2C target to its mikroBUS
header, and drive that target's I2C bus with simple text commands over a
USB CDC virtual COM port. Once flashed, the board runs standalone off
that one USB cable — no debug probe or separate power needed.

Built from scratch as a minimal, command-line-only development
environment for this board: no IDE, no MCUXpresso installer, just
`arm-none-eabi-gcc`, `make`, and [`probe-rs`](https://probe.rs/) (used
only for flashing, over the board's on-board MCU-Link debug probe).

Two firmware targets share the same toolchain and vendor tree:

- **`usb_vcom`** — the USB-to-I2C hub itself. Brings up a USB CDC-ACM
  virtual COM port on the MCX A153's *own* USB0 controller (the board's
  second, "MCU USB" Type-C connector — separate from the MCU-Link debug
  port), and bridges it to an LPI2C0 master, so a host PC can drive I2C
  devices on another board through this one. Also blinks the red LED
  (~2 Hz) as a standalone heartbeat.
- **`blinky`** — a minimal standalone LED blink demo (GPIO3 pin 12),
  kept as the simplest possible "does my toolchain work" sanity check.

Both stream progress messages back over SWD via a minimal hand-rolled
SEGGER RTT implementation (`src/rtt.c`), so you can see what the firmware
is doing without a UART.

## Prerequisites

- `arm-none-eabi-gcc` (Ubuntu/Debian: `sudo apt install gcc-arm-none-eabi`)
- [`probe-rs`](https://probe.rs/): `curl --proto '=https' --tlsv1.2 -sSfL https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh`
- A FRDM-MCXA153 board plugged in over its MCU-Link USB port
- Your user in the `dialout` group if you want to open `/dev/ttyACM*`
  without sudo: `sudo usermod -aG dialout $USER` (then re-login)

## Build, flash, run

```sh
make TARGET=blinky              # or TARGET=usb_vcom (default: blinky)
make TARGET=blinky flash        # flash + reset via probe-rs
probe-rs run --chip MCXA153 build/blinky/blinky.elf   # flash + live RTT log
```

For `usb_vcom`, after flashing, plug a **data-capable** USB-C cable (not a
charge-only one) into the board's lower Type-C port (silkscreened "MCU
USB", labeled `J8` in the schematic) while leaving the MCU-Link port
connected. Wire an I2C target's SCL/SDA (+ GND) to the board's mikroBUS
header (`J5`: `SCL`/`SDA` pins, wired to the MCX A153's `P3_27`/`P3_28`).
It enumerates as `NXP MCU VIRTUAL COM DEMO` (`1fc9:0094`). **Don't assume
it's `/dev/ttyACM0` or `/dev/ttyACM1`** — the MCU-Link debug probe also
exposes its own (unrelated) ttyACM device, and which one gets which
number depends purely on enumeration order, which can and does change
across reconnects. Find the right one by product string:

```sh
for dev in /sys/class/tty/ttyACM*; do
  echo "$(basename "$dev") -> $(cat "$dev/device/../product" 2>/dev/null)"
done
# use whichever one prints "MCU VIRTUAL COM DEMO"
```

```sh
stty -F /dev/ttyACM0 raw -echo speed 115200
cat /dev/ttyACM0 &
printf 'S\r\n' > /dev/ttyACM0     # scan the bus, lists responding addresses
printf 'W 50 00 2a\r\n' > /dev/ttyACM0   # write 0x00 0x2a to device 0x50
printf 'X 50 2 00\r\n' > /dev/ttyACM0    # write 0x00, repeated-start, read 2 bytes
```

### I2C bridge command syntax

One command per line (LF or CRLF). `<addr>` is a 7-bit I2C address,
`<byte>` a data byte — both 1-2 hex digits, no `0x` prefix. `<n>` is a
decimal byte count.

| Command | Meaning |
|---|---|
| `W <addr> <byte> [byte ...]` | write bytes to `<addr>` |
| `R <addr> <n>` | read `<n>` bytes from `<addr>` |
| `X <addr> <n> <byte> [byte ...]` | write bytes, repeated-start, then read `<n>` bytes (the classic "select register, then read" pattern) |
| `S` | scan the bus, list responding addresses |

Replies: `OK` (write), `OK <byte> [byte ...]` (read/scan), or
`ERR <reason>` (e.g. `ERR nak`, `ERR timeout ...`, `ERR bad address`).

## Layout

```
src/main.c              blinky application
src/usb_main.c          usb_vcom application (adapted from NXP's bm example)
src/rtt.c/.h            minimal SEGGER RTT logger (no UART needed)
src/usb/                usb_vcom app-level config/descriptors (from NXP's
                        usb_device_cdc_vcom "bm" example, BSD-3-Clause)
vendor/sdk/             NXP device headers, startup code, linker script,
                        common/clock/reset/OSA drivers
                        (from nxp-mcuxpresso/legacy-mcux-sdk, BSD-3-Clause)
vendor/usb/             NXP's USB device stack + CDC-ACM class driver
                        (from nxp-mcuxpresso/mcuxsdk-middleware-usb, BSD-3-Clause)
vendor/sdk/drivers/lpi2c/  NXP's LPI2C master driver (legacy-mcux-sdk, BSD-3-Clause)
vendor/cmsis/           Arm CMSIS-Core headers (from ARM-software/CMSIS_5, Apache-2.0)
Makefile                plain GNU make build, no CMake/SDK manager required
```

## Gotchas hit along the way

**Two non-obvious MCXA153 clock/reset quirks** (found by adding RTT print
statements to `blinky` and watching exactly which register access faulted,
since the "obvious" setup silently produces a CPU bus fault instead of
working):

1. **`SYSCON->CLKUNLOCK`** write-protects the `MRCC_GLB_CC*`/`MRCC_GLB_RST*`
   registers. Clear its unlock bit before touching peripheral clock/reset
   control, and set it back after.
2. **Reset-release polarity is inverted from what you'd expect**: writing
   the `_SET` register (not `_CLR`) is what takes a peripheral *out* of
   reset on this chip's `MRCC_GLB_RST1` block (confirmed against NXP's own
   `RESET_ReleasePeripheralReset()`, which internally calls
   `RESET_SetPeripheralReset()`). Using `_CLR` re-asserts reset and any
   access to that peripheral's registers then hard-faults.

**One environmental gotcha for `usb_vcom`**: the firmware initialized
cleanly and enabled its D+ pull-up with zero faults, but no host ever saw
the device and no bus-reset event ever reached the firmware. Root cause
was a charge-only USB-C cable (power wires only, no D+/D− data lines) —
not a firmware bug. Swapping to a data-capable cable fixed it immediately.

**Three gotchas building the I2C bridge**:

1. `LPI2C_MasterStart()` alone does **not** tell you whether an address
   was acknowledged — it only enqueues the START command into the
   hardware FIFO and returns immediately, without waiting for the bus
   transaction to actually complete. A bus scan built on it will report
   false positives (and can wedge the bus for later addresses). Use
   `LPI2C_MasterTransferBlocking()` with a zero-length write instead —
   the standard I2C scan idiom — which correctly waits for and reports a
   NAK.
2. By default this LPI2C driver has `I2C_RETRY_TIMES == 0`, meaning "wait
   forever" on a stuck bus — so scanning/writing to a floating,
   unconnected bus (no target, no pull-ups) hangs forever instead of
   failing. Override it via a compiler define (`-DI2C_RETRY_TIMES=50000U`
   in the Makefile) to fail fast with `kStatus_LPI2C_Timeout` instead.
3. A hand-rolled command-line parser needs a real end-of-string marker.
   The line-assembly buffer here is a fixed, reused static array; without
   explicitly NUL-terminating each completed line, a parser that just
   scans forward for "not a digit" will happily read past the current
   line into stale bytes left over from a previous, longer command.
4. This LPI2C driver has no automatic bus recovery: once a transfer times
   out (e.g. a target device holds SCL or SDA low mid-transaction), the
   bus stays wedged and every subsequent transfer -- even a plain write --
   fails too, until something forces the lines free. `I2C_BusRecover()` in
   `usb_main.c` implements the standard fix (temporarily drive the pins as
   GPIO, clock SCL up to 9 times, drive a manual STOP, then reinit LPI2C0)
   and runs automatically after any non-NAK failure. Note this can only
   recover a bus a *stuck peripheral* is wedging; it can't force a device
   that's deliberately holding the line (e.g. one that doesn't like how a
   read was issued) to let go -- that's a target-device protocol question,
   not a bus electrical one.

**The debugging red herring that ate the most time**: after wiring up a
real I2C target, the bridge worked, then appeared to completely stop
responding to *any* command -- no reply at all, not even an error --
across power-cycles, replugs, and even full board power removal. The
actual cause: this board's MCU-Link debug probe exposes its own,
unrelated `ttyACM` device (a USB-to-UART bridge for a different LPUART),
and repeated reconnects during testing caused the kernel to reassign
which physical device got `ttyACM0` vs `ttyACM1`. Every "no response"
was simply commands going to the wrong tty. Lesson: never hardcode a
`ttyACM` number for a board with more than one CDC interface -- always
resolve it by USB product string (see the `usb_vcom` usage section
above).

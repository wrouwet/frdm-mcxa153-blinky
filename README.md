# FRDM-MCXA153 Bare-Metal Demos

A minimal, from-scratch, command-line-only development environment for NXP's
[FRDM-MCXA153](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXA153)
board (MCX A153, Arm Cortex-M33). No IDE, no MCUXpresso installer — just
`arm-none-eabi-gcc`, `make`, and [`probe-rs`](https://probe.rs/) talking to
the board's on-board MCU-Link debug probe over SWD.

Two demos share the same toolchain and vendor tree:

- **`blinky`** — toggles the on-board RGB LED's red channel (GPIO3 pin 12).
- **`usb_vcom`** — brings up a USB CDC-ACM virtual COM port on the MCX
  A153's *own* USB0 controller (the board's second, "MCU USB" Type-C
  connector — separate from the MCU-Link debug port) and echoes back
  whatever bytes the host sends it.

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
connected. It enumerates as `NXP MCU VIRTUAL COM DEMO` (`1fc9:0094`),
typically at `/dev/ttyACM1`:

```sh
stty -F /dev/ttyACM1 raw -echo speed 115200
cat /dev/ttyACM1 &
printf 'hello board\r\n' > /dev/ttyACM1   # should echo straight back
```

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

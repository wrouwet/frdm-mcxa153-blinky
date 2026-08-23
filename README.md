# FRDM-MCXA153 Blinky

A minimal, from-scratch, command-line-only development environment for NXP's
[FRDM-MCXA153](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXA153)
board (MCX A153, Arm Cortex-M33), proven with a bare-metal LED blink. No IDE,
no MCUXpresso installer — just `arm-none-eabi-gcc`, `make`, and
[`probe-rs`](https://probe.rs/) talking to the board's on-board MCU-Link
debug probe over SWD.

## What it does

Toggles the **red** channel of the on-board RGB LED (GPIO3 pin 12) roughly
every 200ms, and streams progress messages back over SWD via a minimal
hand-rolled SEGGER RTT implementation (`src/rtt.c`) so you can see what the
firmware is doing without a UART.

## Prerequisites

- `arm-none-eabi-gcc` (Ubuntu/Debian: `sudo apt install gcc-arm-none-eabi`)
- [`probe-rs`](https://probe.rs/): `curl --proto '=https' --tlsv1.2 -sSfL https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh`
- A FRDM-MCXA153 board plugged in over its MCU-Link USB port

## Build, flash, run

```sh
make            # -> build/blinky.elf, .bin, .hex
make flash      # flash + reset via probe-rs
probe-rs run --chip MCXA153 build/blinky.elf   # flash + live RTT log
```

## Layout

```
src/main.c        the actual application
src/rtt.c/.h       minimal SEGGER RTT logger (no UART needed)
vendor/sdk/        NXP device headers, startup code, linker script
                   (from nxp-mcuxpresso/legacy-mcux-sdk, BSD-3-Clause)
vendor/cmsis/      Arm CMSIS-Core headers (from ARM-software/CMSIS_5, Apache-2.0)
Makefile           plain GNU make build, no CMake/SDK manager required
```

## Two non-obvious MCXA153 gotchas

Found by adding RTT print statements and watching exactly which register
access faulted, since the "obvious" clock/reset setup silently produces a
CPU bus fault instead of working:

1. **`SYSCON->CLKUNLOCK`** write-protects the `MRCC_GLB_CC*`/`MRCC_GLB_RST*`
   registers. Clear its unlock bit before touching peripheral clock/reset
   control, and set it back after.
2. **Reset-release polarity is inverted from what you'd expect**: writing
   the `_SET` register (not `_CLR`) is what takes a peripheral *out* of
   reset on this chip's `MRCC_GLB_RST1` block (confirmed against NXP's own
   `RESET_ReleasePeripheralReset()`, which internally calls
   `RESET_SetPeripheralReset()`). Using `_CLR` re-asserts reset and any
   access to that peripheral's registers then hard-faults.

# FPB-RA0E2 Board Support

HAL implementation for the Renesas FPB-RA0E2 evaluation board with LoRa Plus Expansion Board (WIO LR2021 radio).

## Overview

This HAL enables LoRaWAN Class A operation on the ultra-low-power RA0E2 MCU (Cortex-M23). Key achievements:

- **1.3 µA sleep current** (MCU + Radio combined) using software standby mode
- Full LoRaWAN stack with US915 region support
- ~79KB flash footprint (production build)

## Hardware Requirements

- **MCU Board**: Renesas FPB-RA0E2 (R7FA0E209)
- **Radio Board**: LoRa Plus Expansion Board (LR2021EVK1XCS1) with WIO LR2021 (with XIAO nRF54L15 module removed from the expansion board)
- **Debugger**: On-board J-Link (included on FPB-RA0E2)
- **Power Measurement**: Nordic Power Profiler Kit II (or similar) for current measurements

### Hardware Modification Required

The WIO LR2021 uses DIO8 for interrupts, but DIO8 routes to a pin without external interrupt capability on the FPB-RA0E2. A jumper wire is required:

1. On the LoRa Plus Expansion Board, connect **U2-D0** to **U2-D15**
2. This routes DIO8 to Arduino header D5 (P201/IRQ5)

### LED Usage

LED2 (P102): OFF = sleep/standby, ON = awake.

## Software Requirements

- **Toolchain**: ARM GCC (`arm-none-eabi-gcc` 13.2.1)
- **Build System**: CMake 3.20+, Ninja
- **Flash Tool**: SEGGER J-Link

## Build Commands

### Development Build (With Trace Output)

HAL and LBM trace enabled for debugging. Uses `-Os` optimization with debug symbols (`-g`) to fit in flash while enabling ELF debugging.

Note: Software standby mode breaks the debug connection. For interactive debugging, add low-power disable flags:
```
-DCMAKE_C_FLAGS_DEBUG="-g -Os -DLOW_POWER_MODE=0 -DHW_DEBUG_PROBE=1"
```

**Size**: ~97KB flash, ~11KB RAM

```bash
cmake -S examples/main_examples/periodical_uplink_example -B build \
  -DBOARD=FPB_RA0E2 -DRAC_RADIO=lr2021 -DLBM_RADIO=lr2021 \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_FLAGS_DEBUG="-g -Os" \
  -DRAC_LOG_PROFILE=MINIMAL -DLBM_REGIONS=US_915 \
  -DLBM_CLASS_B=OFF -DLBM_CLASS_C=OFF \
  -DHAL_DBG_TRACE=ON -DLBM_MODEM_TRACE=ON -G Ninja

cmake --build build
```

### Production Build (Minimal Size)

No trace output, US915 only, Class A only.

**Size**: ~79KB flash, ~11KB RAM

```bash
cmake -S examples/main_examples/periodical_uplink_example -B build \
  -DBOARD=FPB_RA0E2 -DRAC_RADIO=lr2021 -DLBM_RADIO=lr2021 \
  -DCMAKE_BUILD_TYPE=MinSizeRel \
  -DRAC_LOG_PROFILE=MINIMAL -DLBM_REGIONS=US_915 \
  -DLBM_CLASS_B=OFF -DLBM_CLASS_C=OFF \
  -DLBM_MULTICAST=OFF -DLBM_CSMA=OFF -DLBM_ALC_SYNC=OFF \
  -DLBM_FUOTA=OFF -DLBM_ALMANAC=OFF -DLBM_STREAM=OFF \
  -DLBM_LFU=OFF -DLBM_DEVICE_MANAGEMENT=OFF -DLBM_GEOLOCATION=OFF \
  -DLBM_STORE_AND_FORWARD=OFF -DLBM_MODEM_TRACE=OFF \
  -DHAL_DBG_TRACE=OFF -G Ninja

cmake --build build
```

### Flashing

```bash
JLinkExe -device R7FA0E209 -if SWD -speed 4000 -autoconnect 1 -CommandFile flash.jlink
```

Where `flash.jlink` contains:
```
r
h
loadfile build_fpb_ra0e2/periodical_uplink.hex
r
g
exit
```

For a full chip erase (required when changing OFS0 option bytes):
```
r
h
erase
loadfile build_fpb_ra0e2/periodical_uplink.hex
r
g
exit
```

## Debugging with VS Code

### Prerequisites

1. Install the **Cortex-Debug** extension in VS Code
2. Install `gdb-multiarch`:
   ```bash
   sudo apt install gdb-multiarch
   ```

### Launch Configuration

The `.vscode/launch.json` includes debug configurations for the FPB-RA0E2. Key settings:

- **Device**: R7FA0E209
- **Interface**: SWD
- **Server**: J-Link GDB Server
- **GDB**: gdb-multiarch

To debug:
1. Build with the development build command above (includes `-g` debug symbols)
2. Select the appropriate launch configuration in VS Code (optionally select the right "armToolchainPath" setting)
3. Press F5 to start debugging
```

## Status

### Completed

- [x] All HAL modules (GPIO, SPI, UART, RTC, Timers, Flash, RNG, Watchdog)
- [x] Software standby mode (1.3 µA)
- [x] LoRaWAN Class A join and uplink
- [x] US915 region support

### TODO

- [ ] Verify RX window wake timing accuracy
  - Measure actual software standby wakeup time (HOCO restart + peripheral reinit)
  - Adjust `smtc_modem_hal_get_board_delay_ms()` if needed (currently 1ms)
  - Verify `RP_MARGIN_DELAY` is sufficient (currently 8ms default)
- [ ] Integration into production USP codebase
- [ ] Full Class A validation testing
- [ ] Class B support (not tested)
- [ ] Class C support (not tested)
- [ ] HW Modem UART (stubs only)

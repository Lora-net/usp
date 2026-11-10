# LoRa Basics Modem (LBM) Porting Guide to USP

## Overview

This guide explains how to port existing LoRa Basics Modem applications to the **USP** environment. It covers the migration from SWL2001, the key API changes and the initialization sequence.

**Target Audience:** Developers porting SWL2001 LBM applications to USP.

---

## Table of Contents

1. [Migrating from SWL2001 to USP](#migrating-from-swl2001-to-usp)
   - [Why migrate](#why-migrate)
   - [What changes vs. SWL2001](#what-changes-vs-swl2001)
   - [Migration steps](#migration-steps)
   - [Application code changes](#application-code-changes)
   - [From Makefile to CMake](#from-makefile-to-cmake)
   - [Memory footprint](#memory-footprint)
2. [Key Changes Summary](#key-changes-summary)
3. [Initialization Sequence](#initialization-sequence)
4. [Main Loop Integration](#main-loop-integration)
5. [Reference Examples](#reference-examples)

---

## Migrating from [SWL2001](https://github.com/Lora-net/SWL2001) to USP

The **[SWL2001](https://github.com/Lora-net/SWL2001)** LBM Git repository is now in maintenance mode. The last release (v4.9.3) remains supported: our support and critical bug fixing will continue, but no new features or optimizations will be added. New developments of LoRa Basics Modem (LBM) are only available as part of USP, and we encourage you to consider migrating at your own pace. For more details, please contact the [Semtech Support](https://www.semtech.com/company/contact/support).

This section explains what changes for existing SWL2001 users, and how to move a project to USP.

### Why migrate

- The v4.9.3 LBM release is delivered **simultaneously** in SWL2001 and USP as a last, aligned release for LBM and all the dependent software components (drivers, ral, ralf, modem hal). You can keep using it with confidence.
- Beyond this release, new updates are produced **in USP only** and will not be added to SWL2001. All new LBM work lands in USP, for example LR11xx and LR20xx driver updates, ongoing Regional Parameters updates, alignment with new LoRaWAN specifications, and new features.

### What changes vs. SWL2001

| Topic | SWL2001 | USP |
|---|---|---|
| Git Repository | `Lora-net/SWL2001` | `Lora-net/usp` |
| LBM location | `lbm/lbm_lib/` (v4.9.3; `lbm_lib/` at repo root in earlier versions) | `protocols/lbm_lib/` |
| Build entry point | `lbm_examples` (Make or CMake) | `examples/main_examples/<example>` (CMake only) |
| Radio selector | `LBM_RADIO=<radio>` (CMake), Make target `<radio>` / `full_<radio>` | `RAC_RADIO=<radio>` — same radio names (`sx1261/sx1262/sx1268/sx1272/sx1276/sx1280/sx1281/lr1110/lr1120/lr1121/lr2021`), plus `lr2012/lr2022` |
| Board selector | `BOARD=NUCLEO_L476` / `NUCLEO_L073` | `BOARD=NUCLEO_L476` / `NUCLEO_L073` / `FPB_RA0E2` / `LINUX` / `LINUX_ARM` / `LINUX_ARM64` |
| Radio access | LBM sits on top of radio planner | LBM sits on top of the RAC (Radio Access Component), which arbitrates radio access across LoRaWAN, FLRP, ranging, etc. |
| Validation | Validation performed through LCTT + non-regression | Validation performed through LCTT + non-regression (see [Validation & Performances](https://github.com/Lora-net/usp/blob/main/doc/VALIDATION_AND_PERFORMANCES.md)) |

### Migration steps

1. **Clone USP** instead of SWL2001:
   ```
   git clone https://github.com/Lora-net/usp.git
   ```
2. **Locate your LBM configuration.** Application-level LBM options (regions, FUOTA packages, class B/C, crypto backend, relay, store & forward, etc.) map 1:1 to the same CMake symbols; the library now lives under `protocols/lbm_lib/` instead of `lbm/lbm_lib/` (`lbm_lib/` in SWL2001 versions before v4.9.3).
3. **Update your build command.** For example, an existing SWL2001 build (SWL2001's own build commands, as documented in its README):
   ```
   make -C lbm_examples full_lr1110 MODEM_APP=PERIODICAL_UPLINK
   ```
   or
   ```
   cd lbm_examples
   cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=MinSizeRel -DLBM_CMAKE_CONFIG_AUTO=ON -DBOARD=NUCLEO_L476 -DLBM_RADIO=lr1110 -DAPP=periodical_uplink
   ```
   becomes, in USP:
   ```
   cmake -S examples/main_examples/periodical_uplink_example -B build \
       -DCMAKE_BUILD_TYPE=MinSizeRel \
       -DBOARD=NUCLEO_L476 \
       -DRAC_RADIO=lr1110 \
       -G Ninja
   cmake --build build
   ```
4. **Update your application code** to initialize and run the RAC — see [Application code changes](#application-code-changes) below.
5. **If you reuse your own LR20xx radio BSP** (`ral_lr20xx_bsp.c` of a custom board), check that it provides the `ral_lr20xx_bsp_get_rssi_calibration_table()` function, and add it if needed: it is called by the RAL at radio initialization and is mandatory since USP v1.3.0 (link error otherwise). It returns the RSSI calibration tables of your receive chain, either one of the reference tables provided in `examples/radio_hal/ral_lr20xx_bsp.c` (board equivalent to the Semtech reference design) or a table characterized on your board — see [RSSI Calibration Tables Selection](usp_porting_guide.md#rssi-calibration-tables-selection-lr20xx) in the USP porting guide.
6. **Pass your LoRaWAN keys/region and any custom `LBM_*` compilation defines** the same way as before, using `CFLAGS` if no dedicated CMake symbol exists yet — see [cmake compilation symbols & C compilation definitions](https://github.com/Lora-net/usp/blob/main/README.md#cmake-compilation-symbols--c-compilation-definitions) in the USP README.
7. **Re-flash and validate** on your target board (`openocd` commands are unchanged in form, only the built binary path changes to USP's `build/` output).
8. If you use one of the SWL2001 `lbm_applications` ports:
    - **ThreadX/STM32U5: do not exist anymore**.
    - Nordic nRF52840: can be addressed through [usp_zephyr](https://github.com/Lora-net/usp_zephyr).
    - LoRa Edge geolocation: check the corresponding example/porting folder in USP (`examples/`) for the equivalent.

### Application code changes

Because LBM now sits on top of the RAC, a USP application must also initialize and run the RAC, in addition to the usual modem calls (`#include "smtc_rac_api.h"`):

- call **`smtc_rac_init( )`** once at startup, **before** `smtc_modem_init( )` — this order is mandatory, since LBM relies on the RAC for all radio operations (if your port uses `smtc_rac_set_radio_context( )`, it must be called before `smtc_rac_init( )`);
- call **`smtc_rac_run_engine( )`** in the main loop, right after `smtc_modem_run_engine( )`.

See [Key Changes Summary](#key-changes-summary), [Initialization Sequence](#initialization-sequence) and [Main Loop Integration](#main-loop-integration) below for the full code (including IRQ handling), or `examples/main_examples/periodical_uplink_example/main_periodical_uplink.c` for a complete example.

### From Makefile to CMake

SWL2001's `lbm_examples` could be built with **either Make or CMake**. USP is **CMake-only** (there is no top-level `Makefile` in `examples/main_examples/`). If your project currently invokes `make -C lbm_examples ...`, you need to translate the Make target/variables into CMake `-D` options.

**General pattern:**

| SWL2001 (Make) | USP (CMake) |
|---|---|
| `make -C lbm_examples <target> VAR1=val1 VAR2=val2 ...` | `cmake -S examples/main_examples/<example> -B build -DCMAKE_BUILD_TYPE=<...> -DBOARD=<...> -DRAC_RADIO=<...> -DVAR1=val1 ... -G Ninja && cmake --build build` |
| Make **target** selects the app + radio combo | CMake **source directory** selects the app (`examples/main_examples/<example>`); `-DRAC_RADIO=<radio>` selects the radio |
| `MODEM_APP=<APP_NAME>` (`PERIODICAL_UPLINK`, `HW_MODEM`, `PORTING_TESTS`, `LCTT_CERTIF`) | Pick the matching example folder directly: `periodical_uplink_example`, `hw_modem`, `porting_tests_example`, `lctt_certif_example` |
| `LBM_RADIO=<radio>` (when using SWL2001's CMake flavor) | `RAC_RADIO=<radio>` — same radio names |
| `BOARD=NUCLEO_L476` | unchanged: `BOARD=NUCLEO_L476` (also `NUCLEO_L073`, `FPB_RA0E2`, `LINUX`, `LINUX_ARM`, `LINUX_ARM64`) |
| `REGION=<region>` (Make, e.g. `EU_868`, `US_915`; default: all) | `-DLBM_REGIONS=<region>` (default `ALL`); same region names |
| `CRYPTO=SOFT/LR11XX/LR11XX_WITH_CREDENTIALS` (Make) | `-DLBM_CRYPTO=<value>` |
| `ALLOW_RELAY_TX=yes` / `ALLOW_RELAY_RX=yes` (Make) | `-DLBM_RELAY_TX=ON` / `-DLBM_RELAY_RX=ON` |
| `LBM_TRACE=yes/no` / `APP_TRACE=yes/no` (Make) | `-DLBM_MODEM_TRACE=ON/OFF` / `-DAPP_TRACE=ON/OFF` (check the example's CMake options) |
| Other feature flags (class B/C, multicast, FUOTA packages, CSMA, store & forward, geolocation, etc.) | Same `LBM_*` boolean/string CMake symbols (`LBM_CLASS_B`, `LBM_CLASS_C`, `LBM_MULTICAST`, `LBM_FUOTA`, `LBM_CSMA`, `LBM_STORE_AND_FORWARD`, etc.) — pass them as `-D<NAME>=<VALUE>` |
| Keys/DevEUI/JoinEUI passed as Make `CFLAGS`/defines | Same approach: pass via `env CFLAGS="-DUSER_LORAWAN_DEVICE_EUI='{...}' ..." cmake ... -UCMAKE_C_FLAGS` [see example](../README.md#passing-custom-c-flags) |
| Board programming | SWL2001's Make flow has no `flash` target (only the CMake+Ninja flow offers `ninja flash`); with USP, program the board with `openocd` (see USP README build sections) or `ninja flash` if you keep the CMake+Ninja flow |

**Worked example.** Taking the same `periodical_uplink`/`lr1110` command as in [Migration steps](#migration-steps) above and adding a region restriction plus Class B support, a typical SWL2001 Make invocation:

```
make -C lbm_examples full_lr1110 MODEM_APP=PERIODICAL_UPLINK REGION=EU_868 LBM_CLASS_B=yes
```

becomes, in USP, the same `cmake` command as in step 3 plus `-DLBM_REGIONS=EU_868 -DLBM_CLASS_B=ON`:

```
cmake -S examples/main_examples/periodical_uplink_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr1110 \
    -DLBM_REGIONS=EU_868 \
    -DLBM_CLASS_B=ON \
    -G Ninja
cmake --build build
```

If you were instead using SWL2001's own CMake flow (`cmake ... -DLBM_RADIO=lr1110 -DAPP=periodical_uplink`), the change is smaller — mainly `LBM_RADIO` → `RAC_RADIO`, plus pointing `-S` at the matching `examples/main_examples/<example>` folder instead of `-DAPP=<example>`.

**Custom/derived Makefiles:** if your team maintains its own wrapper Makefile on top of SWL2001's (e.g. to inject board files, custom `CFLAGS`, or CI steps), the safest migration path is to keep that wrapper but change its body to call the CMake commands above instead of SWL2001's `make -C lbm_examples`. The table above covers the variables exposed by SWL2001's `make help`; project-specific variables of your own wrapper must be mapped manually.

> **Note**: Work is in progress to make CMake easier to use in USP. USP is planned to support CMake features like `FetchContent`, allowing the USP Git repository to be used as a dependency, locally or from a Git repository.

### Memory footprint

Moving from SWL2001 to USP typically adds a **small flash/RAM overhead**: **about +0.5 KB of flash and +60 B of RAM (RAC glue)**. But BSP/mcu-hal bring more (c.f. below).

**Indicative example.** Overhead of the USP `periodical_uplink_example` compared to the SWL2001 `lbm_examples` `periodical_uplink` (lr2021, NUCLEO_L476, `MinSizeRel`). These figures are given for information only and may vary with the version and the build options:

- **STM32L4 SPI HAL (flash +4.2 KB, RAM +148 B).** Almost all the flash overhead comes from the SPI HAL of the USP examples: the ST HAL SPI/DMA layers (DMA, buffers) weigh more than the byte-by-byte access used by SWL2001. This HAL is an example porting that can be reviewed by the customer according to the needs of the application, and the whole SPI path can be optimized globally, for instance with a polling byte-wise implementation instead of the DMA one.
- **Radio planner (RAM +1.3 KB).** The RAC adds 5 priority hooks to the radio planner (`RP_HOOK_RAC_VERY_HIGH/HIGH/MEDIUM/LOW/VERY_LOW_PRIORITY`, about 270 B of RAM each, since the planner arrays are sized by `RP_NB_HOOKS` = `RP_HOOK_ID_MAX`). They are only used by applications that access the radio through the RAC API (`smtc_rac_open_radio`, `smtc_rac_immediate_radio_access`); LBM does not use them, and the RAC itself only initializes `RP_HOOK_ID_SUSPEND`. In an LBM-only application they are therefore allocated for nothing and can be removed, see [Removing the unused RAC hooks](#removing-the-unused-rac-hooks) below.
- **Others** (RAC glue, small HAL differences): about +0.5 KB of flash and +60 B of RAM, for a total of about +4.7 KB of flash and +1.5 KB of RAM on STM32L4 if mcu-hal is kept as-is.

The memory footprint can increase by adding LBM features or USP/RAC features (e.g. in a multi-protocol context):

- **LBM-only builds** (no other protocols linked in) carry a limited overhead.
- **Builds that also link FLRP, ranging (RTToF), or multiple concurrent protocols** carry a larger overhead, since more of the RAC's scheduler and additional protocol code is included.
- Enabling more `LBM_*` features (class B, all regions via `LBM_REGIONS=ALL`, FUOTA packages, geolocation, etc.) inflates flash size the same way it already did in SWL2001 — this part of the footprint is unchanged by the migration itself.

#### Removing the unused RAC hooks

If your application only uses LBM (no FLRP, no ranging, no direct call to the RAC API), you can recover this RAM by providing your own hook list:

1. Define `RP_HOOK_ID_REDEFINE`: the default `RP_HOOK_ID_DEF` enum of `smtc_rac_lib/radio_planner/src/radio_planner_hook_id_defs.h` is then not compiled.
2. Provide your own `RP_HOOK_ID_DEF` enum in a header that is included first (`-include`). Start from the default enum and keep only `RP_HOOK_ID_SUSPEND` and the LBM hooks of the features you enabled (the `ADD_*` blocks, same order as the default). `smtc_rac.h` still references the `RP_HOOK_RAC_*_PRIORITY` names, so keep them as **aliases of `RP_HOOK_ID_SUSPEND`**: they do not reserve any slot.
3. Pass both to the build, for example with `env CFLAGS="-DRP_HOOK_ID_REDEFINE -include /path/to/rp_hooks_lbm_only.h" cmake ... -UCMAKE_C_FLAGS`.

Example for `periodical_uplink` (no optional LBM feature enabled):

```c
#define NUMBER_OF_STACKS 1

enum RP_HOOK_ID_DEF
{
    RP_HOOK_ID_SUSPEND = 0,

    /* RAC priority hooks: aliases of SUSPEND, no slot reserved */
    RP_HOOK_RAC_VERY_HIGH_PRIORITY = RP_HOOK_ID_SUSPEND,
    RP_HOOK_RAC_HIGH_PRIORITY      = RP_HOOK_ID_SUSPEND,
    RP_HOOK_RAC_MEDIUM_PRIORITY    = RP_HOOK_ID_SUSPEND,
    RP_HOOK_RAC_LOW_PRIORITY       = RP_HOOK_ID_SUSPEND,
    RP_HOOK_RAC_VERY_LOW_PRIORITY  = RP_HOOK_ID_SUSPEND,

    RP_HOOK_ID_LR1MAC_STACK,
    RP_HOOK_ID_LBT              = RP_HOOK_ID_LR1MAC_STACK + NUMBER_OF_STACKS,
    RP_HOOK_ID_TEST_MODE        = RP_HOOK_ID_LBT + NUMBER_OF_STACKS,
    RP_HOOK_ID_DIRECT_RP_ACCESS = RP_HOOK_ID_TEST_MODE + NUMBER_OF_STACKS,
    RP_HOOK_ID_MAX              = RP_HOOK_ID_DIRECT_RP_ACCESS + NUMBER_OF_STACKS,
};
```

On this example, the radio planner goes back from 2804 B to 1468 B, the same size as in SWL2001 (about 1.3 KB of RAM saved, no flash impact).

> **Warning:** with this list, all the RAC priorities share the `SUSPEND` hook. Do not use it if the application (or another protocol) opens the radio through the RAC API (`smtc_rac_open_radio`, `smtc_rac_immediate_radio_access`), or links FLRP or ranging.

**How to measure the overhead on your own configuration:**

1. Build the **same example, radio, region set and `LBM_*` options** on both SWL2001 and USP, with `-DCMAKE_BUILD_TYPE=MinSizeRel` on both sides for a fair comparison.
2. Read flash and RAM usage from the linker's size summary at the end of the build (or run `arm-none-eabi-size build/<binary>.elf` for a `.text`/`.data`/`.bss` breakdown: flash = `.text` + `.data`, RAM = `.data` + `.bss`).
3. For a finer breakdown, generate the `.map` file (`-Wl,-Map=...`) for both builds and compare the size of each object file/library: this is the way to see how much comes specifically from `smtc_rac_lib/` vs. `protocols/lbm_lib/`. `arm-none-eabi-nm --size-sort build/<binary>.elf` lists the largest symbols.

#### Example of compilation with minimal memory footprint (standard `periodical_uplink` application without hooks redefinition)

| Git Repository | Compilation Line | Estimated Footprint |
| --- | --- | --- |
| SWL2001 | cmake -S lbm_examples -B build -G Ninja -DAPP=periodical_uplink -DBOARD=NUCLEO_L476 -DLBM_RADIO=lr2021 -DAPP_TRACE=OFF -DCMAKE_BUILD_TYPE=MinSizeRel -DLBM_ALC_SYNC=OFF -DLBM_BEACON_TX=OFF -DLBM_CLASS_B=OFF -DLBM_CLASS_C=OFF -DLBM_CMAKE_CONFIG_AUTO=ON -DLBM_CSMA=OFF -DLBM_DEVICE_MANAGEMENT=OFF -DLBM_FUOTA=OFF -DLBM_LFU=OFF -DLBM_MODEM_TRACE=OFF -DLBM_MODEM_TRACE_DEEP=OFF -DLBM_MULTICAST=OFF -DLBM_REGIONS=EU_868 -DLBM_RELAY_RX=OFF -DLBM_RELAY_TX=OFF -DLBM_STORE_AND_FORWARD=OFF -DLBM_STREAM=OFF && cmake --build build | ROM: ~78KB RAM: ~11KB |
| USP | cmake -S examples/main_examples/periodical_uplink_example -B build -G Ninja -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -DCMAKE_BUILD_TYPE=MinSizeRel -DHAL_DBG_TRACE=OFF -DLBM_ALC_SYNC=OFF -DLBM_BEACON_TX=OFF -DLBM_CLASS_B=OFF -DLBM_CLASS_C=OFF -DLBM_CSMA=OFF -DLBM_DEVICE_MANAGEMENT=OFF -DLBM_FUOTA=OFF -DLBM_LFU=OFF -DLBM_MODEM_TRACE=OFF -DLBM_MODEM_TRACE_DEEP=OFF -DLBM_MULTICAST=OFF -DLBM_REGIONS=EU_868 -DLBM_RELAY_RX=OFF -DLBM_RELAY_TX=OFF -DLBM_STORE_AND_FORWARD=OFF -DLBM_STREAM=OFF -DMODEM_HAL_DBG_TRACE=OFF -DRAC_LOG_ENABLE=OFF -DRAC_LOG_PROFILE=MINIMAL -DSMTC_RADIO_PLANNER_TRACE=OFF -DSMTC_RADIO_PLANNER_TRACE_DBG=OFF && cmake --build build --target periodical_uplink | ROM: ~83KB RAM: ~12KB |



---

## Key Changes Summary

When porting from traditional LBM to USP, you need to:

| Traditional LBM | USP | Reason |
|----------------|----------------|--------|
| `smtc_modem_init()` only | **`smtc_rac_init()` + `smtc_modem_init()`** | RAC (Radio Access Controller) must be initialized first |
| `smtc_modem_run_engine()` only | **`smtc_modem_run_engine()` + `smtc_rac_run_engine()`** | Both engines need to run |

---

## Initialization Sequence

### ✅ Correct Initialization Order

```c
#include <smtc_rac_api.h>
#include <smtc_modem_api.h>

int main(void)
{
    // 1. Initialize RAC (Radio Access Controller) - MUST BE FIRST
    smtc_rac_init();

    // 2. Initialize LBM modem - AFTER RAC
    smtc_modem_init(&modem_event_callback);

    // Your application code...
}
```

### ⚠️ Critical: Order Matters!

**`smtc_rac_init()` MUST be called BEFORE `smtc_modem_init()`**

This is because:
- RAC manages the radio hardware abstraction layer
- LBM depends on RAC for radio operations
- Calling them in the wrong order will cause initialization failures

---

## Main Loop Integration

### Traditional LBM Main Loop

```c
// Traditional (bare-metal or FreeRTOS)
while(1) {
    uint32_t sleep_time_ms = smtc_modem_run_engine();
    // Sleep or handle events
    sleep(sleep_time_ms);
}
```

### USP Main Loop

```c
// USP - TWO engines to run!
while(true) {
    // 1. Run the modem engine
    uint32_t sleep_time_ms = smtc_modem_run_engine();

    // 2. Run the RAC engine - REQUIRED!
    smtc_rac_run_engine();

    // 3. Check for pending IRQ before sleeping
    hal_mcu_disable_irq( );
    if(smtc_rac_is_irq_flag_pending())
    {
        hal_mcu_enable_irq( );
        continue; // Don't sleep if radio IRQ is pending
    }

    // 4. Wait for events or timeout
    hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
    hal_mcu_enable_irq( );
}
```

### Key Points

1. **`smtc_rac_run_engine()` is MANDATORY** - Must be called in addition to `smtc_modem_run_engine()`
2. **Check IRQ flags** - Use `smtc_rac_is_irq_flag_pending()` to avoid missing radio events

---

## Reference Examples

### Minimal Working Example
See: `examples/main_examples/periodical_uplink_example/`

This example demonstrates:
- Proper initialization sequence
- Both run engines
- Relay TX configuration

See: `examples/main_examples/porting_tests_example/`
This example checks the minimal HW requirements for your porting: SPI, IRQ, timer, etc.


### Key Files to Study
1. `examples/main_examples/periodical_uplink_example/main_periodical_uplink.c` - Complete application example
2. `doc/usp_porting_guide.md` - Hardware porting guide

---

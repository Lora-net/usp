# USP changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.3.0] 2026-11-09

### Added

- FLRP
  - TX statistics given to the TX done callback: new `smtc_flrp_tx_stats_t` / `smtc_flrp_burst_tx_stats_t` (frequency used, frequency offset, payload bit rate including Adaptive Link + Burst + Burst ACK + retries, burst bit rate)
  - Frequency used, frequency offset (from the WOR), payload bit rate & burst bit rate added in RX burst statistics (`smtc_flrp_burst_rx_stats_t`)
  - New API `smtc_flrp_is_transaction_active()`: true while a one-shot transaction is in progress (WOR_TX or DATA), false when idle or in periodic listening
  - `frequency_offset_enabled` field in the advanced FLRC radio configuration (`smtc_flrp_api_tests.h`, also available in hw_modem protobuf) to disable the application of the computed frequency offset (enabled by default)
- LBM: see [LoRa Basics Modem v4.9.3](#lora-basics-modem-v493) below
- Radio / RAL
  - LR20XX RSSI calibration: new BSP function `ral_lr20xx_bsp_get_rssi_calibration_table()` called at RAL init, with 3 ready-to-use reference tables in `examples/radio_hal/ral_lr20xx_bsp.c` (LF 868/915MHz, LF 490MHz, HF 2.4GHz) & documentation in the [USP porting guide](doc/usp_porting_guide.md)
  - SX127X (v1.1.0) and SX128X (v1.1.0) drivers added as submodules of `smtc_radio_interface`
  - LR2021 PRAM version checked at radio init
- hw_modem
  - Commands ported from modem-e: `CMD_WIFI_SCAN_MODE`, `CMD_GNSS_ALM_DEMOD_STOP`, `CMD_WIFI_GET_SCAN_DONE_SCAN_DATA_RESULT`, `CMD_GET/SET_NO_RX_PACKET_THRESHOLD`, `CMD_GET/SET_CHANNEL_MASK`
  - Single-UART transport without external bridge (`hw_modem_serial.c`, default on STM32L4 with USART2): commands, responses and traces are multiplexed on the same UART. The physical bridge (GPIO + UART4) is still available with `-DHW_MODEM_USE_BRIDGE=ON`
  - `python_test/python_bridge.py` host tool (Linux & Windows) demultiplexing traces & data coming from the single UART (see `README-python-bridge.md`)
  - `python_test/flrp_configuration.py`: python FLRP configuration aligned with the C headers (`get_flrp_configuration`)
  - FLRP frequency & frequency offset information
- LINUX
  - SX126X and LR11XX Linux HAL (`sx126x_hal_linux.c`, `lr11xx_hal_linux.c`) & pinout (`modem_pinout_linux.h`)
  - Radio sleep/wake-up management on Linux for LR20XX, LR11XX & SX126X (wake-up through a NSS pulse, new `hal_spi_nss_pulse()`)
  - Optional `LP_TIMER_LATENCY_LOG` build option to log the low power timer latency/jitter
- One Channel Hub example (Linux only): 1-channel LoRaWAN gateway (packet forwarder over RAC) + optional LBM end-device on a single radio, with a pytest test harness
- FHSS example (`fhss_example`): LoRa intra-packet frequency hopping PING/PONG between two LR20XX radios
- Traces can be removed from the build with the CMake options `MODEM_HAL_DBG_TRACE` (core HAL traces used by LBM, RAC & radio planner) and `HAL_DBG_TRACE` (MCU HAL traces: STM32L0, STM32L4 & RA0E2), both ON by default
- Toolchain: a user-provided toolchain file (`-DCMAKE_TOOLCHAIN_FILE=`) is now taken into account, and the cross-compiler prefix is configurable in all the toolchain files (`TOOLCHAIN_CROSS_PREFIX`, e.g. ARM `aarch64-none-linux-gnu-` toolchain for `LINUX_ARM64`)
- STM32L4 HAL: `hal_trace_set_tx_function()` to override the trace output, `hal_uart_rx_start()` / `hal_uart_rx_write_index()` circular DMA RX on the trace UART (`HAL_UART_RX_ENABLED`)

### Changed

- New modular architecture (LBM & RAC decoupling)
  - Radio drivers, RAL & RALF are now provided by the new `smtc_rac_lib/smtc_radio_interface` submodule, which also checks the selected radio and exposes `RADIO_FAMILY`, `RADIO_FAMILY_UPPER` & `RADIO_UPPER` to the caller CMake. The `smtc_rac_lib/smtc_ral`, `smtc_rac_lib/smtc_ralf` & `smtc_rac_lib/radio_drivers/*` submodules are removed
  - New `smtc_rac_lib/smtc_core_hal_interface` submodule: the HAL is split in a "core" HAL (common to RAC, FLRP, radio planner…) and the LBM specific modem HAL
    - `smtc_modem_hal.h` renamed `smtc_core_hal.h`, `smtc_modem_hal_dbg_trace.h` renamed `smtc_core_hal_trace.h`: applications includes must be updated
    - CMake target `smtc_modem_hal_interface` renamed `smtc_core_hal_interface` (the LBM specific `smtc_modem_hal_interface` is now provided by lbm_lib and only linked when LBM is used)
  - LBM is now optional in RAC: new CMake option `INCLUDE_LBM` (set by each LBM based example) & `USE_LBM` define. When LBM is used, `smtc_rac_init()` gives the radio planner & RALF contexts to LBM (`smtc_modem_set_rp_ralf_contextes()`)
  - Radio planner decoupled from LBM (builds without RAC/LBM, own trace macros, LBM/FLRP hook IDs under `#ifdef`)
  - `-DLBM_RADIO=` is deprecated: only `-DRAC_RADIO=` is required, `LBM_RADIO` is automatically set
  - CMake minimum version is now 3.25; RAC logging configuration moved in `smtc_rac_lib/rac_logging.cmake`; FLRP crypto built as a dedicated `smtc_flrp_crypto` library
  - `smtc_modem_hal_protect_api_call()` / `smtc_modem_hal_unprotect_api_call()` are now weak no-op functions that multi-threaded applications can override
  - All examples updated for the new architecture
- FLRP
  - API: `smtc_flrp_tx_done_f` TX callback has a new `smtc_flrp_tx_stats_t flrp_stats` parameter (user callbacks must be updated)
  - `smtc_flrp_initiate_transmission()` / `smtc_flrp_initiate_reception()` return `SMTC_FLRP_RC_BUSY` if a transmission/reception is already in progress
  - Sub-GHz defines renamed: `WOR_865MHz_RF_FREQ_IN_HZ` -> `WOR_SUBGIG_RF_FREQ_IN_HZ`, `DATA_FLRC_865MHz_RF_FREQ_IN_HZ` -> `DATA_FLRC_SUBGIG_RF_FREQ_IN_HZ`, `DATA_FLRC_CHANNEL_STEP_865MHz_HZ` -> `DATA_FLRC_CHANNEL_STEP_SUBGIG_HZ`
  - Default sub-GHz WOR / data frequency moved from 865.1MHz to 865.7MHz
  - The FLRP core goes back to IDLE after the user callback is called (parameters are not reset anymore before the callback) or when the MAC layer can't be started
- LBM (lbm_lib): upgrade from v4.9.0 to v4.9.3, see [LoRa Basics Modem v4.9.3](#lora-basics-modem-v493) below
- Radio drivers & RAL
  - Upgrade LR20xx driver to version v2.1.0
    - FLRC: FEI in packet status, BT 0.3, double rate sampling workaround (`lr20xx_workarounds_flrc_configure_double_rate_sampling()`)
    - CCA with gain & CCA results with half dB resolution
    - LR-FHSS read/write hopping table
    - IRQ for LoRa intra packet hopping, missing IRQs added in the "all IRQ" mask
    - RTToF extended results: rssi2 removed; LoRa CR 0x00 removed
    - LoRa disable RX length check workaround
    - Z-Wave: set scan replaced by execute scan operation
    - FSK time on air fixed for 16-bit header
  - LR11xx driver v3.0.0 & SX126x driver are unchanged
  - RAL/RALF direct dispatch (CMake option `RAL_DRIVER_DIRECT_DISPATCH`, ON by default): the radio family driver is called directly instead of through function pointers (not available for udp_pf)
  - SX1272/SX1276 unified under a single `SX127X` define (examples, HAL & LBM)
  - smtc_ral: LR20XX RSSI calibration applied at RAL init, FLRC FEI in packet status, `IRQ_ADDR_ERROR` / `LR20XX_SYSTEM_IRQ_ERROR` / `LR20XX_SYSTEM_IRQ_CMD_ERROR` & missing LR20XX IRQ flags mapped in RAL, new LoRa LDRO computation for all radios, `ral_set_sleep()` re-enabled on Linux
  - RAC: FLRC double rate sampling now uses the LR20xx driver workaround instead of a direct register access
- hw_modem
  - Circular RX buffer management moved from `smtc_hal_l4` to hw_modem (HAL is now unaware of hw_modem)
  - hw_modem requires `LOW_POWER_MODE=0` (WFI) to keep the UART & DMA running during sleep
  - `CMD_FLRP_GET_STATS` returns `CMD_RC_BUSY` while a transaction is in progress
  - `CMD_FLRP_START_PERIODIC_LISTENING` (uint32 big-endian) and `CMD_FLRP_INITIATE_RECEPTION` (new protobuf `flrp_initiate_reception_pb_t`) now take the expected payload size (host scripts must be updated)
  - Python scripts adapted to `python_bridge.py`
- LINUX
  - Low power timer delivered through a `timerfd` thread instead of a POSIX signal (removes data races reported by ThreadSanitizer)
  - Main loop wake-up centralized in `hal_mcu_wake_main_thread()`
- Examples
  - LBM is enabled when compiling all examples
  - multiprotocol: periodical uplink delay set to 60s (instead of 600s)
  - flrp_api_example: TX statistics (bit rates, frequency & frequency offset) and RX frequency information printed
  - flrc_without_protocol_example moved to `unmaintained`
- Documentation: USP porting guide (LR20XX RSSI calibration tables), [SWL2001 (LBM) to USP porting guide](doc/usp_lbm_porting_guide.md) (new architecture, memory footprint, RAC hooks), LBM porting guide (core / modem HAL split), Linux HAL README, FLRP documentation

### Fixed

- FLRP
  - Channel index boundary check (`>=` number of frequencies); radio configuration now rejects `nb_channels == 0` and `default_channel >= nb_channels`
  - Integrity of the default channel checked before setting it
  - `memcpy()` size of the channel frequencies
  - Busy guards & IDLE state transition; `smtc_flrp_is_transaction_active()` was reporting busy during periodic listening
  - `smtc_wor_get_single_symbol_time_us()` never returns 0 (division by zero)
  - Payload length converted to `uint64_t` for the bit rate computation (overflow)
  - FLRC burst TX: radio IRQs are cleared before being processed (IRQ clear issue)
- LBM: security hardening of the downlink parsers, relay, regions, Class B/C multicast..., see [LoRa Basics Modem v4.9.3](#lora-basics-modem-v493) below
- Radio planner: lock tasks issue, `rp_get_immediate_radio_access()` checks that `irq_callback` is not NULL, duty-cycle band max value propagation, IRQ status cleared before setting the radio in sleep
- RAL/RALF: SX126X packet type set before the LoRa symbol timeout, SX128X packet payload read, udp_pf missing `check_lora_parameters` entry
- Examples: `SMTC_MODEM_EVENT_RESET_REQUEST` managed in lctt_certif_example
- LINUX
  - Main loop is woken up immediately on radio backends without GPIO IRQ thread (udp_pf: join/downlink processing was delayed by 10-15s)
  - GPIO context table writes protected by `gpio_irq_mutex`
  - LINUX target binary is not objcopied anymore (the delivered binary was not executable)
  - LIBM not found in cross-compilation; toolchain libraries linking with `LINK_LIBRARIES_ONLY_TARGETS`
- hw_modem: corrupted `CMD_FLRP_GET_STATS` values, compilation with LR1121 and with `-DCONFIG_USP_FLRP=y`, wrong FUOTA define, Linux PTY command framing
- One Channel Hub: packet forwarder & libloragw thread-safety (data races on `report_ready`, `gmtime()`, exit signals, gateway state)

### Bugfixing

- Coverity: integer overflow, division by zero, uninitialized variables, `PANIC_IF_STACK_ID_TOO_HIGH` scope reduced (LBM), FLRP API initiator added to the Coverity build
- LBM: AU915 / IN865 typos, AU915/LR-FHSS wrong index boundary check when decrementing DR, `fmp_test()` duplicated declaration, coding style, missing `smtc_modem_test_bw_m_t`, unused definitions removed, traces re-enabled
- FLRP: `FREQOFFSET_GRPX` defines cleaned, missing header, unit tests (ceedling `project.yml`) fixed
- CMake: SX126X flag propagated to `smtc_hal` & `radio_hal`, `radio_hal` was defining `SX1272` instead of `SX1276` for the sx1276 radio, geolocation only enabled for lr1110/lr1120 (not lr1121), `smtc_modem_hal_implem` linked to the RAL for udp_pf, flrc_without_protocol_example CMake issue
- hw_modem: unused FUOTA callbacks removed, python bridge PTY issues & raw TTY mode, circular buffer size, Windows compatibility
- One Channel Hub: `strncpy()` truncation warning, redundant `mx_concent` removed, confirmed uplink setting reverted
- Linux: aarch64 toolchain documentation does not require to source a script anymore
- Porting tests: typo in the starting message
- Code formatting (clang-format) & comments coding style harmonized; parson external dependency excluded from the clang-format check
- CI: Coverity reports pushed to Coverity Connect (failing on unclassified defects), artifacts kept one week for pipelines triggered on tags, validation pipeline trigger fix, recursive submodule strategy

### Deprecated

- `smtc_rac_lib/smtc_modem_hal` removed (replaced by `smtc_rac_lib/smtc_core_hal_interface`)
- hw_modem FLRC protocol commands 0xB0 to 0xB2 (`CMD_FLRC_PROTOCOL_INIT`, `CMD_SET_FLRC_PROTOCOL_PARAMS`, `CMD_GET_FLRC_PROTOCOL_STATS`) and associated protobuf structures removed
- `-DLBM_RADIO=` CMake option deprecated (use `-DRAC_RADIO=`)

### LoRa Basics Modem v4.9.3

USP v1.3.0 embeds LoRa Basics Modem v4.9.3 (`protocols/lbm_lib`).

`protocols/lbm_lib` is common to USP and SWL2001, its detailed changelogs are:
- [Modem API changelog](protocols/lbm_lib/smtc_modem_api/CHANGELOG.md)
- [Modem HAL changelog](protocols/lbm_lib/smtc_modem_hal/CHANGELOG.md)

#### Added

- Regional Parameters RP002-1.0.4 and RP002-1.0.5 support (CFList type 1 for dynamic plans, up to 80 channels on EU868, new US915 channel plan, Relay channel 1 SF update, AS923 group 4), selected with the CMake option `RP_VERSION` (`RP2_101`, `RP2_103`, `RP2_104`, `RP2_105`; default remains `RP2_103`)
- LR2022 added to the list of WW_2G4-compatible radios
- CMake: `LBM_CMAKE_CONFIG_AUTO` option
- Relay Tx: `number_of_miss_wor_ack_to_switch_in_nosync_mode` can now be configured by the user

#### Changed

- The radio planner and RALF are provided externally to LBM (RAC dependency removed), the LBM HAL is split in the LBM specific `smtc_modem_hal.h` and the core HAL `smtc_core_hal.h` (see the Modem HAL changelog), simplified CMake architecture, LBM specific compilation definitions are now PUBLIC
- Class B beacon DPLL: `BEACON_PLL_PHASE_GAIN_MUL/DIV` set to 1/2 for a faster convergence and accept a bigger crystal error
- Multicast Class B/C: the session now starts 2 s before its session time
- Duty-cycle frequency list now takes the 80 channels into account

#### Fixed

- NbTrans > 1: the duty-cycle time off is now updated on retransmission, all retransmissions are sent before the duty-cycle delay is enforced
- Wrong index boundary check when decrementing the datarate (AU915 / LR-FHSS)
- `lr1_stack_mac_cmd_ans_cut()` could block the modem because DeviceTimeAns and Class B MAC commands were missing in the array used for length check
- Class B/C Multicast v1/v2 ping slot hopping when frequency is 0
- Modem was not able to retain sleep when `lorawan_dwn_ack_management` variable was less than or equal to 96 bytes (context added)
- Missing check of `SMTC_MULTICAST` definition (Class B or C only)
- `smtc_modem_get_suspend_radio_communications()` can no longer be called in test mode
- Porting guide: `CONTEXT_SECURE_ELEMENT` needs 484 bytes
- Issue [#141](https://github.com/Lora-net/SWL2001/issues/141) Missing semicolons when `SMTC_TRACE` is compiled
- IN865, AU915 and AS923_GRP4 region typos/out-of-bound fixes
- MAC commands answer size table: `LINK_CHECK_ANS` replaced by `LINK_CHECK_REQ` (command sent by the end-device)
- SX127x (SX1272/SX1276) build and management

#### Security

Robustness fixes found by static analysis and code audit (out-of-bound access, unvalidated lengths and indexes):

- FOpts extraction read past the valid payload before MIC verification
- FUOTA: fragment payload size validated against the received RF payload; `FragDecoderProcess()` rejects `fragCounter == 0` (row index underflow); `frag_index` bounds checked before use; `frag_req_cmd_size` table initialisation; unicast DataFragment refused on multicast-only sessions; `fmp_test()` duplicated declaration
- Remote Multicast Setup: `MC_GROUP_STATUS_REQ` answer buffer bound check
- Multi Package Access: `MULTI_PACK_BUFFER_REQ` read past the payload
- Certification: `TX_PERIODICITY_CHANGE_REQ` and `EchoPlayReq` size validation
- `link_adr_parser()` size check performed before dereference
- Function pointers are checked (`SMTC_LBM_IS_VALID_FN_PTR`) before being called; `smtc_lbt_get_obj()` panics on out-of-bound instead of returning NULL
- Coverity findings: uninitialised variables, integer overflow, `MIN/MAX` macros brackets, `CRYPTO_MAXMESSAGE_SIZE` limited to 255 in the LR11xx crypto engine

## [v1.2.1] 2026-06-26

### Added

- FLRP protocol & example (for more details cf the example README.md & FLRP API README.md)
- More parameter checking in LoRa modulation of USP/RAC submit API
- LINUX ARM64 support (aarch64)
- SX126X support : PA tables, TCXO, cmake files
- hw_modem updated with FLRP feature
- Porting Tests SPI integrity tests (SPI endurance)

### Changed

- LR20XX Drivers updated to 2.0.2
- Default ALC SYNC & FUOTA versions set from 1 to 2
- More performance on LINUX support (SIGUSR2 wake up smtc_rac_run_engine())
- Documentation updates (FLRC, README)
- LoRa Plus LR20XX EVK PA Table updated

### Fixed

- Examples & associated documentation fixes (porting tests, full almanac update, tx_cw, geolocation, hw_modem, cad, direct_driver_access, per_flrc, per_fsk, per_lora, ranging_demo, rf_certification)
- Add calibrate after configuring tcxo on LR20XX
- Porting Tests is now OK with LR2021 & STM32L4 & xiao_nRF54L15
- Issues with SX126X support
- LRFHSS/AU : Fix index wrongly calculated when decrementing DR
- RTToF : configure the responder PA / TX power (auto-transmitted response was sent at default power, leading to a weak signal / RSSI at the initiator)

### Deprecated

- NA

## [v1.1.2] 2026-04-16

### Added

- LR2012 & LR2022 support
- New USP/RAC API
  - to get the current radio ID in post_callbacks (`smtc_rac_get_callback_radio_id()`)
  - to get the version of API
  - to get signal rssi result in smtc_rac_data_result_t

### Changed

- v2.0.2 LR20xx Drivers support (including patch ram level2) [CHANGELOG](/smtc_rac_lib/radio_drivers/lr20xx_driver/CHANGELOG.md)
- v3.0.0 LR11xx Drivers support (including security patches) [CHANGELOG](/smtc_rac_lib/radio_drivers/lr11xx_driver/CHANGELOG.md)
- CMake files for the examples have been reworked: each example has now its own CMake file instead of sharing a single one.
- SPI is more optimized for STM32L4
- geolocation example & Tools in a dedicated directory
- hw_modem updated with last features & fixes

### Fixed

- FLRC BURST fixes
- LINUX support fixes
- Examples & associated documentation fixes (porting tests, tx, cw, per_lora, per_fsk, spectral scan, rf_certification, cad, direct_driver_access, immediate_radio_access)
- CAD TO RX (CAD_LBT) fixed
- USP Linux SPI Speed fixed (16MHz instead of 20MHz)
- Comments in USP/RAC API

### Deprecated

- NA

## [v1.1.1] 2026-02-27

### Added

- FPB-RA0E2 board support (Renesas R7FA0E209 Cortex-M23)
  - Complete HAL implementation (GPIO, SPI, UART, RTC, Flash, RNG, Timer, Watchdog)
  - Software standby mode with 1.3µA sleep current
  - US915 region validated with LoRaWAN Class A
- Experimental FLRP Examples (flrc_burst) & Protocol API are still in progress.
- LINUX & LINUX_ARM board support
- New API to allow Immediate Access to the radio : `smtc_rac_immediate_radio_access()` / `smtc_rac_release_immediate_radio_access()` with immediat_access example
- Geolocation Tools : full_almanac_update & wifi_region_detection examples ported from LoRa Basics Modem
- New HAL GPIO/LED management API
- STM32L0: LED and BUTTON HAL support
- README added for all examples

### Changed

- Update LR20XX radio drivers
- Radio planner performances improvement
- Update STM32L4 HAL drivers (CMSIS Device L4 v1.7.2, Core v5.6.0_cm4)
- `symb_nb_timeout` type changed from `uint8_t` to `uint16_t` in RALF & USP/RAC
- Documentation updates (FLRC, LINUX, README)

### Fixed

- HF RX PATH starts from 1.5 GHz instead of 1.6 GHz

### Deprecated

- NA

## [v1.0.0] 2025-12-15

This version is based on branch v0.5.1-alpha of USP.

### Added

- CAD example
- geolocation example
- hw_modem example
- VSCode Integration helper
- New RAC API : smtc_rac_lock_radio_access()/smtc_rac_unlock_radio_access()

### Changed

- Upgrade of LBM to v4.9
- Documentation (getting started, API, LBM porting guide, MCU & Radio porting guide, samples)
- rf_certification example temporary removed
- Radio drivers updates:
  - LR20XX to version [v1.3.4](smtc_rac_lib/radio_drivers/lr20xx_driver/README.md)

### Fixed

- Workaround for bad standard deviation of RTToF results with fractional bandwidths
- PER FLRC crashes
- Fixes on all samples
- Issue [#3](https://github.com/Lora-net/usp/issues/3) CMake compilation issues & improvement
- Issue [#2](https://github.com/Lora-net/usp_zephyr/issues/2) modem_set_radio_ctx() was not called (required for wifi & gnss)

### Deprecated

- NA

## [v0.5.1] - 2025-10-15

### Added

- Initial version

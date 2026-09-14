# USP changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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

- FLRC BURST application removed (replaced with FLRP examples)

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

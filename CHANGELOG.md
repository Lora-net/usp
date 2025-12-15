# USP changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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

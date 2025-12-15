# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.3.4] - 2025-11-25

### Added

- [LoRa] Helper function `lr20xx_radio_convert_nb_symb_to_mant_exp`
- [FSK] Add value `LR20XX_RADIO_FSK_PULSE_SHAPE_GAUSSIAN_BT_2_0` in `lr20xx_radio_fsk_pulse_shape_t`
- [bpsk] Add pulse shape values:
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_GAUSSIAN_BT_2_0`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_4`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_3`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_5`
  - `LR20XX_RADIO_BPSK_MOD_PARAMS_SHAPE_FILTER_RRC_0_7`

### Removed

- [LoRa] Remove unsupported bandwidths 7, 10, 15, 20 kHz.
- [FLRC] Remove `LR20XX_RADIO_FLRC_PULSE_SHAPE_BT_07`
- Fix comments for derivative parts

### Changed

- [LoRa] `lr20xx_radio_lora_configure_timeout_by_number_of_symbols` takes parameter as `uint16_t` and propagate to `lr20xx_radio_lora_configure_timeout_by_mantissa_exponent_symbols` when appropriate
- [FSK] `lr20xx_radio_fsk_mod_params_t` has field `br` replaced by `bitrate` and `bitrate_unit` to clarify the bitrate unit
- [LR-FHSS] `lr20xx_radio_lr_fhss_build_frame` takes configuration from structure `lr20xx_radio_lr_fhss_params_t`
- [LR-FHSS] Structure `lr20xx_radio_lr_fhss_params_t` and corresponding types does not depend on LR-FHSS V1 base types
- [LR-FHSS] Hopping modes includes test modes `LR20XX_RADIO_LR_FHSS_HOPPING_TEST_PAYLOAD` and `LR20XX_RADIO_LR_FHSS_HOPPING_TEST_PA`

### Fixed

- [LoRa] Typo for `lr20xx_radio_lora_hopping_ctrl_t`
- [OOK] Fix available pulse shape values in `lr20xx_radio_ook_pulse_shape_t` which becomes an enumeration, remove `lr20xx_radio_ook_pulse_shape_filter_t` and `lr20xx_radio_ook_pulse_shape_bt_t`

## [v1.3.1] - 2025-10-15

### Added

- Initial version

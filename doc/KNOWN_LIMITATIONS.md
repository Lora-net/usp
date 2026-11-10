# Known Limitations

This document presents the current known limitations of USP and their workarounds, when available.

### This USP version includes FLRP

Known Limitations specific to FLRP are gathered in the [FLRP documentation](FLRP_guidelines.md#limitations).

### Not validated in this release

The following are **not validated / not fully tested** in this release:
- **geolocation**
- **rf_certification**
- **FPB-RA0E2 Renesas port**

### rf_certification example & FCC duty-cyle limit

FCC test application currently may not reach the required 98% channel duty-cycle limit. A fix is in progress.

### porting_tests application limitation

On STM32L476RG, 2 tests are not passing :
- With LR2021 : `porting_test_get_time/Get time in millisecond` : `NOK: Time is not coherent with radio irq : expected 1966ms / get 1968ms (margin +/-1ms)`
  - This issue is under investigation, but currently, It did not prevent to pass OK through the Semtech full Validation Process
- `porting_test_stop_timer` : the `hal_lp_timer_stop()` function is not functional, this is under fix for next release.

At least on `nucleo_l476rg`, `porting_test_get_time/Get time in millisecond` also fails with the **SX126x (SX1261)** and **LR11xx (LR1120)** shields: the measured time is not coherent with the radio IRQ (error is largest on SX1261). Under investigation (issue 267).


### Support of NUCLEO-L073RZ & Renesas FPB-RA0E2 is experimental

Not all samples compile with those platforms.
Only periodical_uplink sample was tested with limited validation.

### US915 TX power index offset via LinkAdrRequest on LR11xx/SX126x (#300)

Network servers can modify the transmit power of devices with the LinkAdrRequest command.
On US915, with LR11xx and SX126x radios, we observed a power offset: the first two power indices produce the same transmit power, which creates an offset throughout the test for all indices

### SX126x RSSI_READ shows dBm offset (#299)

Using the RSSI_READ test command on the SX126X radios, we observe an offset negative or positive depending on usp or usp zephyr.

### SX126x on usp receives LoRaWAN at ~20 dBm too low

On USP, the SX126x radios receive LoRaWAN messages at an RSSI roughly 20 dBm lower than they should. Not seen on usp zephyr.

### Some programmed packets could be dropped (seen in Relay RX) (#130)

During validation, it was discovered that some packets could be dropped under certain circumstances. When this occurs, the following message may appear:
> task schedule aborted because in the past -1

This issue was observed during validation of Relay RX with STM32L476RG & Wio-LR2021, but may also occur occasionally with other features and radios.
If this issue occurs, try extending the `RP_MARGIN_DELAY` value from `8` to `12` in the following file: `smtc_rac_lib/radio_planner/src/radio_planner_types.h`.

### Geolocation Tools are missing (#129)

The geolocation application from Legacy LoRa Basics Modem 3_geolocation_on_lora_edge Application suite was ported to USP.
Nevertheless, the following tools are not yet available for USP:
- lr11xx_flasher

If required, It can be retrieved from [LR11xx Updater tool](https://github.com/Lora-net/SWTL001).

### USP API: `smtc_rac_submit_radio_transaction()` with out-of-range frequency is accepted (#98)

When requested frequency is out of range, no error is returned and the software seems to run normally, but the requested frequency is not used. Instead, the previously set frequency is used.
In future releases, an error will be returned or the firmware will reset with panic for out-of-range frequency.

### USP API: `smtc_rac_submit_radio_transaction()` with LR20xx & BW 7, 10, 15, 20 causes Division by zero (#94)

When using the LR20xx radio with LoRa modulation and BW 7, 10, 15, 20, the software crashes with a Division by zero exception.
In future releases, an error will be returned or the firmware will reset with panic for out-of-range BW.

### SX127X & SX128X radio shields

The support of those boards is
- not available on usp_zephyr
- limited to LoRa Basics Modem on usp

### OnChannelHub & SX126X radio shields (#385)

The sensitivity of the device side of the one_channel_hub in US915 or AU915 with an SX126x may be degraded by about 20 dB when RX bandwidth is updated without sleep.
Observed with Rx BW125 -> standby -> Rx BW500 .


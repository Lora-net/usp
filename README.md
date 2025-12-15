# Unified Software Platform (USP)

> **USP RELEASE v1.0.0**

## Overview

USP provides an abstraction layer for scheduling and managing multiple radio access across available modulations (LoRa, FSK, LR-FHSS, FLRC) and protocols (LoRaWAN). The library enables applications to request radio access, configure transmissions/receptions, and schedule operations with priority management.

Current Version is v1.0.0:
- [Changelog](CHANGELOG.md)
- [known limitations](doc/KNOWN_LIMITATIONS.md)

### Supported Software & Hardware

The USP repository includes LoRa Basics Modem 4.9.0.

The supported Semtech radios are:
- Validated[<sup>1</sup>](#notes) on [LoRa Plus EVK(LoRa Plus Expansion Board + Wio-LR2021 radio)](https://www.semtech.com/products/wireless-rf/lora-plus/lr2021)[<sup>2</sup>](#notes)
- buildable[<sup>1</sup>](#notes) on [LR11xx shield radios](https://www.semtech.com/products/wireless-rf/lora-connect/lr1121)
- buildable[<sup>1</sup>](#notes) on [SX126x shield radios](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262)

The supported MCU boards are:
- Validated[<sup>1</sup>](#notes) on STMicro NUCLEO-STM32L476RG
- might work[<sup>1</sup>](#notes) on STMicro NUCLEO-STM32L073RZ

#### Notes
> **<sup>1</sup>** `Validated` : passed the Semtech nominal validation process, `Buildable` : can be compiled but did not go through full Semtech validation process and can be considered experimental, `might work` : was compiled and tested on `periodical_pulink` sample only with low validation
>
> **<sup>2</sup>WIO-LR2021 CN version** ⚠️
> For WIO-LR2021 China (CN) versions the PA table configuration shall be adjusted as defined in LR2021 Datasheet page 134 to (CN - 490Mhz) band for optimal performances. Refer to [USP Porting Guide](doc/usp_porting_guide.md) for more details.

### Software Components

| Component | Description | Documentation |
|-----------|-------------|---------------|
| **USP/RAC Library** | Radio Access Component (RAC) API for Semtech transceiver management, including also RAL & Semtec Radio Drivers | **[View Full API Documentation →](smtc_rac_lib/README.md)** |
| **LoRa Basics Modem** | Integrated LoRaWAN stack (v4.9.0) | **[LBM User Guide →](protocols/lbm_lib/README.md)** |
| **Semtec Radio Drivers** | Legacy Drivers for supported Semtec Radios | **[Semtec Radio Drivers](smtc_rac_lib/radio_drivers)**
| **Examples Core** | Sample applications demonstrating RAC API usage that can be compiled for baremetal and are also used by USP Zephyr | **[USP / USP Zephyr Samples Common Guide →](https://github.com/Lora-net/usp_zephyr/blob/main/samples/usp)** |

The architecture is described with more detailed in the [USP Zephyr repository](https://github.com/Lora-net/usp_zephyr/blob/main/doc/USP_Architecture.md)

### Key Features

- **Priority-based scheduling** - Manage radio access with configurable priorities
- **Multi-modulation support** - LoRa, FSK, LR-FHSS, and FLRC modulations
- **LoRa capabilities** - Full support for transmission, reception, and ranging
- **Precise timing** - Schedule radio operations with accurate timing control
- **Asynchronous operations** - Callback support for non-blocking execution
- **Seamless integration** - Built on Semtech's radio planner

## Getting Started

### Recommended Development Software

The USP software was tested with:
- gcc 13.3 or higher
- CMake 3.28 or higher
- OpenOCD 0.12 or higher
- Ninja build tool 1.11 or higher

### Available Applications

The Samples documentation is available here : **[USP / USP Zephyr Samples Common Guide →](https://github.com/Lora-net/usp_zephyr/blob/main/samples/usp)**.<br>
The following applications are available in the [examples/](examples/)` directory:

#### Ranging (RTToF)
- **`rttof_manager`**: RTToF ranging manager device
- **`rttof_subordinate`**: RTToF ranging subordinate device

#### Communication Examples
- **`ping_pong`**: Ping-pong communication example
- **`periodical_uplink`**: Periodical uplink transmission example
- **`multiprotocol`**: Multiprotocol example (LoRa + Ranging)

#### Packet Error Rate (PER) Tests
- **`per_tx`**: LoRa packet error rate - transmitter
- **`per_rx`**: LoRa packet error rate - receiver
- **`per_fsk_tx`**: FSK packet error rate - transmitter
- **`per_fsk_rx`**: FSK packet error rate - receiver
- **`per_flrc_tx`**: FLRC packet error rate - transmitter
- **`per_flrc_rx`**: FLRC packet error rate - receiver

#### Modulation Examples
- **`lrfhss_tx`**: LR-FHSS transmission example

#### Certification & Testing
- **`lctt_certif`**: LCTT certification example

#### Advanced Examples
- **`spectral_scan`**: Spectral scan analysis example
- **`tx_cw`**: Continuous wave transmission example
- **`direct_driver_access`**: Direct radio driver access example (Use RAL or Drivers API instead of USP/RAC API to manage radio, and fine-tune radio sleeping operations)
- **`geolocation`**: Manage geolocation of LR1110 & LR1120 radio family
- **`hw_modem`**: Drive USP based MCU through UART (only LBM is currently stable)

### Build Basics

Compilation is done through the cmake command line.

#### Targets

In [CMakeLists.txt](examples/CMakeLists.txt) 3 main targets are available :
- `example` : will compile a example binary based on `-DAPP=XXX` XXX being the upper case of one of the "Available Applications" above,
- the `xxx` example name as defined below in "Available Applications" chapter and in [CMakeLists.txt](examples/CMakeLists.txt) (for example `per_fsk_tx` in the below list),
- `all_examples` : to compile all "Available Applications" example targets defined in [CMakeLists.txt](examples/CMakeLists.txt)

Have a look on cmake trace to get the list of "Available Applications" targets
```
-- Available examples:
--   - hw_modem         : Hardware modem
--   - rttof_manager    : ranging (RTToF) manager
--   - rttof_subordinate: ranging (RTToF) subordinate
--   - ping_pong        : ping-pong communication example
--   - per_tx           : packet error rate - LoRa (transmitter)
--   - per_rx           : packet error rate - LoRa (receiver)
--   - per_fsk_tx       : packet error rate - FSK (transmitter)
--   - per_fsk_rx       : packet error rate - FSK (receiver)
--   - per_flrc_tx      : packet error rate - FLRC (transmitter)
--   - per_flrc_rx      : packet error rate - FLRC (receiver)
--   - lrfhss_tx        : LR-FHSS transmission example
--   - spectral_scan    : spectral scan analysis example
--   - periodical_uplink      : Periodical uplink transmission example
--   - multiprotocol          : Multiprotocol example
--   - porting_tests          : Porting_tests example
--   - lctt_certif            : LCTT certification example
--   - geolocation            : geolocation example
--   - tx_cw            : continuous transmission (TX CW) example
--   - direct_driver_access : direct radio driver access example
--   - cad              : CAD (Channel Activity Detection) example
```

For example to compile `periodical_uplink` both `example` & `periodical_uplink` targets can be used :
```
rm -Rf build/ ; cmake -L -S examples  -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target periodical_uplink
```
or
```
rm -Rf build/ ; cmake -L -S examples  -B build -DAPP=PERIODICAL_UPLINK -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target example
```

To compile all predefined targets with predefined cmake symbols :
```
rm -Rf build/ ; cmake -L -S examples  -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target all_examples
```

One more example with rttof example:
```
rm -Rf build/ ; cmake -L -S examples  -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -DRANGING_DEVICE_MODE=SUBORDINATE -DCONTINUOUS_RANGING=false -G Ninja; cmake --build build --target rttof_subordinate
```

geolocation & hw_modem are not included in `all_examples` target. They shall be compiled whatever the `example`, `hw_modem`, or `geolocation` by specifying the `-DAPP=xxx` cmake symbol in order to force the GEOLOCATION cmake symbols by default :
- geolocation for lr1120:

``` bash
rm -Rf build/ ; cmake -L -S examples  -B build -DAPP=GEOLOCATION -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr1120 -G Ninja; cmake --build build --target geolocation
```

- hw_modem for lr2021 (no geolocation)
```
rm -Rf build/ ; cmake -L -S examples  -B build -DAPP=HW_MODEM -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -DLBM_GEOLOCATION=OFF -G Ninja; cmake --build build --target hw_modem
```

selected cmake symbols can defer depending on command line, have a look on the following chapter.

#### cmake compilation symbols & C compilation definitions

Management of compilation symbols
- When cmake symbols are available (often activating compiler definitions), use them directly in cmake configuration command line with `-D` option (e.g. -DBOARD=NUCLEO_L476) :
    - cmake symbols are described in example documentation,
    - for advanced users, some cmake symbols are defined in cmake sub components like [examples/CMakeLists.txt](examples/CMakeLists.txt), [smtc_rac_lib/CMakeLists.txt](smtc_rac_lib/CMakeLists.txt), [protocols/lbm_lib/CMakeLists.txt](protocols/lbm_lib/CMakeLists.txt), [protocols/lbm_lib/options.cmake](protocols/lbm_lib/options.cmake), [protocols/lbm_lib/smtc_modem_core/CMakeLists.txt](protocols/lbm_lib/smtc_modem_core/CMakeLists.txt)
- Some important compilation defines are not yet available through cmake symbols. In this case, with care, they can be update in cmake command line by using the `CFLAGS` & `-UCMAKE_C_FLAGS`. For example, for LoRa Basics Modem examples, you can use the following command to pass LoRaWAN keys & regions in cmake command lines:
```
env CFLAGS="-DMODEM_EXAMPLE_REGION=SMTC_MODEM_REGION_WW_2G4 -DUSER_LORAWAN_DEVICE_EUI='{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}' -DUSER_LORAWAN_JOIN_EUI='{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}' -DUSER_LORAWAN_APP_KEY='{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}'" cmake -L -S examples  -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr1120 -UCMAKE_C_FLAGS -G Ninja ; cmake --build build --target periodical_uplink
```
`-DAPP=xxx`use shall be checked on [examples/CMakeLists.txt](examples/CMakeLists.txt) :
- specifying `-DAPP=xxx` for `HW_MODEM` & `GEOLOCATION` will activate most LBM features configuration
- specifying `-DAPP=xxx` for `periodical_uplink`, `lctt_certif`, `multiprotocol` will activate minimal LBM features configuration
- NOT specifying `-DAPP=xxx` for `periodical_uplink`, `lctt_certif`, `multiprotocol` will activate most LBM features configuration

Have a look on traces when compiling to understand which cmake symbols are activated or not :
```
APP_MODE:STRING=APP_MODE_CERTIFICATION
CCACHE_PROGRAM:FILEPATH=/usr/bin/ccache
CMAKE_BUILD_TYPE:STRING=MinSizeRel
CMAKE_INSTALL_PREFIX:PATH=/usr/local
CMAKE_TOOLCHAIN_FILE:FILEPATH=xxx/examples/smtc_hal_l4/cmake_stm32l4_toolchain.cmake
INFINITE_PREAMBLE:BOOL=OFF
LBM_ALC_SYNC:BOOL=ON
LBM_ALC_SYNC_VERSION:STRING=1
LBM_ALMANAC:BOOL=OFF
LBM_BEACON_TX:BOOL=OFF
LBM_CLASS_B:BOOL=ON
LBM_CLASS_C:BOOL=ON
LBM_CMAKE_CONFIG_AUTO:BOOL=ON
LBM_CRYPTO:STRING=SOFT
LBM_CSMA:BOOL=ON
LBM_CSMA_BY_DEFAULT:BOOL=OFF
LBM_DEVICE_MANAGEMENT:BOOL=ON
LBM_FUOTA:BOOL=ON
LBM_FUOTA_FMP:BOOL=ON
LBM_FUOTA_FRAGMENTS_MAX_NUM:STRING=
LBM_FUOTA_FRAGMENTS_MAX_REDUNDANCY:STRING=
LBM_FUOTA_FRAGMENTS_MAX_SIZE:STRING=
LBM_FUOTA_MPA:BOOL=ON
LBM_FUOTA_VERSION:STRING=1
LBM_GEOLOCATION:BOOL=OFF
LBM_LFU:BOOL=ON
LBM_MODEM_TRACE:BOOL=ON
LBM_MODEM_TRACE_DEEP:BOOL=OFF
LBM_MULTICAST:BOOL=ON
LBM_NUMBER_OF_STACKS:STRING=1
LBM_PERF_TEST:BOOL=OFF
LBM_RADIO:STRING=lr2021
LBM_REGIONS:STRING=ALL
LBM_RELAY_RX:BOOL=ON
LBM_RELAY_TX:BOOL=ON
LBM_STORE_AND_FORWARD:BOOL=ON
LBM_STREAM:BOOL=ON
LBM_TEST_BYPASS_JOIN_DUTY_CYCLE:BOOL=OFF
LEGACY_EVK_LR20XX:BOOL=OFF
NOTIFICATION_MODE:STRING=NOTIFICATIONS_OFF
RAC_CORE_LOG_API_ENABLE:BOOL=OFF
RAC_CORE_LOG_CONFIG_ENABLE:BOOL=ON
RAC_CORE_LOG_DEBUG_ENABLE:BOOL=OFF
RAC_CORE_LOG_ERROR_ENABLE:BOOL=ON
RAC_CORE_LOG_INFO_ENABLE:BOOL=OFF
RAC_CORE_LOG_RADIO_ENABLE:BOOL=OFF
RAC_CORE_LOG_WARN_ENABLE:BOOL=OFF
RAC_FSK_LOG_ENABLE:BOOL=OFF
RAC_LIB_LOG_PROFILE:STRING=DEFAULT
RAC_LOG_ENABLE:BOOL=ON
RAC_LOG_PROFILE:STRING=DEFAULT
RAC_LORA_LOG_CONFIG_ENABLE:BOOL=ON
RAC_LORA_LOG_DEBUG_ENABLE:BOOL=OFF
RAC_LORA_LOG_ENABLE:BOOL=OFF
RAC_LORA_LOG_ERROR_ENABLE:BOOL=ON
RAC_LORA_LOG_INFO_ENABLE:BOOL=OFF
RAC_LORA_LOG_RX_ENABLE:BOOL=ON
RAC_LORA_LOG_TX_ENABLE:BOOL=ON
RAC_LORA_LOG_WARN_ENABLE:BOOL=OFF
RAC_LRFHSS_LOG_ENABLE:BOOL=OFF
RAC_RADIO:STRING=lr2021
RP_MARGIN_DELAY:STRING=8
RP_VERSION:STRING=RP2_103
TYPE_OF_CAD:STRING=CAD_ONLY
```

### Build Examples on NUCLEO-STM32L476RG

The `-DBOARD=NUCLEO_L476` cmake symbol shall be selected :
```bash
rm -Rf build/ ; cmake -L -S examples  -B build -DAPP=PERIODICAL_UPLINK -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target periodical_uplink
```

Options
- **`RAC_RADIO`**: Target radio (`sx1261`, `sx1262`, `sx1268`, `lr1110`, `lr1120`, `lr1121`, `lr2021`)
- **`BOARD`**: Target MCU : NUCLEO_L476
- Other options are related to examples

Example of `openocd`command to flash:
```
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial 0671FF495648807567102644" -c "program build/periodical_uplink verify reset exit"
```

### Build & flash periodical_uplink on NUCLEO-L073RZ

Notes:
- Only periodical_uplink was tested with low validation.
- The `-DBOARD=NUCLEO_L073` cmake symbol shall be selected.
- Store & Forward feature shall be deactivated.

Build Periodical uplink:
```
rm -Rf build/ ; cmake -L -S examples  -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L073 -DRAC_RADIO=lr2021 -DLBM_STORE_AND_FORWARD=OFF -G Ninja; cmake --build build --target periodical_uplink
```

Build LCTT Certif

``` bash
rm -Rf build/ ; cmake -L -S examples  -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L073 -DRAC_RADIO=lr2021 -DLBM_STORE_AND_FORWARD=OFF -G Ninja; cmake --build build --target lctt_certif
```

Example of `openocd`command to flash:
```
openocd -f interface/stlink.cfg -f target/stm32l0_dual_bank.cfg -c "adapter serial 066DFF515055657867152019" -c "adapter speed 500" -c "reset_config srst_only connect_assert_srst" -c "init" -c "program build/periodical_uplink verify reset exit"
```

Note : Not all examples are compiling on NUCLEO-L073RZ. Only periodical_uplink was tested.

## Samples

More details and how to build & use Samples are available on [USP Zephyr Sample Documentation](https://github.com/Lora-net/usp_zephyr/blob/main/samples/usp)

## Porting Guide

This [chapter](doc/usp_porting_guide.md) explains how to port USP
- on other MCU
- on other radio PCB

This [chapter](doc/usp_lbm_porting_guide.md) explains how to port existing LoRa Basics Modem application to USP

## Troubleshooting

Below is a non-exhaustive list of errors that can cause panics when using the RAC API or LoRa Basics Modem (LBM).
A panic will trig when the modem software is in an invalid state. Most of the time when the modem is in an invalid or unsupported combination of settings in `smtc_rac_context_t` or in LBM configuration.
The printed message will use this format:
```
Modem panic: function():line_number end of message
```
To debug, you can search the file where the function is defined, and open it at the line number.
If not sufficient to understand the issue, a debugger can be used to find out the sequence of calls and branching that led to the error.

Main RAC API Panics are:

### `ERROR: Modem panic: rp_hook_init:<line number>`
This error occurs when invoking `smtc_rac_open_radio(priority)` a second time with the same priority.
It comes from the file `smtc_rac_lib/radio_planner/src/radio_planner.c`, in the function `rp_hook_init`.
To fix it, please make sure that no two calls to `smtc_rac_open_radio` have the same priority.

### `ERROR: Modem panic: radio_access_id is out of range`
This error occurs when using an invalid `radio_access_id` as a parameter in API functions requiring it.
To fix it, ensure that you use an ID returned by `smtc_rac_open_radio()` and that no `smtc_rac_close_radio()` were called with it.

### `ERROR: Modem panic: smtc_rac_submit_radio_transaction:<line number>`
This error occurs when one field member in `smtc_rac_context_t` associated with the radio ID has been filled incorrectly, usually the size of the RX buffer.
It comes from the file `smtc_rac_lib/smtc_rac/smtc_rac.c`, in the function `smtc_rac_submit_radio_transaction`.
To fix it, ensure that `size_of_rx_payload_buffer` is greater or equal to `max_rx_size` of the selected modulation.
For example, in LoRa, ensure `ctx->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer >= ctx->radio_params.lora.max_rx_size`.

### `HARDFAULT_Handler`
This error usually occurs when invoking a NULL callback.
It might comes from the file `smtc_rac_lib/smtc_rac/smtc_rac.c`, in the function `smtc_rac_rp_callback`.
To fix it, ensure that `ctx->scheduler_config.callback_post_radio_transaction != NULL`.

## License

Clear BSD License.
Copyright Semtech Corporation.

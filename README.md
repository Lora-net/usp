# Unified Software Platform (USP)

> **USP RELEASE v1.2.1' - Stable Release**
>
## Overview

USP provides an abstraction layer for scheduling and managing multiple radio access across available modulations (LoRa, FSK, LR-FHSS, FLRC) and protocols (LoRaWAN). The library enables applications to request radio access, configure transmissions/receptions, and schedule operations with priority management.

Current Version is v1.2.1:
- [Changelog](CHANGELOG.md)
- [known limitations](doc/KNOWN_LIMITATIONS.md)

<table width="100%">
<tr>
<td>

<h3>&#10024; What's new in this release</h3>

<a href="doc/FLRP.md"><img alt="New feature: FLRP" src="doc/assets/badge_flrp.svg"></a>
&nbsp;<b><a href="doc/FLRP.md">FLRP &mdash; Fast LoRa communication Protocol</a></b><br>
Principles, API and examples for the high-speed LoRa&nbsp;+&nbsp;FLRC protocol.

<br>

<a href="doc/LORA_PLUS_EVK.md"><img alt="New hardware: LoRa Plus EVK" src="doc/assets/badge_evk.svg"></a>
&nbsp;<b><a href="doc/LORA_PLUS_EVK.md">LoRa Plus&trade; Evaluation Kit</a></b><br>
New EVK based on LR2021 / LR2022 / LR2012 (+ XIAO nRF54L15).

<br>

<a href="doc/VALIDATION_AND_PERFORMANCES.md"><img alt="Validation and Performances" src="doc/assets/badge_validation.svg"></a>
&nbsp;<b><a href="doc/VALIDATION_AND_PERFORMANCES.md">Validation &amp; Performances</a></b><br>
Validation configuration, build options and SPI clock speeds (memory footprint coming next).

</td>
</tr>
</table>

### Supported Software & Hardware

The USP repository includes LoRa Basics Modem 4.9.0.

The supported Semtech radios are:
- Validated[<sup>1</sup>](#notes) on [LoRa Plus EVK(LoRa Plus Expansion Board + Wio-LR2021/Wio-LR2022/Wio-LR2012 radios)](https://www.semtech.com/products/wireless-rf/lora-plus/lr2021)[<sup>2</sup>](#notes)
- buildable[<sup>1</sup>](#notes) on [LR11xx shield radios](https://www.semtech.com/products/wireless-rf/lora-connect/lr1121)
- buildable[<sup>1</sup>](#notes) on [SX126x shield radios](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262)

The supported platforms are:
- Validated[<sup>1</sup>](#notes) on STMicro NUCLEO-STM32L476RG
- buildable[<sup>1</sup>](#notes) on Linux (x86/x86_64 native + ARM cross-compilation for Raspberry Pi, embedded Linux).
  - For documentation, see the "Build Examples on ARM Linux" in the following chapters below
  - if required, check also the [Linux Porting Documentation](examples/smtc_hal_linux/README.md) )
  - Linux porting was only tested with **LR2021**. The radio_hal for other radios shall be implemented before use.
- Experimental[<sup>1</sup>](#notes) on Renesas FPB-RA0E2 (R7FA0E209, see [FPB-RA0E2 Porting Documentation](examples/smtc_hal_ra0e2/README.md))
  - only tested with LoRa Plus EVK (LR2021)
  - only tested with porting_tests & periodical_uplink applications (CLASS A, US915 region)
- Experimental[<sup>1</sup>](#notes) on STMicro NUCLEO-STM32L073RZ

#### Notes
> **<sup>1</sup>** `Validated` : passed the Semtech nominal validation process, `Buildable` : can be compiled but did not go through full Semtech validation process, `Experimental` : was compiled and tested on `periodical_uplink` sample only with low validation
>
> **<sup>2</sup>WIO-LR20xx CN version** ⚠️
> For WIO-LR20xx China (CN) versions the PA table configuration shall be adjusted as defined in the datasheet. For example, in LR2021 Datasheet page 134 to (CN - 490Mhz) band for optimal performances. Refer to [USP Porting Guide](doc/usp_porting_guide.md) for more details.

### Software Components

The USP architecture, its main SW components (RAC, protocols, MCU/Radio HAL & BSP), the RAC API and the dynamic behaviour & priorities are described in the **[USP Architecture documentation →](doc/USP_Architecture.md)**.

| Component | Description | Documentation |
|-----------|-------------|---------------|
| **USP/RAC Library** | Radio Access Component (RAC) API for Semtech transceiver management, including also RAL & Semtech Radio Drivers | **[View Full API Documentation →](smtc_rac_lib/README.md)** |
| **LoRa Basics Modem** | Integrated LoRaWAN stack (v4.9.0) | **[LBM User Guide →](protocols/lbm_lib/README.md)** |
| **FLRP (Fast LoRa communication Protocol)** | High-speed LoRa + FLRC protocol (up to 2.6 Mbps) with LoRa WOR time/frequency synchronization | **[FLRP Documentation →](doc/FLRP.md)** &middot; **[flrp_api Example →](examples/main_examples/flrp_api_example/README.md)** |
| **Semtech Radio Drivers** | Legacy Drivers for supported Semtech Radios | **[Semtech Radio Drivers](smtc_rac_lib/radio_drivers)**
| **Examples Core** | Sample applications demonstrating RAC API usage that can be compiled for baremetal | **[USP Samples Guide →](examples/main_examples)** |

### Key Features

- **Priority-based scheduling** - Manage radio access with configurable priorities
- **Multi-modulation support** - LoRa, FSK, LR-FHSS, and FLRC modulations
- **LoRa capabilities** - Full support for transmission, reception, and ranging
- **Precise timing** - Schedule radio operations with accurate timing control
- **Asynchronous operations** - Callback support for non-blocking execution
- **Seamless integration** - Built on Semtech's radio planner

<details>
<summary><h2>Getting Started</h2></summary>

### Recommended Development Software

The USP software was tested with:
- gcc 13.3 or higher
- CMake 3.28 or higher
- OpenOCD 0.12 or higher
- Ninja build tool 1.11 or higher

### Available Applications

The Samples & documentation &re available here : **[USP Samples Guide →](examples/main_examples)**.<br>

#### FLRP (Fast LoRa communication Protocol)

- **`flrp_api_initiator`**: FLRP API peer-to-peer high-speed transfer (initiator role)
- **`flrp_api_slave`**: FLRP API peer-to-peer high-speed transfer (slave role)

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

- **`rf_certification_etsi`**: RF certification for ETSI region
- **`rf_certification_arib`**: RF certification for ARIB region
- **`rf_certification_fcc`**: RF certification for FCC region
- **`lctt_certif`**: LCTT certification example
- **`porting_tests`**: porting test example


#### Advanced Examples

- **`spectral_scan`**: Spectral scan analysis example
- **`tx_cw`**: Continuous wave transmission example
- **`direct_driver_access`**: Direct radio driver access example (Use RAL or Drivers API instead of USP/RAC API to manage radio, and fine-tune radio sleeping operations)
- **`immediate_radio_access`**: Immediate radio access example (Use USP/RAC API to manage radio)
- **`geolocation`**: Manage geolocation of LR1110 & LR1120 radio family
- **`full_almanac_update`**: Manage almanac update of LR1110 & LR1120 radio family
- **`wifi_region_detect`**: Manage wifi region detection for LR1110 & LR1120 radio family
- **`hw_modem`**: Drive USP based MCU through UART (only LBM is currently stable)
- **`cad`**: Channel Activity Detection example


### Build Basics

Compilation is done through the cmake command line.

#### Build Structure

Each example has its own `CMakeLists.txt` in its directory under `examples/main_examples/`. You can either:
- Build a **single example** by pointing cmake to its directory
- Build **all examples** by pointing cmake to `examples/main_examples`
- Build a **single example** by pointing cmake to `examples/main_examples` and using `--target <example>`

#### Building a Single Example

To build a specific example, point cmake to its directory:

```bash
rm -Rf build/
cmake -S examples/main_examples/periodical_uplink_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr2021 \
    -G Ninja
cmake --build build
```

#### Building All Examples

To build all examples at once, point cmake to `examples/main_examples`:

```bash
rm -Rf build/
cmake -S examples/main_examples -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr2021 \
    -G Ninja
cmake --build build --target all_examples
```

You can also build a specific example from the all_examples configuration:

```bash
cmake --build build --target periodical_uplink
```

#### Available Examples

When pointing to `examples/main_examples`, the cmake configuration will display available examples:

```
-- Available examples:
--   - flrp_api_initiator     : FLRP API (initiator)
--   - flrp_api_slave         : FLRP API (slave)
--   - flrc_burst_tx          : FLRC burst data transfer (transmitter)
--   - flrc_burst_rx          : FLRC burst data transfer (receiver)
--   - full_almanac_update    : Full almanac update (LR11XX only)
--   - geolocation            : Geolocation example (LR11XX only)
--   - cad                    : Channel Activity Detection (LR20XX only)
--   - direct_driver_access   : Direct radio driver access (LR20XX only)
--   - immediate_radio_access : Immediate radio access (LR20XX only)
--   - hw_modem               : Hardware modem with serial interface
--   - lctt_certif            : LCTT LoRaWAN certification example
--   - lrfhss_tx              : LR-FHSS transmission example
--   - multiprotocol          : Multiprotocol (LoRaWAN + ranging) example
--   - per_tx                 : Packet error rate - LoRa (transmitter)
--   - per_rx                 : Packet error rate - LoRa (receiver)
--   - per_flrc_tx            : Packet error rate - FLRC (transmitter)
--   - per_flrc_rx            : Packet error rate - FLRC (receiver)
--   - per_fsk_tx             : Packet error rate - FSK (transmitter)
--   - per_fsk_rx             : Packet error rate - FSK (receiver)
--   - periodical_uplink      : Periodical LoRaWAN uplink example
--   - ping_pong              : Ping-pong communication example
--   - porting_tests          : HAL porting verification tests
--   - radio_planner_test     : Radio Planner stress test
--   - rttof_manager          : Ranging (RTToF) manager
--   - rttof_subordinate      : Ranging (RTToF) subordinate
--   - rf_certification_etsi  : RF certification (ETSI region)
--   - rf_certification_arib  : RF certification (ARIB region)
--   - rf_certification_fcc   : RF certification (FCC region)
--   - spectral_scan          : Spectral scan analysis example
--   - tx_cw                  : Continuous wave (TX CW) transmission
--   - wifi_region_detection  : WiFi region detection (LR11XX only)
```

#### Radio-Specific Examples

Some examples have radio-specific requirements:
- **geolocation**: Only available for LR11XX radios (lr1110, lr1120, lr1121). Automatically skipped for other radios.
- **hw_modem**: Supports all radios. Geolocation features are automatically enabled only for LR11XX radios.

Example building geolocation for lr1120:

```bash
rm -Rf build/
cmake -S examples/main_examples/geolocation/geoloc_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr1120 \
    -G Ninja
cmake --build build
```

Example building hw_modem for lr2021 (automatically without geolocation):

```bash
rm -Rf build/
cmake -S examples/main_examples/hw_modem -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr2021 \
    -G Ninja
cmake --build build
```

#### Passing Custom C Flags

For RTToF example with custom flags:

```bash
rm -Rf build/
env CFLAGS="-DCONTINUOUS_RANGING=false" \
cmake -S examples/main_examples/ranging_demo -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr2021 \
    -UCMAKE_C_FLAGS \
    -G Ninja
cmake --build build --target rttof_subordinate rttof_manager
```

#### cmake compilation symbols & C compilation definitions

Management of compilation symbols
- When cmake symbols are available (often activating compiler definitions), use them directly in cmake configuration command line with `-D` option (e.g. -DBOARD=NUCLEO_L476) :
    - cmake symbols are described in example documentation,
    - for advanced users, some cmake symbols are defined in cmake sub components like [examples/common.cmake](examples/common.cmake), [smtc_rac_lib/CMakeLists.txt](smtc_rac_lib/CMakeLists.txt), [protocols/lbm_lib/CMakeLists.txt](protocols/lbm_lib/CMakeLists.txt), [protocols/lbm_lib/options.cmake](protocols/lbm_lib/options.cmake), [protocols/lbm_lib/smtc_modem_core/CMakeLists.txt](protocols/lbm_lib/smtc_modem_core/CMakeLists.txt)
- Some important compilation defines are not yet available through cmake symbols. In this case, with care, they can be updated in cmake command line by using the `CFLAGS` & `-UCMAKE_C_FLAGS`. For example, for LoRa Basics Modem examples, you can use the following command to pass LoRaWAN keys & regions in cmake command lines:

```bash
env CFLAGS="-DMODEM_EXAMPLE_REGION=SMTC_MODEM_REGION_WW_2G4 \
    -DUSER_LORAWAN_DEVICE_EUI='{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}' \
    -DUSER_LORAWAN_JOIN_EUI='{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}' \
    -DUSER_LORAWAN_APP_KEY='{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}'" \
cmake -S examples/main_examples/periodical_uplink_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr1120 \
    -UCMAKE_C_FLAGS \
    -G Ninja
cmake --build build
```

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
rm -Rf build/
cmake -L -S examples/main_examples/periodical_uplink_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L476 \
    -DRAC_RADIO=lr2021 \
    -G Ninja
cmake --build build
```

Options

- **`RAC_RADIO`**: Target radio (`sx1261`, `sx1262`, `sx1268`, `lr1110`, `lr1120`, `lr1121`, `lr2021`, `udp_pf`)
- **`BOARD`**: Target platform: `NUCLEO_L476`, `NUCLEO_L073`, `FPB_RA0E2`, `LINUX`, or `LINUX_ARM`
- Other options are related to examples

Example of `openocd`command to flash:
```
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <serial_number>" -c "program build/periodical_uplink verify reset exit"
```

---

### Build Examples on ARM Linux (Raspberry Pi) with LR2021 Radio

For deployment on ARM Linux devices with physical LR2021 radio:

```bash
# Cross-compile for ARM with LR2021 radio
rm -Rf build/
cmake -S examples/main_examples/periodical_uplink_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=LINUX_ARM \
    -DRAC_RADIO=lr2021 \
    -G Ninja
cmake --build build

# Transfer to target device
scp build/periodical_uplink pi@raspberrypi.local:~/
```

**Prerequisites:**
- SPI enabled: `/dev/spidev0.0`
- GPIO access: `/dev/gpiochip0`
- User in `spi` and `gpio` groups

**For detailed Linux HAL implementation and hardware setup, see** → [Linux HAL Documentation](examples/smtc_hal_linux/README.md)

---

### Build Examples on Native Linux with Virtual Radio (UDP_PF)

The virtual radio (`udp_pf`) implements the Semtech UDP Packet Forwarder protocol to connect directly to a LoRaWAN Network Server (TTN, ChirpStack, etc.) without physical radio hardware or gateway. Suitable for development, testing, and CI/CD integration.

Configure via environment variables:

```bash
# Build with virtual radio (native x86/x86_64)
rm -Rf build/
cmake -S examples/main_examples/periodical_uplink_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=LINUX \
    -DRAC_RADIO=udp_pf \
    -G Ninja
cmake --build build

# Run the application
./build/periodical_uplink

# Configure server address/port and gateway EUI (optional)
UDP_PF_SERVER_ADDR=eu1.cloud.thethings.network \
UDP_PF_SERVER_PORT=1700 \
UDP_PF_GATEWAY_EUI=AA555AFFFE000000 \
./build/periodical_uplink
```

**Environment variables for configuration:**
- `UDP_PF_SERVER_ADDR` - Network server address (default: `127.0.0.1`)
- `UDP_PF_SERVER_PORT` - Network server port (default: `1700`)
- `UDP_PF_GATEWAY_EUI` - Gateway EUI identifier (default: `000000FFFE000000`)

---

### Build & flash periodical_uplink on NUCLEO-L073RZ

Notes:
- Only periodical_uplink was tested with low validation.
- The `-DBOARD=NUCLEO_L073` cmake symbol shall be selected.
- Store & Forward feature shall be deactivated.

Build Periodical uplink:

```bash
rm -Rf build/
cmake -S examples/main_examples/periodical_uplink_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L073 \
    -DRAC_RADIO=lr2021 \
    -DLBM_STORE_AND_FORWARD=OFF \
    -G Ninja
cmake --build build
```

Build LCTT Certif:

```bash
rm -Rf build/
cmake -L -S examples/main_examples/lctt_certif_example -B build \
    -DCMAKE_BUILD_TYPE=MinSizeRel \
    -DBOARD=NUCLEO_L073 \
    -DRAC_RADIO=lr2021 \
    -DLBM_STORE_AND_FORWARD=OFF \
    -G Ninja
cmake --build build
```

Example of `openocd`command to flash:
```
openocd -f interface/stlink.cfg -f target/stm32l0_dual_bank.cfg -c "adapter serial 066DFF515055657867152019" -c "adapter speed 500" -c "reset_config srst_only connect_assert_srst" -c "init" -c "program build/periodical_uplink verify reset exit"
```

Note : Not all examples are compiling on NUCLEO-L073RZ. Only periodical_uplink was tested.

</details>

<details>
<summary><h2>Samples</h2></summary>

More details and how to build & use Samples are available on [USP Sample Documentation](/examples/main_examples/README.md)

</details>

<details>
<summary><h2>Porting Guide</h2></summary>

This [chapter](doc/usp_porting_guide.md) explains how to port USP
- on other MCU
- on other radio PCB

This [chapter](doc/usp_lbm_porting_guide.md) explains how to port existing LoRa Basics Modem application to USP

</details>

<details>
<summary><h2>Troubleshooting</h2></summary>

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

</details>

<details>
<summary><h2>License</h2></summary>

Clear BSD License.
Copyright Semtech Corporation.

</details>

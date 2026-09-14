# LR11xx full almanac update example

## Description

This application executes a full almanac update from a given almanac binary image, by using dedicated LoRa Basics Modem
API.

This example also provides a simple python script 'get_full_almanac.py' which is available in usp `examples/main_examples/geolocation/full_almanac_update`. This script
fetches almanac content from traxmate.io and generate a C header file that is compiled with the embedded binary. Refer to USP Documentation for more details.

**NOTE**: This example is only applicable to LR1110 / LR1120 chips.

## Generation of almanac C header file

The python script usage to generate the almanac C header file can be obtained with:

```bash
$ python ./get_full_almanac.py --help
```

For example, in order to get the latest almanac image, one can execute the following:

```bash
$ python get_full_almanac.py -f almanac.h PUT_YOUR_TRAXMATE_TOKEN_HERE
```

In order to get an almanac image for a specific date (for testing purpose), one can execute the following:

```bash
$ python get_full_almanac.py -f almanac.h -g 1419724818 PUT_YOUR_TRAXMATE_TOKEN_HERE
```

> Note: update the GPS time provided with the -g option with the desired GPS time.

## Compile and flash the binary code

The example code expects the almanac C header file produced by *get_full_almanac.py* python script to be named ['almanac.h'](../../../../../../modules/lib/usp/examples/main_examples/geolocation/full_almanac_update/almanac.h).

The full almanac flasher tool can be compiled with CMake or make.

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/geolocation/full_almanac_update -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr1110 -G Ninja; cmake --build build --target full_almanac_update
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/full_almanac_update verify reset exit"
```

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

```
            *** Booting Zephyr OS build vx.y.z ***
[00:00:00.000,000] <inf> full_almanac_update: Full almanac flasher example
INFO: Event received: RESET
[00:00:00.241,000] <inf> lorawan: DefiINFO: LR11XX FW: 0x0401, type: 0x01
ned Hook IDs:
[00:00:00.241,000] <inf> lorawan: RP_HOOK_ID_SUSPEND: 0
[00:00:00.24INFO: Success: almanac source date Sun 2026-01-18 00:00:00 GMT
1,000] <inf> lorawan: RP_HOOK_RAC_VERY_HIGH_PRIORITY: 1
[00:00:00.241,000] <inf> lorawan: RP_HOOK_RAC_HIGH_PRIORITY: 3
[00:00:00.241,000] <inf> lorawan: RP_HOOK_RAC_MEDIUM_PRIORITY: 4
[00:00:00.241,000] <inf> lorawan: RP_HOOK_RAC_LOW_PRIORITY: 13
[00:00:00.241,000] <inf> lorawan: RP_HOOK_RAC_VERY_LOW_PRIORITY: 16
[00:00:00.241,000] <inf> lorawan: RP_HOOK_RAC_FLRP_MAC: 2
[00:00:00.241,000] <inf> lorawan: RP_HOOK_RAC_FLRP_WOR_TX: 14
[00:00:00.241,000] <inf> lorawan: RP_HOOK_RAC_FLRP_WOR_RX: 15
[00:00:00.241,000] <inf> lorawan: RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS_ALMANAC: 5
[00:00:00.241,000] <inf> lorawan: RP_HOOK_ID_DIRECT_RP_ACCESS_GNSS: 6
[00:00:00.242,000] <inf> lorawan: RP_HOOK_ID_DIRECT_RP_ACCESS_WIFI: 7
[00:00:00.242,000] <inf> lorawan: RP_HOOK_ID_LR1MAC_STACK: 8
[00:00:00.242,000] <inf> lorawan: RP_HOOK_ID_LBT: 9
[00:00:00.242,000] <inf> lorawan: RP_HOOK_ID_CAD: 10
[00:00:00.242,000] <inf> lorawan: RP_HOOK_ID_TEST_MODE: 11
[00:00:00.242,000] <inf> lorawan: RP_HOOK_ID_DIRECT_RP_ACCESS: 12
[00:00:00.242,000] <inf> lorawan: RP_HOOK_ID_MAX: 17
[00:00:00.242,000] <inf> lorawan: Modem Initialization
[00:00:00.242,000] <inf> lorawan: Use soft secure element for cryptographic functionalities
[00:00:00.242,000] <inf> lorawan_hal: Opened flash area - size 32768 bytes
[00:00:00.242,000] <wrn> lorawan: No valid DevNonce in NVM, use default (0)
[00:00:00.242,000] <wrn> lorawan: No valid lr1mac context --> Factory reset
[00:00:00.335,000] <inf> lorawan: stack_id 0
[00:00:00.335,000] <inf> lorawan:  DevNonce = 0
[00:00:00.335,000] <inf> lorawan:  JoinNonce = 0xff ff ff, NetID = 0xff ff ff
[00:00:00.335,000] <inf> lorawan:  Region = EU868
[00:00:00.335,000] <inf> lorawan: LoRaWAN Certification is disabled on stack 0
[00:00:00.335,000] <inf> lorawan: mw_gnss_scan_services_init task_id 7, service_id 0, CURRENT_STACK:0
[00:00:00.335,000] <inf> lorawan: mw_gnss_send_services_init task_id 8, service_id 0, CURRENT_STACK:0
[00:00:00.335,000] <inf> lorawan: mw_gnss_almanac_services_init task_id 9, service_id 0, CURRENT_STACK:0
[00:00:00.336,000] <inf> lorawan: mw_wifi_scan_services_init task_id 10, service_id 0, CURRENT_STACK:0
[00:00:00.336,000] <inf> lorawan: mw_wifi_send_services_init task_id 11, service_id 0, CURRENT_STACK:0
[00:00:00.398,000] <inf> lorawan: mw_gnss_almanac_full_update: already up-to-date
```
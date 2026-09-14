# LoRaWAN Geolocation

### Description

This example illustrates how a typical application running LoRa Basics Modem (LBM) can benefit from the **geolocation services** using LR1110/LR1120 transceivers to perform GNSS and Wi-Fi scans and send it either with direct uplinks, or using the Store & Forward service.

This example demonstrates:
- how to configure LoRa Basics Modem library
- how to start the needed services (almanac demodulation update for GNSS scan, store & forward, ...)
- how to program GNSS/Wi-Fi scans
- how to get events from the services to sequence the geolocation scans

### Geolocation services

This example application relies on various services provided by LoRa Basics Modem to simplify the usage of a LR11xx radio to perform geolocation based on GNSS and Wi-Fi scans.

The services used are:
* GNSS scan & send
* GNSS almanac demodulation
* Wi-Fi scan & send
* Store & Forward

For more details about the GNSS and Wi-Fi services, please refer to the documentation here:

Geolocation services documentation: [LOCAL](/protocols/lbm_lib/smtc_modem_core/geolocation_services/README.md)  [ONLINE](https://github.com/Lora-net/usp/blob/main/protocols/lbm_lib/smtc_modem_core/geolocation_services/README.md)
## Key Features

- **Dual Geolocation Methods**: GNSS (GPS + BeiDou) and Wi-Fi scanning
- **Automatic Scanning**: Periodic GNSS (2 min) and Wi-Fi (1 min) scans
- **Almanac Demodulation**: Enhanced GNSS performance with constellation data
- **Store and Forward**: Geolocation data buffering and transmission
- **Custom ADR Profile**: Optimized for geolocation transmission requirements
- **LED Indicators**: Visual feedback for scanning and transmission states
- **Keep-Alive Messages**: Regular status updates every 30 seconds

## Configuration

### Hardware Requirements

| Component     | Requirement                    |
|---------------|--------------------------------|
| Transceiver   | LR1110 or LR1120               |
| Antenna       | GNSS + LoRa antenna or dual antenna setup |
| Network       | LoRaWAN network with geolocation support |

### LoRa Basics Modem configuration
Once the modem has reset, the application configures the various services used and initiate both GNSS and Wi-Fi scans. Then, it completely relies on LBM arbitration to handle LR11xx radio access.

In this example, the device does not need to successfully join a LoRaWAN network to start scanning.
If "Store & Forward" is enabled the scan results are stored in the device's flash memory, and forwarded later once the device has joined a network. Otherwise, the scan results are lost.

Once the modem has joined the LoRaWAN network, the application configures the ADR custom profile.
It is necessary to use a custom profile due to payload size constraints.

### LoRaWAN related configuration

The `example_options.h` header file defines several constants to configure the LoRaWAN parameters (region, keys).

* MODEM_EXAMPLE_REGION: Selects the regulatory region. This example has been tested for SMTC_MODEM_REGION_EU_868.
* USER_LORAWAN_DEVICE_EUI: LoRaWAN device EUI for user defined credentials.
* USER_LORAWAN_JOIN_EUI: LoRaWAN Join EUI for user defined credentials.
* USER_LORAWAN_APP_KEY: LoRaWAN App Key for user defined credentials.

```dts
#define USER_LORAWAN_DEVICE_EUI                        \
    {                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#define USER_LORAWAN_JOIN_EUI                          \
    {                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#define USER_LORAWAN_APP_KEY                                                                           \
    {                                                                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
```

#### Common

The custom ADR profile parameters are defined in the `main_geolocation.c` file.

* CUSTOM_NB_TRANS: The number of retransmission to be done for each LoRaWAN uplink
* ADR_CUSTOM_LIST: The custom datarate distribution to be used for LoRaWAN uplink

Those parameters have to be carefully defined depending of the region of operation, the scan period, the payload size etc...

### Geolocation services events

After the initial scan has been initiated, the application has to rely on the events sent by the GNSS/Wi-Fi services to get results and program the next scan.

The events to be handled are the following:
* SMTC_MODEM_EVENT_GNSS_SCAN_DONE: the GNSS scan service has completed the multiframe scan sequence. The application can retrieve the associated data.
* SMTC_MODEM_EVENT_GNSS_TERMINATED: the GNSS send service has completed the multiframe send sequence according to the selected send mode (direct uplink, store & forward, bypass...). The application must wait for this event before programming the next GNSS scan.
* SMTC_MODEM_EVENT_GNSS_ALMANAC_DEMOD_UPDATE: the GNSS almanac demodulation service has progressed. This event is for information, there is no action to take from the application.
* SMTC_MODEM_EVENT_WIFI_SCAN_DONE: the Wi-Fi scan service has completed the scan. The application can retrieve the associated data.
* SMTC_MODEM_EVENT_WIFI_TERMINATED: the Wi-Fi send service has completed the send sequence according to the selected send mode (direct uplink, store & forward, bypass...). The application must wait for this event before programming the next Wi-Fi scan.



### Geolocation Parameters

| Parameter                        | Value | Description                      |
|----------------------------------|-------|----------------------------------|
| `GEOLOCATION_GNSS_SCAN_PERIOD_S` | `120` | GNSS scan interval               |
| `GEOLOCATION_WIFI_SCAN_PERIOD_S` | `60`  | Wi-Fi scan interval              |
| `KEEP_ALIVE_PERIOD_S`            | `30`  | Keep-alive message interval      |
| `KEEP_ALIVE_PORT`                | `2`   | LoRaWAN port for status messages |

* GEOLOCATION_GNSS_SCAN_PERIOD_S: Defines the duration between the end of a GNSS scan & send sequence and the start of next sequence, in seconds.
* GEOLOCATION_WIFI_SCAN_PERIOD_S: Defines the duration between the end of a Wi-Fi scan & send sequence and the start of next sequence, in seconds.
* By default, the selected GNSS scan mode is SMTC_MODEM_GNSS_MODE_MOBILE, and the GNSS send mode is SMTC_MODEM_SEND_MODE_STORE_AND_FORWARD.
* By default, the selected WiFi send mode is SMTC_MODEM_SEND_MODE_UPLINK.

## Tools

`Almanac full update` and `WiFi region detector` Tools are delivered in this software release.
`LR11xx flasher` Tool is not delivered in this software release.

## Compilation

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/geolocation/geoloc_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr1120 -G Ninja; cmake --build build --target geolocation
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/geolocation verify reset exit"
```

## Usage

1. **Network Setup**: Configure LoRaWAN credentials and ensure network supports geolocation services
2. **Deploy Device**: Place in location with GNSS satellite and Wi-Fi access point visibility
3. **Operation**:
   - **Automatic Join**: Device joins LoRaWAN network on startup
   - **Almanac Download**: Begins downloading GNSS almanac for improved performance
   - **Periodic Scans**: Alternates between GNSS and Wi-Fi scanning
   - **Data Transmission**: Sends geolocation data via LoRaWAN uplinks

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Initialization
```
[00:00:01.000,000] <inf> geolocation_sample: GEOLOCATION example is starting
[00:00:01.100,000] <inf> geolocation_sample: LR11XX FW: 0x0401, type: 0x01
[00:00:02.200,000] <inf> geolocation_sample: Event received: JOINED
```

### Scanning Operations
```
[00:00:30.300,000] <inf> geolocation_sample: Event received: GNSS_TERMINATED
[00:01:15.400,000] <inf> geolocation_sample: Event received: WIFI_TERMINATED
[00:01:20.500,000] <inf> geolocation_sample: GPS progress: 45%
[00:01:20.500,000] <inf> geolocation_sample: BDS progress: 23%
```

## Technical Notes

- **Constellation Support**: GPS and BeiDou satellites for improved accuracy
- **Custom ADR**: Optimized transmission parameters for geolocation payloads
- **Store and Forward**: Buffers scan results for reliable transmission
- **Firmware Validation**: Automatic check for compatible LR11XX firmware versions
- **LED Feedback**: TX LED pulses during scan completion and transmission events

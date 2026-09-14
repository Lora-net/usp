# LoRaWAN Certification (LCTT)

This application provides testing for the LoRaWAN Certification Test Tool (LCTT). Please see <https://resources.lora-alliance.org/home/getting-started-with-the-lorawan-certification-test-tool-lctt-2> for more information. It enables LoRa Basics Modem compliance testing by offering both normal operation and certification modes that can be toggled via button press.

## Key Features

- **Dual Operation Modes**: Normal periodic uplinks and LCTT certification mode
- **Button-Controlled Toggle**: Switch between modes using user button
- **Advanced LBM Features**: Class B/C, multicast, CSMA support for comprehensive testing
- **Persistent Mode**: Certification mode setting persists across reboots

## Configuration

### Network Credentials (Required)

Configure your LoRaWAN credentials in `main_examples/example_options.h`:

```dts
#ifndef USER_LORAWAN_DEVICE_EUI
#define USER_LORAWAN_DEVICE_EUI                        \
    {                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#endif
#ifndef USER_LORAWAN_JOIN_EUI
#define USER_LORAWAN_JOIN_EUI                          \
    {                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
    }
#endif
#ifndef USER_LORAWAN_GEN_APP_KEY
#define USER_LORAWAN_GEN_APP_KEY                                                                       \
    {                                                                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 \
    }
#endif
#ifndef USER_LORAWAN_APP_KEY
#define USER_LORAWAN_APP_KEY                                                                           \
    {                                                                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 \
    }
#endif
```

### LoRaWAN Regions

| Region    | Description           |
|-----------|-----------------------|
| `EU_868`  | Europe 868 MHz        |
| `US_915`  | North America 915 MHz |
| `AS_923_GRP1`     | Asia-Pacific 923 MHz (GRP1)  |
| `AS_923_GRP2`     | Asia-Pacific 923 MHz (GRP2)  |
| `AS_923_GRP3`     | Asia-Pacific 923 MHz (GRP3)  |
| `AS_923_GRP4`     | Asia-Pacific 923 MHz (GRP4)  |
| `AU_915`  | Australia 915 MHz     |
| `CN_470`  | China 470 MHz (RP2)         |
| `CN_470_RP_1_0` | China 470 MHz (RP1) |
| `IN_865`  | India 865 MHz         |
| `KR_920`     | South Korea 920 MHz |
| `RU_864`     | Russia 864 MHz  |
| `WW_2G4`     | WW 2.4GHz  |

### Certification Requirements

- **LCTT Software**: LoRaWAN Certification Test Tool from LoRa Alliance
- **Gateway Connection**: Direct gateway link to LCTT Network Server
- **Isolated Environment**: Use only in isolated RF environments during certification

## Compilation

**Build sample:**
``` bash
rm -Rf build/ ; cmake -L -S examples/main_examples/lctt_certif_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L073 -DRAC_RADIO=lr2021 -DLBM_STORE_AND_FORWARD=OFF -G Ninja; cmake --build build --target lctt_certif
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/lctt_certif verify reset exit"
```

## Usage

1. **Network Setup**: Configure LoRaWAN credentials (see `periodical_uplink` documentation)
2. **LCTT Setup**: Connect device to gateway linked with LCTT software
3. **Operation**:
   - **Normal Mode**: Automatic periodic uplinks every 10 seconds on port 101
   - **Button Press**: Toggles between normal and certification modes
   - **Certification Mode**: LCTT takes full control, bypassing duty cycle limits

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Mode Toggle
```
[00:00:01.000,000] <inf> lctt_certif: Certification example is starting
[00:00:01.100,000] <inf> lctt_certif: Push button to enable/disable certification
[00:00:30.200,000] <inf> lctt_certif: Button pushed
[00:00:30.300,000] <inf> lctt_certif: Certification mode enabled
```

### Normal Operation
```
[00:00:02.200,000] <inf> lctt_certif: Event received: JOINED
[00:00:12.300,000] <inf> lctt_certif: Event received: ALARM
[00:00:12.400,000] <inf> lctt_certif: Event received: TXDONE
```

## Important Notes

- **Certification Mode**: Should only be used in isolated RF environments for compliance testing
- **Mode Persistence**: Certification mode setting survives device resets
- **LCTT Control**: In certification mode, the LCTT software controls all device operations
- **Compliance Testing**: Used to verify LoRaWAN specification conformance

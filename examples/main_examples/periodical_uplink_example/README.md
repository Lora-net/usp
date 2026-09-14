# LoRaWAN Periodical Uplink

This application demonstrates **LoRaWAN network integration** using LoRa Basics Modem (LBM) with automatic periodic manual button-triggered uplinks on port 102.

## Key Features

- **LoRaWAN Network Join**: Automatic OTAA (Over-The-Air Activation) network joining
- **Periodic Uplinks**: Configurable automatic uplinks every 60 seconds (default)
- **Button-Triggered Uplinks**: Manual uplink transmission on button press
- **Downlink Reception**: Receives and processes downlink messages
- **Relay TX Support**: Optional LoRaWAN relay transmission capability
- **Multiple LoRaWAN Regions**: Support for EU868, US915, and other regions
- **Credentials Management**: Device provisioning via device tree overlays
- **Low Power Mode**: Optional power-optimized configuration available

## Configuration

### Network Credentials (Required)

Configure your LoRaWAN credentials in [LOCAL](/examples/main_examples/example_options.h) [ONLINE](https://github.com/Lora-net/usp/blob/main/examples/main_examples/example_options.h)

```dts
#ifndef USER_LORAWAN_DEVICE_EUI
#define USER_LORAWAN_DEVICE_EUI                        \
    {                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 \
    }
#endif
#ifndef USER_LORAWAN_JOIN_EUI
#define USER_LORAWAN_JOIN_EUI                          \
    {                                                  \
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 \
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

An alternative possibility is to compile the example passing the keys :
### Using CMake

| Parameter                    | Default Value | Description                                    |
|------------------------------|---------------|------------------------------------------------|
| `PERIODICAL_UPLINK_DELAY_S`  | `60`          | Interval between periodic uplinks (seconds)    |
| `DELAY_FIRST_MSG_AFTER_JOIN` | `60`          | Delay after join before first uplink (seconds) |
| `STACK_ID`                   | `0`           | LBM stack identifier                           |

### USP Zephyr Build Configurations

| Configuration    | File                | Description                          |
|------------------|---------------------|--------------------------------------|
| Standard         | `prj.conf`          | Full logging and debug capabilities  |
| Low Power        | `prj_lowpower.conf` | Power-optimized with minimal logging |

### LoRaWAN Regions

Set the selected region in `/examples/main_examples/example_options.h` file for USP zephyr project or in [USP credentials](/examples/main_examples/example_options.h) for USP

| Region          | Description                 |
|-----------------|-----------------------------|
| `EU_868`        | Europe 868 MHz              |
| `US_915`        | North America 915 MHz       |
| `AS_923_GRP1`   | Asia-Pacific 923 MHz (GRP1) |
| `AS_923_GRP2`   | Asia-Pacific 923 MHz (GRP2) |
| `AS_923_GRP3`   | Asia-Pacific 923 MHz (GRP3) |
| `AS_923_GRP4`   | Asia-Pacific 923 MHz (GRP4) |
| `AU_915`        | Australia 915 MHz           |
| `CN_470`        | China 470 MHz (RP2)         |
| `CN_470_RP_1_0` | China 470 MHz (RP1)         |
| `IN_865`        | India 865 MHz               |
| `KR_920`        | South Korea 920 MHz         |
| `RU_864`        | Russia 864 MHz              |
| `WW_2G4`        | WW 2.4GHz                   |

## Compilation

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/periodical_uplink_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target periodical_uplink
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/periodical_uplink verify reset exit"
```

## Usage

### Prerequisites

1. **LoRaWAN Network**: Access to a LoRaWAN network (TTN, ChirpStack, etc.)
2. **Network Credentials**: Valid DevEUI, JoinEUI, and AppKey from your network provider
3. **Gateway Coverage**: Ensure LoRaWAN gateway coverage in your area

### Setup Steps

1. **Configure Credentials**: Edit `/examples/main_examples/example_options.h` with your LoRaWAN credentials or [USP credentials](/examples/main_examples/example_options.h)
2. **Select Region**: Set appropriate region in user_keys.overlay
3. **Flash Firmware**: Build and flash the application
4. **Monitor Logs**: Connect serial console to observe join process and uplinks

### Operation

1. **Automatic Join**: Device automatically attempts to join the network on startup
2. **Periodic Uplinks**: After successful join, sends uplinks on port 101 every 60 seconds
3. **Button Uplinks**: Press user button to send immediate uplink on port 102
4. **Downlink Handling**: Receives and logs any downlink messages from network

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Startup and Join
```
[00:00:01.000,000] <inf> usp: LoRaWAN Periodical uplink (60 sec) example is starting
[00:00:02.100,000] <inf> usp: Event received: RESET
[00:00:02.200,000] <inf> usp: Event received: JOINED
[00:00:02.200,000] <inf> usp: Modem is now joined
```

### Periodic Uplinks
```
[00:01:02.300,000] <inf> usp: Event received: ALARM
[00:01:02.400,000] <inf> usp: Event received: TXDONE
[00:01:02.400,000] <inf> usp: Transmission done
[00:02:02.500,000] <inf> usp: Event received: ALARM
```

### Button-Triggered Uplink
```
[00:00:30.200,000] <inf> usp: Button pushed
[00:00:30.300,000] <inf> usp: Event received: TXDONE
[00:00:30.300,000] <inf> usp: Transmission done
```

### Downlink Reception
```
[00:01:05.100,000] <inf> usp: Event received: DOWNDATA
[00:01:05.100,000] <dbg> usp: Data received on port 1
[00:01:05.100,000] <dbg> usp: Received payload: 48 65 6C 6C 6F
```

## Technical Notes

- **Uplink Ports**: Periodic uplinks use port 101, button uplinks use port 102
- **Payload Format**: 4-byte counter (big-endian) incremented with each transmission
- **Join Method**: OTAA (Over-The-Air Activation) only
- **Button Debouncing**: 500ms debounce time to prevent multiple triggers
- **Event-Driven Architecture**: Comprehensive handling of all LBM events
- **Relay Support**: Optional LoRaWAN relay TX functionality for network extension
- **Thread Integration**: Full USP thread integration with cooperative scheduling

## Advanced Features

### Relay TX Configuration
When `CONFIG_LORA_BASICS_MODEM_RELAY_TX=y`:
- Dynamic relay activation mode
- Smart level: 8 consecutive uplinks
- Backoff: 4 WOR frames
- CSMA protection for WOR transmissions

### Event Handling
- **Network Events**: JOIN, TXDONE, DOWNDATA, JOINFAIL
- **Service Events**: FUOTA, multicast, firmware management
- **Advanced Events**: Relay TX, regional duty cycle, test mode

## Troubleshooting

### Common Issues

1. **Join Failures**:
   - Verify credentials in user_keys.overlay or example_options.h
   - Check LoRaWAN region setting
   - Ensure gateway coverage

2. **No Uplinks**:
   - Confirm successful join (JOINED event)
   - Check duty cycle restrictions
   - Verify network connectivity

3. **Build Errors**:
   - Ensure user_keys.overlay or example_options.h are properly configured
   - Check shield compatibility with board
   - Verify USP module configuration

### Network Server Configuration

- **Payload Decoder**: 4-byte counter (bytes to uint32, big-endian)
- **Port Assignment**: Port 101 (periodic), Port 102 (button)
- **Confirmed Uplinks**: Unconfirmed by default (configurable)

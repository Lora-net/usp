# TX Continuous Wave (CW)

This application demonstrates **continuous transmission** capabilities for testing and development purposes. It provides two transmission modes: standard continuous wave transmission and infinite preamble mode. The application transmits using LoRa modulation with user-controlled start/stop functionality via button press.

## Key Features

- **Continuous Transmission**: Sustained RF output for testing and measurement
- **Dual Transmission Modes**: Standard continuous wave or infinite preamble
- **Button-Controlled Operation**: Start/stop transmission via user button
- **LED Feedback**: Visual TX activity indication
- **Configurable Parameters**: Frequency, power, modulation settings
- **Packet Counting**: Tracks total number of transmitted packets
- **Scheduled Transmission**: 1-second interval between packet transmissions
- **High Priority Access**: Uses very high priority RAC access for reliable transmission

### Operation Modes

- **Standard CW Mode**: Continuous packet transmission with 1-second intervals
- **Infinite Preamble Mode**: Transmits extended preamble for carrier testing

### Transmission Control

- **Button Press**: Toggle between start and stop transmission
- **Real-time Control**: Immediate response to user commands
- **Safe Stop**: Clean transmission termination with statistics display

## Configuration

### Using CMake

| Parameter               | Default Value              | Description                                      |
|-------------------------|----------------------------|--------------------------------------------------|
| `RF_FREQ_IN_HZ`         | `868100000`                | Operating frequency in Hz (868.1 MHz)            |
| `TX_OUTPUT_POWER_DBM`   | `14`                       | Transmit power in dBm                            |
| `PAYLOAD_SIZE`          | `128`                      | Payload size in bytes                            |
| `INFINITE_PREAMBLE`     | *undefined*                | Enable infinite preamble mode (define to enable) |
| `LORA_SPREADING_FACTOR` | `RAL_LORA_SF7`             | LoRa spreading factor                            |
| `LORA_BANDWIDTH`        | `RAL_LORA_BW_125_KHZ`      | LoRa bandwidth                                   |
| `LORA_CODING_RATE`      | `RAL_LORA_CR_4_5`          | LoRa coding rate                                 |
| `LORA_PREAMBLE_LENGTH`  | `12`                       | LoRa preamble length in symbols                  |
| `LORA_PKT_LEN_MODE`     | `RAL_LORA_PKT_EXPLICIT`    | LoRa packet length mode                          |
| `LORA_IQ`               | `false`                    | LoRa IQ inversion                                |
| `LORA_CRC`              | `true`                     | LoRa CRC enable/disable                          |

## Compilation

**Build standard LoRa CW mode with INFINITE PREAMBLE:**
``` bash
rm -Rf build/ ; cmake -L -S examples/main_examples/tx_cw_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -DINFINITE_PREAMBLE=ON -G Ninja; cmake --build build --target tx_cw
```

**Build standard LoRa CW mode with CONTINUOUS WAVE:**
```
rm -Rf build/ ; cmake -L -S examples/main_examples/tx_cw_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target tx_cw
```

Flash:
``` bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program build/tx_cw verify reset exit"
```

## Usage

1. **Power On Device**: Boot the device with TX CW firmware

2. **Monitor Output**: Observe UART logs for initialization messages

3. **Start Transmission**: Press the user button to begin continuous transmission

4. **Monitor Activity**:
   - LED indicates TX activity
   - UART shows transmission progress and packet count

5. **Stop Transmission**: Press the user button again to stop transmission

6. **View Statistics**: Check UART output for total packet count

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Initialization and Configuration Display

```
[00:00:00.036,000] <inf> usp: Initialization done
[00:00:00.040,000] <inf> usp: Tx continuous application started
[00:00:00.045,000] <inf> usp: TX CW initialized - press button to start/stop continuous transmission
```

### Starting Transmission (Standard CW Mode)

```
[00:00:05.200,000] <inf> usp: button pressed
[00:00:05.200,000] <inf> usp: Starting continuous transmission...
[00:00:05.200,000] <inf> usp: Continuous wave
[00:00:05.200,000] <inf> usp: Modulation type: LORA
[00:00:05.200,000] <inf> usp: Frequency: 868100000 Hz
[00:00:05.200,000] <inf> usp: Power: 14 dBm
[00:00:05.200,000] <inf> usp: Spread factor: SF7
[00:00:05.200,000] <inf> usp: Bandwidth: 125 kHz
[00:00:05.200,000] <inf> usp: Coding rate: 4/5
[00:00:05.200,000] <inf> usp: Preamble length: 12
[00:00:05.200,000] <inf> usp: Packet length mode: Explicit
[00:00:06.205,000] <inf> usp: usp/rac: transmission #0 starting
```

### Starting Transmission (Infinite Preamble Mode)

```
[00:00:05.200,000] <inf> usp: button pressed
[00:00:05.200,000] <inf> usp: Starting continuous transmission...
[00:00:05.200,000] <inf> usp: Infinite preamble
[00:00:05.200,000] <inf> usp: Modulation type: LORA
[00:00:05.200,000] <inf> usp: Frequency: 868100000 Hz
[00:00:05.200,000] <inf> usp: Power: 14 dBm
[00:00:05.200,000] <inf> usp: Spread factor: SF7
[00:00:05.200,000] <inf> usp: Bandwidth: 125 kHz
[00:00:06.205,000] <inf> usp: usp/rac: transmission #0 starting
```

## Use Cases

### RF Testing and Measurement

1. **Spectrum Analysis**:
   - Use standard CW mode for continuous carrier measurement
   - Verify frequency accuracy and stability
   - Measure spurious emissions

2. **Power Measurement**:
   - Conducted power testing with continuous transmission
   - Antenna pattern measurements
   - EIRP validation

3. **Modulation Analysis**:
   - LoRa modulation characteristics
   - Symbol rate accuracy verification

4. **Regulatory Compliance**:
   - Continuous transmission for certification testing
   - Power spectral density measurements
   - Occupied bandwidth verification

## Technical Notes

- **Transmission Interval**: 1-second delay between packets in standard mode
- **Priority Level**: Uses `RAC_VERY_HIGH_PRIORITY` for reliable access
- **Payload Buffer**: 255-byte maximum capacity, configurable size
- **Button Debouncing**: 500ms debounce time to prevent multiple triggers
- **Thread Safety**: Cooperative scheduling for deterministic behavior

## Safety Considerations

- **RF Exposure**: Ensure compliance with local SAR/MPE limits during continuous transmission
- **Thermal Management**: Monitor device temperature during extended operation
- **Regulatory Compliance**: Verify transmission parameters comply with local regulations
- **Antenna Connection**: Ensure proper antenna connection to avoid PA damage
- **Duty Cycle**: Some regions may have duty cycle restrictions for continuous transmission

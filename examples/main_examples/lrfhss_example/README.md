# LR-FHSS Example

This application demonstrates **Long Range - Frequency Hopping Spread Spectrum (LR-FHSS)** transmission capabilities using the USP/RAC framework. LR-FHSS is a transmission-only modulation that combines frequency hopping with spread spectrum techniques for robust long-range communication. The application sends periodic transmissions with structured payloads and provides button-triggered immediate transmission cycles.

## Key Features

- **Frequency Hopping**: Automatic frequency hopping across multiple channels for interference resilience
- **Spread Spectrum**: Enhanced sensitivity and range through coding rate optimization
- **Structured Payload**: Packets contain header ("LR-FHSS:") and incremental counter
- **Button-Controlled Operation**: Immediate transmission trigger via user button
- **Periodic Transmissions**: Configurable transmission intervals with automatic scheduling
- **LED Feedback**: Visual TX activity indication

## Configuration

### Using CMake

| Parameter               | Default Value       | Description                                    |
|-------------------------|---------------------|------------------------------------------------|
| `LRFHSS_FREQUENCY_HZ`   | `866500000`         | Operating frequency in Hz (866.5 MHz)          |
| `LRFHSS_TX_POWER_DBM`   | `14`                | Transmit power in dBm                          |
| `LRFHSS_PAYLOAD_SIZE`   | `50`                | Payload size in bytes                          |
| `LRFHSS_TX_INTERVAL_MS` | `5000`              | Interval between transmissions in milliseconds |

## Compilation

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/lrfhss_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target lrfhss_tx
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/lrfhss_tx verify reset exit"
```

## Usage

1. **Flash Firmware**: Load the compiled firmware to your device

2. **Connect Console**: Open serial console to monitor transmission activity
   ```bash
   minicom -D /dev/ttyACM0 -b 115200
   ```

3. **Start Transmission**: Press the user button to begin LR-FHSS transmission cycle

4. **Monitor Activity**:
   - LED indicates TX activity during transmissions
   - UART shows transmission progress and packet counters

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Initialization
```
[00:00:01.000,000] <inf> usp: ===== LR - FHSS example        started - press button to trigger immediate transmission cycle =====
[00:00:01.100,000] <inf> usp: Radio access opened with ID: 0
[00:00:01.100,000] <inf> usp: LR-FHSS Configuration:
[00:00:01.100,000] <inf> usp:   - Frequency: 866500000 Hz
[00:00:01.100,000] <inf> usp:   - TX Power: 14 dBm
[00:00:01.100,000] <inf> usp:   - Payload Size: 50 bytes
[00:00:01.100,000] <inf> usp:   - TX Interval: 5000 ms
[00:00:01.100,000] <inf> usp: LR-FHSS application initialized successfully
```

### Button Press and Transmission
```
[00:00:05.200,000] <inf> usp: Button pushed
[00:00:05.200,000] <inf> usp: Starting LR-FHSS transmission cycle
[00:00:06.200,000] <inf> usp: TX LR-FHSS TX #1 scheduled for 6200 ms
[00:00:06.300,000] <inf> usp: TX : LR-FHSS TX #1 completed successfully
[00:00:11.300,000] <inf> usp: TX LR-FHSS TX #2 scheduled for 11300 ms
[00:00:11.400,000] <inf> usp: TX : LR-FHSS TX #2 completed successfully
```

## Technical Notes

- **Transmission Interval**: 5-second default interval between automatic transmissions
- **Priority Level**: Uses `RAC_HIGH_PRIORITY` for reliable radio access
- **Payload Structure**: "LR-FHSS:" header (8 bytes) + counter (4 bytes) + random data
- **Button Debouncing**: 500ms debounce time to prevent multiple triggers
- **Frequency Hopping**: Automatic hop sequence generation for each transmission
- **Thread Safety**: Cooperative scheduling for deterministic behavior

## Regulatory Considerations

- **Frequency Compliance**: Ensure operating frequency complies with local regulations
- **Power Limits**: Verify transmission power is within regional limits
- **Duty Cycle**: Some regions may have duty cycle restrictions for continuous transmission
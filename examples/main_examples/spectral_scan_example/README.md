# Spectral Scan

This application provides **RF spectral analysis** capabilities for radio frequency environment assessment and interference detection. It performs systematic RSSI (Received Signal Strength Indicator) measurements across configurable frequency ranges to generate detailed spectral information including histograms, minimum/maximum/average RSSI values, and statistical analysis for each frequency channel.

## Key Features

- **Multi-Channel Scanning**: Configurable frequency range with adjustable channel spacing
- **Statistical Analysis**: Min/max/average RSSI values with histogram generation
- **Flexible Configuration**: Customizable frequency range, channel count, and sampling parameters
- **Real-Time Display**: Detailed results logging with frequency and RSSI information
- **RSSI Histogram**: Binned RSSI distribution analysis for signal characterization

## Configuration

### Using CMake

| Parameter               | Default Value         | Description                          |
|-------------------------|-----------------------|--------------------------------------|
| `FREQ_START_HZ`         | `866500000`           | Starting frequency in Hz (866.5 MHz) |
| `NB_CHAN`               | `3`                   | Number of frequency channels to scan |
| `WIDTH_CHAN_HZ`         | `1000000`             | Channel spacing in Hz (1 MHz)        |
| `NB_SCAN`               | `50`                  | Number of RSSI samples per channel   |
| `RSSI_TOP_LEVEL_DBM`    | `0`                   | Maximum RSSI level in dBm            |
| `RSSI_BOTTOM_LEVEL_DBM` | `-128`                | Minimum RSSI level in dBm            |
| `RSSI_SCALE`            | `4`                   | RSSI histogram bin size in dBm       |

## Compilation

**Build with default configuration**
```bash
rm -Rf build ; cmake -L -S examples/main_examples/spectral_scan_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target spectral_scan
```

**Flash using openocd**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/spectral_scan verify reset exit"
```

**Flash using pyocd**
```bash
pyocd flash --target stm32l476rgtx --uid <SERIAL_NUMBER> --connect under-reset --format elf build/spectral_scan
```

## Usage

1. **Prepare Test Setup**: Position device in the environment to be analyzed
2. **Power On Device**: The application initializes and displays configuration
3. **Trigger Scan**: Press the user button to start a spectral scan cycle
4. **Monitor Results**: Observe UART logs for detailed measurements and statistics
5. **Repeat Scans**: Press button again for additional scan cycles
6. **Analyze Data**: Review histogram and statistical data for each channel

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

The output can be seen using minicom
```
minicom -D /dev/USB0 -b 921600
```
Please note that the `spectral_scan` application is configured for a baudrate of 921600

### Application Startup

```
[00:00:00.035,000] <inf> usp: ================================
[00:00:00.035,000] <inf> usp:   Spectral Scan Application v1.0.0
[00:00:00.035,000] <inf> usp:   Built: Oct 16 2025 14:51:54
[00:00:00.035,000] <inf> usp: ================================
[00:00:00.035,000] <inf> usp: CONFIG Spectral Scan Configuration:
[00:00:00.036,000] <inf> usp: CONFIG   - Frequency range: 866.500 - 868.500 MHz
[00:00:00.036,000] <inf> usp: CONFIG   - Number of channels: 3
[00:00:00.036,000] <inf> usp: CONFIG   - Channel spacing: 1.000 MHz
[00:00:00.036,000] <inf> usp: CONFIG   - Scans per channel: 50
[00:00:00.036,000] <inf> usp: CONFIG   - RSSI range: -128 to 0 dBm
[00:00:00.036,000] <inf> usp: CONFIG   - RSSI scale: 4 dBm/bin
[00:00:00.036,000] <inf> usp: CONFIG   - Histogram bins: 33
[00:00:00.036,000] <inf> usp: Radio initialized successfully (ID: 3)
[00:00:00.036,000] <inf> usp: Spectral scan application initialized successfully
[00:00:00.036,000] <inf> usp: Spectral scan example started - press button to trigger immediate scan
```

### Results Summary

```
[00:00:18.273,000] <inf> usp: STATS === Spectral Scan Results (Cycle 1) ===
[00:00:18.274,000] <inf> usp: 866.500 MHz: 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 4 13 27 6 0 0
[00:00:18.274,000] <inf> usp:   Stats: 50 samples, Min: -121 dBm, Max: -110 dBm, Avg: -118 dBm
[00:00:18.274,000] <inf> usp:     ASCII Histogram for 866.500 MHz:
[00:00:18.274,000] <inf> usp:                                  █
[00:00:18.274,000] <inf> usp:                                  █
[00:00:18.274,000] <inf> usp:                                  █
[00:00:18.274,000] <inf> usp:                                  █
[00:00:18.274,000] <inf> usp:                                  █
[00:00:18.274,000] <inf> usp:                                 ██
[00:00:18.274,000] <inf> usp:                                 ██
[00:00:18.274,000] <inf> usp:                                ████
[00:00:18.275,000] <inf> usp:     |---|---|---|---|---|---|---|---|
[00:00:18.275,000] <inf> usp:      0     -32     -64     -96    -128     dBm
```

## Technical Notes

- **State Machine**: Manages scan progression through channels and measurements
- **RSSI Measurements**: Uses LoRa receiver for RSSI sampling at each frequency
- **Thread Safety**: Uses RAC transaction system for radio access coordination

## Applications

- **Spectrum Analysis**: RF environment assessment before deployment
- **Interference Detection**: Identify occupied frequency channels
- **Site Survey**: Characterize RF conditions at installation locations
- **Regulatory Compliance**: Verify spectrum usage in licensed bands
- **Troubleshooting**: Diagnose RF interference issues

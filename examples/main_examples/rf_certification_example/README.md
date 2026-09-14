# RF Certification

This application provides **comprehensive RF certification testing** capabilities for regulatory compliance across multiple regions (ETSI, FCC, ARIB). It supports automated testing sequences with multiple modulation types (LoRa, FSK, LR-FHSS, FLRC) and regional frequency plans. The application systematically cycles through predefined test configurations to validate RF performance according to regulatory requirements.

This tool is provided by Semtech to assist customers in preparing for radio certification on custom hardware. It does not guarantee regulatory approval. Certification results depend on board design, implementation, and accredited laboratory testing. Semtech accepts no liability for certification outcomes. For guidance and support, please contact Semtech support.
**FCC duty-cycle limitations are known and being addressed.**

## Key Features

- **Multi-Regional Support**: ETSI (Europe), FCC (USA), and ARIB (Japan) certification sequences
- **Multiple Modulations**: LoRa, FSK, LR-FHSS, and FLRC modulation testing
- **Automated Test Sequences**: Predefined state machines for systematic testing
- **Frequency Sweeping**: FCC FHSS and hybrid mode frequency hopping
- **LED Feedback**: Visual TX activity indication
- **LBT Support**: Listen-Before-Talk for ARIB region compliance
- **Configurable Payload**: 255-byte maximum payload with packet counters
- **Real-Time Configuration Display**: Detailed parameter logging for each test
- **Notification System**: Another device can optionally receive a detailled notification for each transaction

### Operation Modes

- **Certification Mode**: Continuous transmission for regulatory testing
- **RX-Only Mode**: Reception-only mode for notification monitoring
- **Regional Compliance**: Automated sequences tailored for specific regulatory domains

### Supported Test Sequences

#### ETSI - 868 MHz Band
- **LoRa Tests**: Multiple frequency/SF/BW combinations (863.1-869.9 MHz)
- **FSK Tests**: LoRaWAN-compliant FSK parameters across EU channels
- **LR-FHSS Tests**: Frequency hopping spread spectrum validation
- **FLRC Tests**: High-speed communication testing

#### FCC - 902-928 MHz Band
- **DTS Mode**: 500 kHz bandwidth testing (903.0, 909.4, 914.2 MHz)
- **Hybrid Mode**: 125 kHz bandwidth with multiple SF/DR combinations
- **FHSS Sweep**: 64-channel pseudo-random frequency hopping
- **Hybrid Sweep**: Subband-specific 8-channel testing

#### ARIB - 920-928 MHz Band
- **LoRa Tests**: 125 kHz bandwidth with LBT compliance
- **FSK/LR-FHSS/FLRC**: Complete modulation coverage with LBT

## Configuration

### Using CMake

| Parameter               | Default Value                      | Description                              |
|-------------------------|------------------------------------|------------------------------------------|
| `RF_CERT_REGION`        | `RF_CERT_REGION_ETSI`              | Certification region: ETSI, FCC, or ARIB |
| `RF_FREQ_IN_HZ`         | `868100000`                        | Base operating frequency in Hz           |
| `TX_OUTPUT_POWER_DBM`   | `14`                               | Transmit power in dBm                    |
| `PAYLOAD_SIZE`          | `255`                              | Payload size in bytes (max 255)          |
| `LORA_SPREADING_FACTOR` | `RAL_LORA_SF7`                     | Default LoRa spreading factor            |
| `LORA_BANDWIDTH`        | `RAL_LORA_BW_125_KHZ`              | Default LoRa bandwidth                   |
| `LORA_CODING_RATE`      | `RAL_LORA_CR_4_5`                  | Default LoRa coding rate                 |
| `LORA_PREAMBLE_LENGTH`  | `8`                                | Default LoRa preamble length             |
| `LORA_PKT_LEN_MODE`     | `RAL_LORA_PKT_EXPLICIT`            | LoRa packet length mode                  |
| `LORA_IQ`               | `false`                            | LoRa IQ inversion                        |
| `LORA_CRC`              | `true`                             | LoRa CRC enable/disable                  |
| `FSK_BITRATE`           | `50000`                            | FSK bitrate in bps (50 kbps)             |
| `FSK_FDEV`              | `25000`                            | FSK frequency deviation in Hz            |
| `FSK_BANDWIDTH`         | `117000`                           | FSK bandwidth in Hz (117 kHz)            |
| `LRFHSS_CODING_RATE`    | `LR_FHSS_V1_CR_1_3`                | LR-FHSS coding rate (1/3)                |
| `LRFHSS_BANDWIDTH`      | `LR_FHSS_V1_BW_136719_HZ`          | LR-FHSS bandwidth (136.719 kHz)          |
| `LRFHSS_ENABLE_HOPPING` | `true`                             | Enable LR-FHSS frequency hopping         |
| `FLRC_RAW_BIT_RATE`     | `RAL_FLRC_RAW_BIT_RATE_2_600_MBPS` | FLRC bandwidth in Hz (2.666 MHz)         |
| `FLRC_CODING_RATE`      | `RAL_FLRC_CR_1_1`                  | FLRC coding rate (1/1)                   |
| `NOTIFICATIONS_ENABLED` | `false`                            | Enable notification system               |
| `RX_ONLY`               | `false`                            | RX-only mode (notification monitoring)   |

## Compilation

**Build for ETSI region:**
```
rm -Rf build/ ; cmake -L -S examples/main_examples/rf_certification_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target rf_certification_etsi
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/rf_certification_etsi verify reset exit"
```

## Usage

1. **Prepare Test Setup**: Position device in appropriate test environment (anechoic chamber, conducted setup)

2. **Configure Region**: Build firmware for target certification region (ETSI/FCC/ARIB)

3. **Connect Test Equipment**:
   - Spectrum analyzer for emission testing
   - Power meter for conducted power measurements
   - Antenna for radiated measurements

4. **Start Test Sequence**: Press user button to initiate automated certification sequence

5. **Monitor Progress**: Observe UART logs for test progression and parameter changes

6. **Record Results**: Document measurements for each test configuration

7. **Notifications (optional)**: Use another device to log every transaction

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### ETSI Certification Sequence

```
[00:00:00.036,000] <inf> usp: Initialization done
[00:00:00.040,000] <inf> usp: RF Certification application started
[00:00:00.045,000] <inf> usp: ==============================
[00:00:00.045,000] <inf> usp:    RF Certification Example
[00:00:00.045,000] <inf> usp: ==============================
[00:00:00.045,000] <inf> usp: Region: ETSI (Europe)
[00:00:00.045,000] <inf> usp: ==============================
[00:00:00.050,000] <inf> usp: Press the blue button to start transmission
[00:00:05.200,000] <inf> usp: button pressed
[00:00:05.200,000] <inf> usp: ===== LoRa Configuration =====
[00:00:05.200,000] <inf> usp: Frequency:        863100000 Hz (863.1 MHz)
[00:00:05.200,000] <inf> usp: TX Power:         14 dBm
[00:00:05.200,000] <inf> usp: Spreading Factor: SF7
[00:00:05.200,000] <inf> usp: Bandwidth:        125 kHz
[00:00:05.200,000] <inf> usp: Coding Rate:      4/5
[00:00:05.200,000] <inf> usp: Payload Length:   255 bytes
[00:00:05.205,000] <inf> usp: TX Scheduling LoRa transmission #1
[00:00:05.240,000] <inf> usp: TX Scheduling LoRa transmission #2
...
[00:01:30.500,000] <inf> usp: CONFIG LoRa parameters changed: Frequency: 863100000 Hz, SF: SF12, BW: 125 kHz, TX Power: 14 dBm
[00:01:30.505,000] <inf> usp: TX Scheduling LoRa transmission #1847
...
[00:05:45.200,000] <inf> usp: ===== FSK Configuration =====
[00:05:45.200,000] <inf> usp: Modulation:       FSK
[00:05:45.200,000] <inf> usp: Frequency:        863100000 Hz (863.1 MHz)
[00:05:45.200,000] <inf> usp: TX Power:         14 dBm
[00:05:45.200,000] <inf> usp: Bitrate:          50000 bps (50.0 kbps)
[00:05:45.200,000] <inf> usp: Freq Deviation:   25000 Hz (25.0 kHz)
[00:05:45.200,000] <inf> usp: Bandwidth:        117.0 kHz
```

### FCC FHSS Sweep Mode

```
[00:10:30.200,000] <inf> usp: CONFIG Starting FHSS SWEEP mode - SF10/DR0
[00:10:30.205,000] <inf> usp: TX Scheduling LoRa transmission #3842
[00:10:30.240,000] <inf> usp: Frequency: 902500000 Hz
[00:10:30.275,000] <inf> usp: TX Scheduling LoRa transmission #3843
[00:10:30.310,000] <inf> usp: Frequency: 905300000 Hz
[00:10:30.345,000] <inf> usp: TX Scheduling LoRa transmission #3844
[00:10:30.380,000] <inf> usp: Frequency: 907900000 Hz
[00:10:30.415,000] <inf> usp: TX Scheduling LoRa transmission #3845
...
[00:15:45.200,000] <inf> usp: CONFIG FCC certification test sequence completed
[00:15:45.200,000] <inf> usp: ===== Transmission stopped by user =====
[00:15:45.200,000] <inf> usp: Total packets sent: 4256
[00:15:45.200,000] <inf> usp: Transmission sequence completed
```

### ARIB with LBT

```
[00:00:05.200,000] <inf> usp: Frequency:        920600000 Hz (920.6 MHz)
[00:00:05.200,000] <inf> usp: TX Power:         9 dBm
[00:00:05.200,000] <inf> usp: LBT:              Enabled (5ms, -80dBm threshold)
[00:00:05.205,000] <inf> usp: TX Scheduling LoRa transmission #1
[00:00:05.215,000] <inf> usp: LBT: Channel clear, transmitting
[00:00:05.250,000] <inf> usp: TX Scheduling LoRa transmission #2
```

## Architecture

The example is built following RAC's transaction-based architecture:

- **main_rf_certification.c**: Main initialization, RAC setup, and event loop
- **app_rf_certification.c**: Application logic for RF certification
- **app_rf_certification.h**: Configuration parameters and public interfaces

This separation allows for clean code organization and easy maintenance.

## Technical Notes

- **State Machine**: Each region implements a comprehensive test sequence
- **Packet Structure**: Includes "RF_CERT" header and incremental packet counter
- **Timing**: Minimal inter-packet delay (0ms) for maximum throughput testing
- **Memory Usage**: 255-byte payload requires sufficient RAM allocation
- **Thread Priority**: Uses cooperative threading for predictable timing

## License

This software is provided under The Clear BSD License.
Copyright Semtech Corporation 2025. All rights reserved.

## Support

For questions and support, please refer to the Semtech documentation or contact technical support.

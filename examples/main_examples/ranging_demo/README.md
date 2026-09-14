# Ranging Demo

This application demonstrates **high-precision distance measurement** between two LoRa devices using Time-of-Flight (ToF) ranging with frequency hopping. One device acts as a manager coordinating the ranging process, while the other acts as a subordinate responding to ranging requests. The system performs measurements across multiple frequency channels and computes a median distance for improved accuracy.

## Key Features

- **Frequency Hopping**: Uses different channels for robust measurements
- **Statistical Processing**: Computes median distance from multiple measurements
- **OLED Display Support**: Visual feedback with real-time distance and RF parameters
- **LED Indicators**: Visual status indication for TX/RX and ranging operations
- **Multiple Data Rates**: Optional support for different spreading factors and bandwidths
- **Continuous Mode**: Optional automatic continuous ranging
- **Periodic Uplink**: Optional periodic LoRa transmissions for network connectivity

### Operation Modes

- **Manager Mode**: Initiates ranging exchanges, coordinates frequency hopping, and calculates results
- **Subordinate Mode**: Responds to ranging requests and participates in frequency hopping sequence

### Ranging Process

The ranging process involves several phases:
1. **Configuration Exchange**: Manager sends ranging parameters to subordinate (using LoRa)
2. **Acknowledgment**: Subordinate confirms reception and readiness
3. **Frequency Hopping**: Both devices perform ranging on multiple channels
4. **Result Processing**: Manager computes median distance from all measurements

## Configuration

### Using CMake

| Parameter                     | Default Value                   | Description                                           |
|-------------------------------|---------------------------------|-------------------------------------------------------|
| `RANGING_DEVICE_MODE`         | `1`                             | Device role: Subordinate (1) or Manager (2)           |
| `RF_FREQ_IN_HZ`               | `868100000`                     | Base operating frequency in Hz                        |
| `TX_OUTPUT_POWER_DBM`         | `14`                            | Transmit power in dBm                                 |
| `LORA_SPREADING_FACTOR`       | `RAL_LORA_SF9`                  | LoRa spreading factor                                 |
| `LORA_BANDWIDTH`              | `RAL_LORA_BW_500_KHZ`           | LoRa bandwidth (500 kHz)                              |
| `LORA_CODING_RATE`            | `RAL_LORA_CR_4_5`               | LoRa coding rate (4/5)                                |
| `LORA_PREAMBLE_LENGTH`        | `12`                            | Preamble length in symbols (critical for timing)      |
| `LORA_PKT_LEN_MODE`           | `RAL_LORA_PKT_EXPLICIT`         | Packet length mode                                    |
| `LORA_IQ`                     | `false`                         | IQ inversion (keep standard for calibration)          |
| `LORA_CRC`                    | `true`                          | Enable CRC                                            |
| `LORA_SYNCWORD`               | `LORA_PRIVATE_NETWORK_SYNCWORD` | LoRa sync word                                        |
| `ACTIVATE_MULTIPLE_DATA_RATE` | `false`                         | Enable multiple data rate testing                     |
| `CONTINUOUS_RANGING`          | `false`                         | Enable continuous ranging mode                        |
| `PERIODIC_UPLINK_ENABLED`     | `false`                         | Enable periodic uplink transmissions                  |
| `TX_PERIODICITY_IN_MS`        | `10000`                         | Periodic uplink interval (10 seconds)                 |
| `PAYLOAD_LENGTH`              | `7`                             | Ranging payload length in bytes                       |
| `RANGING_ADDR_1`              | `0x32101222`                    | Ranging address value to ease concurent ranging tests |


### Hardware Requirements

- **OLED Display**: Optional SSD1306 I2C display for visual feedback
- **User Button**: Required for manual ranging initiation
- **LEDs**: TX/RX status indicators

## Compilation

Notes: 
- The USP version do not manage the I2C screen.
- The output of the USP version is not managed as json format

**Build manager (manager) device:**
```bash
rm -Rf build/ ; env CFLAGS="-DCONTINUOUS_RANGING=false" cmake -L -S examples/main_examples/ranging_demo -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -UCMAKE_C_FLAGS -G Ninja; cmake --build build --target rttof_manager
```

**Build subordinate (subordinate) device:**
```bash
rm -Rf build/ ; env CFLAGS="-DCONTINUOUS_RANGING=false" cmake -L -S examples/main_examples/ranging_demo -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -UCMAKE_C_FLAGS -G Ninja; cmake --build build --target rttof_subordinate
```

**Example of `openocd`command to flash ranging manager:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/rttof_manager verify reset exit"
```

## Usage

1. **Setup Two Devices**:
   - Flash one device as manager (`RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_MANAGER`)
   - Flash second device as subordinate (`RANGING_DEVICE_MODE=RANGING_DEVICE_MODE_SUBORDINATE`)

2. **Position Devices**: Place devices at desired measurement distance

3. **Power On**: Start both devices (subordinate first recommended)

4. **Initiate Ranging**:
   - **Manual Mode**: Press user button on manager device
   - **Continuous Mode**: Ranging starts automatically if enabled

5. **Monitor Results**:
   - **UART Output**: Distance measurements and statistics
   - **OLED Display**: Real-time distance, SF, and BW (if available)
   - **LEDs**: TX/RX activity indication

6. **Distance Calculation**: Manager performs measurements across all frequency channels and computes median distance

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Manager device output

```
*** Booting Zephyr OS build vx.y.z ***
start oled_init()
[00:00:00.007,216] <inf> usp: Starting loop...
[00:00:00.372,926] <inf> usp: ===== ranging and frequency hopping example =====
[00:00:00.410,983] <inf> lorawan: Defined Hook IDs:
[00:00:00.411,017] <inf> lorawan: RP_HOOK_ID_SUSPEND: 0
[00:00:00.411,047] <inf> lorawan: RP_HOOK_RAC_VERY_HIGH_PRIORITY: 1
[00:00:00.411,078] <inf> lorawan: RP_HOOK_RAC_HIGH_PRIORITY: 2
[00:00:00.411,109] <inf> lorawan: RP_HOOK_RAC_MEDIUM_PRIORITY: 3
[00:00:00.411,139] <inf> lorawan: RP_HOOK_RAC_LOW_PRIORITY: 8
[00:00:00.411,183] <inf> lorawan: RP_HOOK_RAC_VERY_LOW_PRIORITY: 9
[00:00:00.411,213] <inf> lorawan: RP_HOOK_ID_LR1MAC_STACK: 4
[00:00:00.411,243] <inf> lorawan: RP_HOOK_ID_LBT: 5
[00:00:00.411,273] <inf> lorawan: RP_HOOK_ID_TEST_MODE: 6
[00:00:00.411,304] <inf> lorawan: RP_HOOK_ID_DIRECT_RP_ACCESS: 7
[00:00:00.411,336] <inf> lorawan: RP_HOOK_ID_MAX: 10
[00:00:00.412,392] <inf> usp: Running in ranging manager mode
button_pressed[00:00:04.121,822] <inf> usp: Button pushed


{
"SF": "RAL_LORA_SF9",
"BW": "RAL_LORA_BW_500_KHZ",
"ROLE": "MANAGER",
"LoRa RSSI": "-65 dBm",
"LoRa SNR": -5,
"RngResult": {
	"Num": 29,
	"Results": [
		{"FreqIndex": "0", "Freq": "863.75 MHz", "RawDistance": "0x0000002a", "Distance": "3 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "1", "Freq": "865.10 MHz", "RawDistance": "0x00000022", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "2", "Freq": "864.80 MHz", "RawDistance": "0x00fffff2", "Distance": "-1 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "3", "Freq": "868.40 MHz", "RawDistance": "0x0000003d", "Distance": "4 m", "RSSI": "-64 dBm" },
		{"FreqIndex": "4", "Freq": "865.25 MHz", "RawDistance": "0x0000001d", "Distance": "2 m", "RSSI": "-70 dBm" },
		{"FreqIndex": "5", "Freq": "867.50 MHz", "RawDistance": "0x00000026", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "6", "Freq": "865.55 MHz", "RawDistance": "0x00000026", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "7", "Freq": "867.65 MHz", "RawDistance": "0x0000001e", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "8", "Freq": "866.15 MHz", "RawDistance": "0x00000027", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "9", "Freq": "864.05 MHz", "RawDistance": "0x00000024", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "10", "Freq": "864.50 MHz", "RawDistance": "0x00000024", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "11", "Freq": "866.45 MHz", "RawDistance": "0x00000024", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "12", "Freq": "865.40 MHz", "RawDistance": "0x00fffff9", "Distance": "0 m", "RSSI": "-70 dBm" },
		{"FreqIndex": "13", "Freq": "868.70 MHz", "RawDistance": "0x0000002e", "Distance": "3 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "14", "Freq": "863.15 MHz", "RawDistance": "0x00000024", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "15", "Freq": "866.75 MHz", "RawDistance": "0x0000002b", "Distance": "3 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "16", "Freq": "866.30 MHz", "RawDistance": "0x00000024", "Distance": "2 m", "RSSI": "-68 dBm" },
		{"FreqIndex": "17", "Freq": "864.95 MHz", "RawDistance": "0x0000002b", "Distance": "3 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "18", "Freq": "864.35 MHz", "RawDistance": "0x00000028", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "19", "Freq": "866.00 MHz", "RawDistance": "0x00000025", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "20", "Freq": "866.90 MHz", "RawDistance": "0x0000001f", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "21", "Freq": "868.25 MHz", "RawDistance": "0x00000023", "Distance": "2 m", "RSSI": "-68 dBm" },
		{"FreqIndex": "22", "Freq": "865.85 MHz", "RawDistance": "0x00000026", "Distance": "2 m", "RSSI": "-68 dBm" },
		{"FreqIndex": "23", "Freq": "865.70 MHz", "RawDistance": "0x00000028", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "24", "Freq": "867.35 MHz", "RawDistance": "0x00000022", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "25", "Freq": "868.10 MHz", "RawDistance": "0x00000042", "Distance": "4 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "26", "Freq": "863.60 MHz", "RawDistance": "0x00000028", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "27", "Freq": "866.60 MHz", "RawDistance": "0x00000027", "Distance": "2 m", "RSSI": "-69 dBm" },
		{"FreqIndex": "28", "Freq": "864.20 MHz", "RawDistance": "0x00000028", "Distance": "2 m", "RSSI": "-69 dBm" }
		],
	"DistanceRng": "2 m",
	"PER": "0 %"
	}
},

```

### Subordinate device output

```
*** Booting Zephyr OS build vx.y.z ***
[00:00:00.000,000] <inf> usp: ===== ranging and frequency hopping example =====
[00:00:00.000,000] <inf> usp: Starting loop...
[00:00:00.254,000] <inf> lorawan: Defined Hook IDs:
[00:00:00.254,000] <inf> lorawan: RP_HOOK_ID_SUSPEND: 0
[00:00:00.254,000] <inf> lorawan: RP_HOOK_RAC_VERY_HIGH_PRIORITY: 1
[00:00:00.254,000] <inf> lorawan: RP_HOOK_RAC_HIGH_PRIORITY: 2
[00:00:00.254,000] <inf> lorawan: RP_HOOK_RAC_MEDIUM_PRIORITY: 3
[00:00:00.254,000] <inf> lorawan: RP_HOOK_RAC_LOW_PRIORITY: 8
[00:00:00.254,000] <inf> lorawan: RP_HOOK_RAC_VERY_LOW_PRIORITY: 9
[00:00:00.254,000] <inf> lorawan: RP_HOOK_ID_LR1MAC_STACK: 4
[00:00:00.254,000] <inf> lorawan: RP_HOOK_ID_LBT: 5
[00:00:00.254,000] <inf> lorawan: RP_HOOK_ID_TEST_MODE: 6
[00:00:00.255,000] <inf> lorawan: RP_HOOK_ID_DIRECT_RP_ACCESS: 7
[00:00:00.255,000] <inf> lorawan: RP_HOOK_ID_MAX: 10
[00:00:00.255,000] <inf> usp: Running in ranging subordinate mode

{
"SF": "RAL_LORA_SF9",
"BW": "RAL_LORA_BW_500_KHZ",
"ROLE": "SUBORDINATE",
"LoRa RSSI": "-62 dBm",
"LoRa SNR": 7,
"RngResult": {
	"Num": 0
	}
},



```

## Troubleshooting

### Common Issues

1. **No Ranging Results**:
   - Check device positioning (line of sight recommended)
   - Verify both devices use same configuration
   - Ensure adequate distance (minimum ~1 meter)

2. **Timeout Errors**:
   - Check RF environment for interference
   - Verify antenna connections
   - Try different frequency or power settings

3. **Inconsistent Results**:
   - Ensure stable device positioning
   - Check for RF reflections or multipath
   - Consider environmental factors (weather, obstacles)

4. **OLED Display Issues**:
   - Verify I2C connections and address
   - Check display power supply
   - Ensure SSD1306 driver configuration

## Technical Notes

- **Timing Critical**: Preamble length and IQ settings affect ranging accuracy
- **Calibration**: Uses factory calibration tables for optimal performance
- **Range**: Typically accurate from 1m to several kilometers depending on conditions
- **Precision**: Can achieve sub-meter accuracy under ideal conditions

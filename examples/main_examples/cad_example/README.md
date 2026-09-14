# CAD (Channel Activity Detection)

This application demonstrates **Channel Activity Detection (CAD)** for detecting LoRa signal activity without receiving the full packet. CAD is a low-power feature that allows the radio to quickly detect whether a LoRa preamble is present on the channel, making it ideal for Listen Before Talk (LBT), receiver wake-up, and energy-efficient operation. The application uses button-triggered periodic CAD operations with detailed statistics reporting.

## Key Features

- **Channel Activity Detection**: Fast detection of LoRa preambles on the channel
- **Multiple CAD Modes**: Support for CAD-only, CAD-to-RX, and CAD-to-TX modes
- **Button-Controlled Operation**: Manual trigger for periodic CAD operations
- **Detailed Statistics**: Tracks positive/negative detections and success rates
- **Pre/Post Transaction Callbacks**: Fine-grained control over CAD operations
- **LED Feedback**: Visual TX activity indication during CAD operations
- **Continuous Monitoring**: Automatic re-scheduling after each CAD operation

## Configuration

| Parameter               | Default Value           | Description                                |
|-------------------------|-------------------------|--------------------------------------------|
| `CAD_DELAY_MS`          | `1000`                  | Delay between CAD operations in ms         |
| `CAD_DURATION_MS`       | `2000`                  | Maximum duration for CAD operation         |
| `TYPE_OF_CAD`           | `RAL_LORA_CAD_RX`       | CAD mode: ONLY/RX/LBT (see below)          |
| `RF_FREQ_IN_HZ`         | `868100000`             | Operating frequency in Hz (868.1 MHz)      |
| `LORA_SYNCWORD`         | `0x34`                  | LoRa synchronization word                  |
| `LORA_SPREADING_FACTOR` | `RAL_LORA_SF7`          | Spreading factor (SF7)                     |
| `LORA_BANDWIDTH`        | `RAL_LORA_BW_125_KHZ`   | Bandwidth (125 kHz)                        |
| `LORA_CODING_RATE`      | `RAL_LORA_CR_4_5`       | Coding rate (4/5)                          |
| `LORA_IQ`               | `true`                  | IQ inversion disabled                      |
| `TX_OUTPUT_POWER_DBM`   | `14`                    | Output power in dBm (for CAD-to-TX mode)   |
| `PAYLOAD_SIZE`          | `255`                   | Payload size in bytes (for CAD-to-TX mode) |
| `LORA_PREAMBLE_LENGTH`  | `255`                   | LoRa preamble length in symbols            |
| `LORA_CRC`              | `false`                 | CRC enabled                                |
| `LORA_PKT_LEN_MODE`     | `RAL_LORA_PKT_EXPLICIT` | Explicit header mode                       |

### CAD Modes

The `TYPE_OF_CAD` parameter determines the behavior after CAD detection:

- **`RAL_LORA_CAD_ONLY`**: (default) Perform CAD and stop the radio
  - Used for simple channel sensing
  - Lowest power consumption
  - Returns `RP_STATUS_CAD_POSITIVE` or `RP_STATUS_CAD_NEGATIVE`

- **`RAL_LORA_CAD_RX`**: Perform CAD, then receive if activity detected
  - Used for receiver wake-up and energy-efficient reception
  - Automatically switches to RX on positive CAD
  - Returns `RP_STATUS_RX_PACKET` or `RP_STATUS_RX_TIMEOUT` on positive CAD

- **`RAL_LORA_CAD_LBT`**: Perform CAD, then transmit if channel is clear
  - Used for Listen Before Talk (LBT) implementations
  - Automatically switches to TX on negative CAD
  - Returns `RP_STATUS_TX_DONE` on successful transmission

## Compilation

**Build sample: TYPE_OF_CAD=RAL_LORA_[CAD_LBT|CAD_ONLY|CAD_RX]**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/cad_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -DTYPE_OF_CAD=RAL_LORA_CAD_LBT -G Ninja; cmake --build build --target cad
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/cad verify reset exit"
```

## Usage

1. **Hardware Requirements**:
   - LR20XX radio module
   - Board with user button support

2. **Flash Firmware**: Load the compiled firmware to your device

3. **Connect Console**: Open serial console to monitor CAD activity
   ```bash
   minicom -D /dev/ttyACM0 -b 115200
   ```

4. **Start CAD Operations**: Press the user button to initiate periodic CAD operations

5. **Monitor Activity**:
   - LED indicates TX activity during CAD operations
   - UART shows detailed CAD statistics including positive/negative counts
   - Each CAD result is logged with cumulative statistics

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Initialization
```
[00:00:01.000,000] <inf> usp: ===== CAD (Channel Activity Detection) example =====
[00:00:01.100,000] <inf> usp: CAD initialized - press button to start CAD operation
```

### Button Press and CAD Operations (CAD_RX Mode)
```
[00:00:05.200,000] <inf> usp: Button pushed
[00:00:05.300,000] <inf> usp: Button launched periodic CAD operation
[00:00:06.400,000] <inf> usp: >>> CAD #1: NEGATIVE ( 0 positive, 1 negative, Last positive: #0)
[00:00:07.500,000] <inf> usp: >>> CAD #2: NEGATIVE ( 0 positive, 2 negative, Last positive: #0)
[00:00:08.600,000] <inf> usp: >>> CAD #3: POSITIVE ( 1 positive, 2 negative, Last positive: #3)
[00:00:09.700,000] <inf> usp: >>> CAD #4: RX PACKET ( 2 positive, 2 negative, Last positive: #4)
[00:00:10.800,000] <inf> usp: >>> CAD #5: RX TIMEOUT ( 3 positive, 2 negative, Last positive: #5)
```

### CAD Statistics Interpretation

- **CAD #N**: Sequential CAD operation number
- **POSITIVE**: Preamble detected on the channel
- **NEGATIVE**: No preamble detected on the channel
- **RX PACKET**: Packet successfully received (in CAD_RX mode after positive detection)
- **RX TIMEOUT**: No packet received within timeout (in CAD_RX mode after positive detection)
- **TX DONE**: Transmission completed (in CAD_LBT mode after negative detection)
- **Last positive**: CAD number of the most recent positive detection

## Technical Notes

- **Low Power Operation**: CAD is significantly more power-efficient than continuous RX
- **Fast Detection**: CAD completes in approximately 1-4 symbol periods
- **Automatic Re-scheduling**: Each CAD operation automatically schedules the next one
- **High Priority**: CAD operations use `RAC_VERY_HIGH_PRIORITY` for minimal latency
- **Transaction Callbacks**: Pre-callback enables TX LED, post-callback handles results and statistics
- **Configurable CAD Symbols**: Uses 4 symbol CAD (`RAL_LORA_CAD_04_SYMB`) for detection

## Limitations

- **LoRa Only**: CAD is specific to LoRa modulation (not available for FSK/GFSK)
- **Preamble Detection**: Detects preambles only, not packet validity
- **False Positives**: Possible in high-noise environments
- **Parameter Matching**: CAD parameters must match expected signal (SF, BW, etc.)
- **Detection Range**: Limited to approximately 4 symbol periods

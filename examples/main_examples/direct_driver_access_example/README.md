# Direct Driver Access

This application demonstrates **low-level radio driver access** by bypassing the USP/RAC abstraction layers to directly control LR20xx radio chips. It provides maximum flexibility for custom radio configurations and advanced features not available through standard APIs. The application uses button-triggered transmissions with direct LR20xx driver calls for precise radio parameter control.

## Key Features

- **Direct LR20xx Driver Access**: Bypasses RAC abstraction for maximum control
- **Button-Controlled Transmission**: Manual trigger for direct radio operations
- **Custom Radio Configuration**: Direct parameter setting via LR20xx APIs
- **Pre/Post Transaction Callbacks**: Fine-grained control over radio operations
- **LED Feedback**: Visual TX activity indication
- **LR20xx Specific**: Optimized for LR20xx chip family with compile-time validation

## Configuration

| Parameter                          | Default Value                   | Description                           |
|------------------------------------|---------------------------------|---------------------------------------|
| `RF_FREQ_IN_HZ`                    | `868100000`                     | Operating frequency in Hz (868.1 MHz) |
| `OUT_POWER_IN_DBM`                 | `14`                            | Output power in dBm                   |
| `LORA_PAYLOAD_LENGTH`              | `10`                            | LoRa payload size in bytes            |
| `LORA_PREAMBLE_LENGTH`             | `12`                            | LoRa preamble length in symbols       |
| `LORA_SYNCWORD`                    | `0x34`                          | LoRa synchronization word             |
| `DIRECT_DRIVER_ACCESS_DELAY_MS`    | `100`                           | Delay before transmission starts      |
| `DIRECT_DRIVER_ACCESS_DURATION_MS` | `2000`                          | Maximum transaction duration          |
| `LORA_SF`                          | `LR20XX_RADIO_LORA_SF8`         | Spreading factor (SF8)                |
| `LORA_BW`                          | `LR20XX_RADIO_LORA_BW_125`      | Bandwidth (125 kHz)                   |
| `LORA_CR`                          | `LR20XX_RADIO_LORA_CR_4_5`      | Coding rate (4/5)                     |
| `LORA_CRC`                         | `LR20XX_RADIO_LORA_CRC_ENABLED` | CRC enabled                           |
| `LORA_IQ`                          | `LR20XX_RADIO_LORA_IQ_STANDARD` | Standard IQ polarity                  |

## Compilation

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/direct_driver_access_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target direct_driver_access
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/direct_driver_access verify reset exit"
```


## Usage

1. **Hardware Requirements**:
   - LR20xx-based radio module (LR2021/LR2021W)
   - Board with user button support

2. **Flash Firmware**: Load the compiled firmware to your device

3. **Connect Console**: Open serial console to monitor transmission activity
   ```bash
   minicom -D /dev/ttyACM0 -b 115200
   ```

4. **Trigger Transmission**: Press the user button to initiate direct driver transmission

5. **Monitor Activity**:
   - LED indicates  TX activity during transmissions
   - UART shows detailed radio configuration and transaction status

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Initialization
```
[00:00:01.000,000] <inf> usp: ===== Direct Driver Access example =====
[00:00:01.100,000] <inf> usp: Direct driver access initialized - press button to start/stop transmission
```

### Button Press and Direct Radio Access
```
[00:00:05.200,000] <inf> usp: Button pushed
[00:00:05.300,000] <inf> usp: usp/rac: provide direct radio access
[00:00:05.350,000] <inf> usp: usp/rac: transaction (transmission) has ended (success)
```

## Technical Notes

- **LR20xx Only**: This example is specifically designed for LR20xx chips and includes compile-time validation
- **Direct API Access**: Bypasses RAC abstraction for maximum performance and flexibility
- **Transaction Callbacks**: Pre-callback configures radio, post-callback handles completion
- **LED Integration**: TX LED provides visual feedback during transmission
- **Custom Parameters**: All radio parameters directly configurable via LR20xx driver APIs

## Limitations

- **LR20xx Specific**: Not portable to other radio chips without modification
- **Lower-Level Complexity**: Requires detailed knowledge of LR20xx driver APIs
- **No Abstraction Benefits**: Manual management of radio state and parameters
- **Development Complexity**: More complex than standard RAC-based applications

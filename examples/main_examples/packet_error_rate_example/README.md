# Packet Error Rate (LoRa) Example

This application performs **Packet Error Rate (PER) testing** between two devices to evaluate radio link quality and reliability using LoRa modulation. One device acts as a transmitter sending numbered packets, while the other acts as a receiver counting successful receptions and calculating error rates in real-time.

## Key Features

- **Dual Role Operation**: Configurable as either transmitter or receiver at compile time
- **Structured Payload**: Packets contain header ("PER"), separator, and incremental counter
- **Real-time Statistics**: Tracks exchange count, failure count, and calculates PER during test
- **Series-based Testing**: Runs batches of 100 exchanges with inter-series delays
- **Payload Validation**: Verifies header integrity and counter sequence
- **Missing Packet Detection**: Automatically detects and counts skipped packets
- **LED Feedback**: Visual indication of TX/RX activity

### Operation Modes

- **Transmitter Mode**: Sends numbered packets sequentially with configurable delays
- **Receiver Mode**: Receives packets, validates payload, and calculates PER statistics

### Protocol Details

The PER test uses an 8-byte structured payload:
- **Header**: "PER" (3 bytes)
- **Separator**: 0x00 (1 byte)
- **Counter**: 32-bit exchange counter (4 bytes)

## Configuration

### Using CMake

| Parameter               | Default Value                  | Description                                      |
|-------------------------|--------------------------------|--------------------------------------------------|
| `ROLE`                  | *Required*                     | Device role: `RECEIVER` (1) or `TRANSMITTER` (2) |
| `MAX_EXCHANGE_COUNT`    | `100`                          | Number of packets per test series                |
| `RX_TIMEOUT`            | `30000`                        | Reception timeout in ms                          |
| `INTER_EXCHANGE_DELAY`  | `20`                           | Delay between packets in ms                      |
| `INTER_SERIES_DELAY`    | `10000`                        | Delay between test series in ms                  |
| `RF_FREQ_IN_HZ`         | `868100000`                    | Operating frequency in Hz                        |
| `TX_OUTPUT_POWER_DBM`   | `14`                           | Transmit power in dBm                            |
| `LORA_SPREADING_FACTOR` | `RAL_LORA_SF10`                | LoRa spreading factor                            |
| `LORA_BANDWIDTH`        | `RAL_LORA_BW_500_KHZ`          | LoRa bandwidth                                   |
| `LORA_CODING_RATE`      | `RAL_LORA_CR_4_5`              | LoRa coding rate                                 |
| `LORA_PREAMBLE_LENGTH`  | `8`                            | Preamble length in symbols                       |
| `LORA_PKT_LEN_MODE`     | `RAL_LORA_PKT_EXPLICIT`        | Packet length mode                               |
| `LORA_IQ`               | `false`                        | LoRa IQ inversion setting                        |
| `LORA_CRC`              | `true`                         | Enable/disable CRC                               |
| `LORA_SYNCWORD`         | `LORA_PUBLIC_NETWORK_SYNCWORD` | Synchronization word                             |

## Compilation

**Build receiver:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/packet_error_rate_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target per_rx
```

**Build transmitter:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/packet_error_rate_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target per_tx
```

```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/per_rx verify reset exit"
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/per_tx verify reset exit"
```

## Usage

1. Compile and flash receiver firmware to the second device using `ROLE=RECEIVER`
2. Compile and flash transmitter firmware to the first device using `ROLE=TRANSMITTER`
3. Start the receiver first - it will wait for packets
4. Start the transmitter - it will begin sending numbered packets automatically
5. Monitor UART debug output on both devices to observe test progress
6. After 100 packets, statistics are displayed and a new series begins automatically

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Transmitter device output

```
[00:00:00.036,000] <inf> usp: ===== PER example =====
[00:00:00.040,000] <inf> usp: ===== SEMTECH PER TEST INITIALIZED =====
[00:00:00.040,000] <inf> usp: Device Role: TRANSMITTER
[00:00:00.040,000] <inf> usp: Mode: Continuous packet transmission for PER testing
[00:00:00.045,000] <inf> usp: usp/rac: transaction is starting
[00:00:00.186,000] <inf> usp: usp/rac: new event: transaction (transmission) has ended (success)
[00:00:00.186,000] <inf> usp: TX Progress: 1/100 packets sent successfully
[00:00:00.206,000] <inf> usp: usp/rac: transaction is starting
[00:00:00.347,000] <inf> usp: usp/rac: new event: transaction (transmission) has ended (success)
[00:00:00.347,000] <inf> usp: TX Progress: 2/100 packets sent successfully
[00:00:02.027,000] <inf> usp: TX Progress: 50/100 packets sent successfully
[00:00:04.047,000] <inf> usp: ===== PER TEST SERIES COMPLETED =====
[00:00:04.047,000] <inf> usp: Total Packets: 100 | Success: 100 | PER: 0.00%
```

### Receiver device output

```
[00:00:00.036,000] <inf> usp: ===== PER example =====
[00:00:00.040,000] <inf> usp: ===== SEMTECH PER TEST INITIALIZED =====
[00:00:00.040,000] <inf> usp: Device Role: RECEIVER
[00:00:00.040,000] <inf> usp: Mode: Continuous packet reception for PER measurement
[00:00:00.045,000] <inf> usp: usp/rac: transaction is starting
[00:00:01.186,000] <inf> usp: usp/rac: new event: transaction (reception) has ended (success)
[00:00:01.186,000] <inf> usp: Valid packet received with counter: 0
[00:00:01.186,000] <inf> usp: RX Progress: 1/100 packets received successfully
[00:00:01.186,000] <inf> usp: Current Progress: 1/100 | Failures: 0 | PER: 0.00% | Success Rate: 100.00%
[00:00:01.347,000] <inf> usp: usp/rac: new event: transaction (reception) has ended (success)
[00:00:01.347,000] <inf> usp: Valid packet received with counter: 1
[00:00:01.347,000] <inf> usp: RX Progress: 2/100 packets received successfully
[00:00:01.347,000] <inf> usp: Current Progress: 2/100 | Failures: 0 | PER: 0.00% | Success Rate: 100.00%
[00:00:05.047,000] <inf> usp: ===== PER TEST SERIES COMPLETED =====
[00:00:05.047,000] <inf> usp: Final Results: Total: 100 | Failed: 2 | PER: 2.00% | Success Rate: 98.00%
```
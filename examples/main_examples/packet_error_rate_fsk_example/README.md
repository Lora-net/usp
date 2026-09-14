# Packet Error Rate (FSK) Example

This application performs **Packet Error Rate (PER) testing** between two devices to evaluate radio link quality and reliability using FSK modulation. One device acts as a transmitter sending numbered packets, while the other acts as a receiver counting successful receptions and calculating error rates in real-time.

## Key Features

- **Dual Role Operation**: Configurable as either transmitter or receiver at compile time
- **Structured Payload**: Packets contain header ("FSK"), separator, and incremental counter
- **Real-time Statistics**: Tracks exchange count, failure count, and calculates PER during test
- **Series-based Testing**: Runs batches of 100 exchanges with inter-series delays
- **Payload Validation**: Verifies header integrity and counter sequence
- **Missing Packet Detection**: Automatically detects and counts skipped packets
- **LED Feedback**: Visual indication of TX/RX activity
- **CRC Error Tracking**: Separate tracking of CRC failures vs payload corruption

### Operation Modes

- **Transmitter Mode**: Sends numbered packets sequentially with configurable delays
- **Receiver Mode**: Receives packets, validates payload, and calculates PER statistics

### Protocol Details

The PER test uses an 8-byte structured payload:
- **Header**: "FSK" (3 bytes)
- **Separator**: 0x00 (1 byte)
- **Counter**: 32-bit exchange counter (4 bytes)

## Configuration

### Using CMake

| Parameter                        | Default Value              | Description                                       |
|----------------------------------|----------------------------|---------------------------------------------------|
| `ROLE`                           | *Required*                 | Device role: `RECEIVER` (1) or `TRANSMITTER` (2)  |
| `MAX_EXCHANGE_COUNT`             | `100`                      | Number of packets per test series                 |
| `RX_TIMEOUT`                     | `30000`                    | Reception timeout in ms                           |
| `INTER_EXCHANGE_DELAY`           | `0` (RX) / `300` (TX)      | Delay between packets in ms                       |
| `INTER_SERIES_DELAY_TRANSMITTER` | `5000`                     | Delay between test series in ms for `TRANSMITTER` |
| `RF_FREQ_IN_HZ`                  | `866500000`                | Operating frequency in Hz                         |
| `TX_OUTPUT_POWER_DBM`            | `14`                       | Transmit power in dBm                             |
| `FSK_BITRATE`                    | `50000`                    | FSK bitrate in bps (50 kbps)                      |
| `FSK_FDEV`                       | `25000`                    | Frequency deviation in Hz (25 kHz)                |
| `FSK_BANDWIDTH`                  | `138000`                   | FSK bandwidth in Hz (138 kHz)                     |
| `FSK_PREAMBLE_LENGTH`            | `5`                        | Preamble length in bytes                          |
| `FSK_SYNC_WORD_LENGTH`           | `3`                        | Sync word length in bytes                         |
| `FSK_CRC`                        | `RAL_GFSK_CRC_2_BYTES_INV` | CRC type (2-byte inverted CRC)                    |
| `FSK_WHITENING`                  | `true`                     | Enable/disable data whitening                     |
| `FSK_PACKET_TYPE`                | `RAL_GFSK_PKT_VAR_LEN`     | Packet length mode (variable length)              |

## Compilation

**Build receiver:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/packet_error_rate_fsk_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target per_fsk_rx
```

**Build transmitter:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/packet_error_rate_fsk_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target per_fsk_tx
```

```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/per_fsk_rx verify reset exit"
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
[00:00:00.036,000] <inf> usp: ===== PER FSK example =====
[00:00:00.040,000] <inf> usp: ========================================
[00:00:00.040,000] <inf> usp:      FSK PACKET ERROR RATE (PER) TEST
[00:00:00.040,000] <inf> usp: ========================================
[00:00:00.040,000] <inf> usp: Role: TRANSMITTER
[00:00:00.045,000] <inf> usp: Starting FSK transmission #1
[00:00:00.045,000] <inf> usp: Payload: 'FSK' + counter=0
[00:00:00.186,000] <inf> usp: FSK transmission completed successfully
[00:00:00.186,000] <inf> usp: TX Progress: 1/100 FSK packets sent
[00:00:00.686,000] <inf> usp: Starting FSK transmission #2
[00:00:00.686,000] <inf> usp: Payload: 'FSK' + counter=1
[00:00:00.827,000] <inf> usp: FSK transmission completed successfully
[00:00:00.827,000] <inf> usp: TX Progress: 2/100 FSK packets sent
```

### Receiver device output

```
[00:00:00.036,000] <inf> usp: ===== PER FSK example =====
[00:00:00.040,000] <inf> usp: ========================================
[00:00:00.040,000] <inf> usp:      FSK PACKET ERROR RATE (PER) TEST
[00:00:00.040,000] <inf> usp: ========================================
[00:00:00.040,000] <inf> usp: Role: RECEIVER
[00:00:00.045,000] <inf> usp: Starting FSK reception #1 (timeout: 30000 ms)
[00:00:00.045,000] <inf> usp: Awaiting FSK packet...
[00:00:01.186,000] <inf> usp: FSK packet received successfully
[00:00:01.186,000] <inf> usp: RSSI: -42 dBm
[00:00:01.186,000] <inf> usp: FSK payload valid - Header: 'FSK', Counter: 0
[00:00:01.186,000] <inf> usp: RX Progress: 1/100 FSK packets received successfully
[00:00:01.186,000] <inf> usp: === REAL-TIME FSK PER STATISTICS ===
[00:00:01.186,000] <inf> usp: Total FSK Packets: 1/100
[00:00:01.186,000] <inf> usp: Failed Packets: 0
[00:00:01.186,000] <inf> usp: Packet Error Rate: 0%
[00:00:01.186,000] <inf> usp: Success Rate: 100%
...
[00:00:51.047,000] <inf> usp: ========================================
[00:00:51.047,000] <inf> usp:        FSK PER TEST COMPLETED!
[00:00:51.047,000] <inf> usp: ========================================
[00:00:51.047,000] <inf> usp: Final FSK PER RESULTS:
[00:00:51.047,000] <inf> usp: Total Packets Processed: 100
[00:00:51.047,000] <inf> usp: Successful Packets: 98
[00:00:51.047,000] <inf> usp: Failed Packets: 2
[00:00:51.047,000] <inf> usp: Final Packet Error Rate: 2%
[00:00:51.047,000] <inf> usp: Final Success Rate: 98%
[00:00:51.047,000] <inf> usp: Excellent FSK link quality!
```

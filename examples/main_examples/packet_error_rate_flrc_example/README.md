# Packet Error Rate (FLRC) Example

This application performs **Packet Error Rate (PER) testing** between two devices to evaluate radio link quality and reliability using FLRC (Fast Long Range Communication) modulation. One device acts as a transmitter sending numbered packets, while the other acts as a receiver counting successful receptions and calculating error rates in real-time.

## Key Features

- **Dual Role Operation**: Configurable as either transmitter or receiver at compile time
- **Structured Payload**: Packets contain header ("FLRC"), separator, and incremental counter
- **Real-time Statistics**: Tracks exchange count, failure count, and calculates PER during test
- **Series-based Testing**: Runs batches of 100 exchanges with inter-series delays
- **Payload Validation**: Verifies header integrity and counter sequence
- **Missing Packet Detection**: Automatically detects and counts skipped packets
- **LED Feedback**: Visual indication of TX/RX activity
- **CRC Error Tracking**: Separate tracking of CRC failures vs payload corruption
- **High-Speed Communication**: FLRC modulation for fast data transmission (2.6 Mbps)
- **Large Payload Support**: Uses 255-byte packets for comprehensive testing

### Operation Modes

- **Transmitter Mode**: Sends numbered packets sequentially with configurable delays
- **Receiver Mode**: Receives packets, validates payload, and calculates PER statistics

### Protocol Details

The PER test uses a structured payload with variable size (up to 255 bytes):
- **Header**: "FLRC" (4 bytes)
- **Separator**: 0x00 (1 byte)
- **Counter**: 32-bit exchange counter (4 bytes)
- **Data Pattern**: Incremental byte pattern for payload validation

## Configuration

### Using CMake

| Parameter              | Default Value                      | Description                                        |
|------------------------|------------------------------------|----------------------------------------------------|
| `ROLE`                 | *Required*                         | Device role: `RECEIVER` (1) or `TRANSMITTER` (2)   |
| `MAX_EXCHANGE_COUNT`   | `100`                              | Number of packets per test series                  |
| `RX_TIMEOUT`           | `400`                              | Reception timeout in ms (2 × INTER_EXCHANGE_DELAY) |
| `INTER_EXCHANGE_DELAY` | `200`                              | Delay between packets in ms                        |
| `INTER_SERIES_DELAY`   | `10000`                            | Delay between test series in ms                    |
| `RF_FREQ_IN_HZ`        | `866500000`                        | Operating frequency in Hz                          |
| `TX_OUTPUT_POWER_DBM`  | `14`                               | Transmit power in dBm                              |
| `FLRC_RAW_BIT_RATE`    | `RAL_FLRC_RAW_BIT_RATE_2_600_MBPS` | FLRC bitrate in bps (2.6 Mbps)                     |
| `FLRC_CR`              | `RAL_FLRC_CR_3_4`                  | FLRC coding rate (3/4)                             |
| `FLRC_PULSE_SHAPE`     | `RAL_FLRC_PULSE_SHAPE_BT_05`       | FLRC pulse shaping (BT=0.5)                        |
| `FLRC_PREAMBLE_BITS`   | `RAL_FLRC_PREAMBLE_LENGTH_32_BITS` | Preamble length in bits                            |
| `FLRC_SYNCWORD_LEN`    | `RAL_FLRC_SYNCWORD_LENGTH_4_BYTES` | Sync word length (4 bytes)                         |
| `FLRC_TX_SYNCWORD`     | `RAL_FLRC_TX_SYNCWORD_1`           | TX sync word selection                             |
| `FLRC_MATCH_SYNCWORD`  | `RAL_FLRC_RX_MATCH_SYNCWORD_1`     | RX sync word matching                              |
| `FLRC_PLD_IS_FIX`      | `false`                            | Variable length payload mode                       |
| `FLRC_CRC`             | `RAL_FLRC_CRC_2_BYTES`             | CRC type (2-byte CRC)                              |

## Compilation

**Build receiver:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/packet_error_rate_flrc_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target per_flrc_rx
```

**Build transmitter:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/packet_error_rate_flrc_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target per_flrc_tx
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/per_flrc_rx verify reset exit"
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/per_flrc_tx verify reset exit"
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
[00:00:00.040,000] <inf> usp: FLRC PER init
[00:00:00.040,000] <inf> usp: ===== FLRC Transaction #1 Complete =====
[00:00:00.040,000] <inf> usp: === TRANSMITTED PACKET DEBUG ===
[00:00:00.040,000] <inf> usp: TX Packet Structure:
[00:00:00.040,000] <inf> usp:   Header: 'FLR'
[00:00:00.040,000] <inf> usp:   Separator: 0x00
[00:00:00.040,000] <inf> usp:   Counter: 0
[00:00:00.040,000] <inf> usp: =================================
[00:00:00.040,000] <inf> usp: STATS TX Progress: 1/100 FLRC packets sent
[00:00:00.146,000] <inf> usp: ===== FLRC Transaction #2 Complete =====
[00:00:00.146,000] <inf> usp: STATS TX Progress: 2/100 FLRC packets sent
[00:00:10.547,000] <inf> usp: ========================================
[00:00:10.547,000] <inf> usp:        FLRC PER TEST COMPLETED!
[00:00:10.547,000] <inf> usp: ========================================
[00:00:10.547,000] <inf> usp: STATS FINAL FLRC PER RESULTS:
[00:00:10.547,000] <inf> usp: STATS Total Packets Processed: 100
[00:00:10.547,000] <inf> usp: STATS Final Packet Error Rate: 0%
[00:00:10.547,000] <inf> usp: Excellent FLRC link quality!
```

### Receiver device output

```
[00:00:00.036,000] <inf> usp: ===== PER example =====
[00:00:00.040,000] <inf> usp: FLRC PER init
[00:00:00.045,000] <inf> usp: ===== FLRC Transaction #1 Complete =====
[00:00:00.045,000] <inf> usp: RX FLRC packet received successfully RSSI: -45 dBm, SNR: 12 dB
[00:00:00.045,000] <inf> usp: RX Packet Size: 255 bytes
[00:00:00.045,000] <inf> usp: Perfect sequence: counter 0 as expected
[00:00:00.045,000] <inf> usp: STATS RX Progress: 1/100 FLRC packets received successfully
[00:00:00.151,000] <inf> usp: ===== FLRC Transaction #2 Complete =====
[00:00:00.151,000] <inf> usp: RX FLRC packet received successfully RSSI: -45 dBm, SNR: 12 dB
[00:00:00.151,000] <inf> usp: Perfect sequence: counter 1 as expected
[00:00:00.151,000] <inf> usp: STATS RX Progress: 2/100 FLRC packets received successfully
[00:00:10.547,000] <inf> usp: ========================================
[00:00:10.547,000] <inf> usp:        FLRC PER TEST COMPLETED!
[00:00:10.547,000] <inf> usp: ========================================
[00:00:10.547,000] <inf> usp: STATS FINAL FLRC PER RESULTS:
[00:00:10.547,000] <inf> usp: STATS Total Packets Processed: 100
[00:00:10.547,000] <inf> usp: STATS Successful Packets: 98
[00:00:10.547,000] <inf> usp: STATS Failed Packets: 2
[00:00:10.547,000] <inf> usp: STATS Final Packet Error Rate: 2%
[00:00:10.547,000] <inf> usp: STATS Final Success Rate: 98%
[00:00:10.547,000] <inf> usp: Excellent FLRC link quality!
[00:00:10.547,000] <inf> usp: *** RESTARTING NEW FLRC PER SERIES ***
```

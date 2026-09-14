# Ping Pong Example

This application demonstrates **bidirectional LoRa communication** between two devices using a ping-pong exchange protocol. The application features automatic role management where devices can dynamically switch between manager and subordinate modes, along with optional periodic LoRa uplink functionality for multi-protocol demonstration.

## Key Features

- **Dynamic Role Management**: Devices start as subordinates and can become managers by pressing the user button
- **Automatic Exchange Protocol**: Implements PING/PONG message exchange with payload validation
- **Error Handling & Retry Logic**: Includes timeout handling, CRC error recovery, and configurable retry limits
- **Payload Structure**: Uses structured payload with header ("PING"/"PONG"), separator, and counter
- **LED Indicators**: TX and RX LEDs provide visual feedback during transmissions
- **Optional Periodic LoRa Uplink**: If activated at compilation, a periodic LoRa TX is sent in parallel by the Manager

### Operation Modes

- **Manager Mode**: Initiates exchanges, adds delays between transactions, tracks successful/failed exchanges
- **Subordinate Mode**: Listens for PING messages and responds with PONG messages

### Protocol Details

The ping-pong protocol uses a structured 6-byte payload:
- **Header**: "PING" or "PONG" (4 bytes)
- **Separator**: 0x00 (1 byte)
- **Counter**: Exchange counter (1 byte)

## Configuration

### Using CMake

| Parameter                        | Default Value           | Description                                                         |
|----------------------------------|-------------------------|---------------------------------------------------------------------|
| `ENABLE_PERIODIC_UPLINK`         | `OFF`                   | Enable periodic LoRa uplink transmission in parallel with ping-pong |
| `SUBORDINATE_PROCESSING_TIME_MS` | `10`                    | Processing time delay for subordinate responses (ms)                |
| `MAX_EXCHANGE_COUNT`             | `25`                    | Maximum exchanges per session                                       |
| `MAX_RETRY_COUNT`                | `5`                     | Maximum retries on failure                                          |
| `DELAY`                          | `250`                   | Inter-exchange delay for manager (ms)                               |
| `MANAGER_RX_TIMEOUT`             | `500`                   | Receive timeout for manager (ms)                                    |
| `SUBORDINATE_RX_TIMEOUT`         | `30000`                 | Receive timeout for subordinate (ms)                                |
| `RF_FREQ_IN_HZ`                  | `868100000`             | Operating frequency in Hz                                           |
| `TX_OUTPUT_POWER_DBM`            | `14`                    | Transmit power in dBm                                               |
| `LORA_SPREADING_FACTOR`          | `RAL_LORA_SF9`          | LoRa spreading factor                                               |
| `LORA_BANDWIDTH`                 | `RAL_LORA_BW_125_KHZ`   | LoRa bandwidth                                                      |
| `LORA_CODING_RATE`               | `RAL_LORA_CR_4_5`       | LoRa coding rate                                                    |
| `LORA_PREAMBLE_LENGTH`           | `12`                    | Preamble length in symbols                                          |
| `LORA_PKT_LEN_MODE`              | `RAL_LORA_PKT_EXPLICIT` | Packet length mode                                                  |
| `LORA_IQ`                        | `false`                 | LoRa IQ inversion setting                                           |
| `LORA_CRC`                       | `true`                  | Enable/disable CRC                                                  |


## Compilation

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/ping_pong_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target ping_pong
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/ping_pong verify reset exit"
```

## Usage

1. Flash the same firmware to both devices (role is determined dynamically)
2. Power on both devices - they will start in subordinate mode (listening)
3. Press the user button on one device to make it the manager and start the ping-pong exchange
4. Observe TX/RX LEDs and UART debug output for communication status

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Manager device output

```
[00:00:02.615,000] <inf> usp: Button pushed
[00:00:02.615,000] <inf> usp: this device is now manager
[00:00:02.615,000] <inf> usp: aborting current transaction
[00:00:02.615,000] <inf> usp: requesting PING
[00:00:02.616,000] <inf> usp: usp/rac: new event: transaction aborted
[00:00:02.616,000] <inf> usp: PING requested, continuing
[00:00:02.616,000] <inf> usp: sending (value=0, delay=0ms)
[00:00:02.623,000] <inf> usp: usp/rac: transaction is starting
[00:00:02.764,000] <inf> usp: usp/rac: new event: transaction (transmission) has ended (success)
[00:00:02.765,000] <inf> usp: receiving...
[00:00:02.772,000] <inf> usp: usp/rac: transaction is starting
[00:00:02.921,000] <inf> usp: usp/rac: new event: transaction (reception) has ended (success)
[00:00:02.921,000] <inf> usp: payload is correct, continuing
[00:00:02.921,000] <inf> usp: sending (value=1, delay=250ms)
[00:00:03.170,000] <inf> usp: usp/rac: transaction is starting
[00:00:03.312,000] <inf> usp: usp/rac: new event: transaction (transmission) has ended (success)
[00:00:03.312,000] <inf> usp: receiving...
```

### Subordinate device output

```
[00:00:00.036,000] <inf> usp: this device is now subordinate
[00:00:00.036,000] <inf> usp: receiving...
[00:00:00.042,000] <inf> usp: usp/rac: transaction is starting
[00:00:08.544,000] <inf> usp: usp/rac: new event: transaction (reception) has ended (success)
[00:00:08.544,000] <inf> usp: payload is correct, continuing
[00:00:08.544,000] <inf> usp: sending (value=0, delay=10ms)
[00:00:08.552,000] <inf> usp: usp/rac: transaction is starting
[00:00:08.695,000] <inf> usp: usp/rac: new event: transaction (transmission) has ended (success)
[00:00:08.695,000] <inf> usp: receiving...
[00:00:08.702,000] <inf> usp: usp/rac: transaction is starting
[00:00:09.091,000] <inf> usp: usp/rac: new event: transaction (reception) has ended (success)
[00:00:09.091,000] <inf> usp: payload is correct, continuing
[00:00:09.092,000] <inf> usp: sending (value=1, delay=10ms)
[00:00:09.101,000] <inf> usp: usp/rac: transaction is starting
[00:00:09.243,000] <inf> usp: usp/rac: new event: transaction (transmission) has ended (success)
[00:00:09.243,000] <inf> usp: receiving...
```

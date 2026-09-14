# Hardware Modem

This application implements a **complete LoRa Basics Modem hardware interface** that exposes all LBM functionality through UART communication and GPIO control signals. It transforms the device into a standalone modem that can be controlled by external host systems via a standardized command protocol, making it ideal for integrating LoRaWAN capabilities into existing products without direct LBM integration.

## Key Features

- **Complete LBM API Access**: All LoRa Basics Modem functions accessible via commands
- **UART + GPIO Interface**: Standard communication using RX/TX + 3 control GPIOs (CMD, BUSY, EVENT)
- **Bridge Compatible**: Designed for use with bridge boards for host communication
- **Protobuf Serialization**: Complex data structures handled via Protocol Buffers
- **Dual Protocol Support**: Legacy protocol and NHM (New Hardware Modem) protocol
- **Command Validation**: CRC-protected command interface with proper error handling
- **Low Power Support**: Sleep mode management with interrupt-driven wake-up

## Configuration

### Using CMake

| Parameter                            | Default Value | Description                   |
|--------------------------------------|---------------|-------------------------------|
| `CONFIG_MAIN_STACK_SIZE`             | `12288`       | Main thread stack size (12KB) |
| `CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE` | `8192`        | System workqueue stack size   |
| `CONFIG_HEAP_MEM_POOL_SIZE`          | `8192`        | Heap memory pool size         |
| `CONFIG_LOG_BUFFER_SIZE`             | `4096`        | Logging buffer size           |
| `USE_FLRC_PROTOCOL`                  | `OFF`         | Enable FLRP (use 20KB buffer) |

### GPIO Configuration (Device Tree)

```dts
smtc-hal-uart = &uart1;
hw-modem-command-gpios = <&arduino_header 8 GPIO_ACTIVE_HIGH>;
hw-modem-busy-gpios = <&arduino_header 9 GPIO_ACTIVE_HIGH>;
hw-modem-event-gpios = <&arduino_header 10 GPIO_ACTIVE_HIGH>;
hw-modem-led-scan-gpios = <&arduino_header 11 GPIO_ACTIVE_HIGH>;
```

### LBM Features (Enabled)

- LoRaWAN Class B and Class C support
- Multicast and CSMA capabilities
- FUOTA (Firmware Update Over The Air)
- Application Layer Clock Synchronization (ALC Sync v1/v2)
- Geolocation and almanac services
- Stream and Large File Upload (LFU)
- Device management and store-and-forward
- Relay TX/RX functionality

### FLRP / FLRC protocol (disabled by default)

- Support for burst exchanges using FLRP (the FLRC protocol)
- The protocol is available at [`protocols/smtc_flrc_protocol`](../../../protocols/smtc_flrc_protocol)
- As the dedicated buffer consumes 20K bytes in RAM, this feature is disabled by default
- To enable, use the CMake option `-DUSE_FLRC_PROTOCOL=ON` (only available for LR20XX)

## Compilation

**Build sample:**
- hw_modem for lr2021 (no geolocation)
```
rm -Rf build/ ; cmake -L -S examples/main_examples/hw_modem -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -DLBM_GEOLOCATION=OFF -G Ninja; cmake --build build --target hw_modem
```
- hw_modem for lr2021 for FLRP
```
rm -Rf build/ ; cmake -L -S examples/main_examples/hw_modem -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -DLBM_GEOLOCATION=OFF -DUSE_FLRC_PROTOCOL=ON -G Ninja; cmake --build build --target hw_modem
```

- hw_modem for lr1120
```
rm -Rf build/ ; cmake -L -S examples/main_examples/hw_modem -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr1120 -G Ninja; cmake --build build --target hw_modem
```

- hw_modem for lr1120 with LBM_CRYPTO=LR11XX
```
rm -Rf build/ ; cmake -L -S examples/main_examples/hw_modem -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr1120 -DLBM_RELAY_TX=OFF -DLBM_RELAY_RX=OFF -DLBM_CRYPTO=LR11XX -G Ninja; cmake --build build --target hw_modem
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/hw_modem verify reset exit"
```


## Usage

### 1. Hardware Setup

1. **Connect Bridge Board**: Wire UART and GPIO signals through shield to external pins
2. **UART Connection**: Connect `smtc-hal-uart` (UART1) to bridge module
3. **GPIO Wiring**: Connect CMD, BUSY, EVENT GPIOs to bridge board
4. **Power Supply**: Ensure adequate power for modem operations

### 2. Command Interface

Commands follow the format:
```
<command_id> <command_size> <command_data> <crc>
```

**Example - Get Version (0x10):**
```
0x10 0x00 0x10
```

**Example - Open RAC Session (0xA2):**
```
0xA2 0x01 0x01 0xA0
```

### 3. Python Test Suite

Refer to the README.md file present in the python_test directory

### 4. Serial Command Example

**Using Python:**
```python
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

# Get version command
ser.write(b'\x10\x00\x10')
response = ser.read(100)
```

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Initialization
```
[00:00:01.000,000] <inf> hw_modem: Modem is starting
[00:00:01.100,000] <dbg> hw_modem: Commit SHA1: a1b2c3d4
[00:00:01.100,000] <dbg> hw_modem: Build date: 2024-01-15
```

### Command Processing
```
[00:00:05.200,000] <inf> hw_modem: Command received: 0x10 (Get Version)
[00:00:05.250,000] <inf> hw_modem: Response sent: version 040900
[00:00:06.100,000] <inf> hw_modem: Command received: 0xA2 (RAC Open)
[00:00:06.150,000] <inf> hw_modem: RAC session opened with handle 0x01
```

## Available Commands

### Core Commands
- `0x10` - Get LBM Version
- `0x5D` - Get Device Information
- `0xA0` - Submit USP Transaction (Legacy)
- `0xA2` - Open RAC Session
- `0xA3` - Close RAC Session
- `0xA5` - Get RAC Results (Legacy)
- `0xA6` - NHM Extended Command

### LoRaWAN Commands
- `0x41` - Join Network
- `0x42` - Send Uplink
- `0x43` - Request Downlink
- `0x44` - Set Device Class
- `0x45` - Configure Multicast

### Advanced Features
- FUOTA commands (0x60-0x6F)
- Geolocation commands (0x70-0x7F)
- Clock sync commands (0x80-0x8F)
- Device management commands (0x90-0x9F)

## Linux Implementation

On Linux, the UART+GPIO transport is replaced by a **pseudo-terminal (PTY)** implemented in `hw_modem_linux.c`. The rest of the application (`main_hw_modem.c`, `cmd_parser.c`, the LBM stack) is **identical** to the embedded build -- only the transport layer is swapped.

### PTY Setup

During `hw_modem_init()`, the application creates a PTY master/slave pair via `posix_openpt()`, configures the slave side in raw mode (no echo, no canonical processing), and creates a well-known symlink so Python scripts can connect as if it were a real serial port:

```python
ser = serial.Serial('/tmp/ttyHWMODEM', 115200, timeout=2)
```

The symlink path can be overridden via the `HW_MODEM_PTY_PATH` environment variable:

```bash
export HW_MODEM_PTY_PATH=/tmp/myModem
```

### Bridge Emulation

Since there is no physical bridge board, `hw_modem_linux.c` emulates the two transformations the bridge normally performs:

| Direction | Bridge HW behavior | PTY emulation |
|---|---|---|
| **Ingress** (host → modem) | Bridge computes XOR CRC over the command bytes and appends it | `hw_modem_is_a_cmd_available()` computes the CRC after `read()` and appends it to the receive buffer |
| **Egress** (modem → host) | Bridge prepends a 2-byte header (`0x01 0x00`) to the response | `hw_modem_process_cmd()` prepends the same header before `write()` |

This allows the Python test scripts to work unchanged -- they send frames **without** a CRC and expect responses **with** the 2-byte bridge header, regardless of whether the modem runs on a Nucleo board or on Linux.

### GPIO Replacement

The three control GPIOs used on embedded hardware are replaced as follows:

| Embedded GPIO | Purpose | Linux replacement |
|---|---|---|
| **CMD** | Host asserts to signal an incoming command; triggers DMA UART reception | `poll(pty_master_fd, POLLIN)` in `hw_modem_is_a_cmd_available()` detects incoming data |
| **BUSY** | Modem signals ready/busy state to synchronize transfers | Not needed -- the kernel PTY buffers data; no flow-control handshake required |
| **EVENT** | Modem raises to notify host of asynchronous LBM events | Stub (no-op) -- Python scripts poll for events via commands |

### Architecture & Data Flow

```
┌──────────────────────────────────────────────────────────────────┐
│                      PYTHON TEST SCRIPT                          │
│  ser.write([cmd_id, len, data...])           (no CRC)            │
│  ser.read() → [0x01, 0x00, rc, len, data..., crc]                │
└────────────────────────┬─────────────────────────────────────────┘
                         │
                  PTY slave device
                /tmp/ttyHWMODEM → /dev/pts/N
                         │
               ══════════╪══════════  kernel PTY  ══════════
                         │
                  PTY master fd
                         │
┌────────────────────────┴─────────────────────────────────────────┐
│                    hw_modem_linux.c                              │
│                  (bridge emulation)                              │
│                                                                  │
│  INGRESS:  poll() → read() → compute & append CRC                │
│  EGRESS:   prepend [0x01,0x00] bridge header → write()           │
└────────────────────────┬─────────────────────────────────────────┘
                         │
┌────────────────────────┴─────────────────────────────────────────┐
│                    main_hw_modem.c                               │
│                                                                  │
│  while(1)                                                        │
│    ├─ hw_modem_is_a_cmd_available()? → hw_modem_process_cmd()    │
│    ├─ smtc_modem_run_engine()                                    │
│    └─ sleep until next event                                     │
└──────────┬───────────────────────────────┬───────────────────────┘
           │                               │
           ▼                               ▼
┌─────────────────────┐     ┌──────────────────────────────────────┐
│    cmd_parser.c     │     │         LBM / RAC stack              │
│                     │     │                                      │
│  parse_cmd()        │     │  smtc_modem_run_engine()             │
│  ├─ Legacy commands │     │  ├─ LoRaWAN MAC processing           │
│  ├─ NHM extended    │     │  ├─ Class A/B/C scheduling           │
│  └─ Test commands   │     │  └─ FUOTA, ALC Sync, DM, ...         │
│     │               │     │                                      │
│     │ smtc_modem_*()│     │                                      │
│     └───────────────┼────►│                                      │
└─────────────────────┘     └──────────────┬───────────────────────┘
                                           │
                                           ▼
                            ┌──────────────────────────┐
                            │   Radio HAL (SPI)        │
                            │   lr1120 / lr2021        │
                            └──────────────────────────┘
```

## Technical Notes

- **Protocol Buffers**: Complex data serialization for RAC context and results
- **Segmentation**: Automatic for payloads exceeding 251 bytes (NHM protocol)
- **CRC Protection**: All commands include CRC validation
- **Bridge Interface**: GPIO signals routed through shield connector

## Integration Notes

- **Host Requirements**: Serial interface + 3 GPIO control lines
- **Bridge Board**: Required for connecting to host systems

## Limitations

- On NUCLEO-STM32L476RG, due to the USART, the power mode is deactivated
- The NUCLEO-U575ZI-Q is not supported

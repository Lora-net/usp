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

### Using the hardware bridge

#### 1. Hardware Setup

1. **Connect Bridge Board**: Wire UART and GPIO signals through shield to external pins
2. **UART Connection**: Connect `smtc-hal-uart` (UART1) to bridge module
3. **GPIO Wiring**: Connect CMD, BUSY, EVENT GPIOs to bridge board
4. **Power Supply**: Ensure adequate power for modem operations

#### 2. Command Interface

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

#### 3. Python Test Suite

Refer to the README.md file present in the python_test directory.

#### 4. Serial Command Example

**Using Python:**
```python
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

# Get version command
ser.write(b'\x10\x00\x10')
response = ser.read(100)
```

### Using the software bridge

The `python_bridge.py` is a software replacement for the former physical bridge that the hw_modem example mentioned previously. Commands follow the same interface as before. The script will connect to a device (through either a tty endpoint or a COM port) and expose two sockets, one for the traces (output of SMTC_HAL_TRACE_* functions) and one for the data (bidirectional connection using a ring buffer)

#### 1. Parameters

  * device selection and configuration
    --port PORT: set the port used for the connection
    --baudrate BAUDRATE: set the baudrate of the connection
    --timeout TIMEOUT: set the timeout of the connection
  * output setup
    --trace-out SINK: set the destination(s) for the trace channel (repeatable)
    --data-out SINK: set the destination(s) for the data channel (repeatable)

The SINK can be any of:
- 'stdout' or 'stderr' (this process's streams)
- 'pty' (Unix only; allocates a PTY and prints its path, forwards raw bytes both ways)
- 'socket[:PORT]' (TCP server, bidirectional, default port 5551 if PORT is omitted)
- 'file:PATH' (writes raw bytes to PATH, truncated at startup)

#### 2.1 Usage as a module

You can import the following symbols from the script:
  * add_python_bridge_options: add the script options to a user-provided ArgumentParser
  * extract_python_bridge_config: extract a HostConfig from the parsed arguments
  * open_device: a context manager that takes a HostConfig, opens the configured device, exposes and display trace and data output

Example:
```python
from python_bridge import *

def main():
    parser = ArgumentParser(description=__doc__)
    add_python_bridge_options(parser)
    args = parser.parse_args()
    config = extract_python_bridge_config(args)

    with open_device(config) as device:
        device.write(b"...")
        reply = device.read()
        ...
```

Using the `with` syntax, the hub will live as long as the scope.

#### 2.2 Usage as a standalone

The script is intended to be invoked by the `uv` tool, that takes care of the dependencies. If `uv` is not installed, you can still invoke the script with `python3`, but you will need to manually install the dependencies. Typical use:


#### 3.1 Linux example

Terminal 1 (bridge running in the background):
```
$ uv run python_bridge.py --port /dev/ttyACM0 --trace-out stdout
Python bridge on /dev/ttyACM0 @ 115200
  trace -> stdout
  data  -> socket: 127.0.0.1:5552
Ctrl+C to stop.
```

Terminal 2 (application communicating with hw_modem):
```python
from python_bridge import *

def main():
    parser = ArgumentParser(description=__doc__)
    add_python_bridge_options(parser)
    args = parser.parse_args()
    config = extract_python_bridge_config(args)

    with open_device(config, attach=True) as device:
        device.write(b"...")
        reply = device.read()
        ...
```

Note the `attach = True` parameter of `open_device`, it allows to connect to an already running python_bridge (the hub keeps on living after the scope). This option is not compatible with `--data-out pty`.

#### 3.2 Windows example

The bridge runs the same way on Windows, with a few differences:

  * `--port` takes a COM port instead of a tty path, e.g. `--port COM5`. Check Device Manager > Ports (COM & LPT) for the assigned number.
  * `--trace-out pty` / `--data-out pty` are Unix only. Consider using the default option (TCP sockets).
  * `uv run python_bridge.py ...` works the same from PowerShell or Command Prompt.

Example (Command Prompt or PowerShell):
```
> uv run python_bridge.py --port COM5 --trace-out stdout
Python bridge on COM5 @ 115200
  trace -> stdout
  data  -> socket: 127.0.0.1:5552
Ctrl+C to stop.
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
- Geolocation commands (0x70-0x89, requires `LBM_GEOLOCATION=ON`)
- Device management commands (0x90-0x9F)

### Geolocation Commands
GNSS (0x70-0x7F):
- `0x7B` - Start almanac demodulation
- `0x7C` - Set almanac demodulation constellations
- `0x7D` - Get almanac update event data. The response starts with a `service_state` byte (almanac demodulation service running or stopped)
- `0x7E` / `0x7F` - Start / stop cloud almanac update (`ADD_ALMANAC` only)

The GNSS scan done event data (`0x72`) includes a `time_available` byte right after the token (radio synchronized with GPS time, needed for an immediate almanac demodulation start).

WiFi (0x80-0x89):
- `0x80` - Start WiFi scan
- `0x81` - Cancel WiFi scan
- `0x82` - Get WiFi scan done data. Returns the header only (number of results, power consumption, scan duration); results are read one by one with `0x89`
- `0x83` - Get WiFi terminated event data
- `0x84` - Set WiFi port
- `0x85` - Set WiFi send mode
- `0x86` - Set WiFi payload format
- `0x87` - Set WiFi scan mode (1 byte, country code / SSID demodulation for region detection)
- `0x88` - Stop almanac demodulation
- `0x89` - Get one WiFi scan result (1 byte: result index, must be lower than the number of results). The response contains the result type, MAC address, channel, type and RSSI, followed by the country code and the SSID for extended results

### Network Commands
- `0x9B` - Get no-RX-packet threshold (number of uplinks without downlink before the stack is reset). Response: 2 bytes, big endian
- `0x9C` - Set no-RX-packet threshold. 2 bytes, big endian
- `0x9D` - Get LoRaWAN channel mask. Response: the channel mask bitfield, its size depends on the region (e.g. 9 bytes for US915/AU915)
- `0x9E` - Set LoRaWAN channel mask. 1 to 6 groups of 3 bytes: `ChMaskCntl` (1 byte) + `ChMask` (2 bytes, big endian), same encoding as `LinkADRReq`. Must be called before join for ABP with fixed channel plan regions

### Events
- `0x37` - Reset request: the modem needs to be reset (e.g. the LoRaWAN certification can request a DUT reset). 1 byte of status

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

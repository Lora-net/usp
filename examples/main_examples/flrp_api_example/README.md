# FLRP API Example

This application demonstrates **Fast LoRa communication Protocol (FLRP) API** for peer-to-peer communication between an initiator and slave device with optional integrity (MIC) support.

> For FLRP principles and limitations, see the [FLRP documentation](/doc/FLRP.md).

## Key Features

- **Dual Role Support**: Both initiator and slave device modes
- **Peer-to-Peer Communication**: Direct device-to-device communication using FLRP
- **Optional integrity (MIC)**: per-packet AES-CMAC message integrity check (the payload is **not** encrypted)
- **Two-Way Communication**: Supports both transmission and reception
- **Radio Configuration**: Customizable radio parameters and frequency plans
- **Device Identification**: EUI-based device addressing
- **Frequency Plan Support**: Multiple frequency bands (865MHz, 2.4GHz)
- **Button-Triggered Transfer**: Manual transfer trigger on button press for both roles
- **Periodic Listen Mode**: Continuous reception capability for slave devices
- **Validated MCU**: STM32L4

## Architecture

### Initiator Role
- Listens for incoming transmissions from slave
- Supports immediate transfer on button press
- **Automatically and periodically initiates transfers** with the target slave (every
  `FLRP_API_INTIATOR_PERIODIC_TRANSFER` ms, default 20 s), **alternating** transmit and receive:
  transmit → receive → transmit → … and so on.

### Slave Role
- Listens for incoming transmissions from initiator
- Supports immediate transfer on button press

## Configuration

> For the **FLRP protocol configuration** (default values, PA ramp time, burst interframe, and
> how/where to configure FLRP), see [FLRP — Configuration](/doc/FLRP_guidelines.md#configuration).

### FLRP API Parameters

Define the following macros to configure the example:

| Macro                                 | Default Value | Description                                                |
|---------------------------------------|---------------|------------------------------------------------------------|
| `FLRP_API_ROLE`                       | `0`           | `1` for Initiator, `0` for Slave (default: Slave)          |
| `FLRP_API_IS_LOW_FREQUENCY`           | `true`        | Use sub-giga frequency or 2.4GHz                           |
| `FLRP_API_IS_LISTENING`               | `true`        | Enable periodic listening mode (must be enabled for slave) |
| `FLRP_API_INTIATOR_PERIODIC_TRANSFER` | `20000`       | Initiator transmit period in ms                            |
| Device EUI                            | `0x00..0x07`  | Unique device identifier (8 bytes)                         |
| Target EUI                            | `0x10..0x07`  | Target device EUI for initiator                            |
| MIC Key                               | `0x00..0x0F`  | 16-byte AES-128 MIC (integrity) key                         |

**TX power, FLRC data rate, frequency plan and channels** are taken from the FLRP configuration
defaults (`flrp_configuration.h`) and the FLRP init configuration. **For any precise / detailed FLRP
configuration, refer to the [FLRP — Guidelines](/doc/FLRP_guidelines.md) documentation.**

This example implements a **point-to-point (P2P)** exchange using the **`SMTC_FLRP_BIDIRECTIONAL`**
communication mode (with `SMTC_FLRP_LINK_ADAPTATION_CHANNEL_SELECTION_ONLY`). **`SMTC_FLRP_BIDIRECTIONAL`
is the mode that has been mainly validated** in this release; **broadcast (one-way) configurations are
still being improved.**

### Device configuration
Some local variables to consider to update configuration

#### Local and target device EUI

**Initiator Device Configuration:**
```c
#if( FLRP_API_ROLE == FLRP_API_ROLE_INITIATOR )
// Initiator EUI
static uint8_t flrp_api_device_eui[SMTC_FLRP_EUI_LENGTH] =
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
// Target Slave EUI
static uint8_t flrp_api_target_device_eui[SMTC_FLRP_EUI_LENGTH] =
    { 0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
#endif
```

**Slave Device Configuration:**
```c
#elif( FLRP_API_ROLE == FLRP_API_ROLE_SLAVE )
// Slave EUI
static uint8_t flrp_api_device_eui[SMTC_FLRP_EUI_LENGTH] =
    { 0x10, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
// Initiator EUI (for reference)
static uint8_t flrp_api_target_device_eui[SMTC_FLRP_EUI_LENGTH] =
    { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
#endif
```

#### MIC key

Default MIC key (AES-128):
```c
static uint8_t flrp_api_key[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};
```

Both initiator and slave must use the same MIC key for successful communication.

## Compilation

### Build Initiator

```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/flrp_api_example -B build \
  -DCMAKE_BUILD_TYPE=MinSizeRel \
  -DBOARD=NUCLEO_L476 \
  -DRAC_RADIO=lr2021 \
  -G Ninja
cmake --build build --target flrp_api_initiator
```

### Build Slave

```bash
cmake --build build --target flrp_api_slave
```

### Build Both Targets

```bash
cmake --build build --target flrp_api
```

### Flashing with OpenOCD

**Initiator:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "adapter serial <SERIAL_NUMBER>" \
  -c "program build/flrp_api_initiator verify reset exit"
```

**Slave:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "adapter serial <SERIAL_NUMBER>" \
  -c "program build/flrp_api_slave verify reset exit"
```

## Expected Output

Trace captured on the USP (baremetal) example. Both devices (initiator and slave) produce the same kind of output (alternating RX/TX transfers).

```
INFO: [RX #66 OK] size=20482 ok/err/nok=74/0/0 rssi=-44 dBm wor_rssi=-44 dBm wor_snr=15 dB
[RX] data preview - (64 bytes):
 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F
 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F
 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F
INFO: [TX #64 OK] counter=63
[TX] data preview - (64 bytes):
 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
 10 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F
 20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F
 30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F
```

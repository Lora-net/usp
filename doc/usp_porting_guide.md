# USP Baremetal Porting Guide

## Introduction

This guide explains how to port **USP (Unified Software Platform)** baremetal applications to new MCUs and radios. Refer to the [USP Repository README](https://github.com/Lora-net/usp#readme)
- for general information about USP features,
- architecture,
- Supported MCU : **Reference Implementations:** STM32L476 (`examples/smtc_hal_l4/`) and STM32L073 (`examples/smtc_hal_l0_LL/`),
- Supported Radios,
- SW Requirements.

---

## Prerequisites

### Hardware Requirements

**MCU Requirements:**

USP contains source code example for the STMicroelectronics STM32L476 Nucleo board, however, it can be easily ported to other MCUs. The following MCU features are required:

**Architecture:**
- 32-bit native operation (no specific CPU core needed)
- Little-endian byte order
- Software MCU reset

**Memory Requirements:**
- Varies significantly by application and enabled features
- **Reference:** `periodical_uplink` example on STM32L476 + LR2021 (compiled with `-Os` MinSizeRel):
  - Region: IN_865, CLASS B: ON, Relay TX: ON, FUOTA: ON, CSMA: OFF
  - Flash: ~143KB, RAM: ~18KB
- Actual usage depends on your application code and enabled LBM features

**Required Peripherals:**
- **SPI:** For radio communication (full-duplex master, 8-bit transfers, MISO, MOSI, SCK, NSS)
- **Timer:** 100μs resolution or better (In LoRaWAN, Timer accuracy compensation is possible by widening the reception windows)
- **GPIOs:** Interrupt-capable GPIO for radio events, control pins as required by radio (RESET, BUSY, IRQs)
- **Non-volatile Storage:** for modem state storage and if application requires context persistence
- **Entropy Source:** hardware or software

**Detailed LoRa Basics Modem Requirements:**
Note that reliable Class A LoRaWAN communication can be obtained without any major time constraints on the MCU oscillator, or the oscillator used to clock the devices that implement the time-related LoRa Basics Modem HAL functions.
To compensate for time-related oscillator frequency errors, calling the smtc_modem_set_crystal_error_ppm() modem API function with an appropriate value is sufficient.
This results in a widening of the LoRaWAN reception window and increased power consumption.
In the case of Class B, however, it is desirable to be able to remain synchronized with the beacon over relatively long time intervals, even if beacons are sometimes not received due to poor RF conditions.
In this case, it is recommended to use an accurate crystal oscillator to clock the devices used to implement the time-related LoRa Basics Modem HAL functions.

In certain situations, such as the use of GNSS reception with the LR11xx, a transceiver TCXO is required. When using GNSS advanced scan on the LR11xx, the TCXO must have a relatively fast settling time, and the 32.768 kHz crystal oscillator must have 20ppm accuracy at 25 degrees.

### System Design Considerations

There are numerous requirements and options to consider when developing a device that implements USP and/or LoRaWAN.

It is important to consider the transceiver and timer interrupt behavior and configuration. The LoRa Basics Modem is designed to use a specific transceiver DIO line as the radio interrupt source. For SX126x, this is the DIO1 line, and for LR11xx, this is the DIO9 line.
Two principal interrupt sources interact with USP: a timer interrupt, and a radio interrupt.
The system interrupt priorities must be configured in such a way that the timer and radio interrupts do not nest or interrupt each another.

Therefore, when designing hardware that will run USP, it is recommended that the MCU GPIO lines selected for the transceiver's DIO interrupt request line do not share an MCU interrupt flag with other timing-critical hardware. If MCU interrupt flags are shared, it may not always be possible to react immediately to interrupts originating from these other devices.

No radio operations over the transceiver SPI bus are performed during timer and radio interrupt service routines.
The Radio shall have an **exclusive SPI controller**.
See reference implementation in `examples/modem_pinout.h` for interrupt pin assignments for your radio

---

## Porting Scenarios

Based on your hardware configuration:

| Scenario | MCU | Radio | Action Required |
|----------|-----|-------|-----------------|
| **A** | Custom MCU | Supported radio | Implement PART 1 and PART 2 |
| **B** | Custom MCU | Custom radio board | Implement PART 1, PART 2, and PART 3 |

---

## PART 1: Platform HAL Functions (MCU-Specific)

Implement these MCU-specific functions for your platform.

**Reference Implementation:** `examples/smtc_hal_l4/` (STM32L476) or `examples/smtc_hal_l0_LL/` (STM32L073)
**HAL Interface Definition:** `protocols/lbm_lib/smtc_modem_hal/smtc_modem_hal.h`

**Implementation Strategy:**
- Study reference implementation in `examples/smtc_hal_l4/`
- Implement same functions using your platform's APIs
- Replace STM32L4-specific calls with your MCU's driver functions

#### Function Categories:

| Category | Functions | Reference File |
|----------|-----------|----------------|
| **Time** | `smtc_modem_hal_get_time_in_s()`, `smtc_modem_hal_get_time_in_ms()`, `smtc_modem_hal_get_time_in_100us()`, `smtc_modem_hal_get_radio_irq_timestamp_in_100us()` | `smtc_hal_rtc.c` |
| **Timer** | `smtc_modem_hal_start_timer()`, `smtc_modem_hal_stop_timer()` | `smtc_hal_lp_timer.c` |
| **SPI** | SPI communication with radio | `smtc_hal_spi.c` |
| **GPIO** | GPIO control and interrupt handling | `smtc_hal_gpio.c` |
| **Interrupts** | `smtc_modem_hal_disable_modem_irq()`, `smtc_modem_hal_enable_modem_irq()`, `smtc_modem_hal_irq_config_radio_irq()`, `smtc_modem_hal_radio_irq_clear_pending()` | `smtc_hal_gpio.c` |
| **Flash/NVM** | `smtc_modem_hal_context_store()`, `smtc_modem_hal_context_restore()` | `smtc_hal_flash.c` |
| **Random** | `smtc_modem_hal_get_random_nb_in_range()` | `smtc_hal_rng.c` |
| **Radio** | `smtc_modem_hal_start_radio_tcxo()`, `smtc_modem_hal_stop_radio_tcxo()`, `smtc_modem_hal_get_radio_tcxo_startup_delay_ms()` | `smtc_modem_hal.c` |
| **System** | `smtc_modem_hal_reset_mcu()`, `smtc_modem_hal_get_board_delay_ms()` | `smtc_hal_mcu.c` |
| **Crashlog** | `smtc_modem_hal_crashlog_store()`, `smtc_modem_hal_crashlog_restore()`, `smtc_modem_hal_crashlog_set_status()`, `smtc_modem_hal_crashlog_get_status()` | `smtc_modem_hal.c` |
| **Optional** | `smtc_modem_hal_reload_wdog()`, `smtc_modem_hal_get_battery_level()`, `smtc_modem_hal_get_temperature()`, `smtc_modem_hal_get_voltage_mv()`, `smtc_modem_hal_print_trace()` | Various |

**Note:** All function names are prefixed with `smtc_modem_hal_`

**Complete Function Descriptions:** See [SMTC Modem HAL Implementation](#smtc-modem-hal-implementation) section below for detailed API descriptions.

**Reference - GPIO Function Example:**

```c
// Reference: examples/smtc_hal_l4/smtc_hal_gpio.c (STM32L4)
void hal_gpio_set_value(const hal_gpio_pin_names_t pin, const uint32_t value) {
    ...
    HAL_GPIO_WritePin(...);  // STM32L4 HAL call
}

// Your implementation - replace with your platform's API
void hal_gpio_set_value(const hal_gpio_pin_names_t pin, const uint32_t value) {
    your_gpio_write(pin, value);  // Use your platform's GPIO API
}
```

## PART 2: MCU-to-Radio Pin Mapping

Define GPIO pin assignments for your board.

**Reference:** `examples/modem_pinout.h`

**Required Pin Definitions:**
- SPI pins: MOSI, MISO, SCLK, NSS
- Radio control: RESET, BUSY
- Radio interrupt: DIOX (interrupt-capable GPIO)

**Note:** Pin assignments in `modem_pinout.h` show MCU GPIO assignments. The actual radio DIO pin (e.g., DIO1, DIO8, DIO9) used depends on the radio family and is configured in the radio HAL/BSP.

## PART 3: Radio BSP Configuration (Custom Radio Boards Only)

Configure board-specific radio settings based on your hardware schematic. For the Semtech LR2021, follow the [**Semtech LR2021 Reference Design**](https://www.semtech.com/products/wireless-rf/lora-plus/lr2021).

**Reference Implementation:** `examples/radio_hal/ral_{radio}_bsp.c` (shows how STM32L476 configures BSP)
**Function Definitions:** `smtc_rac_lib/smtc_radio_interface/smtc_ral/src/ral_{radio}_bsp.h` (defines required BSP functions)

**What to configure:**
- Power regulator type (DCDC or LDO) & consumption configuration
- RF switch configuration
- Clock source (XTAL or TCXO)
- RSSI calibration tables matching the RF bands used by your board (see [RSSI Calibration Tables Selection](#rssi-calibration-tables-selection-lr20xx))
- Keep radio-specific parameters (TX power tables, calibration) from reference

> ⚠️ **LR20xx, new in USP v1.3.0**: the BSP must implement `ral_lr20xx_bsp_get_rssi_calibration_table()`, called by the RAL at radio initialization. There is no default implementation: a LR20xx BSP written for an earlier USP version does not link anymore until this function is added. See [RSSI Calibration Tables Selection](#rssi-calibration-tables-selection-lr20xx).

If your RF shield hardware **matches the LR2021 reference design**, you can **reuse the existing power PA table** provided in `examples/radio_hal/pa_tables_lr20xx_common.c` (copy of the smtc-shields reference tables `examples/radio_hal/smtc_shields/lr20xx/src/smtc_shield_lr20xx_common.c`).

**Radio HAL files:** `examples/radio_hal/{radio}_hal.c` - Include in build without modification

**Note:** The `smtc_rac_lib/smtc_radio_interface/smtc_ral/src/` folder also contains RAL core implementation files (e.g., `ral_sx126x.c`, `ral_lr11xx.c`) - these are part of the library and should **not be modified**.

**Details:** See [Radio Driver HAL Implementation](#radio-driver-hal-implementation) and [RAL BSP Implementation](#ral-bsp-implementation) sections below for complete details.

**For Standard Shields:** Use reference BSP without modification.

---

## Implementation Details

### Radio Driver HAL Implementation

The USP software depends on Semtech's radio driver, which, in turn, requires a radio driver HAL implementation.
A brief description of the necessary steps for this implementation follows.

The HAL implementation must provide platform-specific read, write, reset, and wake-up implementations.

- Radio driver API functions call the HAL implementation to perform the actual reset, wake, and communication operations needed by the driver.
- For the `LR20xx`, these functions are documented in [lr20xx_hal.h](/smtc_rac_lib/smtc_radio_interface/radio_drivers/lr20xx_driver/inc/lr20xx_hal.h).
- For the `LR11xx`, these functions are documented in [lr11xx_hal.h](/smtc_rac_lib/smtc_radio_interface/radio_drivers/lr11xx_driver/src/lr11xx_hal.h).
- For the `SX126x`, these functions are documented in [sx126x_hal.h](/smtc_rac_lib/smtc_radio_interface/radio_drivers/sx126x_driver/src/sx126x_hal.h).

All radio driver API functions take a 'const void* context' argument:

- This argument is opaque to both the radio driver and USP software.
- It may be used by the HAL implementer to differentiate between different transceivers, which makes it easy to communicate with several radios inside the same application.
- Driver API functions do not use the context argument but pass it directly to the HAL implementation.

The USP software imposes a specific requirement on the radio driver HAL implementation:

- If a radio driver API function is called while the transceiver is in sleep mode, the HAL implementation must properly wake the transceiver and wait until it is ready before initiating any SPI communication.
- This typically requires that the HAL keeps track of whether the radio is awake or asleep, potentially by monitoring any commands sent to the transceiver to detect the SetSleep command.
- For a concrete LR20xx example, see the file: [lr20xx_hal.c](/examples/radio_hal/lr20xx_hal.c).
- For a concrete LR11xx example, see the file: [lr11xx_hal.c](/examples/radio_hal/lr11xx_hal.c).
- For a concrete SX126x example, see the file: [sx126x_hal.c](/examples/radio_hal/sx126x_hal.c).

When compiling the radio driver HAL implementation, it is necessary to add the radio driver source directory to the include path.
For example, for LR11xx:

- `smtc_rac_lib/smtc_radio_interface/radio_drivers/lr11xx_driver/src`

---

### RAL BSP Implementation

When porting the USP software to a new radio + MCU implementation, a Radio Abstraction Layer (RAL) board support package (BSP) implementation is necessary.
A brief description of the necessary steps follows.

The RAL provides radio-independent API functions that are similar to those provided by each radio driver.
The RAL, and a complementary layer called the RALF, are described in the following header functions:

- [ral.h](/smtc_rac_lib/smtc_radio_interface/smtc_ral/src/ral.h)
- [ralf.h](/smtc_rac_lib/smtc_radio_interface/smtc_ralf/src/ralf.h)

The RAL requires the implementer to define a few BSP API functions for the selected transceiver, by providing platform or radio-specific information to the RAL.

- For the `LR20xx`, these functions are described in [ral_lr20xx_bsp.h](/smtc_rac_lib/smtc_radio_interface/smtc_ral/src/ral_lr20xx_bsp.h) (including `ral_lr20xx_bsp_get_rssi_calibration_table()`, mandatory since USP v1.3.0).
- For the `LR11xx`, these functions are described in [ral_lr11xx_bsp.h](/smtc_rac_lib/smtc_radio_interface/smtc_ral/src/ral_lr11xx_bsp.h).
- For the `SX126x`, these functions are described in [ral_sx126x_bsp.h](/smtc_rac_lib/smtc_radio_interface/smtc_ral/src/ral_sx126x_bsp.h).

- An LR20xx sample implementation is in the file [ral_lr20xx_bsp.c](/examples/radio_hal/ral_lr20xx_bsp.c).
- An LR11xx sample implementation is in the file [ral_lr11xx_bsp.c](/examples/radio_hal/ral_lr11xx_bsp.c).
- An SX126x sample implementation is in the file [ral_sx126x_bsp.c](/examples/radio_hal/ral_sx126x_bsp.c).

The role of the 'const void* context' variable is described in previous section. It is typically used to store radio-specific information, but depending on the radio driver BSP implementation, it may be NULL if a single transceiver is used.
When compiling the RAL BSP implementation, it is necessary to add the radio driver source directory and the RAL source directory to the include path. For example, for LR11xx:

- `smtc_rac_lib/smtc_radio_interface/radio_drivers/lr11xx_driver/src`
- `smtc_rac_lib/smtc_radio_interface/smtc_ral/src`

#### RSSI Calibration Tables Selection (LR20xx)

The LR20xx RSSI accuracy depends on the gain and noise figure of the whole receive chain, which includes the matching network and the front-end of the board. For this reason the calibration values are not part of the radio driver: they are provided by the BSP through `ral_lr20xx_bsp_get_rssi_calibration_table()`, which the RAL calls once during radio initialization.

> ⚠️ This BSP function is **mandatory since USP v1.3.0** for all the LR20xx radios (LR2021, LR2022, LR2012): it has no default (weak) implementation, so a custom BSP that does not provide it fails at link time. When migrating a custom BSP from an earlier USP version, add it to your `ral_lr20xx_bsp.c`, either selecting one of the reference tables (board equivalent to the Semtech reference design) or a table characterized on your board, as described below.

The function returns two independent tables, one per RF path:

- `rssi_calibration_table_lf`: sub-GHz path (e.g. 490 MHz, 868/915 MHz)
- `rssi_calibration_table_hf`: 2.xGHz path

Each table is a `lr20xx_radio_common_rssi_calibration_gain_table_t` structure holding 27 entries: one per gain step from `g1` to `g11`, then eight entries for `g12` and eight for `g13` (`g12_boost0` to `g12_boost7` and `g13_boost0` to `g13_boost7`), which correspond to the Rx boost levels available on these two highest gain steps. All 27 entries must be filled: the driver serializes the whole structure (3 bytes per entry, 81 bytes per table) and sends it to the chip as is.

Each entry contains:

- `gain_value`: gain of the receive chain at this gain step, on 10 bits, signed fixed point format 8.2 (2 fractional bits, i.e. a 0.25 resolution). The field is declared as `uint16_t` and transmitted on 16 bits, but only the 10 least significant bits are meaningful: negative gains are written as negative values (as done in the reference tables) and the 6 upper bits are RFU.
- `noise_figure`: noise figure of the receive chain at this gain step, on 8 bits, unsigned fixed point format 6.2 (2 fractional bits, i.e. a 0.25 resolution).

Refer to the LR2021 datasheet for the reference unit and encoding of these two fields before computing your own values.

**Selecting a reference table**

The reference implementation [ral_lr20xx_bsp.c](/examples/radio_hal/ral_lr20xx_bsp.c) provides ready-to-use tables, characterized on the Semtech LR20xx reference design, one per supported band. Selection is done by simply assigning the address of the wanted table to the corresponding output pointer:

```c
void ral_lr20xx_bsp_get_rssi_calibration_table(
    const void* context, const lr20xx_radio_common_rssi_calibration_gain_table_t** rssi_calibration_table_lf,
    const lr20xx_radio_common_rssi_calibration_gain_table_t** rssi_calibration_table_hf )
{
    //Make sure pointers is not affected
    *rssi_calibration_table_lf = NULL;
    *rssi_calibration_table_hf = NULL;

    /*Getting High Frequency Calibration Table*/

    //Choosing HF 2.4GHz
    *rssi_calibration_table_hf = &table_hf_2G4;


    /*Getting Low Frequency Calibration Table*/

    //Choosing LF 868 MHz to 915 MHz
    *rssi_calibration_table_lf = &table_lf_868_915;

    //Choosing LF 490 MHz
    //*rssi_calibration_table_lf = &table_lf_490;
}
```

Guidelines:

- Select the low frequency table according to the sub-GHz band used by your application: keep `table_lf_868_915` for the 868 MHz / 915 MHz regions, or comment it out and uncomment `table_lf_490` (and its definition at the top of the file) when operating around 490 MHz.
- Only one table can be active per RF path. Uncommenting a second assignment for the same path simply overwrites the previous one.
- Leaving an output pointer to `NULL` is allowed: the driver then skips the calibration of that RF path, and the radio keeps its default internal values. This is the recommended setting for a path that is not routed on your board (for example, leave `rssi_calibration_table_hf` to `NULL` on a sub-GHz only design).

**Using a custom table for your own board**

The provided tables are only valid for a receive chain equivalent to the reference design. If your board uses a different matching network, an external LNA, a front-end module, an RF switch with a different insertion loss, or a different antenna port routing, the reference values will result in an RSSI offset. In that case, characterize your own board and map a custom table:

1. Measure, for each LNA gain step, the actual gain and noise figure of your receive chain at the frequency of interest.
2. Declare a new `const lr20xx_radio_common_rssi_calibration_gain_table_t` in your BSP file, following the same layout as the existing tables (all gain steps must be filled), and encode the values in the fixed point formats described above.
3. Assign its address to the relevant output pointer in `ral_lr20xx_bsp_get_rssi_calibration_table()`, instead of the reference table.

```c
/* Custom table characterized on <my_board> at 868 MHz */
static const lr20xx_radio_common_rssi_calibration_gain_table_t table_lf_my_board =
{
    .g1  = { .gain_value = -32, .noise_figure = 219 },
    .g2  = { .gain_value =  -8, .noise_figure = 204 },
    /* ... all gain steps up to g13_boost7 ... */
    .g13_boost7 = { .gain_value = 232, .noise_figure = 9 }
};

void ral_lr20xx_bsp_get_rssi_calibration_table(
    const void* context, const lr20xx_radio_common_rssi_calibration_gain_table_t** rssi_calibration_table_lf,
    const lr20xx_radio_common_rssi_calibration_gain_table_t** rssi_calibration_table_hf )
{
    *rssi_calibration_table_lf = &table_lf_my_board;
    *rssi_calibration_table_hf = NULL;  // 2.xGHz path not used on this board
}
```

If several radios or several front-end configurations coexist in the same application, the `const void* context` argument can be used to return a different table per transceiver.

The table structure and the fixed point formats are defined in [lr20xx_radio_common_types.h](/smtc_rac_lib/smtc_radio_interface/radio_drivers/lr20xx_driver/inc/lr20xx_radio_common_types.h), and the BSP function is documented in [ral_lr20xx_bsp.h](/smtc_rac_lib/smtc_radio_interface/smtc_ral/src/ral_lr20xx_bsp.h).

---

### SMTC Modem HAL Implementation

Porting the USP software to a new MCU architecture requires implementing the modem Hardware Abstraction Layer (HAL) API commands. Since USP v1.3.0, the HAL is split in two headers:
- the core HAL [smtc_core_hal.h](/smtc_rac_lib/smtc_core_hal_interface/smtc_core_hal.h), always required (used by RAC, radio planner, FLRP, LBM, ...): reset, watchdog, time, timer, IRQ, context storage, panic, random, radio environment, trace and RTOS related functions,
- the LBM specific modem HAL [smtc_modem_hal.h](/protocols/lbm_lib/smtc_modem_hal/smtc_modem_hal.h), only required when LBM is used: battery level, FUOTA, device management, crashlog and store & forward related functions.

Among other things, these API implementations define how timing information is provided to the USP, how random numbers are generated, and how data is stored in non-volatile memory.
If a TCXO is used on the radio part, its startup timing behavior should be specified in the RAL BSP implementation, and the documentation of the smtc_modem_hal_start_radio_tcxo(), smtc_modem_hal_stop_radio_tcxo(), and smtc_modem_hal_get_radio_tcxo_startup_delay_ms() functions, should be consulted.
Logging macros are defined in [smtc_core_hal_trace.h](/smtc_rac_lib/smtc_core_hal_interface/smtc_core_hal_trace.h) (formerly `smtc_modem_hal_dbg_trace.h`). In case the OS has a macro-based logging implementation, the HAL can provide its own header (see [Trace management related functions](#trace-management-related-functions)).

An example of implementation on STM32L476 and STM32L073 can be found in [smtc_modem_hal.c](/examples/smtc_modem_hal/smtc_modem_hal.c)

The following sections provide the list and more details on the different modem HAL APIs.

### Reset related functions

#### `void smtc_modem_hal_reset_mcu( void )`

**Brief**: Reset the MCU.

USP may need to reset the MCU during LoRaWAN certification process or in case Device Management service is used and cloud is asking for a device reset.

Recommendation is also to reset the mcu in case of modem_panic (in `smtc_modem_hal_on_panic()`)

### Watchdog related functions

#### `void smtc_modem_hal_reload_wdog( void )`

**Brief**: Reload the watchdog timer.

If the HAL implementation configures a watchdog timer, then this function should be implemented to reload the watchdog timer. Currently, the only code in USP that calls this HAL API command is the test code in smtc_modem_test.c.

### Time management related functions

#### `uint32_t smtc_modem_hal_get_time_in_s( void )`

**Brief**:
Provide the time since startup, in seconds.
USP uses this command to help perform various LoRaWAN® activities that do not have significant time accuracy requirements, such as NbTrans retransmissions.
**Return**:
The current system uptime in seconds.

#### `uint32_t smtc_modem_hal_get_time_in_ms( void )`

**Brief**:
Provide the time since startup, in milliseconds.
**Return**:
The system uptime, in milliseconds. The value returned by this function must monotonically increase all the way to 0xFFFFFFFF and then overflow to 0x00000000.

### Timer management related functions

#### `void smtc_modem_hal_start_timer( const uint32_t milliseconds, void ( *callback )( void* context ), void* context )`

**Brief**:
Start a timer that will expire at the requested time.
Upon expiration, the provided callback is called with context as its sole argument.
The current design of the LoRa Basics Modem has only been tested in the case where the provided callback is executed in an interrupt context, with interrupts disabled.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|milliseconds|Number of milliseconds before callback execution|
|[in]|callback|Callback to execute|
|[in]|context|Argument that is passed to callback|

#### `void smtc_modem_hal_stop_timer( void )`

**Brief**:
Stop the timer that may have been started with `smtc_modem_hal_start_timer()`

### IRQ management related functions

#### `void smtc_modem_hal_disable_modem_irq( void )`

**Brief**:
Disable the two interrupt sources that execute the USP code: the timer, and the transceiver DIO interrupt source.
Please also refer to System Design Considerations.

#### `void smtc_modem_hal_enable_modem_irq( void )`

**Brief**:
Enable the two interrupt sources that execute the USP code: the timer, and the transceiver DIO interrupt source.
Please also refer to System Design Considerations.

### Context saving management related functions

#### `void smtc_modem_hal_context_restore( const modem_context_type_t ctx_type, uint32_t offset, uint8_t* buffer, const uint32_t size )`

**Brief**:
Restore to RAM a data structure of type `ctx_type` that has previously been stored in non-volatile memory by calling `smtc_modem_hal_context_store()`.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|ctx_type|Type of modem context to be restored|
|[out]|buffer|Buffer where context must be restored|
|[in]|size|Number of bytes of context to restore|

#### `void smtc_modem_hal_context_store( const modem_context_type_t ctx_type, uint32_t offset, const uint8_t* buffer, const uint32_t size )`

**Brief**:
Store a data structure of type `ctx_type` from RAM to non-volatile memory.

**Details of each type of context**:

|Context type|Size in bytes|Purpose|
|---    |---    |---    |
|CONTEXT_MODEM|16|To save general info of modem, eg reset|
|CONTEXT_KEY_MODEM|20|To save crc of keys in case lr11xx crypto engine is used|
|CONTEXT_LORAWAN_STACK|36|To save stack devnonce, joinonce, region, certification status|
|CONTEXT_FUOTA|variable|To save the fragmented data received|
|CONTEXT_SECURE_ELEMENT|480 or 24|To save all secure element context, needed only for certification purpose|
|CONTEXT_STORE_AND_FORWARD|variable|To save data for store and forward|

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|ctx_type|Type of modem context to be saved|
|[in]|buffer|Buffer which must be saved|
|[in]|size|Number of bytes of context to save|

#### `void smtc_modem_hal_context_flash_pages_erase( const modem_context_type_t ctx_type, uint32_t offset, uint8_t nb_page )`

**Brief**:
Erase a chosen number of flash pages of a context.
This function is only used for Store and Forward service with `ctx_type` parameter set to `CONTEXT_STORE_AND_FORWARD`

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|ctx_type|Type of modem context that need to be erased|
|[in]|offset|Memory offset after ctx_type address|
|[in]|nb_page|Number of pages that|

### Panic management related functions

#### `void smtc_modem_hal_on_panic( uint8_t* func, uint32_t line, const char* fmt, ... )`

**Brief**:
Action to be taken in case on modem panic.
In case Device Management is used, it is recommended to perform the crashlog storage and status update in this function

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|func|The name of the function where the panic occurs|
|[in]|line|The line number where the panic occurs|
|[in]|fmt|String Format|
|[in]|...|String Arguments|

### Random management related functions

#### `uint32_t smtc_modem_hal_get_random_nb_in_range( const uint32_t val_1, const uint32_t val_2 )`

**Brief**:
Return a uniformly-distributed unsigned random integer from the closed interval [val_1, ..., val_2] or [val_2, ..., val_1].
**Return**:
The random integer.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|val_1|first range unsigned value|
|[in]|val_2|second range unsigned value|

### Radio environment management related functions

#### `void smtc_modem_hal_irq_config_radio_irq( void ( *callback )( void* context ), void* context )`

**Brief**:
Store the callback and context argument that must be executed when a radio event occurs.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|callback|Callback that is executed upon radio interrupt service request|
|[in]|context|Argument that is provided to callback|

#### `void smtc_modem_hal_start_radio_tcxo( void )`

**Brief**:
If the TCXO is not controlled by the transceiver, powers up the TCXO.
If no TCXO is used, or if the TCXO has been configured in the RAL BSP to start up automatically, then implement an empty command. If the TCXO is not controlled by the transceiver, then this function must power up the TCXO, and then busy wait until the TCXO is running with the proper accuracy.

#### `void smtc_modem_hal_stop_radio_tcxo( void )`

**Brief**:
If the TCXO is not controlled by the transceiver, stop the TCXO.
If no TCXO is used, or if the TCXO has been configured in the RAL BSP to start up automatically, implement an empty command.

#### `uint32_t smtc_modem_hal_get_radio_tcxo_startup_delay_ms( void )`

**Brief**:
Return the time, in milliseconds, that the TCXO needs to start up with the required accuracy.
This does not implement a delay but is used to perform certain calculations in the LoRa Basics Modem so that this time will be taken into consideration when opening the Rx window.
If the TCXO is configured by the RAL BSP to start up automatically, then the value used here should be the same as the startup delay used in the RAL BSP.
**Return**:
The needed TCXO startup time, in milliseconds. Return 0 if no TCXO is used.

#### `void smtc_modem_hal_set_ant_switch( bool is_tx_on )`

**Brief**:
Set antenna switch for Tx operation or not.
If no antenna switch is used then implement an empty command.

### Environment management related functions

#### `uint8_t smtc_modem_hal_get_battery_level( void )`

**Brief**:
Indicate the current battery state.
According to LoRaWan 1.0.4 spec:

- 0: The end-device is connected to an external power source.
- 1..254: Battery level, where 1 is the minimum and 254 is the maximum.
- 255: The end-device was not able to measure the battery level.
**Return**
A value between 0 (for 0%) and 254 (for 100%) or 255.

#### `int8_t smtc_modem_hal_get_board_delay_ms( void )`

**Brief**:
Return the amount of time that passes between the moment the MCU calls `ral_set_tx()` or `ral_set_rx()`, and the moment the radio transceiver enters RX or TX state.
This varies depending on the MCU clock speed and SPI bus speed.
**Return**:
The board delay, in milliseconds.

### Trace management related functions

The HAL can provide its own trace macros by writing its own `smtc_core_hal_trace.h` header, based on the content of the existing [smtc_core_hal_trace.h](/smtc_rac_lib/smtc_core_hal_interface/smtc_core_hal_trace.h), and making sure that it is found before the default one in the include path.

#### `void smtc_modem_hal_print_trace( const char* fmt, ... )`

**Brief**:
Output a printf-style variable-length argument list to the logging subsystem.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|fmt|String Format|
|[in]|...|Variadic arguments |

### Fuota management related functions (optional)

#### `uint32_t smtc_modem_hal_get_hw_version_for_fuota( void )`

**brief**:
Return the hardware version as defined in FMP Specification TS006-1.0.0.
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.
**Return**:
The hardware version

#### `uint32_t smtc_modem_hal_get_fw_version_for_fuota( void )`

**brief**:
Return the firmware version as defined in FMP Specification TS006-1.0.0.
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.
**Return**:
The firmware version

#### `uint8_t smtc_modem_hal_get_fw_status_available_for_fuota( void )`

**brief**:
Return the firmware status as defined in FMP Specification TS006-1.0.0.
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.
**Return**:
The firmware status field

#### `uint8_t smtc_modem_hal_get_fw_delete_status_for_fuota( uint32_t fw_to_delete_version )`

**brief**:
Delete a chosen firmware version as defined in FMP Specification TS006-1.0.0.
No need to implement this function if FMP package is not build in `LBM_FUOTA` option.
**Return**:
The firmware status field

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|fw_to_delete_version|Version of firmware to delete|

#### `uint32_t smtc_modem_hal_get_next_fw_version_for_fuota( void )`

**brief**:
Return the firmware version that will be running once the firmware update imagine is installed as defined in FMP Specification TS006-1.0.0.

No need to implement this function if FMP package is not build in `LBM_FUOTA` option.
**Return**:
The new firmware version.

### Device Management related functions (optional)

#### `int8_t smtc_modem_hal_get_temperature( void )`

**Brief**:
Indicate the current system temperature.
**Return**:
The temperature, in degrees Celsius.

#### `uint16_t smtc_modem_hal_get_voltage_mv( void )`

**Brief**:
Indicates the current battery voltage.
**Return**:
The battery voltage in mV.

#### `void smtc_modem_hal_crashlog_store( const uint8_t* crash_string, uint8_t crash_string_length )`

**Brief**:
Store the modem crash log to non-volatile memory.
On most MCUs, RAM is preserved upon reset, so it may be possible to use RAM for this purpose.
This function is not called by LoRa Basics Modem directly but the recommendation is to call it in `smtc_modem_hal_on_panic()`implementation.

In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|crash_string|Crashlog string to be stored|
|[in]|crash_string_length|Crashlog string length|

#### `void smtc_modem_hal_crashlog_restore( uint8_t* crash_string, uint8_t* crash_string_length )`

**Brief**:
Retrieve the modem crash log from non-volatile memory.
On most MCUs, RAM is preserved upon reset, so it may be possible to use RAM for this purpose.
In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[out]|crash_string|Crashlog string to be restored|
|[out]|crash_string_length|Crashlog string length|

#### `void smtc_modem_hal_crashlog_set_status( bool available )`

**Brief**:
Store the modem crash log status to non-volatile memory. True indicates that a crash log has been stored and is available for retrieval.
On most MCUs, RAM is preserved upon reset, so it may be possible to use RAM for this purpose.
This function is not called by USP directly but the recommendation is to call it in `smtc_modem_hal_on_panic()`implementation.

In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

**Parameters**:

|       |       |       |
|---    |---    |---    |
|[in]|available|True if a crashlog is available, false otherwise|

#### `bool smtc_modem_hal_crashlog_get_status( void )`

**Brief**:
Get the modem crash log status from non-volatile memory.
**Return**:
The crash log status, as previously written using `smtc_modem_hal_set_crashlog_status()`.
In case Device Management service is running, if status is true the crashlog will be sent automatically after joining the network.

### Store and Forward related functions (optional)

#### `uint16_t smtc_modem_hal_store_and_forward_get_number_of_pages( void )`

**Brief**:
Return the number of reserved pages in flash for the Store and Forward service.
At least 3 pages are needed for the service.
**Return**:
The number of reserved pages

#### `uint16_t smtc_modem_hal_flash_get_page_size( void )`

**Brief**:
Gives the size of a flash page in bytes
**Return**:
The size of a flash page.

### RTOS compatibility related functions

#### `void smtc_modem_hal_user_lbm_irq( void )`

**Brief**:
This function is called by the LBM stack on each LBM interruption (radio interrupt or low-power timer interrupt). It could be convenient in the case of an RTOS implementation to notify the thread that manages the LBM stack

---

## Testing & Validation

### Phase 1: Porting Tests (**REQUIRED**)

**Application:** `porting_tests`
**Source:** `examples/main_examples/porting_tests_example/main_porting_tests.c`

This validates your HAL implementation.

**Expected output:**
```
porting_test_spi : OK
porting_test_radio_irq : OK
porting_test_get_time : OK
porting_test_timer_irq : OK
porting_test_stop_timer : OK
porting_test_disable_enable_irq : OK
porting_test_random : OK
porting_test_config_rx_radio : OK
porting_test_config_tx_radio : OK
porting_test_sleep_ms : OK
porting_test_timer_irq_low_power : OK
```

**If any test fails:** Fix that HAL function before proceeding.

### Phase 2: LoRaWAN Application Test

**Application:** `periodical_uplink`
**Source:** `examples/main_examples/periodical_uplink_example/main_periodical_uplink.c`

Configure LoRaWAN credentials:

**Reference:** `examples/main_examples/example_options.h`

```c
#define USER_LORAWAN_DEVICE_EUI { 0x00, ..., 0x01 }
#define USER_LORAWAN_JOIN_EUI   { 0x00, ..., 0x01 }
#define USER_LORAWAN_APP_KEY    { 0x00, ..., 0x01 }
#define MODEM_EXAMPLE_REGION    SMTC_MODEM_REGION_EU_868
```

**Expected output:**
```
Event received: RESET
Event received: JOINED
Event received: TXDONE
```

---

## Porting Checklist

### Implementation
- [ ] Platform HAL functions (timer, SPI, flash, GPIO, RNG, MCU)
- [ ] Pin definitions for your board
- [ ] Radio Driver HAL implementation
- [ ] RAL BSP configuration (if custom radio board)
- [ ] LR20xx: `ral_lr20xx_bsp_get_rssi_calibration_table()` implemented in the RAL BSP (mandatory since USP v1.3.0)

### Build System
- [ ] Configure build system to compile your HAL implementation
- [ ] Include Modem HAL wrapper: `examples/smtc_modem_hal/`
- [ ] Include Radio HAL: `examples/radio_hal/`
- [ ] Link RAC library: `smtc_rac_lib/`
- [ ] Link LBM library: `protocols/lbm_lib/`

### Testing
- [ ] Build `porting_tests`
- [ ] Run `porting_tests` - all tests pass
- [ ] Build `periodical_uplink`
- [ ] Join LoRaWAN network
- [ ] Verify uplinks on network server

---

## Resources

**USP Repository:**
- Main Repository: https://github.com/Lora-net/usp
- RAC API Documentation: [smtc_rac_lib/README.md](https://github.com/Lora-net/usp/blob/main/smtc_rac_lib/README.md)
- Build Instructions: [USP README](https://github.com/Lora-net/usp#readme)

**LoRa Basics Modem lbm_lib:**
- [LBM README](https://github.com/Lora-net/usp/blob/main/protocols/lbm_lib/README.md)

**Reference Implementation:**
- Platform HAL: `examples/smtc_hal_l4/` (STM32L476) or `examples/smtc_hal_l0_LL/` (STM32L073)
- Pin Configuration: `examples/modem_pinout.h`
- Radio BSP: `examples/radio_hal/`
- HAL Interfaces: `smtc_rac_lib/smtc_core_hal_interface/smtc_core_hal.h` (core HAL) & `protocols/lbm_lib/smtc_modem_hal/smtc_modem_hal.h` (LBM specific HAL)
- BSP Interfaces: `smtc_rac_lib/smtc_radio_interface/smtc_ral/src/ral_xxx_bsp.h`
- Test Application: `examples/main_examples/porting_tests_example/main_porting_tests.c`

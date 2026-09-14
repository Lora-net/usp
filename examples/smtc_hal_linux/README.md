# Linux Hardware Abstraction Layer (HAL)

## Overview

This directory contains the Linux implementation of the USP Hardware Abstraction Layer (HAL). It enables running LoRa Basics Modem and USP applications on Linux platforms, including:

- **Native x86/x86_64** systems for development and testing
- **ARM Linux devices** (Raspberry Pi, embedded Linux boards)

The Linux HAL provides the same interface as the STM32 HAL implementations but uses Linux kernel APIs instead of bare-metal peripherals.

**Key capabilities:**
- Physical radio support via SPI/GPIO (LR20xx)
- Virtual radio support (UDP Packet Forwarder protocol)
- POSIX-based timers and RTC
- File-based flash emulation
- GPIO-based LED control
- Signal-based virtual button

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                   Application / LBM / RAC                       │
├─────────────────────────────────────────────────────────────────┤
│                     smtc_modem_hal API                          │
│              (Platform-independent interface)                   │
├─────────────────────────────────────────────────────────────────┤
│                    Linux HAL Layer (this directory)             │
│   ┌─────────────┬──────────────┬──────────────┬──────────────┐  │
│   │ GPIO        │ SPI          │ Timers       │ Flash        │  │
│   │ RTC         │ RNG          │ Button       │ LED          │  │
│   └─────────────┴──────────────┴──────────────┴──────────────┘  │
├─────────────────────────────────────────────────────────────────┤
│                     Linux Kernel APIs                           │
│   - /dev/gpiochip0 (GPIO character device)                      │
│   - /dev/spidev0.0 (SPI device)                                 │
│   - POSIX timers (timer_create, SIGRTMIN + timer_id)            │
│   - CLOCK_MONOTONIC (monotonic time)                            │
│   - File I/O (flash emulation)                                  │
│   - Signals (SIGUSR1 for button)                                │
└─────────────────────────────────────────────────────────────────┘
```

---

## HAL Module Reference

### GPIO (`smtc_hal_gpio.c`, `smtc_hal_gpio.h`)

**Implementation:** Linux GPIO character device API (`/dev/gpiochip0`)

**Features:**
- Pin configuration (input/output, pull-up/pull-down)
- Read/write GPIO values
- Interrupt handling via edge detection
- Dedicated pthread for IRQ polling

**Key details:**
- Uses `libgpiod` style API (direct ioctl calls to `/dev/gpiochip0`)
- IRQ handling: A background thread (`hal_gpio_irq_thread`) polls GPIO event file descriptors for rising edge detection
- Multiple GPIO interrupts supported (radio IRQ, button, etc.)
- Pin definitions in `smtc_hal_gpio_pin_names.h` (Raspberry Pi BCM numbering)

**Pin mapping example (Raspberry Pi):**
```c
// See smtc_hal_gpio_pin_names.h for complete mapping
NC = 0xFF,          // Not Connected
GPIO_2 = 2,         // BCM GPIO 2
GPIO_3 = 3,         // BCM GPIO 3
...
```

**Important functions:**
- `hal_gpio_init()` - Initialize GPIO subsystem
- `hal_gpio_set_value()` - Set output pin state
- `hal_gpio_get_value()` - Read input pin state
- `hal_gpio_irq_attach()` - Attach interrupt handler
- `hal_gpio_irq_enable()` / `hal_gpio_irq_disable()` - Control interrupts

---

### SPI (`smtc_hal_spi.c`, `smtc_hal_spi.h`)

**Implementation:** Linux spidev driver (`/dev/spidev0.0`)

**Configuration:**
- **Speed:** 20 MHz (configurable via `SPI_SPEED_HZ`)
- **Mode:** SPI Mode 0 (CPOL=0, CPHA=0)
- **Bits per word:** 8
- **Byte order:** MSB first

**Key details:**
- NSS (Chip Select) is automatically managed by the kernel driver
- Full-duplex transfers supported
- Thread-safe (mutex protection)

**Limitations:**
- NSS cannot be manually controlled from userspace with standard spidev
- This prevents waking the radio from sleep mode (radio sleep is disabled on Linux platform)

**Important functions:**
- `hal_spi_init()` - Initialize SPI interface
- `hal_spi_in_out()` - Bidirectional SPI transfer

---

### Timers

#### Low-Power Timer (`smtc_hal_lp_timer.c`, `smtc_hal_lp_timer.h`)

**Implementation:** POSIX `timer_create()` with real-time signals

**Features:**
- One-shot timer for modem operations
- 100μs resolution or better
- Signal-based callback mechanism

**Key details:**
- Uses `CLOCK_MONOTONIC` as time base
- Timer signal: `SIGRTMIN + timer_id` (typically `SIGRTMIN` for HAL_LP_TIMER_ID_1)
- Callback executed in signal handler context (keep it short!)
- Time acceleration supported (see RTC section)

**Important functions:**
- `hal_lp_timer_init()` - Initialize timer subsystem
- `hal_lp_timer_start()` - Start one-shot timer
- `hal_lp_timer_stop()` - Stop running timer
- `hal_lp_timer_irq_enable()` / `hal_lp_timer_irq_disable()` - Control timer interrupts

#### Real-Time Clock (`smtc_hal_rtc.c`, `smtc_hal_rtc.h`)

**Implementation:** `CLOCK_MONOTONIC` via `clock_gettime()`

**Features:**
- Monotonic time source (unaffected by system time adjustments)
- Time acceleration support (experimental)

**Time Acceleration (Experimental):**
```bash
# via environment variable
LBM_TIME_ACCELERATION=5 ./periodical_uplink
```

**Important functions:**
- `hal_rtc_init()` - Initialize RTC
- `hal_rtc_get_time_s()` - Get time in seconds
- `hal_rtc_get_time_ms()` - Get time in milliseconds

---

### Flash Storage (`smtc_hal_flash.c`, `smtc_hal_flash.h`)

**Implementation:** File-based emulation

**Features:**
- Persistent storage for modem context
- Emulates non-volatile memory behavior

**Key details:**
- **File:** `flash_context.bin` (created in working directory)
- **Size:** 128 KB (configurable via `FLASH_USER_END_ADDR`)
- **Access:** Direct file I/O (read/write/seek)
- File created automatically on first write

**Storage layout:**
```
flash_context.bin (128 KB)
├─ Modem context (variable size)
├─ LoRaWAN keys and state
└─ Application data (if used)
```

**Important functions:**
- `hal_flash_init()` - Initialize flash subsystem
- `hal_flash_erase_page()` - Erase flash page (sets to 0xFF)
- `hal_flash_write_buffer()` - Write data to flash
- `hal_flash_read_buffer()` - Read data from flash

---

### Button (`smtc_hal_button.c`, `smtc_hal_button.h`)

**Implementation:** Signal-based virtual button (SIGUSR1)

**Features:**
- Virtual button emulation via UNIX signal
- Callback support

**Key details:**
- No physical button required
- Trigger button press: `kill -SIGUSR1 <pid>`
- Signal handler invokes registered callback

**Usage example:**
```bash
# Find process ID
ps aux | grep periodical_uplink

# Trigger button press
kill -SIGUSR1 12345
```

**Important functions:**
- `hal_button_init()` - Initialize button subsystem
- `hal_button_register_callback()` - Register button press callback

---

### LED (`smtc_hal_led.c`, `smtc_hal_led.h`)

**Implementation:** GPIO-based LED control

**Features:**
- LED on/off/toggle control
- Multiple LED support (TX, RX, SCAN)
- NC (Not Connected) pins safely ignored

**Key details:**
- Uses GPIO HAL for output control
- Pin definitions in `modem_pinout_linux.h`
- `NC` pins (0xFF) are no-ops

**LED definitions (from `modem_pinout_linux.h`):**
```c
#define SMTC_LED_TX    RPI_GPIO2   // TX LED
#define SMTC_LED_RX    RPI_GPIO3   // RX LED
#define SMTC_LED_SCAN  NC          // Scan LED (not connected)
```

**Important functions:**
- `hal_led_init()` - Initialize LED subsystem
- `hal_led_set()` - Set LED on/off
- `hal_led_toggle()` - Toggle LED state

---

### MCU (`smtc_hal_mcu.c`, `smtc_hal_mcu.h`)

**Implementation:** Platform initialization and system control

**Features:**
- Platform initialization (RTC, RNG, GPIO, SPI, timers)
- Microsecond-precision delays
- Sleep emulation with interrupt responsiveness
- Reset control

**Key details:**
- Critical sections (`hal_mcu_disable_irq()` / `hal_mcu_enable_irq()`) are no-ops (placeholder for embedded compatibility)
- Sleep uses `nanosleep()` with maximum 1ms chunks to maintain interrupt responsiveness
- Reset triggers `exit(1)` - relies on external supervisor for restart
- Wait functions use `nanosleep()` for precise timing

**Important functions:**
- `hal_mcu_init()` - Initialize all HAL subsystems (RTC, RNG, GPIO, SPI, timers)
- `hal_mcu_reset()` - Exit process with code 1
- `hal_mcu_wait_us()` - Busy-wait delay in microseconds
- `hal_mcu_set_sleep_for_ms()` - Sleep in 1ms chunks (allows frequent IRQ checking)
- `hal_mcu_disable_irq()` / `hal_mcu_enable_irq()` - No-op (compatibility stubs)
- `hal_mcu_critical_section_begin()` / `hal_mcu_critical_section_end()` - No-op (compatibility stubs)

---

### Random Number Generator (`smtc_hal_rng.c`, `smtc_hal_rng.h`)

**Implementation:** Standard C library `rand()` / `srand()`

**Features:**
- Pseudo-random number generation
- Range-limited random generation

**Key details:**
- Seeded with `time(NULL)` at initialization
- Uses `rand()` for random number generation
- **Note:** could be improved with `/dev/urandom` for stronger randomness

**Important functions:**
- `hal_rng_init()` - Initialize and seed RNG with current time
- `hal_rng_get_random()` - Get 32-bit random number
- `hal_rng_get_random_in_range()` - Get random number within specified range [val_1, val_2]

---

### Trace/Debug (`smtc_hal_trace.c`, `smtc_hal_trace.h`, `smtc_hal_dbg_trace.h`)

**Implementation:** Console output via `vprintf()`

**Key details:**
- Outputs to stdout
- Uses variable argument formatting

---

### Watchdog (`smtc_hal_watchdog.c`, `smtc_hal_watchdog.h`)

**Implementation:** Stub (no-op)

**Note:** Watchdog functionality relies on external supervisor or systemd watchdog integration.

---

## Build Configuration

### CMake Integration

The Linux HAL is automatically selected when building with `BOARD=LINUX` or `BOARD=LINUX_ARM`.

For more details regarding the integration, refer to the [examples/toolchain.cmake](examples/toolchain.cmake] file.

### Toolchain Files

**Native Linux (x86/x86_64):**
- Uses system default compiler (gcc/clang)
- No toolchain file needed

**ARM Cross-Compilation:**
- Toolchain: `cmake_linux_arm_toolchain.cmake`
- Requires ARM cross-compiler: `arm-linux-gnueabihf-gcc`

```cmake
# cmake_linux_arm_toolchain.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
```

### Compile Definitions

| Definition | Description |
|------------|-------------|
| `LINUX_PLATFORM` | Indicates Linux platform build |
| `_POSIX_C_SOURCE=199309L` | Enable POSIX timer APIs |

---

## Pin Configuration

### Customizing Pin Assignments

To use different GPIO pins:

1. Edit `examples/modem_pinout_linux.h`
2. Use Raspberry Pi BCM GPIO numbering (not physical pin numbers)

---

## License

Clear BSD License.
Copyright Semtech Corporation.

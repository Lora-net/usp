# LoRaWAN Porting Tests

This application provides **comprehensive Hardware Abstraction Layer (HAL) testing** for LoRa Basics Modem (LBM) integration. It validates critical system functions required for proper modem operation, including SPI communication, timing, interrupts, and low-power functionality.

## Key Features

- **SPI Communication Testing**: Verifies radio transceiver communication via SPI interface
- **Radio Interrupt Validation**: Tests radio IRQ handling and callback functionality
- **Timing System Tests**: Validates time measurement functions (seconds and milliseconds)
- **Timer Interrupt Testing**: Verifies low-power timer operation and IRQ callbacks
- **Random Number Generation**: Tests hardware random number generation functionality
- **Radio Configuration Tests**: Validates RX/TX radio setup and timing performance
- **Sleep Mode Testing**: Verifies low-power sleep functionality and wake-up timing
- **Flash Storage Tests**: Optional non-volatile memory read/write validation

## Configuration

### Test Modes

| Parameter                                         | Default | Description                                                                            |
|---------------------------------------------------|---------|----------------------------------------------------------------------------------------|
| `ENABLE_TEST_FLASH`                               | `n`     | Enable flash tests, disable others                                                     |
| `NB_LOOP_TEST_SPI`                                | `2`     | Number of SPI test iterations                                                          |
| `NB_LOOP_TEST_CONFIG_RADIO`                       | `2`     | Number of radio config test loops                                                      |
| `LR20XX_PORTING_TEST_SPI_REGMEM_ENDURANCE`        | `n`     | LR20XX_SPI_ENDURANCE : Enable SPI endurance test                                       |
| `LR20XX_PORTING_TEST_SPI_REGMEM_NB_ITERATIONS`    | `10000` | LR20XX_SPI_ENDURANCE : Number of iterations for SPI endurance                          |
| `LR20XX_PORTING_TEST_SPI_REGMEM_SIZE_BYTES`       | `1024`  | LR20XX_SPI_ENDURANCE : Number of bytes per iteration for SPI endurance (multiple of 4) |

## Compilation

**Build sample:**
```
rm -Rf build/ ; cmake -L -S examples/main_examples/porting_tests_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target porting_tests

```build with LR20XX_SPI_ENDURANCE
rm -Rf build/ ; env CFLAGS="-DLR20XX_PORTING_TEST_SPI_REGMEM_ENDURANCE=1" cmake -L -S examples/main_examples/porting_tests_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -UCMAKE_C_FLAGS -G Ninja; cmake --build build --target porting_tests
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/porting_tests verify reset exit"
```

## Usage

1. **Build and Flash**: Compile and flash the application to target hardware
2. **Monitor Output**: Connect to UART/RTT console to view test results
3. **Automatic Execution**: Tests run automatically on startup and report pass/fail status
4. **Flash Tests** (if enabled): Requires MCU reset and relaunch to verify persistent storage

## Expected Output


### Standard Test Sequence
```
PORTING_TEST example is starting

----------------------------------------
 porting_test_spi :  OK
----------------------------------------
 porting_test_spi_regmem_endurance :
 regmem endurance: iteration 0
 regmem endurance: iteration 1000
 regmem endurance: iteration 2000
 regmem endurance: iteration 3000
 regmem endurance: iteration 4000
 regmem endurance: iteration 5000
 regmem endurance: iteration 6000
 regmem endurance: iteration 7000
 regmem endurance: iteration 8000
 regmem endurance: iteration 9000
 regmem endurance: iteration 9999
 OK
----------------------------------------
 porting_test_radio_irq :  OK
----------------------------------------
 porting_test_get_time :
 * Get time in second:  OK
 Time expected 5s / get 5s (no margin)
 * Get time in millisecond:
 Radio irq received but not RAL_IRQ_RX_TIMEOUT -> relaunched test
 * Get time in millisecond:  OK
 Time expected 1966ms / get 1966ms (margin +/-1ms)
----------------------------------------
 porting_test_timer_irq :  OK
 Timer irq configured with 1500ms / get 1500ms (margin +1ms)
----------------------------------------
 porting_test_stop_timer :  OK
----------------------------------------
 porting_test_disable_enable_irq :  OK
----------------------------------------
 porting_test_random :
 * Get random nb :  OK
 random1 = 4078310964, random2 = 3521158465
 * Get random nb in range :  OK
 random1 = 36, random2 = 9 in range [1;42]
 * Get random draw :  OK
 Random draw of 100000 numbers between [1;10] range
----------------------------------------
 porting_test_config_rx_radio : OK
----------------------------------------
 porting_test_config_tx_radio : OK
----------------------------------------
 porting_test_sleep_ms : OK
 Sleep time expected 2000ms / get 1999ms (margin +/-2ms)
----------------------------------------
 porting_test_timer_irq_low_power :  OK
 Timer irq configured with 1500ms / get 1500ms (margin +1ms)
----------------------------------------
END
```

## Technical Notes

- **Test Validation**: Each test validates specific HAL functions with timing tolerances and error margins
- **Hardware Dependencies**: Tests require proper GPIO, SPI, timer, and RTC configuration in device tree
- **Firmware Requirements**: LR11xx transceivers require compatible firmware versions for proper operation
- **Debug Support**: Comprehensive logging shows detailed test progress and failure diagnostics
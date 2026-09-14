# Immediate Radio Access

This application demonstrates **immediate radio access** via the USP/RAC API.
- `smtc_rac_immediate_radio_access()`
- `smtc_rac_release_immediate_radio_access()`

This examples requests radio access with high priority (RAC_VERY_HIGH_PRIORITY) to send LoRa frames immediately without going through the normal scheduling request (`smtc_rac_submit_radio_transaction()`).
The application uses button-triggered transmission: one press starts transmission.

## Key Features

- **Immediate RAC Access**: Pressing the button request Immediate access to the radio. In case a lower priority access was is in progress, It is aborted. Then the drivers of the chosen modulation shall be used. Finally, the radio shall be set to sleep and the Immediate Access released to allow other protocols to access the radio.
- **Button-Controlled Transmission**: Manual trigger to start transmission
- **LR2021**: Built for NUCLEO-L476 with LR2021 radio

## Compilation

**Build sample:**
```bash
rm -Rf build/ ; cmake -L -S examples/main_examples/immediate_radio_access_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target immediate_radio_access
```

**Example of `openocd` command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL_NUMBER>" -c "program build/immediate_radio_access verify reset exit"
```

## Usage

1. **Hardware Requirements**:
   - LR2021-based radio module
   - Board with user button support (e.g. NUCLEO-L476, XIAO-NRF54L15)

2. **Flash Firmware**: Load the compiled firmware to your device

3. **Connect Console**: Open serial console to monitor transmission activity
   ```bash
   minicom -D /dev/ttyACM0 -b 115200
   ```

4. **Trigger Transmission**: Press the user button to start transmission

5. **Monitor Activity**: UART shows hook IDs, button events and TX status

## Expected Output

WARNING: This trace comes from USP-Zephyr. The trace is not the same in USP.

### Initialization
```
INFO: Button: Initialized (GPIO EXTI on pin 0x2D)
Defined Hook IDs:
RP_HOOK_ID_SUSPEND: 0
RP_HOOK_RAC_VERY_HIGH_PRIORITY: 1
RP_HOOK_RAC_HIGH_PRIORITY: 2
RP_HOOK_RAC_MEDIUM_PRIORITY: 3
RP_HOOK_RAC_LOW_PRIORITY: 12
RP_HOOK_RAC_VERY_LOW_PRIORITY: 13
RP_HOOK_ID_RELAY_FORWARD_RXR: 4
RP_HOOK_ID_CLASS_B_BEACON: 5
RP_HOOK_ID_LR1MAC_STACK: 6
RP_HOOK_ID_LBT: 7
RP_HOOK_ID_CAD: 8
RP_HOOK_ID_CLASS_B_PING_SLOT: 9
RP_HOOK_ID_TEST_MODE: 10
RP_HOOK_ID_DIRECT_RP_ACCESS: 11
RP_HOOK_ID_RELAY_RX_CAD: 14
RP_HOOK_ID_RELAY_TX: 15
RP_HOOK_ID_CLASS_C: 16
RP_HOOK_ID_MAX: 17
INFO: Immediate radio access initialized - press button to start/stop transmission
```

### Button Press and Transmission
```
[MAIN-IMMEDIATE-RADIO-INFO ] [    8339 ms] button pressed

INFO: usp/rac: provide immediate radio access for first transmission
INFO: usp/rac: TX sent
INFO: usp/rac: TX sent
INFO: usp/rac: TX sent
INFO: usp/rac:end of execution of immediate radio access
```

## Technical Notes

- **Scheduling**: Immediate access bypasses normal RAC scheduling using smtc_rac_submit(); use with care in multi-stack setups

## Limitations

- **LR2021**: Example is built for LR2021; other radios may require adaptation

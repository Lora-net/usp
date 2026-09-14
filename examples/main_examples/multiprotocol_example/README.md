# Multiprotocol

This sample joins a LoRa network and periodically sends empty plinks.
It also make it possible to configure the ranging test role (manager or subordinate) and the ranging test priority via the shell commands. It starts a ranging test whenever the USER button is pressed or triggered by the shell.
The ranging test result is sent over LoraWan by the manager to the network server.

## Key Features

- **LoRaWAN Integration**: Periodic uplinks every 60 seconds (same as `periodical_uplink` sample)
- **Button-Triggered Ranging**: LoRa ranging tests initiated by button press (same as `ranging_demo` sample)
- **Multi-Protocol Management**: Coordinated access between LoRaWAN and ranging operations
- **Network Credentials**: Device provisioning via device tree overlays
- **shell integration**: Allow to configure and trigger actions

## Requirements

* Two boards with a LoRa transceiver supported by Zephyr.
* A LoRaWAN network
* Two uart consoles to interact with the boards

## Main shell commands

The following shell commands are available:
- mode <manager|subordinate> <priority>: set the ranging test role and priority
- ranging <start|info>: start a ranging test (only on manager) or get ranging result information
- req_time: send a request lorawan mac command to get the current time from the network server
- time: display the current time
- button: simulate a USER button press to start a ranging test (only on manager)
- uplink: send an uplink immediately
- status: display the current status of the application
- help: show all available commands

## Compilation

Note: USP version do not embed a shell.

**Build Sample**
```bash
rm -Rf build ; cmake -L -S examples/main_examples/multiprotocol_example -B build -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 -DRAC_RADIO=lr2021 -G Ninja; cmake --build build --target multiprotocol
```

**Example of `openocd`command to flash:**
```bash
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "adapter serial <SERIAL NUMBER>" -c "program build/multiprotocol verify reset exit"
```

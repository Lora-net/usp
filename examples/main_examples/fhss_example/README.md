# FHSS Example

This application demonstrates LoRa intra-packet frequency hopping between two LR20xx radios through the USP/RAC framework.

## Features

- Intra-packet frequency hopping using the LR20xx LoRa hopping engine
- LR20xx-to-LR20xx PING/PONG exchange
- Deterministic payload validation with retry handling
- TX/RX LED feedback

## Configuration

The defaults in `main_fhss_example.h` target LR20xx radios:

| Parameter | Default | Description |
|---|---:|---|
| `FHSS_HOP_PERIOD_IN_SYMBOLS` | `2` | Hop period in LoRa symbols |
| `TX_OUTPUT_POWER_DBM` | `14` | TX output power |
| `LORA_SPREADING_FACTOR` | `RAL_LORA_SF10` | LoRa spreading factor |
| `LORA_BANDWIDTH` | `RAL_LORA_BW_125_KHZ` | LoRa bandwidth |
| `LORA_CODING_RATE` | `RAL_LORA_CR_4_5` | LoRa coding rate |
| `LORA_PREAMBLE_LENGTH` | `12` | Preamble length |
| `LORA_PKT_LEN_MODE` | `RAL_LORA_PKT_EXPLICIT` | Packet length mode |
| `LORA_IQ` | `false` | IQ inversion |
| `LORA_CRC` | `true` | CRC enable |

## Compilation

```bash
rm -Rf build/
cmake -S examples/main_examples/fhss_example -B build \
  -DCMAKE_BUILD_TYPE=MinSizeRel -DBOARD=NUCLEO_L476 \
  -DRAC_RADIO=lr2021 -G Ninja
cmake --build build --target fhss_example
```

## Usage

1. Flash the same firmware to two LR20xx boards.
2. Start both boards; each starts as subordinate and waits in RX.
3. Press the user button on one board to make it manager and start PING/PONG exchanges.
4. Monitor UART output and TX/RX LEDs.

The example requires an LR20xx radio; the CMake configuration rejects other `RAC_RADIO` selections.

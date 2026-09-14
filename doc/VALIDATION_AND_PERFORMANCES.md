# Validation & Performances

This page summarizes **how** the software was validated, **with which configuration**, and a few
**performance / integration constraints** (SPI clock, upcoming memory footprint).

## Validation methodology

The release goes through Semtech's **nominal (full) validation process**. Validation relies on a
set of example applications, each exercising a different layer of the stack:

- **`lctt_certif_example`** — run against the official **LCTT (LoRaWAN Certification Test Tool)** to
  check LoRaWAN certification behavior (Class A/B/C, regions).
- **`periodical_uplink_example`** — end-to-end **LoRaWAN** behavior (periodic + button-triggered
  uplinks) against a network server.
- **`porting_tests_example`** — validates the **HAL / porting layer** on each platform: SPI access,
  radio IRQ, `get_time`, timers, RNG, sleep, low-power.
- **`hw_modem`** — host-controlled (serial) modem, exercised through its command interface.
- Other **RAC / SDK** examples (`ping_pong`, `packet_error_rate_*`, `cad`, `flrp_api`, …) exercise
  the individual radio features.

## Validation maturity levels

The same wording is used in the [root README](../README.md):

- **Validated** — passed the Semtech nominal full validation process.
- **Buildable** — can be compiled but did **not** go through the full Semtech validation process.
- **Experimental** — compiled and tested on the `periodical_uplink` sample only, with low
  validation.

## Validated configuration

- **Platform:** Validated on **STMicro NUCLEO-STM32L476RG**.
  - **Buildable** on Linux (x86 / x86_64 native + ARM cross-compilation).
  - **Experimental** on Renesas FPB-RA0E2 (only `porting_tests` & `periodical_uplink`, Class A,
    US915) and on STMicro NUCLEO-STM32L073RZ.
- **Radios:** Validated on the **LoRa Plus™ EVK** (LR2021 / LR2022 / LR2012); buildable on LR11xx
  and SX126x shields. See [LoRa Plus™ EVK](LORA_PLUS_EVK.md).
- See [Known Limitations](KNOWN_LIMITATIONS.md) for per-configuration caveats.

## Validation coverage per domain

Validation maturity and the SPI clock at which it was performed, per functional domain:

| Domain | Full validation | Also run / lighter testing | SPI / datarate notes |
|--------|-----------------|----------------------------|----------------------|
| **FLRP** | STM32L4 (NUCLEO-STM32L476RG) @ **10 MHz** | — | **Below 8 MHz the datarate must be lowered / adapted.** |
| **LoRaWAN** | STM32L4 (NUCLEO-STM32L476RG) @ **10 MHz** | `periodical_uplink` (Class A) also known to work on **Renesas FPB-RA0E2 @ 8 MHz**, **USP Linux @ 8 MHz** and **NUCLEO-STM32L073RZ (STM32L0) @ 4 MHz** | |
| **Other example apps** | STM32L4 (NUCLEO-STM32L476RG) @ **10 MHz** | — | Should also work at **much lower** SPI speeds |

## SPI clock speed

The maximum SPI clock used / validated per platform:

- **STM32L4 (NUCLEO-STM32L476RG):** **10 MHz**.
- **USP Linux:** **8 MHz**.
- **Renesas FPB-RA0E2:** **8 MHz**.
- **STM32L0 (NUCLEO-STM32L073RZ):** **4 MHz**.

Notes:
- Saleae Logic Analyzer probes were OK at these speeds.

## Memory footprint (upcoming)

**RAM / Flash footprint** figures per sample/configuration will be added in a **future release**.

---

*See also: [LoRa Plus™ EVK](LORA_PLUS_EVK.md) · [FLRP](FLRP.md) · [Known Limitations](KNOWN_LIMITATIONS.md)*

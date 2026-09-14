# FLRP-BURST — Guidelines

This page gathers the **practical guidelines** for using **FLRP-BURST** (the first feature of the FLRC-based **FLRP** family; more to come): regulatory compliance,
recommendations, configuration (with the default values) and known limitations.

> **More on FLRP-BURST:** for a deeper description of the FLRP-BURST benefits, recommandations & limitations, refer to the **Semtech
> Application Notes** available on the [Semtech website](https://www.semtech.com/products/wireless-rf/lora-plus).
> The **README files** in this repository provide the tools to **configure FLRP-BURST and enable / put in
> practice the FLRP-BURST protocol** (see the [FLRP overview](FLRP.md) and its linked examples).

## Regulatory compliance

- **Regulatory compliance is currently the responsibility of the application** (duty cycle, dwell
  time, channel plan, maximum EIRP, listen-before-talk / CCA where required, etc.). The protocol
  does not enforce these constraints on its own.
- **Work is ongoing** to add **Listen-Before-Talk (LBT)** to the protocol and to provide
  **ready-made configurations adapted to each region**.
- **To change what can be configured** (TX power, data rate, frequency plan, EUIs, MIC key,
  etc.), see the [Configuration](#configuration) section of this document, and the configuration
  section of the FLRP API example:
  [flrp_api_example — Configuration](../examples/main_examples/flrp_api_example/README.md#configuration).

## Recommendations

### Clock accuracy / frequency drift (target ≤ 60 ppm)

The LoRa WOR aligns the two devices in time and frequency **only once**, at the start of the
exchange; the FLRC burst is then sent back-to-back with **no per-packet resynchronization**, so any
residual clock error **accumulates as a timing drift** over the frame and the receiver ends up
**missing the last symbols / last packets**.

- **Manage your reference clock** so the combined error of both devices stays within the FLRC
  frequency tolerance (**~60 ppm**) — see the **LR20xx datasheet §18.1.4 *FLRC Frequency
  Tolerance*** (Table 18-3): the tolerance **shrinks with the raw datarate**, from ±150 kHz at
  2.6 Mbps down to ±25 kHz at 0.26 Mbps.
- FLRP-BURST solves this weakness by **measuring the drift via the LoRa exchange and applies a frequency offset on a
  best-effort basis, and is designed to cope with up to ~60 ppm** (`crystal_error` in the FLRP-BURST init
  config).
- FLRP-BURST also take into account the +-60ppm tolerated drift by shortening the frames depending of the
  used Coding Rate : 300 bytes for CR=1/2, 400 bytes for other CR.

### Coding rate & PA ramp time (sensitivity)

- The **default coding rate is CR 1/2** to reach the best sensitivity.
- For the **best possible sensitivity at CR 1/2, in the 2.4 GHz band, for the 1 Mb/s to 2 Mb/s raw
  data rates**, configure the **PA ramp time at least to 128 µs** (`SetTxParams` `ramp_time = 0x0A` — see the
  LR20xx datasheet Table 7-21 *Tx Ramp_time Parameter Definition*).
- **Warning:** the current default is **272 µs** (`ramp_time = 0x17`) because of constraints at CR 3/4 (see above). **PA ramp time** — depending on the platform:
  - **Baremetal (USP):** in the radio BSP ([`ral_lr20xx_bsp.c`](../examples/radio_hal/ral_lr20xx_bsp.c), `pa_ramp_time`, applied when compiling with `USE_FLRC_PROTOCOL`).
  - **Zephyr:** in the application devicetree overlays (`pa-ramp-time`) — see the board overlays of the [`flrp_api`](https://github.com/Lora-net/usp_zephyr/blob/main/samples/usp/rac/flrp_api/boards) and [`flrp_api_lorawan`](https://github.com/Lora-net/usp_zephyr/blob/main/samples/usp/rac/flrp_api_lorawan/boards) samples.
- **Important — the PA ramp time and the FLRC burst interframe delay must be changed together:**
  **128 µs ramp → 400 µs interframe**, **272 µs ramp → 700 µs interframe**. The interframe is set via
  `DATA_FLRC_MIN_INTERFRAME_DURATION_US` in
  [`flrp_configuration.h`](../protocols/smtc_flrc_protocol/flrp_configuration.h) (see
  [Interframe and PA ramp time tuning](#interframe-and-pa-ramp-time-tuning) in the Configuration section).

### WOR wake-up preset (custom)

The slave wakes up via **periodic LoRa CAD**; the two defaults (preset A / preset B) trade **wake-up
latency** against the **slave RX duty cycle** (see
[Principles → Low-power wake-up](FLRP_principles.md#low-power-wake-up)). You can define your **own
preset** by choosing a coherent `WOR_LORA_LONG_PREAMBLE_MS` / `RESTART_WOR_RX_DELAY_MS` pair in
[`flrp_configuration.h`](../protocols/smtc_flrc_protocol/flrp_configuration.h) — **both must be changed
together**:

- **Rule:** the slave RX period (`RESTART_WOR_RX_DELAY_MS` + the ~12.3 ms CAD window) **must stay
  shorter than the initiator's long preamble** (`WOR_LORA_LONG_PREAMBLE_MS`), keeping a **~5–8 %
  margin** for crystal drift and scheduler jitter — otherwise the slave can miss the WOR.
- **Trade-off:** a **longer** preamble lowers the slave duty cycle (lower slave power) but raises the
  wake-up latency and the initiator TX cost; a **shorter** preamble does the opposite.
- **Rough figures:** wake-up latency ≈ `WOR_LORA_LONG_PREAMBLE_MS`; slave RX duty cycle ≈ CAD window
  (~12.3 ms) / slave RX period.

For the exact formulas and worked examples, see the **WOR preset** comment block in
[`flrp_configuration.h`](../protocols/smtc_flrc_protocol/flrp_configuration.h).

## Configuration

FLRP-BURST can be configured at several levels, from the **most accessible** (application) to the **most
internal** (protocol implementation). Pick the level matching what you need:

- **FLRP API** — application code (`smtc_flrp_api_config_t` at init + a few runtime calls). The normal
  way to configure an application; it can also override the compile-time defaults at runtime.
- **Compile-time defaults** — `#define`s in
  [`flrp_configuration.h`](../protocols/smtc_flrc_protocol/flrp_configuration.h) (protocol & radio
  defaults: datarate, coding rate, channels, interframe, retry, WOR, TX power…). Change and recompile.
- **Radio hardware** — baremetal BSP
  ([`ral_lr20xx_bsp.c`](../examples/radio_hal/ral_lr20xx_bsp.c)) or Zephyr devicetree overlays
  (`pa-ramp-time`).
- **Internal protocol values** — constants baked into the FLRP-BURST implementation (packet size, max
  packets, dwell limit, beacon type, max payload…). Not meant to be changed.

For the parameters exposed directly by the sample application, see
[flrp_api_example — Configuration](../examples/main_examples/flrp_api_example/README.md#configuration).

### Via the FLRP API (application code)

| Parameter | How | Default |
|-----------|-----|---------|
| Device EUI | `smtc_flrp_api_config_t.dev_eui` (init) | — |
| Integrity (CRC vs MIC) | `smtc_flrp_api_config_t.crypto_enabled` (init) | — (see [Integrity: CRC vs MIC](#integrity-crc-vs-mic) below) |
| MIC key | `smtc_flrp_crypto_set_key(SMTC_SE_APP_KEY, …)` after `smtc_flrp_init()` | `flrc_default_key` (both devices must match) |
| Frequency plan | `smtc_flrp_api_config_t.freq_plan` (init) | 865 MHz (or 2.4 GHz) |
| Clock accuracy | `smtc_flrp_api_config_t.crystal_error` (ppm, init) | — (drift margin, ≤ ~60 ppm) |
| Communication mode | `smtc_flrp_com_config_t.com_mode` *(initiator, per transfer)* | `SMTC_FLRP_BIDIRECTIONAL` |
| Target device EUI | `smtc_flrp_com_config_t.slave_dev_eui` *(initiator)* | — |
| EUI filter length | `smtc_flrp_com_config_t.filter_len` *(initiator; used only in `SMTC_FLRP_COM_ONE_WAY`)* | — |
| Adaptive-link mode | `smtc_flrp_com_config_t.link_adaptation_mode` *(initiator)* | channel selection (disable with `SMTC_FLRP_LINK_ADAPTATION_DISABLED`) cf. below |
| Radio config override | `smtc_flrp_get`/`set_new_radio_config()` | defaults from `flrp_configuration.h` (datarate, TX power, delays, PER…) |
| FLRC coding rate & advanced radio params | `smtc_flrp_set_new_advanced_flrc_radio_config()` | `DATA_FLRC_CR` from `flrp_configuration.h` (initiator advertises the CR in the WOR) |

> **Only `SMTC_FLRP_BIDIRECTIONAL` is validated in this release.** `SMTC_FLRP_BIDIRECTIONAL_STREAM`
> and `SMTC_FLRP_COM_ONE_WAY` (one-way / broadcast) are available but **not yet validated**.

> **If the adaptive link is disabled** (`SMTC_FLRP_LINK_ADAPTATION_DISABLED`), the FLRC REQ/ACK phase
> is skipped, so neither the **full payload size** nor the **uniform packet size** is signalled: the
> **receiver expects a payload of the exact size of the RX buffer** it passes to
> `smtc_flrp_start_periodic_listening()` / `smtc_flrp_initiate_reception()` (it must equal what the
> transmitter sends — both sides derive the uniform packet size from it), and the **burst interframe
> must match** on both sides. Datarate, coding rate and channel are still carried by the WOR.

### Compile-time defaults — [`flrp_configuration.h`](../protocols/smtc_flrc_protocol/flrp_configuration.h)

| Parameter | Define | Default |
|-----------|--------|---------|
| TX power | `TX_OUTPUT_POWER_DBM` | 14 dBm |
| FLRC raw bit rate | `DATA_FLRC_RAW_BIT_RATE` | 2.6 Mbps (0.26 – 2.6) |
| FLRC coding rate | `DATA_FLRC_CR` | CR 1/2 |
| FLRC burst interframe | `DATA_FLRC_MIN_INTERFRAME_DURATION_US` | 700 µs (field range 0 – 25.5 ms) — **must be paired with the PA ramp time; see [Interframe and PA ramp time tuning](#interframe-and-pa-ramp-time-tuning) below** |
| Channels count | `DATA_FLRC_NB_FREQ` | 3 |
| Channel hopping step | `DATA_FLRC_CHANNEL_STEP_865MHz_HZ` / `_2GHz4_HZ` | 2.7 MHz / 5 MHz — channel N center = base + step × N; adjust the step to keep all channels within your market's regulatory sub-band |
| Retry / burst target PER | `DATA_FLRC_BURST_TARGET_PER_PERCENTAGE` | 0 % (0 → retry ≤ 3× burst TOA; 100 → no retry; between → retry only missing packets) |
| FLRC CRC (crypto off) | `DATA_FLRC_CRC` | 2 bytes |
| WOR / FLRC frequency | `WOR_865MHz_RF_FREQ_IN_HZ` / `WOR_2GHz4_RF_FREQ_IN_HZ` | 865.1 MHz / 2.412 GHz |
| WOR modulation | `WOR_LORA_SPREADING_FACTOR` / `_BANDWIDTH` / `_CODING_RATE` | SF10, BW 500 kHz, CR 4/8 (LI), private syncword |
| WOR low-power preset | `WOR_LORA_LONG_PREAMBLE_MS` / `RESTART_WOR_RX_DELAY_MS` | preset A (1 s / 950 ms, slave RX ~1.3 %); preset B (100 ms / 80 ms, ~13 %) |
| WOR slave RX window (CAD) | `WOR_SYM_NB_TIMEOUT` | 6 symbols (≈ 12.3 ms at SF10/BW500) |
| WOR ACK delay / preamble | `WOR_ACK_DELAY_MS` / `WOR_ACK_LORA_PREAMBLE_LENGTH` | 5 ms / 12 symbols |
| MIC default key | `flrc_default_key` | 16-byte AES-128 |

### Radio hardware — BSP / devicetree overlays

| Parameter | Where | Default |
|-----------|-------|---------|
| PA ramp time | baremetal BSP [`ral_lr20xx_bsp.c`](../examples/radio_hal/ral_lr20xx_bsp.c) (`pa_ramp_time`, under `USE_FLRC_PROTOCOL`) / Zephyr overlay `pa-ramp-time` | 272 µs (pair with interframe: 128↔400 µs, 272↔700 µs — see [Interframe and PA ramp time tuning](#interframe-and-pa-ramp-time-tuning)) |

### Interframe and PA ramp time tuning

The recommended interframe & PA ramp time settings for the best performance and sensitivity are the
following. The **PA ramp time and the FLRC burst interframe delay must be tuned together**. The expected
gross burst bit-rate (raw payload), from the **max (2.6 Mb/s)** down to the **min (0.52 Mb/s)** datarate,
is also given.

| Use case | Recommended FLRC burst interframe | Recommended PA ramp time | Expected gross bit-rate (2.6 → 0.52 Mb/s datarate) |
|----------|-----------------------------------|--------------------------|----------------------------------------------------|
| **2.4 GHz, low datarate** | **≥ 400 µs** | **≥ 128 µs** (`ramp_time = 0x0A`) | na |
| **CR 1/2 near sensitivity** | **≥ 400 µs** | **≥ 128 µs** (`ramp_time = 0x0A`) | ~1.08 → ~0.24 Mb/s |
| **CR 3/4 near sensitivity** | **≥ 700 µs** | **≥ 272 µs** (`ramp_time = 0x17`) | ~1.37 → ~0.35 Mb/s |
| **Other cases (minimum)** | **200 µs** | **48 µs** | ~1.16 → ~0.25 Mb/s (CR 1/2) · ~1.73 → ~0.37 Mb/s (CR 3/4) |

The current default configuration (interframe = 700 µs & PA ramp time = 272 µs) is focused on the best sensitivity regardless of the use case.

How to configure these values (**always change the PA ramp time and the interframe together**):

- **FLRC burst interframe delay** — set `DATA_FLRC_MIN_INTERFRAME_DURATION_US` in
  [`flrp_configuration.h`](../protocols/smtc_flrc_protocol/flrp_configuration.h).
- **PA ramp time** — depending on the platform:
  - **Baremetal (USP):** in the radio BSP ([`ral_lr20xx_bsp.c`](../examples/radio_hal/ral_lr20xx_bsp.c), `pa_ramp_time`, applied when compiling with `USE_FLRC_PROTOCOL`).
  - **Zephyr:** in the application devicetree overlays (`pa-ramp-time`) — see the board overlays of the [`flrp_api`](https://github.com/Lora-net/usp_zephyr/blob/main/samples/usp/rac/flrp_api/boards) and [`flrp_api_lorawan`](https://github.com/Lora-net/usp_zephyr/blob/main/samples/usp/rac/flrp_api_lorawan/boards) samples.
  - **Warning:** the current default is **272 µs** (`ramp_time = 0x17`).

### Internal protocol values (advanced — normally not changed)

| Value | Source | Default |
|-------|--------|---------|
| WOR beacon type (protocol id) | `SMTC_WOR_TYPE_FLRC` (`smtc_wor.h`) | 56 |
| Max packets per burst | `SMTC_FLRP_PACKETS_IN_BURST_MAX` (`flrp_defs.h`) | 255 |
| FLRC packet size | `get_packet_payload_size()` | 300 (CR 1/2) / 400 bytes |
| Burst dwell-time limit | `MAC_BURST_MAX_TIME_MS` | 400 ms (max single-burst duration) |
| Max data per single burst | derived (255 × 300/400) | ~76 – 102 kB |
| Max total payload | `MAC_DATA_SIZE_MAX` (24-bit field) | 16 MB |
| FLRC REQ start timing | 16-bit × 100 µs field (set by initiator) | 0 – 6.55 s |

### Integrity: CRC vs MIC

`crypto_enabled` (in `smtc_flrp_api_config_t`, chosen at init) selects the per-packet integrity check:

- **disabled** → FLRC packets use the **radio CRC** (`DATA_FLRC_CRC`, default `RAL_FLRC_CRC_2_BYTES`);
- **enabled** → the **radio CRC is turned off** (`RAL_FLRC_CRC_OFF`) and replaced by a per-packet
  **MIC** (AES-CMAC). The two are mutually exclusive.

**Which to choose** (both are integrity only — neither encrypts the payload):

- **CRC** (2 bytes, no key, hardware) — when performance / latency prevails and there is no injection
  threat (max throughput, trusted environment).
- **MIC** (4 bytes, shared AES-128 key required, software) — when authenticity / anti-spoofing is
  needed; its per-packet cost weighs on the interframe budget → validate against the MCU timing.

The MIC uses an AES-128 key stored as `SMTC_SE_APP_KEY` — **not part of `smtc_flrp_api_config_t`**: its
default is `flrc_default_key` in
[`flrp_configuration.h`](../protocols/smtc_flrc_protocol/flrp_configuration.h), and the application
overrides it by calling `smtc_flrp_crypto_set_key(SMTC_SE_APP_KEY, key, 0)` **after `smtc_flrp_init()`**
(as done in [`app_flrp_api.c`](../examples/main_examples/flrp_api_example/app_flrp_api.c)). Both devices
must use the same key.

## Limitations

Not yet fixed / not validated:
- Only **point-to-point** communication is currently used and validated; **multicast mode**
  configuration is not available.
- **Frequency-offset management** is currently applied **only at the start** of the exchange; applying
  it **again later**, depending on the phase durations, is planned.
- CN470 sub-GHz band: Due to the high level of interference in this band, adaptive channel selection and FLRC BURST retry may cause transfer failures in some circumstances. Disabling both is recommended if the issue occurs.
- 2.4 GHz band: Best sensitivity requires a PA Ramp Time of 128 µs or higher (the 272 µs default value meets this). Lower values degrade performance. Additional configurations are under investigation.
- In Phase-3 (Data transfer), each transfer is limited to 255 bursts, and each burst is limited to a maximum of 255 packets or 400 ms of time-on-air. At low datarate the maximum transferable size drops (~1.35 MB worst case vs 16 MB best case) due to the ToA limitation. This 400ms ToA limit can be increased to suit your use case (MAC_BURST_MAX_TIME_MS in smtc_flrp_mac_layer.c).
- In Phase-2 (Adaptive Link), the default configuration supports dynamically configuring up to 3 channels. Using more requires recompiling both the initiator and slave with matching values for DATA_FLRC_NB_FREQ and the channel-step defines (DATA_FLRC_CHANNEL_STEP_865MHz_HZ / DATA_FLRC_CHANNEL_STEP_2GHz4_HZ). These defines shall be identical on both devices.
- Disabling Adaptive Link is not recommended, as only predefined-size bursts can be used in that mode.

Features not yet developed:
- LBT/CSMA is not managed in FLRP-BURST.
- **Data-rate selection** in the adaptive link is **not yet implemented** (channel selection only for now).
- Many parameters are currently set as **static configuration** and are expected to become **automatic**
  in a future release.
- **No payload confidentiality:** FLRP-BURST provides **integrity / authentication only** (optional per-packet
  AES-CMAC MIC); the **payload is not encrypted**. Add application-level encryption if confidentiality is required.
- Security is **AES-128** based; key management is static and shall be managed by the application.

## Troubleshooting

For a symptom → cause → fix table (clock / interframe, PA ramp, MIC keys, adaptive link disabled…),
see [FLRP API — Troubleshooting](../protocols/smtc_flrc_protocol/smtc_flrp_api/README.md#troubleshooting).

---

See also: [FLRP-BURST — Principles](FLRP_principles.md) · [FLRP overview](FLRP.md)

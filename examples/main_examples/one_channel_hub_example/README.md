# One Channel Hub Example

A **dual-function device** based on Semtech's USP architecture that combines:
1. **1-channel LoRaWAN gateway (hub)** — forwards LoRa packets between end-devices and an LNS via UDP.
2. **LoRaWAN end-device (client)** — sends alert messages when UDP connectivity to the LNS is lost/restored.

It integrates the LoRa packet forwarder with RAC (Radio Access Controller) and, optionally, LBM
(LoRa Basics Modem), so both the gateway and an end-device run on a **single radio**.

> **Platform:** Linux only (POSIX threads, SPI/GPIO character devices, UDP packet forwarder).

## Overview

### Gateway functionality (always active)
- Receives uplink packets from LoRa end-devices.
- Forwards them to a LoRaWAN Network Server (LNS) over UDP (Semtech protocol).
- Receives downlinks from the LNS and transmits them via Just-In-Time (JIT) scheduling.
- Monitors UDP connection health.

### LoRaWAN client functionality (optional)
When enabled in configuration, the device also:
- Joins a LoRaWAN network.
- Sends an alert uplink when the gateway↔LNS UDP link is lost or restored.
- Sends periodic keep-alive status updates.

## Directory structure

```
one_channel_hub_example/
├── CMakeLists.txt            # Per-example build (this folder is a cmake source dir)
├── main_one_channel_hub.c    # Application entry point (int main) coordinating both roles
├── main_one_channel_hub.h    # LoRa parameters
├── lorawan_client_conf.c/.h  # LoRaWAN client config parser
├── lorawan_client.json       # LoRaWAN client configuration (optional)
├── smtc_modem_hal_protect.c  # RAC API lock: recursive-mutex override of the protect hooks
├── libloragw/                # Gateway HAL (libloragw API implemented on top of RAC)
│   ├── inc/{loragw_hal.h, loragw_aux.h}
│   └── src/{loragw_hal.c, loragw_aux.c}
└── packet_forwarder/         # Semtech UDP packet forwarder
    ├── inc/{lora_pkt_fwd.h, jitqueue.h, base64.h, parson.h, trace.h}
    ├── src/{lora_pkt_fwd.c, jitqueue.c, base64.c, parson.c}
    ├── global_conf.json      # Gateway configuration file
    └── PROTOCOL.md           # Semtech UDP protocol documentation
```

## Architecture

### Radio coordination
A **single radio** is shared between gateway and client roles:
- **Gateway RX** — continuous listen for uplinks (via RAC).
- **Gateway TX** — scheduled downlinks via the JIT queue (via RAC).
- **Client TX/RX** — LoRaWAN uplinks/downlinks (via LBM → RAC).

The **RAC engine** coordinates all radio access through a transaction-based system. These roles,
together with the radio-IRQ and timer delivery, run across several threads, so all access to the
shared radio planner is serialized by the RAC API lock (see [Synchronization](#synchronization)).

### Thread architecture
The application uses **5 application threads** (the Linux HAL additionally runs two internal threads —
a GPIO radio-IRQ thread and an LP-timer thread; see [Synchronization](#synchronization)):

| Thread | Name | Source | Description |
|--------|------|--------|-------------|
| 0 | Main | `main_one_channel_hub.c` | Runs `smtc_rac_run_engine()` (always) and `smtc_modem_run_engine()` (if client enabled); processes radio transactions for both roles; services connection alerts. |
| 1 | pkt_fwd_thread | `main_one_channel_hub.c` | Wrapper calling `main_pkt_fwd()` — config, sockets, statistics loop. |
| 2 | thread_up | `lora_pkt_fwd.c` | **Upstream**: `lgw_receive()` → JSON → `PUSH_DATA` to LNS. |
| 3 | thread_down | `lora_pkt_fwd.c` | **Downstream**: `PULL_DATA`/`PULL_RESP`, enqueue to JIT; monitors connection health, triggers alerts. |
| 4 | thread_jit | `lora_pkt_fwd.c` | **JIT TX**: sends queued downlinks at their scheduled time via `lgw_send()`. |

**HAL-internal threads (Linux):**
- **GPIO radio-IRQ thread** — polls the radio DIO line and dispatches the radio IRQ callback.
- **LP-timer thread** — a `timerfd`-backed thread delivering LoRaWAN timer expiries.

Both run the radio callback under the RAC API lock and wake the main loop via `SIGUSR2`.

### RAC callbacks (gateway) / LBM events (client)
- RAC: `rx_pre/post_transaction_callback`, `tx_pre/post_transaction_callback` drive the gateway RX queue and TX status.
- LBM: `RESET` (join), `JOINED` (send pending alerts), `ALARM` (keep-alive), `TXDONE`, `DOWNDATA`, `JOINFAIL`.

## Synchronization

| Mutex | Purpose | Used by |
|-------|---------|---------|
| RAC API lock | Serializes all access to the shared radio planner / transceiver (recursive) | `smtc_modem_run_engine()` wrap (main), all `smtc_rac_*` calls internally, GPIO-IRQ & LP-timer HAL threads |
| `lgw_state_mutex` | Gateway HAL state in libloragw: `is_started`, `rx_status`, `tx_status` | `lgw_*` (PF threads), RAC completion callbacks |
| `mx_meas_up` / `mx_meas_dw` | Upstream / downstream statistics | up/down/jit, main_pkt_fwd |
| `mx_stat_rep` | Status report JSON | main_pkt_fwd, thread_up |
| `rx_queue_mutex` | RX packet queue (in HAL) | RX callbacks, `lgw_receive()` |
| `alert_flag.mutex` | Connection alert status | thread_down, main thread |

### Radio access serialization (RAC API lock)
The radio planner and the transceiver are shared singletons but are driven from several threads. Most
access goes through the RAC API, which serializes itself on a single **recursive** lock exposed
through the modem-HAL hooks `smtc_modem_hal_protect_api_call()` / `smtc_modem_hal_unprotect_api_call()`
— the software equivalent of IRQ masking on an MCU. It must be recursive because the engine holds it
across `rp_callback()`, which re-enters the RAC API on the same thread.

The application takes this lock directly only where the planner is touched outside the RAC API: around
`smtc_modem_run_engine()` in the main loop (the LBM engine does not lock itself), and in the two HAL
threads that deliver the radio DIO interrupt (GPIO) and the LoRaWAN timer (`timerfd`) before invoking
the radio callback. The gateway HAL (libloragw) does **not** take this lock — its `smtc_rac_*` calls
self-serialize, and its own mutable state is guarded separately by `lgw_state_mutex`.

These hooks are **weak no-ops** by default (correct for the single-threaded examples); the
one-channel-hub provides a strong recursive-mutex implementation in `smtc_modem_hal_protect.c`, so the
lock is active only for this example.

The HAL IRQ/timer threads also send a thread-directed `SIGUSR2` to the main thread after running their
callback, so a main loop blocked in `nanosleep()` (`hal_mcu_set_sleep_for_ms`) wakes immediately and
re-runs the engines.

## Building

Built with the USP build system: point cmake at this example directory and select a target board
and radio.

**Options**
- `BOARD`: `LINUX` (native x86/x86_64 — compile-checks), `LINUX_ARM`, `LINUX_ARM64` (Raspberry Pi / embedded Linux).
- `RAC_RADIO`: `lr2021` (validated on Linux), `lr1110`/`lr1120`/`lr1121`, `sx1261`/`sx1262`/`sx1268`.

**Raspberry Pi (AArch64), LR2021:**
Ensure the AArch64 cross toolchain (`aarch64-linux-gnu-*`) is in your `PATH`, then:
```bash
cmake -S examples/main_examples/one_channel_hub_example -B build \
    -DBOARD=LINUX_ARM64 -DRAC_RADIO=lr2021 -G Ninja
cmake --build build
```
For 32-bit ARM use `-DBOARD=LINUX_ARM`; for another radio change `-DRAC_RADIO=` (e.g. `sx1262`, `lr1121`).

**Run prerequisites (target):** SPI at `/dev/spidev0.0`, GPIO at `/dev/gpiochip0`, user in the `spi`
and `gpio` groups. See the [Linux HAL documentation](../../smtc_hal_linux/README.md).

## Configuration

Two configuration files (searched in CWD, or via env vars `GLOBAL_CONF` / `LORAWAN_CLIENT_CONF`):

1. **`global_conf.json`** — gateway configuration (**required**): `one_channel_hub_conf` (radio:
   `rx_freq_hz`, `datarate`, `bandwidth`, `tx_*`) and `gateway_conf` (`gateway_ID`, `server_address`,
   `serv_port_up/down`, `keepalive_interval`, `autoquit_threshold`, `stat_interval`, …).
2. **`lorawan_client.json`** — LoRaWAN client (**optional**; absent ⇒ gateway-only): `enabled`,
   `dev_eui`, `join_eui`, `app_key`, `region`, `keep_alive_interval_s`, `alert_port`.

**`autoquit_threshold`** — number of consecutive `PULL_DATA` without `PULL_ACK` that flags the LNS link as lost:
- `0` — connection monitoring disabled (plain gateway).
- `> 0` — flag the LNS connection **lost** after N consecutive `PULL_DATA` without `PULL_ACK`, and
  **restored** on the next `PULL_ACK`; on each change drive the status LEDs (red = lost, green = OK)
  and, if the client is enabled, send an alert uplink on `alert_port`.

**Alert message:** 1-byte payload on `alert_port` (default 102): `0` = OK, `1` = LOST. Sent on status
change and as a periodic keep-alive (`keep_alive_interval_s`); queued until `JOINED` if not yet joined.

## Usage

```bash
# Gateway only (set autoquit_threshold = 0, omit/disable lorawan_client.json)
./one_channel_hub

# Gateway + client: configure both files, register the client device on the LNS, then:
GLOBAL_CONF=/path/global_conf.json LORAWAN_CLIENT_CONF=/path/lorawan_client.json ./one_channel_hub
```

## Testing with ThreadSanitizer (TSan)

The hub is multi-threaded and shares one radio across threads, so it is validated with
**ThreadSanitizer** to prove there are no data races on the radio planner / RAC state.

### 1. Build with TSan (cross-compiled for the Pi)
The AArch64 toolchain ships `libtsan` (including a static archive), so the runtime can be linked
statically — nothing extra needs to be copied to the target.

With the AArch64 cross toolchain in your `PATH`:
```bash
rm -rf build
env CFLAGS="-fsanitize=thread -fno-omit-frame-pointer -g" \
    LDFLAGS="-fsanitize=thread -static-libtsan" \
cmake -S examples/main_examples/one_channel_hub_example -B build \
    -DBOARD=LINUX_ARM64 -DRAC_RADIO=lr2021 \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -UCMAKE_C_FLAGS -G Ninja
cmake --build build
```
Notes:
- `CFLAGS`/`LDFLAGS` are env vars (not `-DCMAKE_C_FLAGS=`) so they **append** to the toolchain's
  flags; `-UCMAKE_C_FLAGS` clears any stale cache entry. This instruments **all** code (LBM, RAC,
  HAL, example) so races are seen across the whole program.
- `RelWithDebInfo` + `-fno-omit-frame-pointer -g` gives usable stacks (avoid `MinSizeRel` for TSan).
- TSan is heavy (much higher CPU/RAM) — expected.

### 2. Run and capture a separate log
TSan writes to **stderr** by default, which interleaves with the app logs. Redirect it to its own
file with `TSAN_OPTIONS=log_path=`:

```bash
TSAN_OPTIONS="log_path=tsan_och.log:history_size=7:second_deadlock_stack=1:halt_on_error=0" \
./one_channel_hub
```
- Reports are written to **`tsan_och.log.<pid>`**; the console keeps only the application logs.
- `history_size=7` deepens race stacks, `second_deadlock_stack=1` shows both lock stacks,
  `halt_on_error=0` keeps running so a whole run's races are collected.

For symbolized stacks the target needs `addr2line`/`llvm-symbolizer` (`sudo apt install binutils`).
Otherwise symbolize on the host against the unstripped binary:
`aarch64-linux-gnu-addr2line -f -e build/one_channel_hub 0x<addr>`.

## License

The Clear BSD License — Copyright Semtech Corporation 2025. All rights reserved.

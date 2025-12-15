# LoRa Basics Modem (LBM) Porting Guide to USP

## Overview

This guide explains how to port existing LoRa Basics Modem applications to the **USP** environment. It covers the key API changes, initialization sequence.

**Target Audience:** Developers porting existing Legacy LBM applications (from bare-metal) to USP.

---

## Table of Contents

1. [Key Changes Summary](#key-changes-summary)
2. [Initialization Sequence](#initialization-sequence)
3. [Main Loop Integration](#main-loop-integration)
4. [Multithreading Considerations](#multithreading-considerations)
5. [Compile Definitions Management](#compile-definitions-management)

---

## Key Changes Summary

When porting from traditional LBM to USP, you need to:

| Traditional LBM | USP | Reason |
|----------------|----------------|--------|
| `smtc_modem_init()` only | **`smtc_rac_init()` + `smtc_modem_init()`** | RAC (Radio Access Controller) must be initialized first |
| `smtc_modem_run_engine()` only | **`smtc_modem_run_engine()` + `smtc_rac_run_engine()`** | Both engines need to run |

---

## Initialization Sequence

### ✅ Correct Initialization Order

```c
#include <smtc_rac_api.h>
#include <smtc_modem_api.h>

int main(void)
{
    // 1. Initialize RAC (Radio Access Controller) - MUST BE FIRST
    smtc_rac_init());
    
    // 2. Initialize LBM modem - AFTER RAC
    smtc_modem_init(&modem_event_callback);
    
    // Your application code...
}
```

### ⚠️ Critical: Order Matters!

**`smtc_rac_init()` MUST be called BEFORE `smtc_modem_init()`**

This is because:
- RAC manages the radio hardware abstraction layer
- LBM depends on RAC for radio operations
- Calling them in wrong order will cause initialization failures

---

## Main Loop Integration

### Traditional LBM Main Loop

```c
// Traditional (bare-metal or FreeRTOS)
while(1) {
    uint32_t sleep_time_ms = smtc_modem_run_engine();
    // Sleep or handle events
    sleep(sleep_time_ms);
}
```

### USP Main Loop

```c
// USP - TWO engines to run!
while(true) {
    // 1. Run the modem engine
    uint32_t sleep_time_ms = smtc_modem_run_engine();
    
    // 2. Run the RAC engine - REQUIRED!
    smtc_rac_run_engine();
    
    // 3. Check for pending IRQ before sleeping
    hal_mcu_disable_irq( );
    if(smtc_rac_is_irq_flag_pending()) 
    {
        hal_mcu_enable_irq( );
        continue; // Don't sleep if radio IRQ is pending
    }
    
    // 4. Wait for events or timeout
    hal_mcu_set_sleep_for_ms( MIN( sleep_time_ms, WATCHDOG_RELOAD_PERIOD_MS ) );
    hal_mcu_enable_irq( );
}
```

### Key Points

1. **`smtc_rac_run_engine()` is MANDATORY** - Must be called in addition to `smtc_modem_run_engine()`
2. **Check IRQ flags** - Use `smtc_rac_is_irq_flag_pending()` to avoid missing radio events

---

## Reference Examples

### Minimal Working Example
See: `samples/usp/lbm/periodical_uplink/`

This example demonstrates:
- Proper initialization sequence
- Both run engines
- Event handling with semaphores
- Relay TX configuration
- Thread-safe API usage

### Key Files to Study
1. `examples/main_examples/main_periodical_uplink.c` - Complete application example
2. `doc/usp_porting_guide.md` - Hardware porting guide

---


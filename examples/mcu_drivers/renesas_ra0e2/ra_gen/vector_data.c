/* generated vector source file - do not edit */
#include "bsp_api.h"
/* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
#if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [2] = r_icu_isr, /* ICU IRQ0 (External pin interrupt 0) */
            [7] = r_icu_isr, /* ICU IRQ5 (External pin interrupt 5) */
            [18] = sau_spi_txrxi_isr, /* SAU0 SPI TXRXI00 (SAU UART TX 0/I2C 00/SPI 00) */
            [19] = tau_tmi_isr, /* TAU0 TMI00 (End of timer channel 00 count or capture) */
            [32] = rtc_c_alarm_prd_or_alm_isr, /* RTC ALARM OR PERIOD (Alarm or Periodic interrupt) */
            [33] = tml_itl_or_isr, /* TML0 ITL OR (TML timer event) */
            [39] = uarta_eri_isr, /* UARTA0 ERRI (UARTA0 reception communication error occurrence) */
            [40] = uarta_txi_isr, /* UARTA0 TXI (UARTA0 transmission transfer end or buffer empty interrupt) */
            [41] = uarta_rxi_isr, /* UARTA0 RXI (UARTA0 reception transfer end) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [2] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ0,GROUP2), /* ICU IRQ0 (External pin interrupt 0) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_ICU_IRQ5,GROUP7), /* ICU IRQ5 (External pin interrupt 5) */
            [18] = BSP_PRV_VECT_ENUM(EVENT_SAU0_SPI_TXRXI00,GROUP2), /* SAU0 SPI TXRXI00 (SAU UART TX 0/I2C 00/SPI 00) */
            [19] = BSP_PRV_VECT_ENUM(EVENT_TAU0_TMI00,GROUP3), /* TAU0 TMI00 (End of timer channel 00 count or capture) */
            [32] = BSP_PRV_VECT_ENUM(EVENT_RTC_ALARM_OR_PERIOD,FIXED), /* RTC ALARM OR PERIOD (Alarm or Periodic interrupt) */
            [33] = BSP_PRV_VECT_ENUM(EVENT_TML0_ITL_OR,FIXED), /* TML0 ITL OR (TML timer event) */
            [39] = BSP_PRV_VECT_ENUM(EVENT_UARTA0_ERRI,FIXED), /* UARTA0 ERRI (UARTA0 reception communication error occurrence) */
            [40] = BSP_PRV_VECT_ENUM(EVENT_UARTA0_TXI,FIXED), /* UARTA0 TXI (UARTA0 transmission transfer end or buffer empty interrupt) */
            [41] = BSP_PRV_VECT_ENUM(EVENT_UARTA0_RXI,FIXED), /* UARTA0 RXI (UARTA0 reception transfer end) */
        };
        #endif
        #endif

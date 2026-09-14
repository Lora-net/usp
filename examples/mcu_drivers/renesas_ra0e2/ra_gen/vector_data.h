/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (9)
#endif
/* ISR prototypes */
void r_icu_isr(void);
void sau_spi_txrxi_isr(void);
void tau_tmi_isr(void);
void rtc_c_alarm_prd_or_alm_isr(void);
void tml_itl_or_isr(void);
void uarta_eri_isr(void);
void uarta_txi_isr(void);
void uarta_rxi_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_ICU_IRQ0 ((IRQn_Type) 2) /* ICU IRQ0 (External pin interrupt 0) */
#define ICU_IRQ0_IRQn          ((IRQn_Type) 2) /* ICU IRQ0 (External pin interrupt 0) */
#define VECTOR_NUMBER_ICU_IRQ5 ((IRQn_Type) 7) /* ICU IRQ5 (External pin interrupt 5) */
#define ICU_IRQ5_IRQn          ((IRQn_Type) 7) /* ICU IRQ5 (External pin interrupt 5) */
#define VECTOR_NUMBER_SAU0_SPI_TXRXI00 ((IRQn_Type) 18) /* SAU0 SPI TXRXI00 (SAU UART TX 0/I2C 00/SPI 00) */
#define SAU0_SPI_TXRXI00_IRQn          ((IRQn_Type) 18) /* SAU0 SPI TXRXI00 (SAU UART TX 0/I2C 00/SPI 00) */
#define VECTOR_NUMBER_TAU0_TMI00 ((IRQn_Type) 19) /* TAU0 TMI00 (End of timer channel 00 count or capture) */
#define TAU0_TMI00_IRQn          ((IRQn_Type) 19) /* TAU0 TMI00 (End of timer channel 00 count or capture) */
#define VECTOR_NUMBER_RTC_ALARM_OR_PERIOD ((IRQn_Type) 32) /* RTC ALARM OR PERIOD (Alarm or Periodic interrupt) */
#define RTC_ALARM_OR_PERIOD_IRQn          ((IRQn_Type) 32) /* RTC ALARM OR PERIOD (Alarm or Periodic interrupt) */
#define VECTOR_NUMBER_TML0_ITL_OR ((IRQn_Type) 33) /* TML0 ITL OR (TML timer event) */
#define TML0_ITL_OR_IRQn          ((IRQn_Type) 33) /* TML0 ITL OR (TML timer event) */
#define VECTOR_NUMBER_UARTA0_ERRI ((IRQn_Type) 39) /* UARTA0 ERRI (UARTA0 reception communication error occurrence) */
#define UARTA0_ERRI_IRQn          ((IRQn_Type) 39) /* UARTA0 ERRI (UARTA0 reception communication error occurrence) */
#define VECTOR_NUMBER_UARTA0_TXI ((IRQn_Type) 40) /* UARTA0 TXI (UARTA0 transmission transfer end or buffer empty interrupt) */
#define UARTA0_TXI_IRQn          ((IRQn_Type) 40) /* UARTA0 TXI (UARTA0 transmission transfer end or buffer empty interrupt) */
#define VECTOR_NUMBER_UARTA0_RXI ((IRQn_Type) 41) /* UARTA0 RXI (UARTA0 reception transfer end) */
#define UARTA0_RXI_IRQn          ((IRQn_Type) 41) /* UARTA0 RXI (UARTA0 reception transfer end) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (42)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */

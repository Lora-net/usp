/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_lpm.h"
#include "r_lpm_api.h"
#include "r_iwdt.h"
#include "r_wdt_api.h"
#include "r_flash_lp.h"
#include "r_flash_api.h"
#include "r_tml.h"
#include "r_timer_api.h"
#include "r_tau.h"
#include "r_timer_api.h"
#include "r_sau_spi.h"
#include "r_spi_api.h"
#include "r_rtc_c.h"
#include "r_rtc_api.h"
#include "r_uarta.h"
#include "r_uart_api.h"
FSP_HEADER
/** lpm Instance */
extern const lpm_instance_t g_lpm0;

/** Access the LPM instance using these structures when calling API functions directly (::p_api is not used). */
extern lpm_instance_ctrl_t g_lpm0_ctrl;
extern const lpm_cfg_t g_lpm0_cfg;
/** WDT on IWDT Instance. */
extern const wdt_instance_t g_wdt0;

/** Access the IWDT instance using these structures when calling API functions directly (::p_api is not used). */
extern iwdt_instance_ctrl_t g_wdt0_ctrl;
extern const wdt_cfg_t g_wdt0_cfg;

#ifndef NULL
void NULL(wdt_callback_args_t *p_args);
#endif
/* Flash on Flash LP Instance. */
extern const flash_instance_t g_flash0;

/** Access the Flash LP instance using these structures when calling API functions directly (::p_api is not used). */
extern flash_lp_instance_ctrl_t g_flash0_ctrl;
extern const flash_cfg_t g_flash0_cfg;

#ifndef NULL
void NULL(flash_callback_args_t *p_args);
#endif
/** Timer on TML Instance. */
extern const timer_instance_t g_timer_tml1;

/** Access the TML instance using these structures when calling API functions directly (::p_api is not used). */
extern tml_instance_ctrl_t g_timer_tml1_ctrl;
extern const timer_cfg_t g_timer_tml1_cfg;

#ifndef tml1_lp_timer_callback
void tml1_lp_timer_callback(timer_callback_args_t *p_args);
#endif
/** Timer on TML Instance. */
extern const timer_instance_t g_timer_tml0;

/** Access the TML instance using these structures when calling API functions directly (::p_api is not used). */
extern tml_instance_ctrl_t g_timer_tml0_ctrl;
extern const timer_cfg_t g_timer_tml0_cfg;

#ifndef tml0_wakeup_callback
void tml0_wakeup_callback(timer_callback_args_t *p_args);
#endif
/** TAU Timer Instance */
extern const timer_instance_t g_timer_tau0;

/** Access the TAU instance using these structures when calling API functions directly (::p_api is not used). */
extern tau_instance_ctrl_t g_timer_tau0_ctrl;
extern const timer_cfg_t g_timer_tau0_cfg;

#ifndef tau0_callback
void tau0_callback(timer_callback_args_t *p_args);
#endif
/** SPI on SAU Instance. */
extern const spi_instance_t g_spi0;

/** Access the SAU_SPI instance using these structures when calling API functions directly (::p_api is not used). */
extern sau_spi_instance_ctrl_t g_spi0_ctrl;
extern const spi_cfg_t g_spi0_cfg;

/** Called by the driver when a transfer has completed or an error has occurred (Must be implemented by the user). */
#ifndef sau_spi_callback
void sau_spi_callback(spi_callback_args_t *p_args);
#endif
/* RTC Instance. */
extern const rtc_instance_t g_rtc;

/** Access the RTC instance using these structures when calling API functions directly (::p_api is not used). */
extern rtc_c_instance_ctrl_t g_rtc_ctrl;
extern const rtc_cfg_t g_rtc_cfg;
extern const rtc_c_extended_cfg g_rtc_cfg_extend;

#ifndef rtc_callback
void rtc_callback(rtc_callback_args_t *p_args);
#endif
/** UART on UARTA Instance. */
extern const uart_instance_t g_uart0;

/** Access the UARTA instance using these structures when calling API functions directly (::p_api is not used). */
extern uarta_instance_ctrl_t g_uart0_ctrl;
extern const uart_cfg_t g_uart0_cfg;
extern const uarta_extended_cfg_t g_uart0_cfg_extend;

#ifndef uart0_callback
void uart0_callback(uart_callback_args_t *p_args);
#endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */

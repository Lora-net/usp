/* generated HAL source file - do not edit */
#include "hal_data.h"

#if BSP_TZ_NONSECURE_BUILD
 #if defined(BSP_CFG_CLOCKS_SECURE) && BSP_CFG_CLOCKS_SECURE
  #error "The LPM module requires CGC registers to be non-secure when it is used in a TrustZone Non-secure project."
 #endif
#endif

lpm_instance_ctrl_t g_lpm0_ctrl;

const lpm_cfg_t g_lpm0_cfg =
{ .low_power_mode = LPM_MODE_STANDBY, .standby_wake_sources = LPM_STANDBY_WAKE_SOURCE_IRQ0
        | LPM_STANDBY_WAKE_SOURCE_IRQ5 | LPM_STANDBY_WAKE_SOURCE_RTC | LPM_STANDBY_WAKE_SOURCE_ITL
        | (lpm_standby_wake_source_t) 0,
#if BSP_FEATURE_ICU_HAS_WUPEN2
    .standby_wake_sources_2     =  (lpm_standby_wake_source_2_t) 0,
#endif
#if BSP_FEATURE_LPM_HAS_SNOOZE
    .snooze_cancel_sources      = LPM_SNOOZE_CANCEL_SOURCE_NONE,
    .snooze_request_source      = (lpm_snooze_request_t) 0,
#if BSP_FEATURE_LPM_SNZEDCR_MASK > 0
    .snooze_end_sources         =  (lpm_snooze_end_t) 0,
#endif
    .dtc_state_in_snooze        = LPM_SNOOZE_DTC_DISABLE,
#endif
#if BSP_FEATURE_LPM_HAS_SBYCR_OPE
    .output_port_enable         = 0,
#endif
#if BSP_FEATURE_LPM_HAS_DEEP_STANDBY
    .io_port_state                = 0,
    .power_supply_state           = 0,
    .deep_standby_cancel_source   =  (lpm_deep_standby_cancel_source_t) 0,
    .deep_standby_cancel_edge     =  (lpm_deep_standby_cancel_edge_t) 0,
#if BSP_FEATURE_LPM_HAS_DPSBYCR_DCSSMODE
    .deep_standby_soft_start_mode = 0,
#endif
#endif
#if BSP_FEATURE_LPM_HAS_PDRAMSCR
    .ram_retention_cfg.ram_retention = (uint16_t) ( 0),
    .ram_retention_cfg.tcm_retention = false,
#endif
#if BSP_FEATURE_LPM_HAS_DPSBYCR_SRKEEP
    .ram_retention_cfg.standby_ram_retention = false,
#endif
#if BSP_FEATURE_LPM_HAS_LDO_SKEEP
    .ldo_standby_cfg.pll1_ldo = false,
    .ldo_standby_cfg.pll2_ldo = false,
    .ldo_standby_cfg.hoco_ldo = false,
#endif
#if BSP_FEATURE_LPM_HAS_FLASH_MODE_SELECT
    .lpm_flash_mode_select = LPM_FLASH_MODE_SELECT_ACTIVE,
#endif
#if BSP_FEATURE_LPM_HAS_HOCO_STARTUP_SPEED_MODE
    .lpm_hoco_startup_speed = LPM_HOCO_STARTUP_SPEED_HIGH_SPEED,
#endif
#if BSP_FEATURE_LPM_HAS_STANDBY_SOSC_SELECT
    .lpm_standby_sosc = LPM_STANDBY_SOSC_ENABLE,
#endif
  .p_extend = NULL, };

const lpm_instance_t g_lpm0 =
{ .p_api = &g_lpm_on_lpm, .p_ctrl = &g_lpm0_ctrl, .p_cfg = &g_lpm0_cfg };
iwdt_instance_ctrl_t g_wdt0_ctrl;

const wdt_cfg_t g_wdt0_cfg =
{ .timeout = (wdt_timeout_t) 0,
  .clock_division = (wdt_clock_division_t) 0,
  .window_start = (wdt_window_start_t) 0,
  .window_end = (wdt_window_end_t) 0,
  .reset_control = (wdt_reset_control_t) 0,
  .stop_control = (wdt_stop_control_t) 0,
  .p_callback = NULL, };

/* Instance structure to use this module. */
const wdt_instance_t g_wdt0 =
{ .p_ctrl = &g_wdt0_ctrl, .p_cfg = &g_wdt0_cfg, .p_api = &g_wdt_on_iwdt };
flash_lp_instance_ctrl_t g_flash0_ctrl;
const flash_cfg_t g_flash0_cfg =
{ .data_flash_bgo = false, .p_callback = NULL, .p_context = NULL, .ipl = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_FCU_FRDYI)
    .irq                 = VECTOR_NUMBER_FCU_FRDYI,
#else
  .irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const flash_instance_t g_flash0 =
{ .p_ctrl = &g_flash0_ctrl, .p_cfg = &g_flash0_cfg, .p_api = &g_flash_on_flash_lp };
tml_instance_ctrl_t g_timer_tml1_ctrl;
const tml_extended_cfg_t g_timer_tml1_extend =
{ .capture_trigger = TML_CAPTURE_TRIGGER_SOFTWARE, .counter_status = TML_COUNTER_STATUS_RETAINED_AFTER_CAPTURING, };
const timer_cfg_t g_timer_tml1_cfg =
{ .mode = TIMER_MODE_16_BIT_COUNTER,
/* Actual period: 2 seconds. */.period_counts = (uint32_t) 0x10000,
  .source_div = (timer_source_div_t) 0, .channel = 2, .p_callback = tml1_lp_timer_callback,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context          = NULL,
#else
  .p_context = (void*) &NULL,
#endif
  .p_extend = &g_timer_tml1_extend,
  .cycle_end_ipl = (2),
#if defined(VECTOR_NUMBER_TML0_ITL_OR)
    .cycle_end_irq      = VECTOR_NUMBER_TML0_ITL_OR,
#else
  .cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_timer_tml1 =
{ .p_ctrl = &g_timer_tml1_ctrl, .p_cfg = &g_timer_tml1_cfg, .p_api = &g_timer_on_tml };
tml_instance_ctrl_t g_timer_tml0_ctrl;
const tml_extended_cfg_t g_timer_tml0_extend =
{ .capture_trigger = TML_CAPTURE_TRIGGER_SOFTWARE, .counter_status = TML_COUNTER_STATUS_RETAINED_AFTER_CAPTURING, };
const timer_cfg_t g_timer_tml0_cfg =
{ .mode = TIMER_MODE_16_BIT_COUNTER,
/* Actual period: 2 seconds. */.period_counts = (uint32_t) 0x10000,
  .source_div = (timer_source_div_t) 0, .channel = 0, .p_callback = tml0_wakeup_callback,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context          = NULL,
#else
  .p_context = (void*) &NULL,
#endif
  .p_extend = &g_timer_tml0_extend,
  .cycle_end_ipl = (2),
#if defined(VECTOR_NUMBER_TML0_ITL_OR)
    .cycle_end_irq      = VECTOR_NUMBER_TML0_ITL_OR,
#else
  .cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_timer_tml0 =
{ .p_ctrl = &g_timer_tml0_ctrl, .p_cfg = &g_timer_tml0_cfg, .p_api = &g_timer_on_tml };
tau_instance_ctrl_t g_timer_tau0_ctrl;
const tau_extended_cfg_t g_timer_tau0_extend =
{ .opirq = TAU_INTERRUPT_OPIRQ_BIT_RESET,
  .tau_func = TAU_FUNCTION_INTERVAL,
  .bit_mode = TAU_BIT_MODE_16BIT,
  .initial_output = TAU_PIN_OUTPUT_CFG_DISABLED,
  .input_source = TAU_INPUT_SOURCE_NONE,
  .tau_filter = TAU_INPUT_NOISE_FILTER_DISABLE,
  .trigger_edge = TAU_TRIGGER_EDGE_RISING,
  .operation_clock = TAU_OPERATION_CK00,
  /* Not used for 16-bit or lower 8-bit mode */
  .period_higher_8bit_counts = (uint16_t) 0x100,
  .higher_8bit_cycle_end_ipl = (BSP_IRQ_DISABLED),
#if defined(VECTOR_NUMBER_TAU0_TMI00H)
    .higher_8bit_cycle_end_irq       = VECTOR_NUMBER_TAU0_TMI00H,
#else
  .higher_8bit_cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
const timer_cfg_t g_timer_tau0_cfg =
{ .mode = (timer_mode_t) 0,
  /* Actual Period: 0.0655360000 seconds. */
  /* Minimum Period ~ Maximum Period: 0.0000010000 ~ 0.06553600 seconds. */.period_counts = (uint32_t) 0x10000,
  .duty_cycle_counts = 0,
  .source_div = (timer_source_div_t) BSP_CFG_TAU_CK00,
  .channel = 0,
  .p_callback = tau0_callback,
  /** If NULL then do not add & */
#if defined(NULL)
    .p_context           = NULL,
#else
  .p_context = (void*) &NULL,
#endif
  .p_extend = &g_timer_tau0_extend,
  .cycle_end_ipl = (2),
#if defined(VECTOR_NUMBER_TAU0_TMI00)
    .cycle_end_irq       = VECTOR_NUMBER_TAU0_TMI00,
#else
  .cycle_end_irq = FSP_INVALID_VECTOR,
#endif
        };
/* Instance structure to use this module. */
const timer_instance_t g_timer_tau0 =
{ .p_ctrl = &g_timer_tau0_ctrl, .p_cfg = &g_timer_tau0_cfg, .p_api = &g_timer_on_tau };
#include "r_sau_spi_cfg.h"
sau_spi_instance_ctrl_t g_spi0_ctrl;
#if SAU_SPI_CFG_DTC_SUPPORT_ENABLE
transfer_info_t RA_NOT_DEFINED_info[2] =
{
    { .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_INCREMENTED,
      .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,
      .transfer_settings_word_b.irq = TRANSFER_IRQ_END,
      .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_EACH,
      .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
      .transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
      .transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
      .p_dest = (void*) NULL,
      .p_src = (void const*) NULL,
      .num_blocks = 0,
      .length = 0, },
    { .transfer_settings_word_b.dest_addr_mode = TRANSFER_ADDR_MODE_FIXED,
      .transfer_settings_word_b.repeat_area = TRANSFER_REPEAT_AREA_DESTINATION,
      .transfer_settings_word_b.irq = TRANSFER_IRQ_END,
      .transfer_settings_word_b.chain_mode = TRANSFER_CHAIN_MODE_DISABLED,
      .transfer_settings_word_b.src_addr_mode = TRANSFER_ADDR_MODE_FIXED,
      .transfer_settings_word_b.size = TRANSFER_SIZE_1_BYTE,
      .transfer_settings_word_b.mode = TRANSFER_MODE_NORMAL,
      .p_dest = (void*) NULL,
      .p_src = (void const*) NULL,
      .num_blocks = 0,
      .length = 0, }
};
const transfer_cfg_t RA_NOT_DEFINED_cfg_sau_spi =
{
  .p_info              = RA_NOT_DEFINED_info,
  .p_extend = &RA_NOT_DEFINED_cfg_extend, };

/* Instance structure to use this module. */
const transfer_instance_t RA_NOT_DEFINED_sau_spi =
{
    .p_ctrl        = &RA_NOT_DEFINED_ctrl,
    .p_cfg         = &RA_NOT_DEFINED_cfg_sau_spi,
    .p_api         = &g_transfer_on_dtc
};

#endif
/** SPI extended configuration */
const sau_spi_extended_cfg_t g_spi0_cfg_extend =
{ .clk_div =
{
/* Actual calculated bitrate: 8000000 */
.stclk = 0,
  .operation_clock = SAU_SPI_OPERATION_CLOCK_CK0, },
  .transfer_mode = SAU_SPI_TRANSFER_MODE_SINGLE, .data_phase = SAU_SPI_DATA_PHASE_HALF_CYCLE_START, .clock_phase =
          SAU_SPI_CLOCK_PHASE_REVERSE,
  .sau_unit = 0,
#if defined(PIN_SCK00)
    .sck_pin_settings.pin = PIN_SCK00,
#else
  .sck_pin_settings.pin = (bsp_io_port_pin_t) UINT16_MAX,
#endif
#if defined(CFG_SCK00)
    .sck_pin_settings.cfg = CFG_SCK00,
#else
  .sck_pin_settings.cfg = (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT,
#endif
#if defined(PIN_SO00)
    .so_pin_settings.pin = PIN_SO00,
#else
  .so_pin_settings.pin = (bsp_io_port_pin_t) UINT16_MAX,
#endif
#if defined(CFG_SO00)
    .so_pin_settings.cfg = CFG_SO00,
#else
  .so_pin_settings.cfg = (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT,
#endif
        };

const spi_cfg_t g_spi0_cfg =
{ .channel = 0,
  .operating_mode = SPI_MODE_MASTER,
  .bit_order = SPI_BIT_ORDER_MSB_FIRST,
  .p_callback = sau_spi_callback,
  .p_context = NULL,
#if defined(VECTOR_NUMBER_SAU0_SPI_TXRXI00)
    .tei_irq         = VECTOR_NUMBER_SAU0_SPI_TXRXI00,
#else
  .tei_irq = FSP_INVALID_VECTOR,
#endif
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_transfer_tx = NULL,
#else
    .p_transfer_tx   = &RA_NOT_DEFINED_sau_spi,
#endif
#undef RA_NOT_DEFINED
  .tei_ipl = (3),
  .p_extend = &g_spi0_cfg_extend, };
/* Instance structure to use this module. */
const spi_instance_t g_spi0 =
{ .p_ctrl = &g_spi0_ctrl, .p_cfg = &g_spi0_cfg, .p_api = &g_spi_on_sau };
rtc_c_instance_ctrl_t g_rtc_ctrl;

/** RTC_C extended configuration */
const rtc_c_extended_cfg g_rtc_cfg_extend =
{ .clock_source_div = RTC_CLOCK_SOURCE_SUBCLOCK_DIV_BY_1, };

const rtc_cfg_t g_rtc_cfg =
{ .p_err_cfg = NULL,
  .p_callback = rtc_callback,
  .p_context = NULL,
  .periodic_ipl = (2),
  .alarm_irq = FSP_INVALID_VECTOR,
#if defined(VECTOR_NUMBER_RTC_ALARM_OR_PERIOD)
    .periodic_irq            = VECTOR_NUMBER_RTC_ALARM_OR_PERIOD,
#else
  .periodic_irq = FSP_INVALID_VECTOR,
#endif
  .p_extend = &g_rtc_cfg_extend, };
/* Instance structure to use this module. */
const rtc_instance_t g_rtc =
{ .p_ctrl = &g_rtc_ctrl, .p_cfg = &g_rtc_cfg, .p_api = &g_rtc_on_rtc_c };
uarta_instance_ctrl_t g_uart0_ctrl;

uarta_baud_setting_t g_uart0_baud_setting =
{
#if (BSP_CFG_UARTA0_CLOCK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_MAIN_OSC)

  /* Baud rate calculated with Acutal_Error 0.22%. */
  /* The permissible baud rate error range during reception: -4.71% ~ 5.20% */
  .utanck_clock_b.utasel = UARTA_CLOCK_SOURCE_MOSC,
  .utanck_clock_b.utanck = UARTA_CLOCK_DIV_1, .brgca = 87, .delay_time = 1
#elif (BSP_CFG_UARTA0_CLOCK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_HOCO)

/* Baud rate calculated with Acutal_Error 0.08%. */
/* The permissible baud rate error range during reception: -4.73% ~ 5.22% */
  .utanck_clock_b.utasel = UARTA_CLOCK_SOURCE_HOCO
, .utanck_clock_b.utanck = UARTA_CLOCK_DIV_1
, .brgca = 139
, .delay_time = 1
 #elif (BSP_CFG_UARTA0_CLOCK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_MOCO)

/* Baud rate calculated with Acutal_Error 2.12%. */ 
/* The permissible baud rate error range during reception: -4.49% ~ 4.94% */
  .utanck_clock_b.utasel = UARTA_CLOCK_SOURCE_MOCO
, .utanck_clock_b.utanck = UARTA_CLOCK_DIV_1
, .brgca = 17
, .delay_time = 1
 #elif ((BSP_CFG_UARTA0_CLOCK_SOURCE == BSP_CFG_FSXP_SOURCE) || (BSP_CFG_UARTA0_CLOCK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_LOCO))

/* Baud rate calculated with Acutal_Error 100%. */
/* The permissible baud rate error range during reception: Invalid Range Error */
  .utanck_clock_b.utasel = 0
, .utanck_clock_b.utanck = 0
, .brgca = 0
, .delay_time = 31
 #elif (BSP_CFG_UARTA0_CLOCK_SOURCE == BSP_CLOCKS_SOURCE_CLOCK_SUBCLOCK)

/* Baud rate calculated with Acutal_Error 100%. */
/* The permissible baud rate error range during reception: Invalid Range Error */
  .utanck_clock_b.utasel = 0
, .utanck_clock_b.utanck = 0
, .brgca = 0
, .delay_time = 31
 #endif
        };

/** UART extended configuration for UART on UARTA HAL driver */
const uarta_extended_cfg_t g_uart0_cfg_extend =
{ .transfer_dir = UARTA_DIR_BIT_LSB_FIRST, .transfer_level = UARTA_ALV_BIT_POSITIVE_LOGIC, .clock_output =
          UARTA_CLOCK_OUTPUT_DISABLED,
  .p_baud_setting = &g_uart0_baud_setting, };

/** UART interface configuration */
const uart_cfg_t g_uart0_cfg =
{ .channel = 0, .data_bits = UART_DATA_BITS_8, .parity = UART_PARITY_OFF, .stop_bits = UART_STOP_BITS_1, .p_callback =
          uart0_callback,
  .p_context = NULL, .p_extend = &g_uart0_cfg_extend,
#define RA_NOT_DEFINED (1)
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_transfer_tx = NULL,
#else
                .p_transfer_tx       = &RA_NOT_DEFINED,
#endif
#if (RA_NOT_DEFINED == RA_NOT_DEFINED)
  .p_transfer_rx = NULL,
#else
                .p_transfer_rx       = &RA_NOT_DEFINED,
#endif
#undef RA_NOT_DEFINED
  .rxi_ipl = (2),
  .txi_ipl = (2), .eri_ipl = (2),
#if defined(VECTOR_NUMBER_UARTA0_RXI)
                .rxi_irq             = VECTOR_NUMBER_UARTA0_RXI,
#else
  .rxi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_UARTA0_TXI)
                .txi_irq             = VECTOR_NUMBER_UARTA0_TXI,
#else
  .txi_irq = FSP_INVALID_VECTOR,
#endif
#if defined(VECTOR_NUMBER_UARTA0_ERRI)
                .eri_irq             = VECTOR_NUMBER_UARTA0_ERRI,
#else
  .eri_irq = FSP_INVALID_VECTOR,
#endif
        };

/* Instance structure to use this module. */
const uart_instance_t g_uart0 =
{ .p_ctrl = &g_uart0_ctrl, .p_cfg = &g_uart0_cfg, .p_api = &g_uart_on_uarta };
void g_hal_init(void)
{
    g_common_init ();
}

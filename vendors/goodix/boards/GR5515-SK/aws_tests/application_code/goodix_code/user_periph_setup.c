/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_periph_setup.h"
#include "gr55xx_sys.h"
#include "app_log.h"
#include "app_assert.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "gr55xx_pwr.h"
#include "gr55xx_hal_dma.h"
#include "gr55xx_hal_uart.h"
#include "gr_config.h"
#include "gr_porting.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Bluetooth device address. */
static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x99, 0x25, 0x32, 0x7e, 0xf1, 0xc0};
//static const uint8_t  s_bd_addr[SYS_BD_ADDR_LEN] = {0x11, 0x00, 0x00, 0x33, 0x22, 0x11};
static app_log_init_t s_app_log_init; 

/*
 * LOCAL  FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void log_assert_init(void)
{
    s_app_log_init.filter.level                 = GR_TRACE_LEVEL;
    s_app_log_init.fmt_set[APP_LOG_LVL_ERROR]   = APP_LOG_FMT_ALL & (~APP_LOG_FMT_TAG);
    s_app_log_init.fmt_set[APP_LOG_LVL_WARNING] = APP_LOG_FMT_LVL;
    s_app_log_init.fmt_set[APP_LOG_LVL_INFO]    = APP_LOG_FMT_LVL;
    s_app_log_init.fmt_set[APP_LOG_LVL_DEBUG]   = APP_LOG_FMT_LVL;

    app_log_init(&s_app_log_init, gr_bsp_uart_send, gr_bsp_uart_flush);
    app_assert_default_cb_register();
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void app_periph_init(void)
{
    hal_flash_init();
    gr_bsp_uart_init();
    log_assert_init();
    nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
    SYS_SET_BD_ADDR(s_bd_addr);
    
    pwr_mgmt_init(pwr_table, CPLL_S64M_CLK);
    pwr_mgmt_mode_set(PMR_MGMT_ACTIVE_MODE);
    
    /* enable sdk log*/
#if GR_BLE_SDK_TRACE_ENABLE > 0u
    ble_stack_debug_setup(0x7FFFFFFF, 0x7FFFFFFF, vprintf);
#endif
    
    gr_ota_startup_check_and_update();
}

 
void hal_pm_resume(void)
{
    log_assert_init();
}


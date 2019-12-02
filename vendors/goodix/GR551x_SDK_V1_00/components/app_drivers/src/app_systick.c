/**
  ****************************************************************************************
  * @file    app_systick.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_systick.h"
#include "gr55xx_hal.h"
#include "app_pwr_mgmt.h"
#include <stdbool.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define SYSTICK_USE_PATTERN     0x47

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool systick_prepare_for_sleep(void);
static void systick_sleep_canceled(void);
static void systick_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t s_systick_use_flag = 0;
static bool    s_sleep_cb_registered_flag = false;

static const app_sleep_callbacks_t systick_sleep_cb =
{
    .app_prepare_for_sleep = systick_prepare_for_sleep,
    .app_sleep_canceled    = systick_sleep_canceled,
    .app_wake_up_ind       = systick_wake_up_ind
};

static bool systick_prepare_for_sleep(void)
{
    return true;
}

static void systick_sleep_canceled(void)
{
}

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void systick_wake_up_ind(void)
{
    if (s_systick_use_flag != SYSTICK_USE_PATTERN)
    {
        return;
    }

    hal_init();
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_systick_init(void)
{
    s_systick_use_flag = SYSTICK_USE_PATTERN;

    if(s_sleep_cb_registered_flag == false)    // register sleep callback
    {
        hal_init();

        s_sleep_cb_registered_flag = true;
        pwr_register_sleep_cb(&systick_sleep_cb);
    }
}

void app_systick_deinit(void)
{

}

uint32_t app_get_systick(void)
{
    if (s_systick_use_flag != SYSTICK_USE_PATTERN)
    {
        return 0;
    }

    return hal_get_tick();
}


__WEAK void SysTick_Handler(void)
{
    hal_increment_tick();
}

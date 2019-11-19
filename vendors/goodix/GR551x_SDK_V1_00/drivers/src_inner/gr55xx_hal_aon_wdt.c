/**
  ****************************************************************************************
  * @file    gr55xx_hal_aon_wdt.c
  * @author  BLE Driver Team
  * @brief   AON WDT HAL module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of GOODIX nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_AON_WDT_MODULE_ENABLED
/** @addtogroup AON_WDT AON_WDT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup AON_WDT_Exported_Functions AON_WDT Exported Functions
  * @{
  */

/** @defgroup AON_WDT_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions.
  * @{
  */

__WEAK hal_status_t hal_aon_wdt_init(aon_wdt_handle_t *p_aon_wdt)
{
    uint32_t wait_count = 1000;

    /* Check the AON_WDT handle allocation */
    if (NULL == p_aon_wdt)
    {
        return HAL_ERROR;
    }

    /* Allocate lock resource and initialize it */
    p_aon_wdt->lock = HAL_UNLOCKED;

    /* Disable AON_WDT */
    ll_aon_wdt_disable();

    /* Set WDT reload counter value */
    ll_aon_wdt_set_reload_counter(p_aon_wdt->init.counter);

    /* Load reload counter value into AON_WDT */
    ll_aon_wdt_reload_counter();

    /* Clear reboot flag */
    ll_aon_wdt_clear_flag_reboot();

    /* Set alarm counter value */
    ll_aon_wdt_set_alarm_counter(p_aon_wdt->init.alarm_counter);

    if (0 != p_aon_wdt->init.alarm_counter)
    {
        /* Clear pending IRQ and eable NVIC interrupt */
        NVIC_ClearPendingIRQ(AON_WDT_IRQn);
        NVIC_EnableIRQ(AON_WDT_IRQn);
    }

    while(wait_count--);

    /* Enable AON_WDT */
    ll_aon_wdt_enable();

    return HAL_OK;
}

__WEAK hal_status_t hal_aon_wdt_deinit(aon_wdt_handle_t *p_aon_wdt)
{
    /* Check the AON_WDT handle allocation */
    if (NULL == p_aon_wdt)
    {
        return HAL_ERROR;
    }

    /* Disable AON_WDT */
    ll_aon_wdt_disable();

    /* Diseable NVIC interrupt and Clear pending IRQ */
    NVIC_DisableIRQ(AON_WDT_IRQn);
    NVIC_ClearPendingIRQ(AON_WDT_IRQn);

    /* Set alarm counter value to default value */
    ll_aon_wdt_set_alarm_counter(20);

    /* Clear reboot flag */
    ll_aon_wdt_clear_flag_reboot();

    /* Process Unlock */
    __HAL_UNLOCK(p_aon_wdt);

    return HAL_OK;
}

/** @} */

/** @defgroup AON_WDT_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
  * @{
  */

__WEAK hal_status_t hal_aon_wdt_refresh(aon_wdt_handle_t *p_aon_wdt)
{
    /* Process Locked */
    __HAL_LOCK(p_aon_wdt);

    /* Set WDT reload counter value */
    ll_aon_wdt_set_reload_counter(p_aon_wdt->init.counter);

    /* Load reload counter value into AON_WDT */
    ll_aon_wdt_reload_counter();

    /* Process Unlock */
    __HAL_UNLOCK(p_aon_wdt);

    return HAL_OK;
}

__WEAK void hal_aon_wdt_irq_handler(aon_wdt_handle_t *p_aon_wdt)
{
    /* Alarm callback  */
    hal_aon_wdt_alarm_callback(p_aon_wdt);

    /* Set reload counter to 0 to RESET system immediately */
    ll_aon_wdt_set_reload_counter(0);
    ll_aon_wdt_reload_counter();

    /* Enable AON_WDT to RESET system */
    ll_aon_wdt_enable();
    
    /* Wait for more than 32us */
    for (uint32_t count = 0; count < 300; count++)
        __asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
}

__WEAK void hal_aon_wdt_alarm_callback(aon_wdt_handle_t *p_aon_wdt)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_aon_wdt);

    /* NOTE: This function should not be modified, when the callback is needed,
             the hal_aon_wdt_alarm_callback could be implemented in the user file
    */
}

/** @} */

/** @} */

#endif /* HAL_AON_WDT_MODULE_ENABLED */
/** @} */

/** @} */

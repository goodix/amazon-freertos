/**
  ****************************************************************************************
  * @file    gr55xx_hal_wdt.c
  * @author  BLE Driver Team
  * @brief   WDT HAL module driver.
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

#ifdef HAL_WDT_MODULE_ENABLED
/** @addtogroup WDT WDT
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup WDT_Exported_Functions WDT Exported Functions
  * @{
  */

/** @defgroup WDT_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions.
  * @{
  */

__WEAK hal_status_t hal_wdt_init(wdt_handle_t *p_wdt)
{
    /* Check the WDT handle allocation */
    if (NULL == p_wdt)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_WDT_ALL_INSTANCE(p_wdt->p_instance));

    /* Allocate lock resource and initialize it */
    p_wdt->lock = HAL_UNLOCKED;

    /* init the low level hardware */
    hal_wdt_msp_init(p_wdt);

    /* Enable write access to WDT_LOAD, WDT_CTRL and WDT_INTCLR registers */
    ll_wdt_enable_write_access(p_wdt->p_instance);

    /* Set WDT Counter Load Value */
    ll_wdt_set_counter_load(p_wdt->p_instance, p_wdt->init.counter);

    /* Set RESET mode */
    if (WDT_RESET_ENABLE == p_wdt->init.reset_mode)
    {
        ll_wdt_enable_reset(p_wdt->p_instance);
    }
    else
    {
        ll_wdt_disable_reset(p_wdt->p_instance);
    }

    /* Clear interrupt status */
    ll_wdt_clear_flag_it(p_wdt->p_instance);
    /* Enable watchdog counter and interrupt event */
    ll_wdt_enable(p_wdt->p_instance);

    /* Disable write access to WDT_LOAD, WDT_CTRL and WDT_INTCLR registers */
    ll_wdt_disable_write_access(p_wdt->p_instance);

    return HAL_OK;
}

__WEAK hal_status_t hal_wdt_deinit(wdt_handle_t *p_wdt)
{
    /* Check the WDT handle allocation */
    if (NULL == p_wdt)
    {
        return HAL_ERROR;
    }

    /* Enable write access to WDT_LOAD, WDT_CTRL and WDT_INTCLR registers */
    ll_wdt_enable_write_access(p_wdt->p_instance);

    /* Disable Reset MODE */
    ll_wdt_disable_reset(p_wdt->p_instance);
    /* Disable WDT Counter and interrupt */
    ll_wdt_disable(p_wdt->p_instance);
    /* Clear interrupt status */
    ll_wdt_clear_flag_it(p_wdt->p_instance);
    /* Set counter load value to 0 */
    ll_wdt_set_counter_load(p_wdt->p_instance, 0x0U);

    /* Disable write access to WDT_LOAD, WDT_CTRL and WDT_INTCLR registers */
    ll_wdt_disable_write_access(p_wdt->p_instance);

    /* Process Unlock */
    __HAL_UNLOCK(p_wdt);

    return HAL_OK;
}

__WEAK void hal_wdt_msp_init(wdt_handle_t *p_wdt)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_wdt);

    /* NOTE: This function should not be modified, when the callback is needed,
        the hal_wdt_msp_init could be implemented in the user file
    */
}

__WEAK void hal_wdt_msp_deinit(wdt_handle_t *p_wdt)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_wdt);

    /* NOTE: This function should not be modified, when the callback is needed,
        the hal_wdt_msp_init could be implemented in the user file
    */
}

/** @} */

/** @defgroup WDT_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
  * @{
  */

__WEAK hal_status_t hal_wdt_refresh(wdt_handle_t *p_wdt)
{
    /* Process Locked */
    __HAL_LOCK(p_wdt);

    /* Enable write access to WDT_INTCLR registers */
    ll_wdt_enable_write_access(p_wdt->p_instance);
    /* Reload Counter */
    ll_wdt_reload_counter(p_wdt->p_instance);
    /* Disable write access to WDT_INTCLR registers */
    ll_wdt_disable_write_access(p_wdt->p_instance);

    hal_nvic_clear_pending_irq(WDT_IRQn);
    hal_nvic_enable_irq(WDT_IRQn);

    /* Process Unlock */
    __HAL_UNLOCK(p_wdt);

    return HAL_OK;
}

__WEAK void hal_wdt_irq_handler(wdt_handle_t *p_wdt)
{
    /* Check if Interrupt occurred */
    if (RESET != ll_wdt_is_active_flag_it(p_wdt->p_instance))
    {
        /* Clear Interrupt status and reload counter if RESET mode was disabled */
        if (RESET == ll_wdt_is_enabled_reset(p_wdt->p_instance))
        {
            /* Enable write access to WDT_INTCLR registers */
            ll_wdt_enable_write_access(p_wdt->p_instance);
            /* Clear Interrupt status and reload counter */
            ll_wdt_clear_flag_it(p_wdt->p_instance);
            /* Disable write access to WDT_INTCLR registers */
            ll_wdt_disable_write_access(p_wdt->p_instance);
        }
        else
        {
            hal_nvic_disable_irq(WDT_IRQn);
        }

        /* Interrupt callback */
        hal_wdt_period_elapsed_callback(p_wdt);
    }
}

__WEAK void hal_wdt_period_elapsed_callback(wdt_handle_t *p_wdt)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_wdt);

    /* NOTE: This function should not be modified, when the callback is needed,
             the hal_wdt_period_elapsed_callback could be implemented in the user file
    */
}

/** @} */

/** @} */

#endif /* HAL_WDT_MODULE_ENABLED */
/** @} */

/** @} */

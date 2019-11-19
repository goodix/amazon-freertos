/**
  ****************************************************************************************
  * @file    gr55xx_hal_pwr.c
  * @author  BLE Driver Team
  * @brief   PWR HAL module driver.
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

#ifdef HAL_PWR_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup PWR_Exported_Functions PWR Exported Functions
  * @{
  */

__WEAK void hal_pwr_set_wakeup_condition(uint32_t condition)
{
    /* Check the parameters */
    gr_assert_param(IS_PWR_WAKEUP_CONDITION(condition));

    /* Select wakeup condition */
    ll_pwr_set_wakeup_condition(condition);
}

__WEAK void hal_pwr_config_timer_wakeup(uint8_t timer_mode, uint32_t load_count)
{
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
    ll_pwr_req_excute_psc_command(LL_PWR_CMD_SLP_TIMER_MODE_DISABLE);

    if (timer_mode != PWR_SLP_TIMER_MODE_DISABLE)
    {
        ll_pwr_set_sleep_timer_value(load_count);
        while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
        ll_pwr_req_excute_psc_command(LL_PWR_CMD_32_TIMER_LD);

        while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
        ll_pwr_req_excute_psc_command(timer_mode + LL_PWR_CMD_SLP_TIMER_MODE_NORMAL);
    }
}

__WEAK void hal_pwr_config_ext_wakeup(uint32_t ext_wakeup_pinx, uint32_t ext_wakeup_type)
{
    /* Check the parameters */
    gr_assert_param(IS_PWR_EXT_WAKEUP_PIN(ext_wakeup_pinx));
    gr_assert_param(IS_PWR_EXTWKUP_TYPE(ext_wakeup_type));

    /* Clear wakeup status of ext_wakeup_pinx */
    ll_pwr_clear_ext_wakeup_status(ext_wakeup_pinx);

    /* Set wakeup type */
    ll_pwr_set_ext_wakeup_type(ext_wakeup_pinx, ext_wakeup_type);

    /* Enable wakeup pin */
    ll_pwr_enable_ext_wakeup_pin(ext_wakeup_pinx);
}

__WEAK void hal_pwr_set_mem_current_power(uint32_t mem, uint32_t power_state)
{
    uint32_t wakp_power = 0;

    /* Check the parameters */
    gr_assert_param(IS_PWR_MEM_BLOCK(mem));
    gr_assert_param(IS_PWR_MEM_POWER_STAT(power_state));

    wakp_power = LL_PWR_ReadReg(MEM_PWR_WKUP);
    ll_pwr_set_mem_wakeup_power(mem, power_state);

    /* Load memoey config */
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
    ll_pwr_req_excute_psc_command(LL_PWR_CMD_LD_MEM_WKUP_CFG);
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());

    LL_PWR_WriteReg(MEM_PWR_WKUP, wakp_power);
}

__WEAK void hal_pwr_set_comm_power(uint32_t timer_power_state, uint32_t core_power_state)
{
    /* Check the parameters */
    gr_assert_param(IS_PWR_COMM_TIMER_POWER_STAT(timer_power_state));
    gr_assert_param(IS_PWR_COMM_CORE_POWER_STAT(core_power_state));

    if (PWR_COMM_TIMER_POWER_UP == timer_power_state)
    {
        ll_pwr_enable_comm_timer_power();
    }
    else
    {
        ll_pwr_disable_comm_timer_power();
    }

    if (PWR_COMM_CORE_POWER_UP == core_power_state)
    {
        ll_pwr_enable_comm_core_power();
    }
    else
    {
        ll_pwr_disable_comm_core_power();
    }
}

__WEAK void hal_pwr_set_comm_mode(uint32_t timer_mode, uint32_t core_mode)
{
    /* Check the parameters */
    gr_assert_param(IS_PWR_COMM_TIMER_MODE(timer_mode));
    gr_assert_param(IS_PWR_COMM_CORE_MODE(core_mode));

    if (PWR_COMM_TIMER_MODE_RESET == timer_mode)
    {
        ll_pwr_enable_comm_timer_reset();
    }
    else
    {
        ll_pwr_disable_comm_timer_reset();
    }

    if (PWR_COMM_CORE_MODE_RESET == core_mode)
    {
        ll_pwr_enable_comm_core_reset();
    }
    else
    {
        ll_pwr_disable_comm_core_reset();
    }
}

__WEAK hal_status_t hal_pwr_get_timer_current_value(uint32_t timer_type, uint32_t *p_value)
{
    uint32_t val0    = 0;
    uint32_t val1    = 1;
    uint32_t timeout = HAL_PWR_TIMEOUT_DEFAULT_VALUE;

    /* Check the parameters */
    gr_assert_param(IS_PWR_PWR_TIMER_TYPE(timer_type));

    /* Check the value allocation */
    if (NULL == p_value)
    {
        return HAL_ERROR;
    }

    ll_pwr_set_timer_read_select(timer_type);

    /* Use count to check timeout */
    timeout *= 1000;
    while (val0 != val1)
    {
        if (0 == timeout)
        {
            return HAL_TIMEOUT;
        }
        timeout--;

        val0 = ll_pwr_get_timer_read_value();
        val1 = ll_pwr_get_timer_read_value();
    }

    *p_value = val0;
    return HAL_OK;
}

__WEAK void hal_pwr_set_mem_wakeup_power(uint32_t full_memory, uint32_t off_memory, uint32_t retention_memory)
{
    /* Check the parameters */
    gr_assert_param(IS_PWR_MEM_BLOCK(full_memory));
    gr_assert_param(IS_PWR_MEM_BLOCK(off_memory));
    gr_assert_param(IS_PWR_MEM_BLOCK(retention_memory));

    ll_pwr_set_mem_wakeup_power(full_memory, LL_PWR_MEM_POWER_FULL);
    ll_pwr_set_mem_wakeup_power(off_memory, LL_PWR_MEM_POWER_OFF);
    ll_pwr_set_mem_wakeup_power(retention_memory, LL_PWR_MEM_POWER_RETENTION);

    return;
}

__WEAK void hal_pwr_set_mem_deepsleep_power(uint32_t full_memory, uint32_t off_memory, uint32_t retention_memory)
{
    /* Check the parameters */
    gr_assert_param(IS_PWR_MEM_BLOCK(full_memory));
    gr_assert_param(IS_PWR_MEM_BLOCK(off_memory));
    gr_assert_param(IS_PWR_MEM_BLOCK(retention_memory));

    ll_pwr_set_mem_deep_sleep_power(full_memory, LL_PWR_MEM_POWER_FULL);
    ll_pwr_set_mem_deep_sleep_power(off_memory, LL_PWR_MEM_POWER_OFF);
    ll_pwr_set_mem_deep_sleep_power(retention_memory, LL_PWR_MEM_POWER_RETENTION);

    return;   
}

__WEAK void __attribute__((section("RAM_CODE"))) hal_pwr_enter_chip_deepsleep(void)
{
    /* Set dpad_le value during sleep and after wake up  */
    ll_pwr_set_dpad_le_value(LL_PWR_DPAD_LE_OFF, LL_PWR_DPAD_LE_OFF);

    /* Enter into deep sleep mode */
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
    ll_pwr_req_excute_psc_command(LL_PWR_CMD_DEEP_SLEEP);

    /* Wait deep sleep config load */
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
    return;
}


__WEAK void  __attribute__((section("RAM_CODE"))) hal_pwr_enter_deepsleep_mode(uint32_t deep_sleep_retention_mem, uint32_t wakeup_power_full_mem)
{
    /* Set dpad_le value during sleep and after wake up  */
    ll_pwr_set_dpad_le_value(LL_PWR_DPAD_LE_OFF, LL_PWR_DPAD_LE_OFF);

    /* Set the memory need to keep power retention in deep sleep mode */
    ll_pwr_set_mem_deep_sleep_power(deep_sleep_retention_mem, LL_PWR_MEM_POWER_RETENTION);
    /* Set the memory need to keep power off in deep sleep mode */
    ll_pwr_set_mem_deep_sleep_power((~deep_sleep_retention_mem) & LL_PWR_MEM_ALL, LL_PWR_MEM_POWER_OFF);

    /* Set the memory need to keep full power after wake up from sleep mode */
    ll_pwr_set_mem_wakeup_power(wakeup_power_full_mem, LL_PWR_MEM_POWER_FULL);
    /* Set the memory need to keep power off after wake up from sleep mode */
    ll_pwr_set_mem_wakeup_power((~wakeup_power_full_mem) & LL_PWR_MEM_ALL, LL_PWR_MEM_POWER_OFF);

    /* Enter into deep sleep mode */
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
    ll_pwr_req_excute_psc_command(LL_PWR_CMD_DEEP_SLEEP);

    /* Wait deep sleep config load */
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
}


__WEAK void hal_pwr_sleep_timer_irq_handler(void)
{
    /* Clear sleep timer wakeup event */
    ll_pwr_clear_wakeup_event(LL_PWR_WKUP_EVENT_TIMER);

    /* Clear pended sleep timer interrupt */
    NVIC_ClearPendingIRQ(SLPTIMER_IRQn);

    /* Call sleep timer elapsed callback */
    hal_pwr_sleep_timer_elapsed_callback();
}

__WEAK void hal_pwr_ext_wakeup_irq_handler(void)
{
    /* Get external wakeup pin */
    uint32_t pin = ll_pwr_get_ext_wakeup_status();

    /* Clear external wakeup status */
    ll_pwr_clear_ext_wakeup_status(PWR_EXTWKUP_PIN_ALL);

    /* Clear external wakeup event */
    ll_pwr_clear_wakeup_event(LL_PWR_WKUP_EVENT_EXT);

    /* Clear pended sleep timer interrupt */
    NVIC_ClearPendingIRQ(EXTWKUP_IRQn);

    /* Call external wakeup callback */
    hal_pwr_ext_wakeup_callback(pin);

}

__WEAK void hal_pwr_sleep_timer_elapsed_callback(void)
{
    return;
}


__WEAK void hal_pwr_ext_wakeup_callback(uint32_t ext_wakeup_pinx)
{
    return;
}

/** @} */

#endif /* HAL_PWR_MODULE_ENABLED */

/** @} */

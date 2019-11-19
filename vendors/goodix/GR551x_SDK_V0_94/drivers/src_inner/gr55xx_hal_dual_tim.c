/**
  ****************************************************************************************
  * @file    gr55xx_hal_dual_tim.c
  * @author  BLE Driver Team
  * @brief   DUAL TIM HAL module driver.
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

#include "gr55xx_hal.h"

#ifdef HAL_DUAL_TIM_MODULE_ENABLED

__WEAK hal_status_t hal_dual_tim_init(dual_tim_handle_t *p_dual_timer)
{
    hal_status_t status              = HAL_OK;
    ll_dual_tim_init_t dual_tim_init = DUAL_TIM_DEFAULT_CONFIG;

    /* Check the DUAL_TIM handle allocation */
    if (NULL == p_dual_timer)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(p_dual_timer->p_instance));
    gr_assert_param(IS_DUAL_TIM_PRESCALER(p_dual_timer->init.prescaler));
    gr_assert_param(IS_DUAL_TIM_COUNTERMODE(p_dual_timer->init.counter_mode));

    /* Process locked */
    __HAL_LOCK(p_dual_timer);

    if (HAL_DUAL_TIM_STATE_RESET == p_dual_timer->state)
    {
        /* Allocate lock resource and initialize it */
        p_dual_timer->lock = HAL_UNLOCKED;

        /* init the low level hardware : GPIO, CLOCK */
        hal_dual_tim_msp_init(p_dual_timer);
    }

    p_dual_timer->state = HAL_DUAL_TIM_STATE_BUSY;

    /* Configure DUAL_TIM Clock Prescaler and Clock Mode */
    dual_tim_init.prescaler  = p_dual_timer->init.prescaler;
    dual_tim_init.auto_reload = p_dual_timer->init.auto_reload;
    ll_dual_tim_init(p_dual_timer->p_instance, &dual_tim_init);
    if (DUAL_TIM_COUNTERMODE_ONESHOT == p_dual_timer->init.counter_mode)
    {
        ll_dual_tim_enable_oneshot(p_dual_timer->p_instance);
    }
    else
    {
        ll_dual_tim_disable_oneshot(p_dual_timer->p_instance);
    }

    /* Initialize the DUAL_TIM state */
    p_dual_timer->state = HAL_DUAL_TIM_STATE_READY;

    /* Release Lock */
    __HAL_UNLOCK(p_dual_timer);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_dual_tim_deinit(dual_tim_handle_t *p_dual_timer)
{
    /* Check the DUAL_TIM handle allocation */
    if (NULL == p_dual_timer)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_dual_timer);

    /* Disable the DUAL_TIM Peripheral Clock */
    ll_dual_tim_deinit(p_dual_timer->p_instance);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_dual_tim_msp_deinit(p_dual_timer);

    /* Initialize the DUAL_TIM state */
    p_dual_timer->state = HAL_DUAL_TIM_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_dual_timer);

    return HAL_OK;
}

__WEAK void hal_dual_tim_msp_init(dual_tim_handle_t *p_dual_timer)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_dual_tim_msp_init can be implemented in the user file
    */
}

__WEAK void hal_dual_tim_msp_deinit(dual_tim_handle_t *p_dual_timer)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_dual_tim_msp_deinit can be implemented in the user file
    */
}

__WEAK void hal_dual_tim_irq_handler(dual_tim_handle_t *p_dual_timer)
{
    if (__HAL_DUAL_TIM_GET_FLAG_IT(p_dual_timer))
    {
        __HAL_DUAL_TIM_CLEAR_FLAG_IT(p_dual_timer);
        if (DUAL_TIM_COUNTERMODE_ONESHOT == p_dual_timer->init.counter_mode)
        {
            p_dual_timer->state = HAL_DUAL_TIM_STATE_READY;
            __HAL_DUAL_TIM_DISABLE(p_dual_timer);
            __HAL_DUAL_TIM_DISABLE_IT(p_dual_timer);
        }
        hal_dual_tim_period_elapsed_callback(p_dual_timer);
    }
}

__WEAK hal_status_t hal_dual_tim_start(dual_tim_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dual_timer);

    if (HAL_DUAL_TIM_STATE_READY == p_dual_timer->state)
    {
        p_dual_timer->state = HAL_DUAL_TIM_STATE_BUSY;
        __HAL_DUAL_TIM_DISABLE_IT(p_dual_timer);
        __HAL_DUAL_TIM_ENABLE(p_dual_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_dual_timer);

    return status;
}

__WEAK hal_status_t hal_dual_tim_stop(dual_tim_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dual_timer);

    if ((HAL_DUAL_TIM_STATE_BUSY == p_dual_timer->state) && (0 == ll_dual_tim_is_enabled_it(p_dual_timer->p_instance)))
    {
        p_dual_timer->state = HAL_DUAL_TIM_STATE_READY;
        __HAL_DUAL_TIM_DISABLE(p_dual_timer);
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_dual_timer);

    return status;
}

__WEAK hal_status_t hal_dual_tim_start_it(dual_tim_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dual_timer);

    if (HAL_DUAL_TIM_STATE_READY == p_dual_timer->state)
    {
        p_dual_timer->state = HAL_DUAL_TIM_STATE_BUSY;
        __HAL_DUAL_TIM_ENABLE_IT(p_dual_timer);
        __HAL_DUAL_TIM_ENABLE(p_dual_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_dual_timer);

    return status;
}

__WEAK hal_status_t hal_dual_tim_stop_it(dual_tim_handle_t *p_dual_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dual_timer);

    if ((HAL_DUAL_TIM_STATE_BUSY == p_dual_timer->state) && (1 == ll_dual_tim_is_enabled_it(p_dual_timer->p_instance)))
    {
        p_dual_timer->state = HAL_DUAL_TIM_STATE_READY;
        __HAL_DUAL_TIM_DISABLE(p_dual_timer);
        __HAL_DUAL_TIM_DISABLE_IT(p_dual_timer);
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_dual_timer);

    return status;
}

__WEAK void hal_dual_tim_period_elapsed_callback(dual_tim_handle_t *p_dual_timer)
{
    return;
}

__WEAK hal_dual_tim_state_t hal_dual_tim_get_state(dual_tim_handle_t *p_dual_timer)
{
    return p_dual_timer->state;
}

__WEAK hal_status_t hal_dual_tim_set_config(dual_tim_handle_t *p_dual_timer, dual_tim_init_t *p_structure)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dual_timer);

    if (HAL_DUAL_TIM_STATE_READY == p_dual_timer->state)
    {
        ll_dual_tim_set_prescaler(p_dual_timer->p_instance, p_structure->prescaler);
        ll_dual_tim_set_auto_reload(p_dual_timer->p_instance, p_structure->auto_reload);
        if (DUAL_TIM_COUNTERMODE_ONESHOT == p_structure->counter_mode)
        {
            ll_dual_tim_enable_oneshot(p_dual_timer->p_instance);
        }
        else
        {
            ll_dual_tim_disable_oneshot(p_dual_timer->p_instance);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_dual_timer);

    return status;
}

#endif


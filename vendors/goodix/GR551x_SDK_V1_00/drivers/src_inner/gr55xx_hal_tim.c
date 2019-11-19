/**
  ****************************************************************************************
  * @file    gr55xx_hal_tim.c
  * @author  BLE Driver Team
  * @brief   TIM HAL module driver.
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

#ifdef HAL_TIM_MODULE_ENABLED

__WEAK hal_status_t hal_tim_init(tim_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;
    ll_tim_init_t tim_init;

    /* Check the TIM handle allocation */
    if (NULL == p_timer)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_TIM_ALL_INSTANCE(p_timer->p_instance));

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIM_STATE_RESET == p_timer->state)
    {
        /* Allocate lock resource and initialize it */
        p_timer->lock = HAL_UNLOCKED;

        /* init the low level hardware : GPIO, CLOCK */
        hal_tim_msp_init(p_timer);
    }

    p_timer->state = HAL_TIM_STATE_BUSY;

    /* Configure TIM Clock Prescaler and Clock Mode */
    tim_init.auto_reload = p_timer->init.auto_reload;
    ll_tim_init(p_timer->p_instance, &tim_init);

    /* Initialize the TIM state */
    p_timer->state = HAL_TIM_STATE_READY;

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_tim_deinit(tim_handle_t *p_timer)
{
    /* Check the TIM handle allocation */
    if (NULL == p_timer)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_timer);

    /* Disable the TIM Peripheral Clock */
    ll_tim_deinit(p_timer->p_instance);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_tim_msp_deinit(p_timer);

    /* Initialize the TIM state */
    p_timer->state = HAL_TIM_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return HAL_OK;
}

__WEAK void hal_tim_msp_init(tim_handle_t *p_timer)
{
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_tim_msp_deinit(tim_handle_t *p_timer)
{
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_tim_irq_handler(tim_handle_t *p_timer)
{
    __HAL_TIM_CLEAR_FLAG_IT(p_timer);
    hal_tim_period_elapsed_callback(p_timer);
}

__WEAK hal_status_t hal_tim_start(tim_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIM_STATE_READY == p_timer->state)
    {
        p_timer->state = HAL_TIM_STATE_BUSY;
        __HAL_TIM_DISABLE_IT(p_timer);
        __HAL_TIM_ENABLE(p_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__WEAK hal_status_t hal_tim_stop(tim_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if ((HAL_TIM_STATE_BUSY == p_timer->state) && (0 == ll_tim_is_enabled_it(p_timer->p_instance)))
    {
        p_timer->state = HAL_TIM_STATE_READY;
        __HAL_TIM_DISABLE(p_timer);
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__WEAK hal_status_t hal_tim_start_it(tim_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIM_STATE_READY == p_timer->state)
    {
        p_timer->state = HAL_TIM_STATE_BUSY;
        __HAL_TIM_ENABLE_IT(p_timer);
        __HAL_TIM_ENABLE(p_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__WEAK hal_status_t hal_tim_stop_it(tim_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if ((HAL_TIM_STATE_BUSY == p_timer->state) && (1 == ll_tim_is_enabled_it(p_timer->p_instance)))
    {
        p_timer->state = HAL_TIM_STATE_READY;
        __HAL_TIM_DISABLE(p_timer);
        __HAL_TIM_DISABLE_IT(p_timer);
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__weak void hal_tim_period_elapsed_callback(tim_handle_t *p_timer)
{
    return;
}

__WEAK hal_tim_state_t hal_tim_get_state(tim_handle_t *p_timer)
{
    return p_timer->state;
}

__WEAK hal_status_t hal_tim_set_config(tim_handle_t *p_timer, tim_init_t *p_structure)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIM_STATE_READY == p_timer->state)
    {
        ll_tim_set_auto_reload(p_timer->p_instance, p_structure->auto_reload);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

#endif


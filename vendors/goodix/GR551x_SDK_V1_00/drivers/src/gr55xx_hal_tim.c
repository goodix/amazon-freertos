/**
  ****************************************************************************************
  * @file    gr55xx_hal_tim.c
  * @author  BLE Driver Team
  * @brief   TIMER HAL module driver.
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

#ifdef HAL_TIMER_MODULE_ENABLED

__WEAK hal_status_t hal_timer_base_init(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;
    ll_timer_init_t timer_init;

    /* Check the TIMER handle allocation */
    if (NULL == p_timer)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_TIMER_ALL_INSTANCE(p_timer->p_instance));

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIMER_STATE_RESET == p_timer->state)
    {
        /* Allocate lock resource and initialize it */
        p_timer->lock = HAL_UNLOCKED;

        /* init the low level hardware : GPIO, CLOCK */
        hal_timer_base_msp_init(p_timer);
    }

    p_timer->state = HAL_TIMER_STATE_BUSY;

    /* Configure TIMER Clock Prescaler and Clock Mode */
    timer_init.auto_reload = p_timer->init.auto_reload;
    ll_timer_init(p_timer->p_instance, &timer_init);

    /* Initialize the TIMER state */
    p_timer->state = HAL_TIMER_STATE_READY;

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_timer_base_deinit(timer_handle_t *p_timer)
{
    /* Check the TIM handle allocation */
    if (NULL == p_timer)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_timer);

    /* Disable the TIMER Peripheral Clock */
    ll_timer_deinit(p_timer->p_instance);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_timer_base_msp_deinit(p_timer);

    /* Initialize the TIMER state */
    p_timer->state = HAL_TIMER_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return HAL_OK;
}

__WEAK void hal_timer_base_msp_init(timer_handle_t *p_timer)
{
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_timer_base_msp_deinit(timer_handle_t *p_timer)
{
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_timer_irq_handler(timer_handle_t *p_timer)
{
    __HAL_TIMER_CLEAR_FLAG_IT(p_timer);
    hal_timer_period_elapsed_callback(p_timer);
}

__WEAK hal_status_t hal_timer_base_start(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIMER_STATE_READY == p_timer->state)
    {
        p_timer->state = HAL_TIMER_STATE_BUSY;
        __HAL_TIMER_DISABLE_IT(p_timer);
        __HAL_TIMER_ENABLE(p_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__WEAK hal_status_t hal_timer_base_stop(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if ((HAL_TIMER_STATE_BUSY == p_timer->state) && (0 == ll_timer_is_enabled_it(p_timer->p_instance)))
    {
        p_timer->state = HAL_TIMER_STATE_READY;
        __HAL_TIMER_DISABLE(p_timer);
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__WEAK hal_status_t hal_timer_base_start_it(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIMER_STATE_READY == p_timer->state)
    {
        p_timer->state = HAL_TIMER_STATE_BUSY;
        __HAL_TIMER_ENABLE_IT(p_timer);
        __HAL_TIMER_ENABLE(p_timer);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__WEAK hal_status_t hal_timer_base_stop_it(timer_handle_t *p_timer)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if ((HAL_TIMER_STATE_BUSY == p_timer->state) && (1 == ll_timer_is_enabled_it(p_timer->p_instance)))
    {
        p_timer->state = HAL_TIMER_STATE_READY;
        __HAL_TIMER_DISABLE(p_timer);
        __HAL_TIMER_DISABLE_IT(p_timer);
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_timer);

    return status;
}

__weak void hal_timer_period_elapsed_callback(timer_handle_t *p_timer)
{
    return;
}

__WEAK hal_timer_state_t hal_timer_get_state(timer_handle_t *p_timer)
{
    return p_timer->state;
}

__WEAK hal_status_t hal_timer_set_config(timer_handle_t *p_timer, timer_init_t *p_structure)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_timer);

    if (HAL_TIMER_STATE_READY == p_timer->state)
    {
        ll_timer_set_auto_reload(p_timer->p_instance, p_structure->auto_reload);
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


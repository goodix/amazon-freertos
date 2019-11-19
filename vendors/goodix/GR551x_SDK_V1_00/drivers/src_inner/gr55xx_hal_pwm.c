/**
  ****************************************************************************************
  * @file    gr55xx_hal_pwm.c
  * @author  BLE Driver Team
  * @brief   PWM HAL module driver.
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

#ifdef HAL_PWM_MODULE_ENABLED

__WEAK hal_status_t hal_pwm_init(pwm_handle_t *p_pwm)
{
    hal_status_t  status   = HAL_OK;
    ll_pwm_init_t pwm_init = LL_PWM_DEFAULT_CONFIG;

    /* Check the PWM handle allocation */
    if (NULL == p_pwm)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_PWM_ALL_INSTANCE(p_pwm->p_instance));
    gr_assert_param(IS_PWM_MODE(p_pwm->init.mode));
    gr_assert_param(IS_PWM_DRIVEPOLARITY(p_pwm->init.channel_a.dirve_polarity));
    gr_assert_param(IS_PWM_DRIVEPOLARITY(p_pwm->init.channel_b.dirve_polarity));
    gr_assert_param(IS_PWM_DRIVEPOLARITY(p_pwm->init.channel_c.dirve_polarity));
    if ((0 == p_pwm->init.freq) || ((p_pwm->init.mode == PWM_MODE_BREATH) && ((0 == p_pwm->init.bperiod) || (0 == p_pwm->init.hperiod))))
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_pwm);

    if (HAL_PWM_STATE_RESET == p_pwm->state)
    {
        /* Allocate lock resource and initialize it */
        p_pwm->lock = HAL_UNLOCKED;

        /* Enable PWM Clock and Automatic turn off PWM clock during WFI. */
        ll_cgc_disable_force_off_pwm_hclk();
        ll_cgc_disable_wfi_off_pwm_hclk();

        /* init the low level hardware : GPIO, CLOCK */
        hal_pwm_msp_init(p_pwm);
    }

    p_pwm->state = HAL_PWM_STATE_BUSY;

    /* Update SystemCoreClock */
    SystemCoreUpdateClock();

    /* Configure PWM Clock Prescaler and Clock Mode */
    pwm_init.mode                     = p_pwm->init.mode;
    pwm_init.prescaler                = SystemCoreClock / p_pwm->init.freq;
    pwm_init.bprescaler               = SystemCoreClock / 1000 * p_pwm->init.bperiod;
    pwm_init.hprescaler               = SystemCoreClock / 1000 * p_pwm->init.hperiod;
    pwm_init.channel_a.duty           = p_pwm->init.channel_a.duty;
    pwm_init.channel_a.dirve_polarity = p_pwm->init.channel_a.dirve_polarity;
    pwm_init.channel_b.duty           = p_pwm->init.channel_b.duty;
    pwm_init.channel_b.dirve_polarity = p_pwm->init.channel_b.dirve_polarity;
    pwm_init.channel_c.duty           = p_pwm->init.channel_c.duty;
    pwm_init.channel_c.dirve_polarity = p_pwm->init.channel_c.dirve_polarity;
    ll_pwm_init(p_pwm->p_instance, &pwm_init);

    if (0 == (p_pwm->active_channel & 0x01))
    {
        ll_pwm_set_action_event_cmp_a0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
        ll_pwm_set_action_event_cmp_a1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
    }
    if (0 == (p_pwm->active_channel & 0x02))
    {
        ll_pwm_set_action_event_cmp_b0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
        ll_pwm_set_action_event_cmp_b1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
    }
    if (0 == (p_pwm->active_channel & 0x04))
    {
        ll_pwm_set_action_event_cmp_c0(p_pwm->p_instance, LL_PWM_ACTIONEVENT_CLEAR);
        ll_pwm_set_action_event_cmp_c1(p_pwm->p_instance, LL_PWM_ACTIONEVENT_NONE);
    }
    /* Pause PWM output */
    ll_pwm_enable_pause(p_pwm->p_instance);

    /* Enable the PWM peripheral */
    __HAL_PWM_ENABLE(p_pwm);

    /* Initialize the PWM state */
    p_pwm->state = HAL_PWM_STATE_READY;

    /* Release Lock */
    __HAL_UNLOCK(p_pwm);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pwm_deinit(pwm_handle_t *p_pwm)
{
    /* Check the PWM handle allocation */
    if (NULL == p_pwm)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_pwm);

    /* Disable the PWM Peripheral Clock */
    ll_pwm_deinit(p_pwm->p_instance);
    
    /* Disable the PWM peripheral */
    __HAL_PWM_DISABLE(p_pwm);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_pwm_msp_deinit(p_pwm);

    /* Initialize the PWM state */
    p_pwm->state = HAL_PWM_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_pwm);

    return HAL_OK;
}

__WEAK void hal_pwm_msp_init(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pwm_msp_init can be implemented in the user file
    */
}

__WEAK void hal_pwm_msp_deinit(pwm_handle_t *p_pwm)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pwm_msp_deinit can be implemented in the user file
    */
}

__WEAK hal_status_t hal_pwm_start(pwm_handle_t *p_pwm)
{
    ll_pwm_disable_pause(p_pwm->p_instance);

    /* Sync-all update enable */
    ll_pwm_enable_update_all(p_pwm->p_instance);
    return HAL_OK;
}

__WEAK hal_status_t hal_pwm_stop(pwm_handle_t *p_pwm)
{
    /* Sync-all update disable */
    ll_pwm_disable_update_all(p_pwm->p_instance);

    ll_pwm_enable_pause(p_pwm->p_instance);
    return HAL_OK;
}

__WEAK hal_status_t hal_pwm_update_freq(pwm_handle_t *p_pwm, uint32_t freq)
{
    /* Check the freq value */
    if ((0 == freq) || ((SystemCoreClock >> 1) < freq))
    {
        return HAL_ERROR;
    }

    p_pwm->init.freq = freq;
    ll_pwm_set_compare_a1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.freq * p_pwm->init.channel_a.duty / 100);
    ll_pwm_set_compare_b1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.freq * p_pwm->init.channel_b.duty / 100);
    ll_pwm_set_compare_c1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.freq * p_pwm->init.channel_c.duty / 100);
    ll_pwm_set_prescaler(p_pwm->p_instance, SystemCoreClock / p_pwm->init.freq);
    return HAL_OK;
}

__WEAK hal_status_t hal_pwm_config_channel(pwm_handle_t *p_pwm, pwm_channel_init_t *p_config, hal_pwm_active_channel_t channel)
{
    uint32_t perscaler = 0;

    /* Check the duty value */
    if (100 < p_config->duty)
    {
        return HAL_ERROR;
    }

    perscaler = ll_pwm_get_prescaler(p_pwm->p_instance);
    if (channel & 0x01)
    {
        if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_config->dirve_polarity)
            ll_pwm_enable_positive_drive_channel_a(p_pwm->p_instance);
        else
            ll_pwm_disable_positive_drive_channel_a(p_pwm->p_instance);

        p_pwm->init.channel_a.duty = p_config->duty;
        ll_pwm_set_compare_a0(p_pwm->p_instance, 0);
        ll_pwm_set_compare_a1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.freq * p_config->duty / 100);
    }
    if (channel & 0x02)
    {
        if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_config->dirve_polarity)
            ll_pwm_enable_positive_drive_channel_b(p_pwm->p_instance);
        else
            ll_pwm_disable_positive_drive_channel_b(p_pwm->p_instance);

        p_pwm->init.channel_b.duty = p_config->duty;
        ll_pwm_set_compare_b0(p_pwm->p_instance, 0);
        ll_pwm_set_compare_b1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.freq * p_config->duty / 100);
    }
    if (channel & 0x04)
    {
        if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_config->dirve_polarity)
            ll_pwm_enable_positive_drive_channel_c(p_pwm->p_instance);
        else
            ll_pwm_disable_positive_drive_channel_c(p_pwm->p_instance);

        p_pwm->init.channel_c.duty = p_config->duty;
        ll_pwm_set_compare_c0(p_pwm->p_instance, 0);
        ll_pwm_set_compare_c1(p_pwm->p_instance, SystemCoreClock / p_pwm->init.freq * p_config->duty / 100);
    }
    ll_pwm_set_prescaler(p_pwm->p_instance, perscaler);
    return HAL_OK;
}

__WEAK hal_pwm_state_t hal_pwm_get_state(pwm_handle_t *p_pwm)
{
    return p_pwm->state;
}

#endif /* HAL_PWM_MODULE_ENABLED */


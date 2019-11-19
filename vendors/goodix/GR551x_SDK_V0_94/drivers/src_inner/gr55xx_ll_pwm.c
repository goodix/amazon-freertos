/**
  ****************************************************************************************
  * @file    gr55xx_ll_pwm.c
  * @author  BLE Driver Team
  * @brief   PWM LL module driver.
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
#include "gr55xx_ll_pwm.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (PWM0) || defined (PWM1)

/**
  * @brief  Set PWMx registers to their reset values.
  * @param  PWMx PWM instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PWMx registers are de-initialized
  *          - ERROR: invalid PWMx instance
  */
__WEAK error_status_t ll_pwm_deinit(pwm_regs_t *PWMx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_PWM_ALL_INSTANCE(PWMx));

    LL_PWM_WriteReg(PWMx, MODE, 0);
    LL_PWM_WriteReg(PWMx, UPDATE, 0);
    LL_PWM_WriteReg(PWMx, PRD, 0);
    LL_PWM_WriteReg(PWMx, CMPA0, 0);
    LL_PWM_WriteReg(PWMx, CMPA1, 0);
    LL_PWM_WriteReg(PWMx, CMPB0, 0);
    LL_PWM_WriteReg(PWMx, CMPB1, 0);
    LL_PWM_WriteReg(PWMx, CMPC0, 0);
    LL_PWM_WriteReg(PWMx, CMPC1, 0);
    LL_PWM_WriteReg(PWMx, AQCTRL, 0);
    LL_PWM_WriteReg(PWMx, BRPRD, 0);
    LL_PWM_WriteReg(PWMx, HOLD, 0);

    return result;
}

/**
  * @brief  Set the fields of the PWM base unit configuration data structure
  *         to their default values.
  * @param  p_pwm_init pointer to a @ref ll_pwm_init_t structure (PWM base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_pwm_struct_init(ll_pwm_init_t *p_pwm_init)
{
    /* Set the default configuration */
    p_pwm_init->mode                     = LL_PWM_FLICKER_MODE;
    p_pwm_init->prescaler                = 10 * LL_PWM_PRESCALER_UNIT;
    p_pwm_init->bprescaler               = 10 * LL_PWM_BREATH_PRESCALER_UNIT * 10 * LL_PWM_PRESCALER_UNIT;
    p_pwm_init->hprescaler               = 10 * LL_PWM_HOLD_PRESCALER_UNIT * 10 * LL_PWM_PRESCALER_UNIT;
    p_pwm_init->channel_a.duty           = 50;
    p_pwm_init->channel_a.dirve_polarity = LL_PWM_DRIVEPOLARITY_POSITIVE;
    p_pwm_init->channel_b.duty           = 50;
    p_pwm_init->channel_b.dirve_polarity = LL_PWM_DRIVEPOLARITY_POSITIVE;
    p_pwm_init->channel_c.duty           = 50;
    p_pwm_init->channel_c.dirve_polarity = LL_PWM_DRIVEPOLARITY_POSITIVE;
}

/**
  * @brief  Configure the PWMx time base unit.
  * @param  PWMx PWM instance
  * @param  p_pwm_init pointer to a @ref ll_pwm_init_t structure (PWMx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PWMx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_pwm_init(pwm_regs_t *PWMx, ll_pwm_init_t *p_pwm_init)
{
    /* Check the parameters */
    gr_assert_param(IS_PWM_ALL_INSTANCE(PWMx));

    /* Sync-all update enable */
    ll_pwm_enable_update_all(PWMx);
    ll_pwm_set_mode(PWMx, p_pwm_init->mode);
    ll_pwm_set_prescaler(PWMx, p_pwm_init->prescaler);
    ll_pwm_set_breath_prescaler(PWMx, p_pwm_init->bprescaler);
    ll_pwm_set_hold_prescaler(PWMx, p_pwm_init->hprescaler);
    if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->channel_a.dirve_polarity)
    {
        ll_pwm_enable_positive_drive_channel_a(PWMx);
    }
    else
    {
        ll_pwm_disable_positive_drive_channel_a(PWMx);
    }
    if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->channel_b.dirve_polarity)
    {
        ll_pwm_enable_positive_drive_channel_b(PWMx);
    }
    else
    {
        ll_pwm_disable_positive_drive_channel_b(PWMx);
    }
    if (LL_PWM_DRIVEPOLARITY_POSITIVE == p_pwm_init->channel_c.dirve_polarity)
    {
        ll_pwm_enable_positive_drive_channel_c(PWMx);
    }
    else
    {
        ll_pwm_disable_positive_drive_channel_c(PWMx);
    }
    ll_pwm_set_compare_a0(PWMx, 0);
    ll_pwm_set_compare_a1(PWMx, p_pwm_init->prescaler * p_pwm_init->channel_a.duty / 100);
    ll_pwm_set_compare_b0(PWMx, 0);
    ll_pwm_set_compare_b1(PWMx, p_pwm_init->prescaler * p_pwm_init->channel_b.duty / 100);
    ll_pwm_set_compare_c0(PWMx, 0);
    ll_pwm_set_compare_c1(PWMx, p_pwm_init->prescaler * p_pwm_init->channel_c.duty / 100);

    ll_pwm_set_action_event_cmp_a0(PWMx, LL_PWM_ACTIONEVENT_SET);
    ll_pwm_set_action_event_cmp_a1(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
    ll_pwm_set_action_event_cmp_b0(PWMx, LL_PWM_ACTIONEVENT_SET);
    ll_pwm_set_action_event_cmp_b1(PWMx, LL_PWM_ACTIONEVENT_CLEAR);
    ll_pwm_set_action_event_cmp_c0(PWMx, LL_PWM_ACTIONEVENT_SET);
    ll_pwm_set_action_event_cmp_c1(PWMx, LL_PWM_ACTIONEVENT_CLEAR);

    return SUCCESS;
}

#endif /* PWM0 || PWM1 */


/**
  ****************************************************************************************
  * @file    gr55xx_ll_dual_tim.c
  * @author  BLE Driver Team
  * @brief   DUAL TIMER LL module driver.
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
#include "gr55xx_ll_dual_tim.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (DUAL_TIMER0) || defined (DUAL_TIMER1)

/**
  * @brief  Set DUAL_TIMERx registers to their reset values.
  * @param  DUAL_TIMERx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DUAL_TIMERx registers are de-initialized
  *          - ERROR: invalid DUAL_TIMERx instance
  */
__WEAK error_status_t ll_dual_timer_deinit(dual_timer_regs_t *DUAL_TIMERx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(DUAL_TIMERx));

    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, CTRL, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, RELOAD, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, BG_LOAD, 0);
    LL_DUAL_TIMER_WriteReg(DUAL_TIMERx, INTCLR, 1);
    return result;
}

/**
  * @brief  Set the fields of the dual_time base unit configuration data structure
  *         to their default values.
  * @param  p_dual_timer_init pointer to a @ref ll_dual_timer_init_t structure (dual_time base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_dual_timer_struct_init(ll_dual_timer_init_t *p_dual_timer_init)
{
    /* Set the default configuration */
    p_dual_timer_init->prescaler    = LL_DUAL_TIMER_PRESCALER_DIV0;
    p_dual_timer_init->counter_size = LL_DUAL_TIMER_COUNTERSIZE_32;
    p_dual_timer_init->counter_mode = LL_DUAL_TIMER_PERIODIC_MODE;
    p_dual_timer_init->auto_reload  = 0xFFFFFFFFU;
}

/**
  * @brief  Configure the DUAL_TIMERx time base unit.
  * @param  DUAL_TIMERx Timer instance
  * @param  p_dual_tim_init pointer to a @ref ll_dual_timer_init_t structure (DUAL_TIMERx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DUAL_TIMERx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_dual_timer_init(dual_timer_regs_t *DUAL_TIMERx, ll_dual_timer_init_t *p_dual_timer_init)
{
    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(DUAL_TIMERx));

    ll_dual_timer_set_auto_reload(DUAL_TIMERx, p_dual_timer_init->auto_reload);
    ll_dual_timer_set_counter_mode(DUAL_TIMERx, p_dual_timer_init->counter_mode);
    ll_dual_timer_set_prescaler(DUAL_TIMERx, p_dual_timer_init->prescaler);
    ll_dual_timer_set_counter_size(DUAL_TIMERx, p_dual_timer_init->counter_size);

    return SUCCESS;
}

#endif /* DUAL_TIMER0 || DUAL_TIMER1 */


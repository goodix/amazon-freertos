/**
  ****************************************************************************************
  * @file    gr55xx_ll_dual_tim.c
  * @author  BLE Driver Team
  * @brief   DUAL TIM LL module driver.
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

#if defined (DUAL_TIM0) || defined (DUAL_TIM1)

/**
  * @brief  Set DUAL_TIMx registers to their reset values.
  * @param  DUAL_TIMx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DUAL_TIMx registers are de-initialized
  *          - ERROR: invalid DUAL_TIMx instance
  */
__WEAK error_status_t ll_dual_tim_deinit(dual_tim_regs_t *DUAL_TIMx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(DUAL_TIMx));

    LL_DUAL_TIM_WriteReg(DUAL_TIMx, CTRL, 0);
    LL_DUAL_TIM_WriteReg(DUAL_TIMx, RELOAD, 0);
    LL_DUAL_TIM_WriteReg(DUAL_TIMx, BG_LOAD, 0);
    LL_DUAL_TIM_WriteReg(DUAL_TIMx, INTCLR, 1);
    return result;
}

/**
  * @brief  Set the fields of the dual_time base unit configuration data structure
  *         to their default values.
  * @param  p_dual_tim_init pointer to a @ref ll_dual_tim_init_t structure (dual_time base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_dual_tim_struct_init(ll_dual_tim_init_t *p_dual_tim_init)
{
    /* Set the default configuration */
    p_dual_tim_init->prescaler    = LL_DUAL_TIM_PRESCALER_DIV0;
    p_dual_tim_init->counter_size = LL_DUAL_TIM_COUNTERSIZE_32;
    p_dual_tim_init->counter_mode = LL_DUAL_TIM_PERIODIC_MODE;
    p_dual_tim_init->auto_reload  = 0xFFFFFFFFU;
}

/**
  * @brief  Configure the DUAL_TIMx time base unit.
  * @param  DUAL_TIMx Timer instance
  * @param  p_dual_tim_init pointer to a @ref ll_dual_tim_init_t structure (DUAL_TIMx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DUAL_TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_dual_tim_init(dual_tim_regs_t *DUAL_TIMx, ll_dual_tim_init_t *p_dual_tim_init)
{
    /* Check the parameters */
    gr_assert_param(IS_DUAL_TIM_ALL_INSTANCE(DUAL_TIMx));

    ll_dual_tim_set_auto_reload(DUAL_TIMx, p_dual_tim_init->auto_reload);
    ll_dual_tim_set_counter_mode(DUAL_TIMx, p_dual_tim_init->counter_mode);
    ll_dual_tim_set_prescaler(DUAL_TIMx, p_dual_tim_init->prescaler);
    ll_dual_tim_set_counter_size(DUAL_TIMx, p_dual_tim_init->counter_size);

    return SUCCESS;
}

#endif /* DUAL_TIM0 || DUAL_TIM1 */


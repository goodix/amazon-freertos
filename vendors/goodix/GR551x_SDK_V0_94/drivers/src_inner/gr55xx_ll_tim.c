/**
  ****************************************************************************************
  * @file    gr55xx_ll_tim.c
  * @author  BLE Driver Team
  * @brief   TIM LL module driver.
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
#include "gr55xx_ll_tim.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (TIM0) || defined (TIM1)

/**
  * @brief  Set TIMx registers to their reset values.
  * @param  TIMx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: invalid TIMx instance
  */
__WEAK error_status_t ll_tim_deinit(tim_regs_t *TIMx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_TIM_ALL_INSTANCE(TIMx));

    LL_TIM_WriteReg(TIMx, CTRL, 0);
    LL_TIM_WriteReg(TIMx, VALUE, 0);
    LL_TIM_WriteReg(TIMx, RELOAD, 0);

    return result;
}

/**
  * @brief  Set the fields of the time base unit configuration data structure
  *         to their default values.
  * @param  p_tim_init pointer to a @ref ll_tim_init_t structure (time base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_tim_struct_init(ll_tim_init_t *p_tim_init)
{
    /* Set the default configuration */
    p_tim_init->auto_reload = 0xFFFFFFFFU;
}

/**
  * @brief  Configure the TIMx time base unit.
  * @param  TIMx Timer instance
  * @param  p_tim_init pointer to a @ref ll_tim_init_t structure (TIMx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_tim_init(tim_regs_t *TIMx, ll_tim_init_t *p_tim_init)
{
    /* Check the parameters */
    gr_assert_param(IS_TIM_ALL_INSTANCE(TIMx));

    ll_tim_set_auto_reload(TIMx, p_tim_init->auto_reload);

    return SUCCESS;
}

#endif


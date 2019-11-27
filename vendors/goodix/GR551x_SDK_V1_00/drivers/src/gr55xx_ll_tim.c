/**
  ****************************************************************************************
  * @file    gr55xx_ll_tim.c
  * @author  BLE Driver Team
  * @brief   TIMER LL module driver.
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

#if defined (TIMER0) || defined (TIMER1)

/**
  * @brief  Set TIMERx registers to their reset values.
  * @param  TIMERx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMERx registers are de-initialized
  *          - ERROR: invalid TIMERx instance
  */
__WEAK error_status_t ll_timer_deinit(timer_regs_t *TIMERx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_TIMER_ALL_INSTANCE(TIMERx));

    LL_TIMER_WriteReg(TIMERx, CTRL, 0);
    LL_TIMER_WriteReg(TIMERx, VALUE, 0);
    LL_TIMER_WriteReg(TIMERx, RELOAD, 0);

    return result;
}

/**
  * @brief  Set the fields of the time base unit configuration data structure
  *         to their default values.
  * @param  p_timer_init pointer to a @ref ll_timer_init_t structure (time base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_timer_struct_init(ll_timer_init_t *p_timer_init)
{
    /* Set the default configuration */
    p_timer_init->auto_reload = 0xFFFFFFFFU;
}

/**
  * @brief  Configure the TIMERx time base unit.
  * @param  TIMERx Timer instance
  * @param  p_timer_init pointer to a @ref ll_timer_init_t structure (TIMERx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMERx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_timer_init(timer_regs_t *TIMERx, ll_timer_init_t *p_timer_init)
{
    /* Check the parameters */
    gr_assert_param(IS_TIMER_ALL_INSTANCE(TIMERx));

    ll_timer_set_auto_reload(TIMERx, p_timer_init->auto_reload);

    return SUCCESS;
}

#endif


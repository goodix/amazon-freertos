/**
  ****************************************************************************************
  * @file    gr55xx_hal_dual_tim_br.c
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

/* Private variables ---------------------------------------------------------*/

static hal_dual_tim_callback_t *p_hal_dual_tim_callback = NULL;

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_dual_tim_init_ext(dual_tim_handle_t *p_dual_timer)
{
    return hal_dual_tim_init(p_dual_timer);
}

hal_status_t hal_dual_tim_deinit_ext(dual_tim_handle_t *p_dual_timer)
{
    return hal_dual_tim_deinit(p_dual_timer);
}

void hal_dual_tim_msp_init(dual_tim_handle_t *p_dual_timer)
{
    if (NULL != p_hal_dual_tim_callback && NULL != p_hal_dual_tim_callback->dual_tim_msp_init)
    {
        p_hal_dual_tim_callback->dual_tim_msp_init(p_dual_timer);
    }
}

void hal_dual_tim_msp_deinit(dual_tim_handle_t *p_dual_timer)
{
    if (NULL != p_hal_dual_tim_callback && NULL != p_hal_dual_tim_callback->dual_tim_msp_deinit)
    {
        p_hal_dual_tim_callback->dual_tim_msp_deinit(p_dual_timer);
    }
}

void hal_dual_tim_period_elapsed_callback(dual_tim_handle_t *p_dual_timer)
{
    if (NULL != p_hal_dual_tim_callback && NULL != p_hal_dual_tim_callback->dual_tim_period_elapsed_callback)
    {
        p_hal_dual_tim_callback->dual_tim_period_elapsed_callback(p_dual_timer);
    }
}

void hal_dual_tim_register_callback(hal_dual_tim_callback_t *hal_dual_tim_callback)
{
    p_hal_dual_tim_callback = hal_dual_tim_callback;
}

#endif


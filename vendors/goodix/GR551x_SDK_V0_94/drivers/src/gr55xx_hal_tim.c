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

#if defined(HAL_TIM_MODULE_ENABLED) && defined(GR551xx_C2)

/* extern function -----------------------------------------------------------*/

extern hal_status_t hal_tim_init_ext(tim_handle_t *p_timer);
extern hal_status_t hal_tim_deinit_ext(tim_handle_t *p_timer);
extern void hal_tim_register_callback(hal_tim_callback_t *hal_tim_callback);

/* Private variables ---------------------------------------------------------*/

static hal_tim_callback_t tim_callback =
{
    .tim_msp_init                   = hal_tim_msp_init,
    .tim_msp_deinit                 = hal_tim_msp_deinit,
    .tim_period_elapsed_callback    = hal_tim_period_elapsed_callback
};

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_tim_init(tim_handle_t *p_timer)
{
    hal_tim_register_callback(&tim_callback);

    return hal_tim_init_ext(p_timer);
}

hal_status_t hal_tim_deinit(tim_handle_t *p_timer)
{
    hal_tim_register_callback(&tim_callback);

    return hal_tim_deinit_ext(p_timer);
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

__weak void hal_tim_period_elapsed_callback(tim_handle_t *p_timer)
{
    return;
}

#endif

/**
  ****************************************************************************************
  * @file    gr55xx_hal_wdt_br.c
  * @author  BLE Driver Team
  * @brief   WDT HAL module driver.
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
#include "gr55xx_hal.h"

#ifdef HAL_WDT_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/

static hal_wdt_callback_t *p_wdt_callback = NULL;

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_wdt_init_ext(wdt_handle_t *p_wdt)
{
    return hal_wdt_init(p_wdt);
}

hal_status_t hal_wdt_deinit_ext(wdt_handle_t *p_wdt)
{
    return hal_wdt_deinit(p_wdt);
}

void hal_wdt_msp_init(wdt_handle_t *p_wdt)
{
    if (NULL != p_wdt_callback && NULL != p_wdt_callback->wdt_msp_init)
    {
        p_wdt_callback->wdt_msp_init(p_wdt);
    }
}

void hal_wdt_msp_deinit(wdt_handle_t *p_wdt)
{
    if (NULL != p_wdt_callback && NULL != p_wdt_callback->wdt_msp_deinit)
    {
        p_wdt_callback->wdt_msp_deinit(p_wdt);
    }
}

void hal_wdt_period_elapsed_callback(wdt_handle_t *p_wdt)
{
    if (NULL != p_wdt_callback && NULL != p_wdt_callback->wdt_period_elapsed_callback)
    {
        p_wdt_callback->wdt_period_elapsed_callback(p_wdt);
    }
}

void hal_wdt_register_callback(hal_wdt_callback_t *hal_wdt_callback)
{
    p_wdt_callback = hal_wdt_callback;
}

#endif /* HAL_WDT_MODULE_ENABLED */

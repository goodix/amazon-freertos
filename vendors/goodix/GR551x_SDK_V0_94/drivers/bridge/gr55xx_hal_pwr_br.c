/**
  ****************************************************************************************
  * @file    gr55xx_hal_pwr_br.c
  * @author  BLE Driver Team
  * @brief   PWR HAL module driver.
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

#ifdef HAL_PWR_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/

static hal_pwr_handler_t pwr_handler;

/* Private function prototypes -----------------------------------------------*/

void hal_pwr_config_timer_wakeup_ext(uint8_t timer_mode, uint32_t load_count)
{
    hal_pwr_config_timer_wakeup(timer_mode, load_count);
}

void hal_pwr_config_ext_wakeup_ext(uint32_t ext_wakeup_pinx, uint32_t ext_wakeup_type)
{
    hal_pwr_config_ext_wakeup(ext_wakeup_pinx, ext_wakeup_type);
}

void hal_pwr_sleep_timer_elapsed_callback(void)
{
    if (NULL != pwr_handler.pwr_slp_elapsed_hander)
    {
        pwr_handler.pwr_slp_elapsed_hander();
    }
}

void hal_pwr_ext_wakeup_callback(uint32_t ext_wakeup_pinx)
{
    if (NULL != pwr_handler.pwr_ext_wakeup_hander)
    {
        pwr_handler.pwr_ext_wakeup_hander(ext_wakeup_pinx);
    }
}

void hal_pwr_register_timer_elaspsed_handler(pwr_slp_elapsed_handler_t pwr_slp_elapsed_hander)
{
    pwr_handler.pwr_slp_elapsed_hander = pwr_slp_elapsed_hander;
}

void hal_pwr_register_external_wakeup_handler(pwr_ext_wakeup_hander_t pwr_ext_wakeup_hander)
{
    pwr_handler.pwr_ext_wakeup_hander = pwr_ext_wakeup_hander;
}

#endif /* HAL_PWR_MODULE_ENABLED */

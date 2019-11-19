/**
  ****************************************************************************************
  * @file    gr55xx_hal_adc_br.c
  * @author  BLE Driver Team
  * @brief   ADC HAL module driver.
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

#ifdef HAL_ADC_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/

static adc_callback_t *p_adc_callback = NULL;

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_adc_init_ext(adc_handle_t *p_adc)
{
    return hal_adc_init(p_adc);
}

hal_status_t hal_adc_deinit_ext(adc_handle_t *p_adc)
{
    return hal_adc_deinit(p_adc);
}

void hal_adc_msp_init(adc_handle_t *p_adc)
{
    if (NULL != p_adc_callback && NULL != p_adc_callback->adc_msp_init)
    {
        p_adc_callback->adc_msp_init(p_adc);
    }
}

void hal_adc_msp_deinit(adc_handle_t *p_adc)
{
    if (NULL != p_adc_callback && NULL != p_adc_callback->adc_msp_deinit)
    {
        p_adc_callback->adc_msp_deinit(p_adc);
    }
}

void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
    if (NULL != p_adc_callback && NULL != p_adc_callback->adc_conv_cplt_callback)
    {
        p_adc_callback->adc_conv_cplt_callback(p_adc);
    }
}

void hal_adc_register_callback(adc_callback_t *adc_callback)
{
    p_adc_callback = adc_callback;
}

#endif /* HAL_ADC_MODULE_ENABLED */


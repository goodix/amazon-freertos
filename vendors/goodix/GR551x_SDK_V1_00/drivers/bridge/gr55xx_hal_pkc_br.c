/**
  ****************************************************************************************
  * @file    gr55xx_hal_pkc_br.c
  * @author  BLE Driver Team
  * @brief   PKC HAL module driver.
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
#include <string.h>

#ifdef HAL_PKC_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/

static hal_pkc_callback_t *p_pkc_callback = NULL;

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_pkc_init_ext(pkc_handle_t *p_pkc)
{
    return hal_pkc_init(p_pkc);
}

hal_status_t hal_pkc_deinit_ext(pkc_handle_t *p_pkc)
{
    return hal_pkc_deinit(p_pkc);
}

void hal_pkc_msp_init(pkc_handle_t *p_pkc)
{
    if (NULL != p_pkc_callback && NULL != p_pkc_callback->pkc_msp_init)
    {
        p_pkc_callback->pkc_msp_init(p_pkc);
    }
}

void hal_pkc_msp_deinit(pkc_handle_t *p_pkc)
{
    if (NULL != p_pkc_callback && NULL != p_pkc_callback->pkc_msp_deinit)
    {
        p_pkc_callback->pkc_msp_deinit(p_pkc);
    }
}

void hal_pkc_done_callback(pkc_handle_t *p_pkc)
{
    if (NULL != p_pkc_callback && NULL != p_pkc_callback->pkc_done_callback)
    {
        p_pkc_callback->pkc_done_callback(p_pkc);
    }
}

void hal_pkc_error_callback(pkc_handle_t *p_pkc)
{
    if (NULL != p_pkc_callback && NULL != p_pkc_callback->pkc_error_callback)
    {
        p_pkc_callback->pkc_error_callback(p_pkc);
    }
}

void hal_pkc_overflow_callback(pkc_handle_t *p_pkc)
{
    if (NULL != p_pkc_callback && NULL != p_pkc_callback->pkc_overflow_callback)
    {
        p_pkc_callback->pkc_overflow_callback(p_pkc);
    }
}

void hal_pkc_abort_cplt_callback(pkc_handle_t *p_pkc)
{
    if (NULL != p_pkc_callback && NULL != p_pkc_callback->pkc_abort_cplt_callback)
    {
        p_pkc_callback->pkc_abort_cplt_callback(p_pkc);
    }
}

void hal_pkc_register_callback(hal_pkc_callback_t *hal_pkc_callback)
{
    p_pkc_callback = hal_pkc_callback;
}

#endif /* HAL_PKC_MODULE_ENABLED */

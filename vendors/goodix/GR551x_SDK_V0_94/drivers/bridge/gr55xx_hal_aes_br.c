/**
  ****************************************************************************************
  * @file    gr55xx_hal_aes_br.c
  * @author  BLE Driver Team
  * @brief   AES HAL module driver.
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

#ifdef HAL_AES_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/

static aes_callback_t *p_aes_callback = NULL;

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_aes_init_ext(aes_handle_t *p_aes)
{
    return hal_aes_init(p_aes);
}

hal_status_t hal_aes_deinit_ext(aes_handle_t *p_aes)
{
    return hal_aes_deinit(p_aes);
}

void hal_aes_msp_init(aes_handle_t *p_aes)
{
    if (NULL != p_aes_callback && NULL != p_aes_callback->aes_msp_init)
    {
        p_aes_callback->aes_msp_init(p_aes);
    }
}

void hal_aes_msp_deinit(aes_handle_t *p_aes)
{
    if (NULL != p_aes_callback && NULL != p_aes_callback->aes_msp_deinit)
    {
        p_aes_callback->aes_msp_deinit(p_aes);
    }
}

void hal_aes_done_callback(aes_handle_t *p_aes)
{
    if (NULL != p_aes_callback && NULL != p_aes_callback->aes_done_callback)
    {
        p_aes_callback->aes_done_callback(p_aes);
    }
}

void hal_aes_error_callback(aes_handle_t *p_aes)
{
    if (NULL != p_aes_callback && NULL != p_aes_callback->aes_error_callback)
    {
        p_aes_callback->aes_error_callback(p_aes);
    }
}

void hal_aes_abort_cplt_callback(aes_handle_t *p_aes)
{
    if (NULL != p_aes_callback && NULL != p_aes_callback->aes_abort_cplt_callback)
    {
        p_aes_callback->aes_abort_cplt_callback(p_aes);
    }
}

void hal_aes_register_callback(aes_callback_t *aes_callback)
{
    p_aes_callback = aes_callback;
}

#endif /* HAL_AES_MODULE_ENABLED */


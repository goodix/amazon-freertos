/**
  ****************************************************************************************
  * @file    gr55xx_hal_i2c_br.c
  * @author  BLE Driver Team
  * @brief   I2C HAL module driver.
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

#ifdef HAL_I2C_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/

static hal_i2c_callback_t *p_i2c_callback = NULL;

/* Private function prototypes -----------------------------------------------*/

hal_status_t hal_i2c_init_ext(i2c_handle_t *p_i2c)
{
    return hal_i2c_init(p_i2c);
}

hal_status_t hal_i2c_deinit_ext(i2c_handle_t *p_i2c)
{
    return hal_i2c_deinit(p_i2c);
}

void hal_i2c_msp_init(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_msp_init)
    {
        p_i2c_callback->i2c_msp_init(p_i2c);
    }
}

void hal_i2c_msp_deinit(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_msp_deinit)
    {
        p_i2c_callback->i2c_msp_deinit(p_i2c);
    }
}

void hal_i2c_master_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_master_tx_cplt_callback)
    {
        p_i2c_callback->i2c_master_tx_cplt_callback(p_i2c);
    }
}

void hal_i2c_master_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_master_rx_cplt_callback)
    {
        p_i2c_callback->i2c_master_rx_cplt_callback(p_i2c);
    }
}

void hal_i2c_slave_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_slave_tx_cplt_callback)
    {
        p_i2c_callback->i2c_slave_tx_cplt_callback(p_i2c);
    }
}

void hal_i2c_slave_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_slave_rx_cplt_callback)
    {
        p_i2c_callback->i2c_slave_rx_cplt_callback(p_i2c);
    }
}

void hal_i2c_listen_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_listen_cplt_callback)
    {
        p_i2c_callback->i2c_listen_cplt_callback(p_i2c);
    }
}

void hal_i2c_mem_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_mem_tx_cplt_callback)
    {
        p_i2c_callback->i2c_mem_tx_cplt_callback(p_i2c);
    }
}

void hal_i2c_mem_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_mem_rx_cplt_callback)
    {
        p_i2c_callback->i2c_mem_rx_cplt_callback(p_i2c);
    }
}

void hal_i2c_error_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_error_callback)
    {
        p_i2c_callback->i2c_error_callback(p_i2c);
    }
}

void hal_i2c_abort_cplt_callback(i2c_handle_t *p_i2c)
{
    if (NULL != p_i2c_callback && NULL != p_i2c_callback->i2c_abort_cplt_callback)
    {
        p_i2c_callback->i2c_abort_cplt_callback(p_i2c);
    }
}

void hal_i2c_register_callback(hal_i2c_callback_t *hal_i2c_callback)
{
    p_i2c_callback = hal_i2c_callback;
}

#endif /* HAL_I2C_MODULE_ENABLED */

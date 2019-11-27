/**
  ****************************************************************************************
  * @file    gr55xx_ll_i2s.c
  * @author  BLE Driver Team
  * @brief   I2S LL module driver.
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
#include "gr55xx.h"
#include "gr55xx_ll_i2s.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (I2S_M) || defined (I2S_S)

/**
  * @brief  Set i2s registers to their reset values.
  * @param  I2Sx I2S instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: i2s registers are de-initialized
  *          - ERROR: invalid i2s instance
  */
__WEAK error_status_t ll_i2s_deinit(i2s_regs_t *I2Sx)
{
    error_status_t status = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_I2S_ALL_INSTANCE(I2Sx));

    ll_i2s_disable(I2Sx);
    ll_i2s_disable_rxblock(I2Sx);
    ll_i2s_disable_txblock(I2Sx);

    if (I2S_M == I2Sx)
        ll_i2s_disable_clock_div();

    ll_i2s_disable_clock(I2Sx);
    WRITE_REG(I2Sx->CLKCONFIG, 0);
    ll_i2s_disable_rx(I2Sx, 0);
    ll_i2s_disable_tx(I2Sx, 0);
    ll_i2s_set_rxsize(I2Sx, 0, LL_I2S_DATASIZE_32BIT);
    ll_i2s_set_txsize(I2Sx, 0, LL_I2S_DATASIZE_32BIT);
    ll_i2s_disable_it(I2Sx, 0, 0x33);
    ll_i2s_set_rx_fifo_threshold(I2Sx, 0, LL_I2S_THRESHOLD_9FIFO);
    ll_i2s_set_tx_fifo_threshold(I2Sx, 0, LL_I2S_THRESHOLD_9FIFO);

    return status;
}

/**
  * @brief  Set the fields of the i2s unit configuration data structure
  *         to their default values.
  * @param  p_i2s_init pointer to a @ref ll_i2s_init_t structure (i2s unit configuration data structure)
  * @retval None
  */
__WEAK void ll_i2s_struct_init(ll_i2s_init_t *p_i2s_init)
{
    /* Set the default configuration */
    p_i2s_init->rxdata_size  = LL_I2S_DATASIZE_16BIT;
    p_i2s_init->txdata_size  = LL_I2S_DATASIZE_16BIT;
    p_i2s_init->rx_threshold = LL_I2S_THRESHOLD_1FIFO;
    p_i2s_init->tx_threshold = LL_I2S_THRESHOLD_9FIFO;
    p_i2s_init->clock_source = LL_I2S_CLOCK_SRC_32M;
    p_i2s_init->audio_freq   = 48000;
}

/**
  * @brief  Configure the i2s unit.
  * @param  I2Sx I2S instance
  * @param  p_i2s_init pointer to a @ref ll_i2s_init_t structure (i2s unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: i2s registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_i2s_init(i2s_regs_t *I2Sx, ll_i2s_init_t *p_i2s_init)
{
    error_status_t status = SUCCESS;
    uint32_t wss, src, div;

    /* Check the parameters */
    gr_assert_param(IS_I2S_ALL_INSTANCE(I2Sx));

    ll_i2s_set_rxsize(I2Sx, 0, p_i2s_init->rxdata_size);
    ll_i2s_set_txsize(I2Sx, 0, p_i2s_init->txdata_size);
    ll_i2s_set_rx_fifo_threshold(I2Sx, 0, p_i2s_init->rx_threshold);
    ll_i2s_set_tx_fifo_threshold(I2Sx, 0, p_i2s_init->tx_threshold);
    ll_i2s_disable_it(I2Sx, 0, 0x33);

    if (I2S_M == I2Sx)
    {
        ll_i2s_set_clock_src(p_i2s_init->clock_source);
        if (LL_I2S_DATASIZE_16BIT >= p_i2s_init->txdata_size)
        {
            ll_i2s_set_wss(I2Sx, LL_I2S_WS_CYCLES_16);
            wss = 16;
        }
        else if (LL_I2S_DATASIZE_24BIT >= p_i2s_init->txdata_size)
        {
            ll_i2s_set_wss(I2Sx, LL_I2S_WS_CYCLES_24);
            wss = 24;
        }
        else
        {
            ll_i2s_set_wss(I2Sx, LL_I2S_WS_CYCLES_32);
            wss = 32;
        }
        /* First enlarge 10 times to reduce div error */
        src = (p_i2s_init->clock_source == LL_I2S_CLOCK_SRC_32M) ? 160000000 : 480000000;
        div = src / p_i2s_init->audio_freq / wss;
        if (5 <= (div % 10))
            div = div / 10 + 1;
        else
            div /= 10;
        if (2 <= div)
            ll_i2s_set_clock_div(div - 2);
        else
            ll_i2s_set_clock_div(0);
        /* Enable I2S clock div */
        ll_i2s_enable_clock_div();
    }

    return status;
}

#endif /* I2S_M || I2S_S */


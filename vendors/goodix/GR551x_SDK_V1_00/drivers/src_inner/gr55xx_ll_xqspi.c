/**
  ****************************************************************************************
  * @file    gr55xx_ll_xqspi.c
  * @author  BLE Driver Team
  * @brief   XQSPI LL module driver.
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
#include "gr55xx_ll_xqspi.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (XQSPI)

/**
  * @brief  Set xqspi registers to their reset values.
  * @param  XQSPIx XQSPI instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: xqspi registers are de-initialized
  *          - ERROR: invalid xqspi instance
  */
__WEAK error_status_t ll_xqspi_deinit(xqspi_regs_t *XQSPIx)
{
    /* Check the parameters */
    gr_assert_param(IS_XQSPI_ALL_INSTANCE(XQSPIx));

    /* CACHE */
    ll_xqspi_disable_cache(XQSPIx);

    /* XIP */
    ll_xqspi_disable_xip(XQSPIx);
    while(ll_xqspi_get_xip_flag(XQSPIx));

    ll_xqspi_set_xip_cmd(XQSPIx, LL_XQSPI_XIP_CMD_READ);
    WRITE_REG(XQSPIx->XIP.CTRL1, 0);
    WRITE_REG(XQSPIx->XIP.CTRL2, 0);
    ll_xqspi_disable_xip_it(XQSPIx);

    /* QSPI */
    WRITE_REG(XQSPIx->QSPI.SPIEN, 0);
    WRITE_REG(XQSPIx->QSPI.CTRL, XQSPI_QSPI_CTRL_MASTER);
    WRITE_REG(XQSPIx->QSPI.AUX_CTRL, XQSPI_QSPI_AUXCTRL_BITSIZE);
    WRITE_REG(XQSPIx->QSPI.SLAVE_SEL, 0);
    WRITE_REG(XQSPIx->QSPI.SLAVE_SEL_POL, 0);
    WRITE_REG(XQSPIx->QSPI.INTEN, 0);

    return SUCCESS;
}

/**
  * @brief  Set the fields of the xqspi unit configuration data structure
  *         to their default values.
  * @param  xqspi_init pointer to a @ref ll_xqspi_init_t structure (xqspi unit configuration data structure)
  * @retval None
  */
__WEAK void ll_xqspi_struct_init(ll_xqspi_init_t *xqspi_init)
{
    /* Set the default configuration */
    xqspi_init->mode           = LL_XQSPI_MODE_QSPI;
    xqspi_init->cache_mode     = LL_XQSPI_CACHE_EN;
    xqspi_init->read_cmd       = LL_XQSPI_XIP_CMD_READ;
    xqspi_init->data_size      = LL_XQSPI_QSPI_DATASIZE_8BIT;
    xqspi_init->data_order     = LL_XQSPI_QSPI_MSB;
    xqspi_init->clock_polarity = LL_XQSPI_SCPOL_LOW;
    xqspi_init->clock_phase    = LL_XQSPI_SCPHA_1EDGE;
    xqspi_init->baud_rate      = LL_XQSPI_BAUD_RATE_16M;
}

/**
  * @brief  Configure the xqspi unit.
  * @param  XQSPIx XQSPI instance
  * @param  xqspi_init pointer to a @ref ll_xqspi_init_t structure (xqspi unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: xqspi registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_xqspi_init(xqspi_regs_t *XQSPIx, ll_xqspi_init_t *xqspi_init)
{
    /* Check the parameters */
    gr_assert_param(IS_XQSPI_ALL_INSTANCE(XQSPIx));

    if (LL_XQSPI_MODE_QSPI == xqspi_init->mode)
    {
        if (ll_xqspi_get_xip_flag(XQSPIx))
        {
            ll_xqspi_disable_cache(XQSPIx);
            ll_xqspi_disable_xip(XQSPIx);
            while(ll_xqspi_get_xip_flag(XQSPIx));
        }

        ll_xqspi_set_qspi_tft(XQSPIx, LL_XQSPI_QSPI_FIFO_WATERMARK_1_4);
        ll_xqspi_set_qspi_rft(XQSPIx, LL_XQSPI_QSPI_FIFO_WATERMARK_1_8);
        ll_xqspi_set_qspi_cpol(XQSPIx, xqspi_init->clock_polarity);
        ll_xqspi_set_qspi_cpha(XQSPIx, xqspi_init->clock_phase);
        ll_xqspi_set_qspi_data_order(XQSPIx, xqspi_init->data_order);
        ll_xqspi_enable_qspi_contxfer(XQSPIx);
        ll_xqspi_enable_qspi_contxfer_extend(XQSPIx);
        ll_xqspi_set_qspi_datasize(XQSPIx, xqspi_init->data_size);
        ll_xqspi_set_flash_write(XQSPIx, LL_XQSPI_FLASH_WRITE_32BIT);
        ll_xqspi_enable_inhibt_rx(XQSPIx);
        ll_xqspi_set_qspi_frf(XQSPIx, LL_XQSPI_QSPI_FRF_QUADSPI);
        ll_xqspi_disable_qspi_it(XQSPIx, 0xFF);
    }
    else
    {
        ll_xqspi_disable_qspi(XQSPIx);

        ll_xqspi_set_xip_cmd(XQSPIx, xqspi_init->read_cmd);
        ll_xqspi_disable_xip_hp(XQSPIx);
        ll_xqspi_set_xip_ss(XQSPIx, LL_XQSPI_XIP_SS0);
        ll_xqspi_set_xip_cpha(XQSPIx, xqspi_init->clock_phase);
        ll_xqspi_set_xip_cpol(XQSPIx, xqspi_init->clock_polarity);
        ll_xqspi_set_xip_endian(XQSPIx, LL_XQSPI_XIP_ENDIAN_LITTLE);
        ll_xqspi_set_xip_hp_cmd(XQSPIx, 0x33);

        ll_xqspi_set_qspi_datasize(XQSPIx, LL_XQSPI_QSPI_DATASIZE_32BIT);
        ll_xqspi_set_flash_write(XQSPIx, LL_XQSPI_FLASH_WRITE_128BIT);
        ll_xqspi_enable_inhibt_rx(XQSPIx);
        switch(xqspi_init->read_cmd)
        {
        case LL_XQSPI_XIP_CMD_READ:
        case LL_XQSPI_XIP_CMD_DUAL_IO_READ:
            ll_xqspi_set_xip_dummycycles(XQSPIx, 0);
            break;
        case LL_XQSPI_XIP_CMD_FAST_READ:
        case LL_XQSPI_XIP_CMD_DUAL_OUT_READ:
        case LL_XQSPI_XIP_CMD_QUAD_OUT_READ:
            ll_xqspi_set_xip_dummycycles(XQSPIx, 1);
            break;
        case LL_XQSPI_XIP_CMD_QUAD_IO_READ:
            ll_xqspi_set_xip_dummycycles(XQSPIx, 2);
            break;
        default:
            break;
        }

        ll_xqspi_disable_cache(XQSPIx);

        if (LL_XQSPI_CACHE_EN == xqspi_init->cache_mode)
        {
            uint32_t __l_ctrl_rest;
            uint32_t __l_ret_rest =  ll_xqspi_is_enable_cache_retention();
            ll_xqspi_disable_cache_retention();

            __l_ctrl_rest = READ_REG(XQSPIx->CACHE.CTRL0) & XQSPI_CACHE_CTRL0_CLK_FORCE_EN;
            MODIFY_REG(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_CLK_FORCE_EN, 0x0);
            ll_xqspi_set_cache_fifo(XQSPIx, LL_XQSPI_CACHE_FIFO_CLEAR);
            ll_xqspi_set_cache_hitmiss(XQSPIx, LL_XQSPI_CACHE_HITMISS_CLEAR);
            ll_xqspi_set_cache_dbgbus(XQSPIx, 0);
            ll_xqspi_disable_cache_dbgmux(XQSPIx);

            ll_xqspi_enable_cache_flush(XQSPIx);

            __asm volatile ("nop; nop; nop; nop; nop; nop;");
            
            //check tag memory flush done flag
            while(ll_xqspi_get_cache_flag(XQSPIx));
            ll_xqspi_set_cache_fifo(XQSPIx, LL_XQSPI_CACHE_FIFO_NORMAL);
            ll_xqspi_disable_cache_flush(XQSPIx);
            ll_xqspi_enable_cache(XQSPIx);
            MODIFY_REG(XQSPIx->CACHE.CTRL0, XQSPI_CACHE_CTRL0_CLK_FORCE_EN, __l_ctrl_rest);
            
            if(__l_ret_rest)
                ll_xqspi_enable_cache_retention();
        }

        ll_xqspi_enable_xip(XQSPIx);
        while(!ll_xqspi_get_xip_flag(XQSPIx));
    }

    ll_xqspi_set_qspi_speed(xqspi_init->baud_rate);
    ll_xqspi_enable_exflash_power();

    return SUCCESS;
}

#endif /* XQSPI */


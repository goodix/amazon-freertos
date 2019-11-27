/**
  ****************************************************************************************
  * @file    gr55xx_hal_xqspi.c
  * @author  BLE Driver Team
  * @brief   XQSPI HAL module driver.
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

#ifdef HAL_XQSPI_MODULE_ENABLED

static hal_status_t xqspi_wait_flag_state_until_retry(xqspi_handle_t *p_xqspi, uint32_t flag, flag_status_t state, uint32_t retry);
static hal_status_t xqspi_transmit(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t retry);
static hal_status_t xqspi_receive(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t retry);
static hal_status_t xqspi_send_inst_addr(xqspi_handle_t *p_xqspi, xqspi_command_t *cmd);

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_init(xqspi_handle_t *p_xqspi)
{
    hal_status_t    status = HAL_OK;
    ll_xqspi_init_t xqspi_init;

    /* Check the XQSPI handle allocation */
    if (NULL == p_xqspi)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_XQSPI_ALL_INSTANCE(p_xqspi->p_instance));
    gr_assert_param(IS_XQSPI_WORK_MODE(p_xqspi->init.work_mode));
    gr_assert_param(IS_XQSPI_CACHE_MODE(p_xqspi->init.cache_mode));
    gr_assert_param(IS_XQSPI_READ_CMD(p_xqspi->init.read_cmd));
    gr_assert_param(IS_XQSPI_BAUD_RATE(p_xqspi->init.baud_rate));
    gr_assert_param(IS_XQSPI_CLOCK_MODE(p_xqspi->init.clock_mode));

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    if (HAL_XQSPI_STATE_RESET == p_xqspi->state)
    {
        /* Allocate lock resource and initialize it */
        p_xqspi->lock = HAL_UNLOCKED;

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_xqspi_msp_init(p_xqspi);

        /* Configure the default retry for the XQSPI memory access */
        hal_xqspi_set_retry(p_xqspi, HAL_XQSPI_RETRY_DEFAULT_VALUE);
    }

    if (!ll_xqspi_get_xip_flag(p_xqspi->p_instance))
        /* Wait till BUSY flag reset */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_BUSY, RESET, p_xqspi->retry);

    if (HAL_OK == status)
    {
        /* Configure XQSPI initial paraments */
        xqspi_init.mode           = p_xqspi->init.work_mode;
        xqspi_init.cache_mode     = p_xqspi->init.cache_mode;
        xqspi_init.read_cmd       = p_xqspi->init.read_cmd;
        xqspi_init.baud_rate      = p_xqspi->init.baud_rate;
        xqspi_init.clock_polarity = (p_xqspi->init.clock_mode & 0x2) ? 1 : 0;
        xqspi_init.clock_phase    = (p_xqspi->init.clock_mode & 0x1) ? 1 : 0;
        xqspi_init.data_size      = LL_XQSPI_QSPI_DATASIZE_8BIT;
        xqspi_init.data_order     = LL_XQSPI_QSPI_MSB;

        if (SUCCESS == ll_xqspi_init(p_xqspi->p_instance, &xqspi_init))
        {
            if (XQSPI_WORK_MODE_QSPI == p_xqspi->init.work_mode)
                /* Enable the XQSPI peripheral */
                __HAL_XQSPI_ENABLE_QSPI(p_xqspi);

            /* Set XQSPI error code to none */
            p_xqspi->error_code = HAL_XQSPI_ERROR_NONE;

            /* Initialize the XQSPI state */
            p_xqspi->state = HAL_XQSPI_STATE_READY;
        }
        else
        {
            status = HAL_ERROR;
        }
    }

    /* Release Lock */
    __HAL_UNLOCK(p_xqspi);

    /* Return function status */
    return status;
}

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_deinit(xqspi_handle_t *p_xqspi)
{
    /* Check the XQSPI handle allocation */
    if (NULL == p_xqspi)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    /* Disable the XQSPI Peripheral Clock */
    ll_xqspi_deinit(p_xqspi->p_instance);

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    hal_xqspi_msp_deinit(p_xqspi);

    /* Set XQSPI error code to none */
    p_xqspi->error_code = HAL_XQSPI_ERROR_NONE;

    /* Initialize the XQSPI state */
    p_xqspi->state = HAL_XQSPI_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_xqspi);

    return HAL_OK;
}

SECTION_RAM_CODE  __WEAK void hal_xqspi_msp_init(xqspi_handle_t *p_xqspi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_xqspi_msp_init can be implemented in the user file
     */
}

SECTION_RAM_CODE  __WEAK void hal_xqspi_msp_deinit(xqspi_handle_t *p_xqspi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_xqspi_msp_deinit can be implemented in the user file
     */
}

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_command_transmit(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd, uint8_t *p_data, uint32_t retry)
{
    hal_status_t status = HAL_ERROR;

    /* Check the parameters */
    gr_assert_param(IS_XQSPI_INSTADDR_MODE(p_cmd->inst_addr_mode));
    gr_assert_param(IS_XQSPI_INSTRUCTION_SIZE(p_cmd->inst_size));
    gr_assert_param(IS_XQSPI_ADDRESS_SIZE(p_cmd->addr_size));
    gr_assert_param(IS_XQSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    if (HAL_XQSPI_STATE_READY == p_xqspi->state)
    {
        p_xqspi->error_code = HAL_XQSPI_ERROR_NONE;

        /* Update XQSPI state */
        p_xqspi->state = HAL_XQSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_BUSY, RESET, retry);

        if (HAL_OK == status)
        {
            p_xqspi->state = HAL_XQSPI_STATE_BUSY_INDIRECT_TX;

            p_xqspi->p_tx_buffer   = p_data;
            p_xqspi->tx_xfer_size  = p_cmd->length;
            p_xqspi->tx_xfer_count = p_cmd->length;

            ll_xqspi_enable_qspi_ssout(p_xqspi->p_instance, 1);
            xqspi_send_inst_addr(p_xqspi, p_cmd);
            status = xqspi_transmit(p_xqspi, p_data, retry);
            ll_xqspi_disable_qspi_ssout(p_xqspi->p_instance, 1);
        }

        p_xqspi->state = HAL_XQSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_xqspi);

    /* Return function status */
    return status;
}

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_command_receive(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd, uint8_t *p_data, uint32_t retry)
{
    hal_status_t status = HAL_ERROR;

    /* Check the parameters */
    gr_assert_param(IS_XQSPI_INSTADDR_MODE(p_cmd->inst_addr_mode));
    gr_assert_param(IS_XQSPI_INSTRUCTION_SIZE(p_cmd->inst_size));
    gr_assert_param(IS_XQSPI_ADDRESS_SIZE(p_cmd->addr_size));
    gr_assert_param(IS_XQSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    if (HAL_XQSPI_STATE_READY == p_xqspi->state)
    {
        p_xqspi->error_code = HAL_XQSPI_ERROR_NONE;

        /* Update XQSPI state */
        p_xqspi->state = HAL_XQSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_BUSY, RESET, retry);

        if (HAL_OK == status)
        {
            p_xqspi->state = HAL_XQSPI_STATE_BUSY_INDIRECT_RX;

            p_xqspi->p_rx_buffer   = p_data;
            p_xqspi->rx_xfer_size  = p_cmd->length;
            p_xqspi->rx_xfer_count = p_cmd->length;

            ll_xqspi_enable_qspi_ssout(p_xqspi->p_instance, 1);
            xqspi_send_inst_addr(p_xqspi, p_cmd);
            status = xqspi_receive(p_xqspi, p_data, retry);
            ll_xqspi_disable_qspi_ssout(p_xqspi->p_instance, 1);
        }

        p_xqspi->state = HAL_XQSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_xqspi);

    /* Return function status */
    return status;
}

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_transmit(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t length, uint32_t retry)
{
    hal_status_t status = HAL_ERROR;

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    if (HAL_XQSPI_STATE_READY == p_xqspi->state)
    {
        p_xqspi->error_code = HAL_XQSPI_ERROR_NONE;

        /* Update XQSPI state */
        p_xqspi->state = HAL_XQSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_BUSY, RESET, retry);

        if (HAL_OK == status)
        {
            /* Configurate frame format */
            ll_xqspi_set_qspi_frf(p_xqspi->p_instance, XQSPI_DATA_MODE_SPI);

            p_xqspi->state = HAL_XQSPI_STATE_BUSY_INDIRECT_TX;

            p_xqspi->p_tx_buffer   = p_data;
            p_xqspi->tx_xfer_size  = length;
            p_xqspi->tx_xfer_count = length;

            ll_xqspi_enable_qspi_ssout(p_xqspi->p_instance, 1);
            status = xqspi_transmit(p_xqspi, p_data, retry);
            ll_xqspi_disable_qspi_ssout(p_xqspi->p_instance, 1);
        }

        p_xqspi->state = HAL_XQSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_xqspi);

    /* Return function status */
    return status;
}

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_receive(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t length, uint32_t retry)
{
    hal_status_t status = HAL_ERROR;

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    if (HAL_XQSPI_STATE_READY == p_xqspi->state)
    {
        p_xqspi->error_code = HAL_XQSPI_ERROR_NONE;

        /* Update XQSPI state */
        p_xqspi->state = HAL_XQSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_BUSY, RESET, retry);

        if (HAL_OK == status)
        {
            /* Configurate frame format */
            ll_xqspi_set_qspi_frf(p_xqspi->p_instance, XQSPI_DATA_MODE_SPI);

            p_xqspi->state = HAL_XQSPI_STATE_BUSY_INDIRECT_RX;

            p_xqspi->p_rx_buffer     = p_data;
            p_xqspi->rx_xfer_size  = length;
            p_xqspi->rx_xfer_count = length;

            ll_xqspi_enable_qspi_ssout(p_xqspi->p_instance, 1);
            status = xqspi_receive(p_xqspi, p_data, retry);
            ll_xqspi_disable_qspi_ssout(p_xqspi->p_instance, 1);
        }

        p_xqspi->state = HAL_XQSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_xqspi);

    /* Return function status */
    return status;
}

SECTION_RAM_CODE  __WEAK hal_xqspi_state_t hal_xqspi_get_state(xqspi_handle_t *p_xqspi)
{
    /* Return XQSPI handle state */
    return p_xqspi->state;
}

SECTION_RAM_CODE  __WEAK uint32_t hal_xqspi_get_error(xqspi_handle_t *p_xqspi)
{
    return p_xqspi->error_code;
}

SECTION_RAM_CODE  __WEAK void hal_xqspi_set_retry(xqspi_handle_t *p_xqspi, uint32_t retry)
{
    p_xqspi->retry = retry;
}

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_set_tx_fifo_threshold(xqspi_handle_t *p_xqspi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    if (HAL_XQSPI_STATE_READY == p_xqspi->state)
    {
        /* Configure XQSPI FIFO Threshold */
        ll_xqspi_set_qspi_tft(p_xqspi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_xqspi);

    /* Return function status */
    return status;
}

SECTION_RAM_CODE  __WEAK hal_status_t hal_xqspi_set_rx_fifo_threshold(xqspi_handle_t *p_xqspi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_xqspi);

    if (HAL_XQSPI_STATE_READY == p_xqspi->state)
    {
        /* Configure XQSPI FIFO Threshold */
        ll_xqspi_set_qspi_rft(p_xqspi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_xqspi);

    /* Return function status */
    return status;
}

SECTION_RAM_CODE  __WEAK uint32_t hal_xqspi_get_tx_fifo_threshold(xqspi_handle_t *p_xqspi)
{
    return ll_xqspi_get_qspi_tft(p_xqspi->p_instance);
}

SECTION_RAM_CODE  __WEAK uint32_t hal_xqspi_get_rx_fifo_threshold(xqspi_handle_t *p_xqspi)
{
    return ll_xqspi_get_qspi_rft(p_xqspi->p_instance);
}

SECTION_RAM_CODE  __WEAK void hal_xqspi_set_xip_present_status(xqspi_handle_t *p_xqspi, uint32_t status)
{
    xqspi_regs_t *p_instance = p_xqspi->p_instance;

    ll_xqspi_disable_cache(p_instance);
    ll_xqspi_disable_xip(p_instance);
    while(ll_xqspi_get_xip_flag(p_instance));
    //Turn off encryption.
    if(XQSPI_DISABLE_PRESENT == status)
    {
        ll_xqspi_set_present_bypass(p_instance, LL_XQSPI_DISABLE_PRESENT);
    }
    else
    {
        ll_xqspi_set_present_bypass(p_instance, LL_XQSPI_ENABLE_PRESENT);
    }
    if(LL_XQSPI_CACHE_EN == p_xqspi->init.cache_mode)
    {
        uint32_t __l_ctrl_rest;
        uint32_t __l_ret_rest =  ll_xqspi_is_enable_cache_retention();
        ll_xqspi_disable_cache_retention();

        __l_ctrl_rest = READ_REG(p_instance->CACHE.CTRL0) & XQSPI_CACHE_CTRL0_CLK_FORCE_EN;
        MODIFY_REG(p_instance->CACHE.CTRL0, XQSPI_CACHE_CTRL0_CLK_FORCE_EN, 0x0);
        ll_xqspi_set_cache_fifo(p_instance, LL_XQSPI_CACHE_FIFO_CLEAR);
        ll_xqspi_enable_cache_flush(p_instance);
        
        __asm volatile ("nop; nop; nop; nop; nop; nop;");
        
        //check tag memory flush done flag
        while(ll_xqspi_get_cache_flag(p_instance));
        ll_xqspi_set_cache_fifo(p_instance, LL_XQSPI_CACHE_FIFO_NORMAL);
        ll_xqspi_disable_cache_flush(p_instance);        
        ll_xqspi_enable_cache(p_instance);
        MODIFY_REG(p_instance->CACHE.CTRL0, XQSPI_CACHE_CTRL0_CLK_FORCE_EN, __l_ctrl_rest);

        if(__l_ret_rest)
            ll_xqspi_enable_cache_retention();
    }
    ll_xqspi_enable_xip(p_instance);
    while(!ll_xqspi_get_xip_flag(p_instance));

    return;
}

SECTION_RAM_CODE  static hal_status_t xqspi_wait_flag_state_until_retry(xqspi_handle_t *p_xqspi, uint32_t flag, flag_status_t state, uint32_t retry)
{
    /* Wait until flag is in expected state */
    while ((__HAL_XQSPI_GET_FLAG(p_xqspi, flag)) != state)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != retry)
        {
            if (retry-- == 0)
            {
                p_xqspi->state       = HAL_XQSPI_STATE_ERROR;
                p_xqspi->error_code |= HAL_XQSPI_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

SECTION_RAM_CODE  static hal_status_t xqspi_transmit(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t retry)
{
    hal_status_t  status   = HAL_OK;
    __O uint32_t *data_reg = &p_xqspi->p_instance->QSPI.TX_DATA;

    ll_xqspi_enable_inhibt_rx(p_xqspi->p_instance);

    while (0U < p_xqspi->tx_xfer_count)
    {
        /* Wait until TFF flag is reset to send data */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_TFF, RESET, retry);

        if (HAL_OK != status)
            break;

        do
        {
            *(__IO uint8_t *)((__IO void *)data_reg) = *p_xqspi->p_tx_buffer++;
            p_xqspi->tx_xfer_count--;
        } while((p_xqspi->tx_xfer_count) && (!ll_xqspi_is_active_qspi_flag(p_xqspi->p_instance, LL_XQSPI_QSPI_STAT_TFF)));
    }

    if (HAL_OK == status)
    {
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_TFE, SET, retry);
        /* Wait until BUSY flag is reset to go back in idle state */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_BUSY, RESET, retry);
    }

    return status;
}

SECTION_RAM_CODE  static hal_status_t xqspi_receive(xqspi_handle_t *p_xqspi, uint8_t *p_data, uint32_t retry)
{
    hal_status_t  status      = HAL_OK;
    __I uint32_t *data_reg    = &p_xqspi->p_instance->QSPI.RX_DATA;
    uint32_t __l_present_rest = ll_xqspi_get_present_bypass(p_xqspi->p_instance);
    uint32_t cont, tx_cont;

    ll_xqspi_disable_inhibt_rx(p_xqspi->p_instance);
    ll_xqspi_set_present_bypass(p_xqspi->p_instance, LL_XQSPI_DISABLE_PRESENT);
    if (p_xqspi->rx_xfer_size > XQSPI_FIFO_DEPTH)
    {
        uint32_t tmp, *buf32b = (uint32_t *)p_data;
        uint8_t  rx_remain = 0;
        ll_xqspi_set_qspi_datasize(p_xqspi->p_instance, LL_XQSPI_QSPI_DATASIZE_32BIT);
        rx_remain = p_xqspi->rx_xfer_size & 0x3;
        p_xqspi->rx_xfer_count = p_xqspi->rx_xfer_size >> 2;

        while (0U < p_xqspi->rx_xfer_count)
        {
            tx_cont = (p_xqspi->rx_xfer_count > XQSPI_FIFO_DEPTH) ? XQSPI_FIFO_DEPTH : p_xqspi->rx_xfer_count;
            cont = tx_cont;

            /* Full TX FIFO to up speed */
            while(cont--)
            {
                p_xqspi->p_instance->QSPI.TX_DATA = 0xFFFFFFFF;
            }

            cont = tx_cont;
            while (cont--)
            {
                /* Ensure the correctness of the data read */
                while(RESET != __HAL_XQSPI_GET_FLAG(p_xqspi, XQSPI_FLAG_RFE));
                tmp = *data_reg;
                *buf32b++ = ((tmp & 0x000000FF) << 24) | \
                            ((tmp & 0x0000FF00) << 8)  | \
                            ((tmp & 0x00FF0000) >> 8)  | \
                            ((tmp & 0xFF000000) >> 24);
            }

            p_xqspi->rx_xfer_count -= tx_cont;
        }
        if (rx_remain)
        {
            p_xqspi->p_instance->QSPI.TX_DATA = 0xFFFFFFFF;
            while(RESET != __HAL_XQSPI_GET_FLAG(p_xqspi, XQSPI_FLAG_RFE));
            tmp = *data_reg;
            for (uint8_t i = 0; i < rx_remain; i++)
            {
                ((uint8_t *)buf32b)[i] = (tmp & 0xFF000000) >> 24;
                tmp <<= 8;
            }
        }
        ll_xqspi_set_qspi_datasize(p_xqspi->p_instance, LL_XQSPI_QSPI_DATASIZE_8BIT);
    }
    else
    {
        cont = p_xqspi->rx_xfer_count;
        /* Full TX FIFO to up speed */
        while(cont--)
        {
            p_xqspi->p_instance->QSPI.TX_DATA = 0xFFFFFFFF;
        }

        cont = p_xqspi->rx_xfer_count;
        while (cont--)
        {
            /* Ensure the correctness of the data read */
            while(RESET != __HAL_XQSPI_GET_FLAG(p_xqspi, XQSPI_FLAG_RFE));
            *p_xqspi->p_rx_buffer++ = *((__IO uint8_t *)data_reg);
        }

        p_xqspi->rx_xfer_count = 0;
    }
    ll_xqspi_set_present_bypass(p_xqspi->p_instance, __l_present_rest);

    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_BUSY, RESET, retry);
    }

    return status;
}

SECTION_RAM_CODE  static hal_status_t xqspi_send_inst_addr(xqspi_handle_t *p_xqspi, xqspi_command_t *p_cmd)
{
    uint8_t  data_byte, dummy_dec;

    if (XQSPI_DATA_MODE_SPI == p_cmd->data_mode)
        dummy_dec = 8;
    else if (XQSPI_DATA_MODE_DUALSPI == p_cmd->data_mode)
        dummy_dec = 4;
    else
        dummy_dec = 2;

    ll_xqspi_enable_inhibt_rx(p_xqspi->p_instance);

    if (XQSPI_INST_ADDR_ALL_IN_SPIFRF == p_cmd->inst_addr_mode)
        ll_xqspi_set_qspi_frf(p_xqspi->p_instance, p_cmd->data_mode);
    else
        ll_xqspi_set_qspi_frf(p_xqspi->p_instance, XQSPI_DATA_MODE_SPI);

    if (XQSPI_INSTSIZE_00_BITS != p_cmd->inst_size)
    {
        for (data_byte = p_cmd->inst_size; data_byte > 0; data_byte--)
        {
            p_xqspi->p_instance->QSPI.TX_DATA = (p_cmd->inst >> ((data_byte - 1) << 3)) & 0xFF;
        }
    }

    if ((XQSPI_INST_IN_SPI_ADDR_IN_SPIFRF == p_cmd->inst_addr_mode) && (XQSPI_DATA_MODE_SPI != p_cmd->data_mode))
    {
        xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_TFE, SET, 1000);
        ll_xqspi_set_qspi_frf(p_xqspi->p_instance, p_cmd->data_mode);
    }

    if (XQSPI_ADDRSIZE_00_BITS != p_cmd->addr_size)
    {
        for (data_byte = p_cmd->addr_size; data_byte > 0; data_byte--)
        {
            p_xqspi->p_instance->QSPI.TX_DATA = (p_cmd->addr >> ((data_byte - 1) << 3)) & 0xFF;
        }
    }

    if ((XQSPI_INST_ADDR_ALL_IN_SPI == p_cmd->inst_addr_mode) && (XQSPI_DATA_MODE_SPI != p_cmd->data_mode))
    {
        xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_TFE, SET, 1000);
        ll_xqspi_set_qspi_frf(p_xqspi->p_instance, p_cmd->data_mode);
    }

    if (0 != p_cmd->dummy_cycles)
    {
        for (data_byte = p_cmd->dummy_cycles; data_byte > 0; data_byte -= dummy_dec)
        {
            p_xqspi->p_instance->QSPI.TX_DATA = 0xFF;
        }
    }

    xqspi_wait_flag_state_until_retry(p_xqspi, XQSPI_FLAG_TFE, SET, 1000);

    return HAL_OK;
}

#endif /* HAL_XQSPI_MODULE_ENABLED */


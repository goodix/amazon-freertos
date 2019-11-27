/**
  ****************************************************************************************
  * @file    gr55xx_hal_i2s.c
  * @author  BLE Driver Team
  * @brief   I2S HAL module driver.
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


#ifdef HAL_I2S_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void i2s_dma_rx_cplt(dma_handle_t *p_dma);
static void i2s_dma_tx_cplt(dma_handle_t *p_dma);
static void i2s_dma_error(dma_handle_t *p_dma);
static void i2s_dma_abort_cplt(dma_handle_t *p_dma);
static void i2s_send_16bit(i2s_handle_t *p_i2s);
static void i2s_send_32bit(i2s_handle_t *p_i2s);
static void i2s_receive_16bit(i2s_handle_t *p_i2s);
static void i2s_receive_32bit(i2s_handle_t *p_i2s);
static hal_status_t i2s_transmit(i2s_handle_t *p_i2s, uint32_t timeout);
static hal_status_t i2s_receive(i2s_handle_t *p_i2s, uint32_t timeout);
static hal_status_t i2s_transmit_receive(i2s_handle_t *p_i2s, uint32_t timeout);

__WEAK hal_status_t hal_i2s_init(i2s_handle_t *p_i2s)
{
    hal_status_t   status    = HAL_OK;
    error_status_t err       = SUCCESS;
    ll_i2s_init_t  i2s_init;

    /* Check the I2S handle allocation */
    if (NULL == p_i2s)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_I2S_ALL_INSTANCE(p_i2s->p_instance));
    gr_assert_param(IS_I2S_DATASIZE(p_i2s->init.data_size));
    if (I2S_M == p_i2s->p_instance)
    {
        gr_assert_param(IS_I2S_AUDIO_FREQUENCY(p_i2s->init.audio_freq));
    }

    if (HAL_I2S_STATE_RESET == p_i2s->state)
    {
        /* Allocate lock resource and initialize it */
        p_i2s->lock = HAL_UNLOCKED;

        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable I2S_M/I2S_S Clock */
        if(p_i2s->p_instance == I2S_M)
        {
            ll_cgc_disable_force_off_i2s_m_hclk();
        }
        else if(p_i2s->p_instance == I2S_S)
        {
            ll_cgc_disable_force_off_i2s_s_hclk();
        }

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_i2s_msp_init(p_i2s);

        /* Configure the default timeout for the I2S memory access */
        p_i2s->timeout = HAL_I2S_TIMEOUT_DEFAULT_VALUE;
    }

    p_i2s->state = HAL_I2S_STATE_BUSY;

    __HAL_I2S_DISABLE(p_i2s);

    /* Set each i2s_init field to default value. */
    ll_i2s_struct_init(&i2s_init);

    /* Configure I2S Clock Prescaler and Clock Mode */
    i2s_init.rxdata_size  = p_i2s->init.data_size;
    i2s_init.txdata_size  = p_i2s->init.data_size;
    i2s_init.clock_source = p_i2s->init.clock_source;
    i2s_init.audio_freq   = p_i2s->init.audio_freq;
    err = ll_i2s_init(p_i2s->p_instance, &i2s_init);

    if (SUCCESS == err)
    {
        /* Enable the I2S peripheral */
        __HAL_I2S_ENABLE(p_i2s);

        /* Set write/read fifo interface */
        if (p_i2s->init.data_size <= I2S_DATASIZE_16BIT)
        {
            p_i2s->write_fifo = i2s_send_16bit;
            p_i2s->read_fifo = i2s_receive_16bit;
        }
        else
        {
            p_i2s->write_fifo = i2s_send_32bit;
            p_i2s->read_fifo = i2s_receive_32bit;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Set I2S error code to none */
    p_i2s->error_code = HAL_I2S_ERROR_NONE;

    /* Initialize the I2S state */
    p_i2s->state = HAL_I2S_STATE_READY;

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_deinit(i2s_handle_t *p_i2s)
{
    /* Check the I2S handle allocation */
    if (NULL == p_i2s)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_i2s);

    if (p_i2s->state != HAL_I2S_STATE_RESET)
    {
        /* Disable the I2S Peripheral Clock */
        ll_i2s_deinit(p_i2s->p_instance);

        /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
        hal_i2s_msp_deinit(p_i2s);

        /* Disable I2S_M/I2S_S Clock */
        if(p_i2s->p_instance == I2S_M)
        {
            ll_cgc_enable_force_off_i2s_m_hclk();
        }
        else if(p_i2s->p_instance == I2S_S)
        {
            ll_cgc_enable_force_off_i2s_s_hclk();
        }

        if((LL_CGC_FRC_I2S_S_HCLK & ll_cgc_get_force_off_hclk_0()) &&
           ((LL_CGC_FRC_SERIALS_HCLK2 & ll_cgc_get_force_off_hclk_2()) == LL_CGC_FRC_SERIALS_HCLK2))
        {
            /* Disable Clock for Serial blocks  */
            ll_cgc_enable_force_off_serial_hclk();
        }

        /* Set I2S error code to none */
        p_i2s->error_code = HAL_I2S_ERROR_NONE;

        /* Initialize the I2S state */
        p_i2s->state = HAL_I2S_STATE_RESET;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_i2s);

    return HAL_OK;
}

__WEAK void hal_i2s_msp_init(i2s_handle_t *p_i2s)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_i2s_msp_init can be implemented in the user file
     */
}

__WEAK void hal_i2s_msp_deinit(i2s_handle_t *p_i2s)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_i2s_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_i2s_irq_handler(i2s_handle_t *p_i2s)
{
    uint32_t itflags = ll_i2s_get_it_flag(p_i2s->p_instance, 0);

    if ((itflags & (I2S_FLAG_TXFO | I2S_FLAG_RXFO)) && \
        (ll_i2s_is_enabled_it(p_i2s->p_instance, 0, I2S_IT_TXFO) || \
         ll_i2s_is_enabled_it(p_i2s->p_instance, 0, I2S_IT_RXFO)))
    {
        /* Disable all the I2S Interrupts */
        __HAL_I2S_DISABLE_IT(p_i2s, (I2S_IT_TXFO | I2S_IT_TXFE | I2S_IT_RXFO | I2S_IT_RXDA));
        /* Clear all interrupt flags */
        __HAL_I2S_CLEAR_FLAG(p_i2s, (I2S_IT_TXFO | I2S_IT_RXFO));

        if (I2S_M == p_i2s->p_instance)
        {
            /* Disable clock */
            __HAL_I2S_DISABLE_CLOCK(p_i2s);
        }

        /* Set error code */
        p_i2s->error_code |= HAL_I2S_ERROR_TRANSFER;

        if (ll_i2s_is_enabled_dma(p_i2s->p_instance))
        {
            /* Abort DMA channel */
            p_i2s->p_dmatx->xfer_abort_callback = i2s_dma_abort_cplt;
            p_i2s->p_dmarx->xfer_abort_callback = i2s_dma_abort_cplt;
            hal_dma_abort_it(p_i2s->p_dmatx);
            hal_dma_abort_it(p_i2s->p_dmarx);
        }
        else
        {
            /* Change state of I2S */
            p_i2s->state = HAL_I2S_STATE_READY;

            /* Error callback */
            hal_i2s_error_callback(p_i2s);
        }
    }

    if ((itflags & I2S_FLAG_RXDA) && ll_i2s_is_enabled_it(p_i2s->p_instance, 0, I2S_IT_RXDA))
    {
        p_i2s->read_fifo(p_i2s);

        if (0 == p_i2s->rx_xfer_count)
        {
            /* All data have been received for the transfer */
            /* Disable the I2S RX Interrupt */
            __HAL_I2S_DISABLE_IT(p_i2s, I2S_IT_RXDA | I2S_IT_RXFO);

            if (HAL_I2S_STATE_BUSY_RX == p_i2s->state)
            {
                /* Change state of I2S */
                p_i2s->state = HAL_I2S_STATE_READY;
                hal_i2s_rx_cplt_callback(p_i2s);
            }
            else if (HAL_I2S_STATE_BUSY_TX_RX == p_i2s->state)
            {
                /* Change state of I2S */
                p_i2s->state = HAL_I2S_STATE_READY;
                hal_i2s_tx_rx_cplt_callback(p_i2s);
            }
        }
    }

    if ((itflags & I2S_FLAG_TXFE) && ll_i2s_is_enabled_it(p_i2s->p_instance, 0, I2S_IT_TXFE))
    {
        if (0 == p_i2s->tx_xfer_count)
        {
            /* Disable the I2S TX Interrupt */
            __HAL_I2S_DISABLE_IT(p_i2s, I2S_IT_TXFE | I2S_IT_TXFO);

            if (HAL_I2S_STATE_BUSY_TX == p_i2s->state)
            {
                /* Change state of I2S */
                p_i2s->state = HAL_I2S_STATE_READY;
                hal_i2s_tx_cplt_callback(p_i2s);
            }
        }
        else
        {
            p_i2s->write_fifo(p_i2s);
        }
    }

}

__WEAK hal_status_t hal_i2s_transmit(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (I2S_DATASIZE_16BIT < p_i2s->init.data_size)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
        }

        /* Configure counters and size of the handle */
        p_i2s->tx_xfer_count = length;
        p_i2s->tx_xfer_size  = length;
        p_i2s->p_tx_buffer   = p_data;

        p_i2s->rx_xfer_count = 0;
        p_i2s->rx_xfer_size  = 0;
        p_i2s->p_rx_buffer   = NULL;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_TX;

        /* Enable channel TX & Disable channel RX */
        __HAL_I2S_ENABLE_TX_CHANNEL(p_i2s, 0);
        __HAL_I2S_DISABLE_RX_CHANNEL(p_i2s, 0);
        /* Enable TX block & Disable RX block */
        __HAL_I2S_ENABLE_TX_BLOCK(p_i2s);
        __HAL_I2S_DISABLE_RX_BLOCK(p_i2s);

        status = i2s_transmit(p_i2s, timeout);

        p_i2s->state = HAL_I2S_STATE_READY;

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_receive(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status   = HAL_OK;

    if ((NULL == p_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (I2S_DATASIZE_16BIT < p_i2s->init.data_size)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
        }

        /* Configure counters and size of the handle */
        p_i2s->rx_xfer_count = length;
        p_i2s->rx_xfer_size  = length;
        p_i2s->p_rx_buffer   = p_data;

        p_i2s->tx_xfer_count = 0;
        p_i2s->tx_xfer_size  = 0;
        p_i2s->p_tx_buffer   = NULL;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_RX;

        /* Enable channel RX & Disable channel TX */
        __HAL_I2S_ENABLE_RX_CHANNEL(p_i2s, 0);
        __HAL_I2S_DISABLE_TX_CHANNEL(p_i2s, 0);
        /* Enable RX block & Disable TX block */
        __HAL_I2S_ENABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_DISABLE_TX_BLOCK(p_i2s);

        status = i2s_receive(p_i2s, timeout);

        p_i2s->state = HAL_I2S_STATE_READY;

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_transmit_receive(i2s_handle_t *p_i2s, uint16_t *p_tx_data, uint16_t *p_rx_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_tx_data) || (NULL == p_rx_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (I2S_DATASIZE_16BIT < p_i2s->init.data_size)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
        }

        /* Configure counters and size of the handle */
        p_i2s->tx_xfer_count = length;
        p_i2s->tx_xfer_size  = length;
        p_i2s->p_tx_buffer   = p_tx_data;

        p_i2s->rx_xfer_count = length;
        p_i2s->rx_xfer_size  = length;
        p_i2s->p_rx_buffer   = p_rx_data;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_TX_RX;

        /* Enable channel TX/RX */
        __HAL_I2S_ENABLE_RX_CHANNEL(p_i2s, 0);
        __HAL_I2S_ENABLE_TX_CHANNEL(p_i2s, 0);
        /* Enable TX/RX block */
        __HAL_I2S_ENABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_ENABLE_TX_BLOCK(p_i2s);

        status = i2s_transmit_receive(p_i2s, timeout);

        p_i2s->state = HAL_I2S_STATE_READY;

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_transmit_it(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (I2S_DATASIZE_16BIT < p_i2s->init.data_size)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
        }

        /* Configure counters and size of the handle */
        p_i2s->tx_xfer_count = length;
        p_i2s->tx_xfer_size  = length;
        p_i2s->p_tx_buffer   = p_data;

        p_i2s->rx_xfer_count = 0;
        p_i2s->rx_xfer_size  = 0;
        p_i2s->p_rx_buffer   = NULL;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_TX;

        if (I2S_M == p_i2s->p_instance)
            NVIC_EnableIRQ(I2S_M_IRQn);
        else if (I2S_S == p_i2s->p_instance)
            NVIC_EnableIRQ(I2S_S_IRQn);
        __HAL_I2S_CLEAR_FLAG(p_i2s, I2S_IT_TXFO);

        /* Enable channel TX & Disable channel RX */
        __HAL_I2S_ENABLE_TX_CHANNEL(p_i2s, 0);
        __HAL_I2S_DISABLE_RX_CHANNEL(p_i2s, 0);
        /* Enable TX block & Disable RX block */
        __HAL_I2S_ENABLE_TX_BLOCK(p_i2s);
        __HAL_I2S_DISABLE_RX_BLOCK(p_i2s);

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);

        __HAL_I2S_ENABLE_IT(p_i2s, I2S_IT_TXFE | I2S_IT_TXFO);
        if (I2S_M == p_i2s->p_instance)
        {
            /* Enable clock */
            __HAL_I2S_ENABLE_CLOCK(p_i2s);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_receive_it(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (I2S_DATASIZE_16BIT < p_i2s->init.data_size)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
        }

        /* Configure counters and size of the handle */
        p_i2s->rx_xfer_count = length;
        p_i2s->rx_xfer_size  = length;
        p_i2s->p_rx_buffer   = p_data;

        p_i2s->tx_xfer_count = 0;
        p_i2s->tx_xfer_size  = 0;
        p_i2s->p_tx_buffer   = NULL;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_RX;

        if (I2S_M == p_i2s->p_instance)
            NVIC_EnableIRQ(I2S_M_IRQn);
        else if (I2S_S == p_i2s->p_instance)
            NVIC_EnableIRQ(I2S_S_IRQn);
        __HAL_I2S_CLEAR_FLAG(p_i2s, I2S_IT_RXFO);

        /* Enable channel RX & Disable channel TX */
        __HAL_I2S_ENABLE_RX_CHANNEL(p_i2s, 0);
        __HAL_I2S_DISABLE_TX_CHANNEL(p_i2s, 0);
        /* Enable RX block & Disable TX block */
        __HAL_I2S_ENABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_DISABLE_TX_BLOCK(p_i2s);

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);

        __HAL_I2S_ENABLE_IT(p_i2s, I2S_IT_RXDA | I2S_IT_RXFO);
        if (I2S_M == p_i2s->p_instance)
        {
            /* Enable clock */
            __HAL_I2S_ENABLE_CLOCK(p_i2s);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_transmit_receive_it(i2s_handle_t *p_i2s, uint16_t *p_tx_data, uint16_t *p_rx_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_tx_data) || (NULL == p_rx_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (I2S_DATASIZE_16BIT < p_i2s->init.data_size)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
        }

        /* Configure counters and size of the handle */
        p_i2s->tx_xfer_count = length;
        p_i2s->tx_xfer_size  = length;
        p_i2s->p_tx_buffer   = p_tx_data;

        p_i2s->rx_xfer_count = length;
        p_i2s->rx_xfer_size  = length;
        p_i2s->p_rx_buffer   = p_rx_data;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_TX_RX;

        if (I2S_M == p_i2s->p_instance)
            NVIC_EnableIRQ(I2S_M_IRQn);
        else if (I2S_S == p_i2s->p_instance)
            NVIC_EnableIRQ(I2S_S_IRQn);
        __HAL_I2S_CLEAR_FLAG(p_i2s, I2S_IT_RXFO);
        __HAL_I2S_CLEAR_FLAG(p_i2s, I2S_IT_TXFO);

        /* Enable channel TX/RX */
        __HAL_I2S_ENABLE_RX_CHANNEL(p_i2s, 0);
        __HAL_I2S_ENABLE_TX_CHANNEL(p_i2s, 0);
        /* Enable TX/RX block */
        __HAL_I2S_ENABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_ENABLE_TX_BLOCK(p_i2s);

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);

        __HAL_I2S_ENABLE_IT(p_i2s, I2S_IT_TXFE | I2S_IT_TXFO | I2S_IT_RXDA | I2S_IT_RXFO);
        if (I2S_M == p_i2s->p_instance)
        {
            /* Enable clock */
            __HAL_I2S_ENABLE_CLOCK(p_i2s);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_transmit_dma(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (DMA_SDATAALIGN_HALFWORD == p_i2s->p_dmatx->init.src_data_alignment)
        {
            p_i2s->tx_xfer_count = length;
        }
        else if (DMA_SDATAALIGN_WORD == p_i2s->p_dmatx->init.src_data_alignment)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
            else
            {
                p_i2s->tx_xfer_count = (length >> 1);
            }
        }
        else
        {
            p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
            return HAL_ERROR;
        }

        /* Configure counters and size of the handle */
        p_i2s->tx_xfer_size = p_i2s->tx_xfer_count << 1;
        p_i2s->p_tx_buffer  = p_data;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_TX;

        /* Set the I2S DMA transfer complete callback */
        p_i2s->p_dmatx->xfer_tfr_callback = i2s_dma_tx_cplt;

        /* Set the DMA error callback */
        p_i2s->p_dmatx->xfer_error_callback = i2s_dma_error;

        /* Clear the DMA abort callback */
        p_i2s->p_dmatx->xfer_abort_callback = NULL;

        /* Set DMA Burst Length for DMA transfer */
        ll_dma_set_destination_burst_length(DMA, p_i2s->p_dmatx->instance, LL_DMA_DST_BURST_LENGTH_8);

        /* Configure the destibation peripheral */
        if (I2S_M == p_i2s->p_instance)
        {
            NVIC_DisableIRQ(I2S_M_IRQn);
        }
        else if (I2S_S == p_i2s->p_instance)
        {
            NVIC_DisableIRQ(I2S_S_IRQn);
        }

        __HAL_I2S_ENABLE_DMA(p_i2s);

        /* Enable channel TX & Disable channel RX */
        __HAL_I2S_ENABLE_TX_CHANNEL(p_i2s, 0);
        __HAL_I2S_DISABLE_RX_CHANNEL(p_i2s, 0);
        /* Enable TX block & Disable RX block */
        __HAL_I2S_ENABLE_TX_BLOCK(p_i2s);
        __HAL_I2S_DISABLE_RX_BLOCK(p_i2s);

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);

        /* Enable the I2S transmit DMA Channel */
        hal_dma_start_it(p_i2s->p_dmatx, (uint32_t)p_data, (uint32_t)&p_i2s->p_instance->TXDMA, p_i2s->tx_xfer_size);
        __HAL_I2S_ENABLE_IT(p_i2s, I2S_IT_TXFE);
        if (I2S_M == p_i2s->p_instance)
        {
            /* Enable clock */
            __HAL_I2S_ENABLE_CLOCK(p_i2s);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_receive_dma(i2s_handle_t *p_i2s, uint16_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (DMA_SDATAALIGN_HALFWORD == p_i2s->p_dmarx->init.src_data_alignment)
        {
            p_i2s->rx_xfer_count = length;
        }
        else if (DMA_SDATAALIGN_WORD == p_i2s->p_dmarx->init.src_data_alignment)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
            else
            {
                p_i2s->rx_xfer_count = (length >> 1);
            }
        }
        else
        {
            p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
            return HAL_ERROR;
        }

        /* Configure counters and size of the handle */
        p_i2s->rx_xfer_size = p_i2s->rx_xfer_count << 1;
        p_i2s->p_rx_buffer  = p_data;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_RX;

        /* Set the I2S DMA transfer complete callback */
        p_i2s->p_dmarx->xfer_tfr_callback = i2s_dma_rx_cplt;

        /* Set the DMA error callback */
        p_i2s->p_dmarx->xfer_error_callback = i2s_dma_error;

        /* Clear the DMA abort callback */
        p_i2s->p_dmarx->xfer_abort_callback = NULL;

        /* Configure the source peripheral */
        if (I2S_M == p_i2s->p_instance)
            NVIC_DisableIRQ(I2S_M_IRQn);
        else if (I2S_S == p_i2s->p_instance)
            NVIC_DisableIRQ(I2S_S_IRQn);

        __HAL_I2S_ENABLE_DMA(p_i2s);

        /* Enable channel RX & Disable channel TX */
        __HAL_I2S_ENABLE_RX_CHANNEL(p_i2s, 0);
        __HAL_I2S_DISABLE_TX_CHANNEL(p_i2s, 0);
        /* Enable RX block & Disable TX block */
        __HAL_I2S_ENABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_DISABLE_TX_BLOCK(p_i2s);

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);

        /* Enable the I2S transmit DMA Channel */
        hal_dma_start_it(p_i2s->p_dmarx, (uint32_t)&p_i2s->p_instance->RXDMA, (uint32_t)p_data, p_i2s->rx_xfer_size);
        __HAL_I2S_ENABLE_IT(p_i2s, I2S_IT_RXDA);
        if (I2S_M == p_i2s->p_instance)
        {
            /* Enable clock */
            __HAL_I2S_ENABLE_CLOCK(p_i2s);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_transmit_receive_dma(i2s_handle_t *p_i2s, uint16_t *p_tx_data, uint16_t *p_rx_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;

    if ((NULL == p_tx_data) || (NULL == p_rx_data) || (0 == length))
    {
        p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        if (DMA_SDATAALIGN_HALFWORD == p_i2s->p_dmatx->init.src_data_alignment)
        {
            p_i2s->tx_xfer_count = length;
        }
        else if (DMA_SDATAALIGN_WORD == p_i2s->p_dmatx->init.src_data_alignment)
        {
            if (length & 1)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
                return HAL_ERROR;
            }
            else
            {
                p_i2s->tx_xfer_count = (length >> 1);
            }
        }
        else
        {
            p_i2s->error_code |= HAL_I2S_ERROR_INVALID_PARAM;
            return HAL_ERROR;
        }

        /* Configure counters and size of the handle */
        p_i2s->tx_xfer_size = p_i2s->tx_xfer_count << 1;
        p_i2s->p_tx_buffer  = p_tx_data;

        p_i2s->rx_xfer_count = p_i2s->tx_xfer_count;
        p_i2s->rx_xfer_size  = p_i2s->tx_xfer_size;
        p_i2s->p_rx_buffer   = p_rx_data;

        /* Process locked */
        __HAL_LOCK(p_i2s);

        /* Update I2S state */
        p_i2s->state = HAL_I2S_STATE_BUSY_TX_RX;

        /* Set the I2S DMA transfer complete callback */
        p_i2s->p_dmatx->xfer_tfr_callback = i2s_dma_tx_cplt;
        p_i2s->p_dmarx->xfer_tfr_callback = i2s_dma_rx_cplt;

        /* Set the DMA error callback */
        p_i2s->p_dmatx->xfer_error_callback = i2s_dma_error;
        p_i2s->p_dmarx->xfer_error_callback = i2s_dma_error;

        /* Clear the DMA abort callback */
        p_i2s->p_dmatx->xfer_abort_callback = NULL;
        p_i2s->p_dmarx->xfer_abort_callback = NULL;

        /* Set DMA Burst Length for DMA transfer */
        ll_dma_set_destination_burst_length(DMA, p_i2s->p_dmatx->instance, LL_DMA_DST_BURST_LENGTH_8);

        /* Configure the destibation peripheral */
        if (I2S_M == p_i2s->p_instance)
        {
            NVIC_DisableIRQ(I2S_M_IRQn);
        }
        else if (I2S_S == p_i2s->p_instance)
        {
            NVIC_DisableIRQ(I2S_S_IRQn);
        }

        __HAL_I2S_ENABLE_DMA(p_i2s);

        /* Enable channel TX/RX */
        __HAL_I2S_ENABLE_RX_CHANNEL(p_i2s, 0);
        __HAL_I2S_ENABLE_TX_CHANNEL(p_i2s, 0);
        /* Enable TX/RX block */
        __HAL_I2S_ENABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_ENABLE_TX_BLOCK(p_i2s);

        /* Process unlocked */
        __HAL_UNLOCK(p_i2s);

        /* Enable the I2S transfer DMA Channel */
        hal_dma_start_it(p_i2s->p_dmatx, (uint32_t)p_tx_data, (uint32_t)&p_i2s->p_instance->TXDMA, p_i2s->tx_xfer_size);
        hal_dma_start_it(p_i2s->p_dmarx, (uint32_t)&p_i2s->p_instance->RXDMA, (uint32_t)p_rx_data, p_i2s->rx_xfer_size);
        __HAL_I2S_ENABLE_IT(p_i2s, I2S_IT_TXFE | I2S_IT_RXDA);
        if (I2S_M == p_i2s->p_instance)
        {
            /* Enable clock */
            __HAL_I2S_ENABLE_CLOCK(p_i2s);
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_start_clock(i2s_handle_t *p_i2s)
{
    if (I2S_M != p_i2s->p_instance)
        return HAL_ERROR;

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        __HAL_I2S_ENABLE_CLOCK(p_i2s);
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2s_stop_clock(i2s_handle_t *p_i2s)
{
    if (I2S_M != p_i2s->p_instance)
        return HAL_ERROR;

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        __HAL_I2S_DISABLE_CLOCK(p_i2s);
        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK void hal_i2s_error_callback(i2s_handle_t *p_i2s)
{

}

__WEAK void hal_i2s_rx_cplt_callback(i2s_handle_t *p_i2s)
{

}

__WEAK void hal_i2s_tx_cplt_callback(i2s_handle_t *p_i2s)
{

}

__WEAK void hal_i2s_tx_rx_cplt_callback(i2s_handle_t *p_i2s)
{

}

__WEAK hal_i2s_state_t hal_i2s_get_state(i2s_handle_t *p_i2s)
{
    /* Return I2S handle state */
    return p_i2s->state;
}

__WEAK uint32_t hal_i2s_get_error(i2s_handle_t *p_i2s)
{
    return p_i2s->error_code;
}

__WEAK hal_status_t hal_i2s_abort(i2s_handle_t *p_i2s)
{
    hal_status_t status    = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_i2s);

    /* Check if the state is in one of the busy states */
    if (0 != (p_i2s->state & 0x2))
    {
        /* Disable all interrupts */
        __HAL_I2S_DISABLE_IT(p_i2s, I2S_IT_TXFO | I2S_IT_TXFE | I2S_IT_RXFO | I2S_IT_RXDA);

        if (I2S_M == p_i2s->p_instance)
        {
            /* Disable clock */
            __HAL_I2S_DISABLE_CLOCK(p_i2s);
        }
        /* Disable TX/RX block */
        __HAL_I2S_DISABLE_RX_BLOCK(p_i2s);
        __HAL_I2S_DISABLE_TX_BLOCK(p_i2s);
        /* Disable TX/RX channel */
        __HAL_I2S_DISABLE_RX_CHANNEL(p_i2s, 0);
        __HAL_I2S_DISABLE_TX_CHANNEL(p_i2s, 0);

        if (ll_i2s_is_enabled_dma(p_i2s->p_instance))
        {
            /* Disable DMA request */
            __HAL_I2S_DISABLE_DMA(p_i2s);
            /* Abort DMA channel */
            if (HAL_DMA_STATE_READY != p_i2s->p_dmatx->state)
                status = hal_dma_abort(p_i2s->p_dmatx);
            if (HAL_DMA_STATE_READY != p_i2s->p_dmarx->state)
                status = hal_dma_abort(p_i2s->p_dmarx);
            if (HAL_OK != status)
            {
                p_i2s->error_code |= HAL_I2S_ERROR_DMA;
            }
        }

        /* Update state */
        p_i2s->state = HAL_I2S_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_i2s);

    return status;
}

__WEAK hal_status_t hal_i2s_set_tx_fifo_threshold(i2s_handle_t *p_i2s, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_i2s);

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        uint32_t ret = ll_i2s_is_enabled(p_i2s->p_instance);
        __HAL_I2S_DISABLE(p_i2s);
        /* Configure I2S FIFO Threshold */
        ll_i2s_set_tx_fifo_threshold(p_i2s->p_instance, 0, threshold);
        if (0 != ret)
            __HAL_I2S_ENABLE(p_i2s);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_i2s);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_i2s_set_rx_fifo_threshold(i2s_handle_t *p_i2s, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_i2s);

    if (HAL_I2S_STATE_READY == p_i2s->state)
    {
        uint32_t ret = ll_i2s_is_enabled(p_i2s->p_instance);
        __HAL_I2S_DISABLE(p_i2s);
        /* Configure I2S FIFO Threshold */
        ll_i2s_set_rx_fifo_threshold(p_i2s->p_instance, 0, threshold);
        if (0 != ret)
            __HAL_I2S_ENABLE(p_i2s);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_i2s);

    /* Return function status */
    return status;
}

__WEAK uint32_t hal_i2s_get_tx_fifo_threshold(i2s_handle_t *p_i2s)
{
    return ll_i2s_get_tx_fifo_threshold(p_i2s->p_instance, 0);
}

__WEAK uint32_t hal_i2s_get_rx_fifo_threshold(i2s_handle_t *p_i2s)
{
    return ll_i2s_get_rx_fifo_threshold(p_i2s->p_instance, 0);
}

static void i2s_dma_rx_cplt(dma_handle_t *p_dma)
{
    i2s_handle_t *p_i2s = ( i2s_handle_t* )((dma_handle_t* )p_dma)->p_parent;

    p_i2s->rx_xfer_count = 0;

    __HAL_I2S_DISABLE_IT(p_i2s, I2S_IT_RXDA);

    if (HAL_I2S_STATE_BUSY_RX == p_i2s->state)
    {
        /* Disable i2s DMA will shuts down both tx & rx */
        __HAL_I2S_DISABLE_DMA(p_i2s);
        /* Change state of I2S */
        p_i2s->state = HAL_I2S_STATE_READY;
        hal_i2s_rx_cplt_callback(p_i2s);
    }
    else if ((HAL_I2S_STATE_BUSY_TX_RX == p_i2s->state) && !ll_i2s_is_enabled_it(p_i2s->p_instance, 0, LL_I2S_INT_TXFE))
    {
        /* Disable i2s DMA will shuts down both tx & rx */
        __HAL_I2S_DISABLE_DMA(p_i2s);
        /* Change state of I2S */
        p_i2s->state = HAL_I2S_STATE_READY;
        hal_i2s_tx_rx_cplt_callback(p_i2s);
    }
}

static void i2s_dma_tx_cplt(dma_handle_t *p_dma)
{
    i2s_handle_t *p_i2s = ( i2s_handle_t* )p_dma->p_parent;

    p_i2s->tx_xfer_count = 0;

    __HAL_I2S_DISABLE_IT(p_i2s, I2S_IT_TXFE);

    if (HAL_I2S_STATE_BUSY_TX == p_i2s->state)
    {
        /* Disable i2s DMA will shuts down both tx & rx */
        __HAL_I2S_DISABLE_DMA(p_i2s);
        /* Change state of I2S */
        p_i2s->state = HAL_I2S_STATE_READY;
        /* If i2s is in full duplex, you can continue to turn on DMA transmission in the completion callback */
        hal_i2s_tx_cplt_callback(p_i2s);
    }
    else if ((HAL_I2S_STATE_BUSY_TX_RX == p_i2s->state) && !ll_i2s_is_enabled_it(p_i2s->p_instance, 0, LL_I2S_INT_RXDA))
    {
        /* Disable i2s DMA will shuts down both tx & rx */
        __HAL_I2S_DISABLE_DMA(p_i2s);
        /* Change state of I2S */
        p_i2s->state = HAL_I2S_STATE_READY;
        hal_i2s_tx_rx_cplt_callback(p_i2s);
    }
}

static void i2s_dma_error(dma_handle_t *p_dma)
{
    i2s_handle_t *p_i2s = ( i2s_handle_t* )p_dma->p_parent;

    p_i2s->error_code  |= HAL_I2S_ERROR_DMA;

    /* Abort the I2S */
    hal_i2s_abort(p_i2s);
}

static void i2s_dma_abort_cplt(dma_handle_t *p_dma)
{
    i2s_handle_t *p_i2s = ( i2s_handle_t* )p_dma->p_parent;

    p_i2s->rx_xfer_count = 0;
    p_i2s->tx_xfer_count = 0;

    /* Disable DMA request */
    __HAL_I2S_DISABLE_DMA(p_i2s);

    /* DMA Abort called due to a transfer error interrupt */
    /* Change state of I2S */
    p_i2s->state = HAL_I2S_STATE_READY;
    /* Error callback */
    hal_i2s_error_callback(p_i2s);
}

static void i2s_send_16bit(i2s_handle_t *p_i2s)
{
    __IO uint32_t *p_ldata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_L;
    __IO uint32_t *p_rdata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_R;

    while (__HAL_I2S_GET_FLAG(p_i2s, I2S_FLAG_TXFE) && p_i2s->tx_xfer_count)
    {
        *((__IO uint16_t *)p_ldata_reg) = *p_i2s->p_tx_buffer++;
        *((__IO uint16_t *)p_rdata_reg) = *p_i2s->p_tx_buffer++;
        p_i2s->tx_xfer_count --;
    }
}

static void i2s_send_32bit(i2s_handle_t *p_i2s)
{
    __IO uint32_t *p_ldata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_L;
    __IO uint32_t *p_rdata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_R;

    while (__HAL_I2S_GET_FLAG(p_i2s, I2S_FLAG_TXFE) && p_i2s->tx_xfer_count)
    {
        *p_ldata_reg = *(uint32_t *)p_i2s->p_tx_buffer;
        p_i2s->p_tx_buffer += 2;
        *p_rdata_reg = *(uint32_t *)p_i2s->p_tx_buffer;
        p_i2s->p_tx_buffer += 2;
        p_i2s->tx_xfer_count -= 2;
    }
}

static void i2s_receive_16bit(i2s_handle_t *p_i2s)
{
    __IO uint32_t *p_ldata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_L;
    __IO uint32_t *p_rdata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_R;

    if (NULL != p_i2s->p_rx_buffer)
    {
        while (__HAL_I2S_GET_FLAG(p_i2s, I2S_FLAG_RXDA) && p_i2s->rx_xfer_count)
        {
            *p_i2s->p_rx_buffer++ = *((__IO uint16_t *)p_ldata_reg);
            *p_i2s->p_rx_buffer++ = *((__IO uint16_t *)p_rdata_reg);
            p_i2s->rx_xfer_count --;
        }
    }
    else
    {
        while (__HAL_I2S_GET_FLAG(p_i2s, I2S_FLAG_RXDA) && p_i2s->rx_xfer_count)
        {
            ll_i2s_receive_ldata(p_i2s->p_instance, 0);
            ll_i2s_receive_rdata(p_i2s->p_instance, 0);
            p_i2s->rx_xfer_count --;
        }
    }
}

static void i2s_receive_32bit(i2s_handle_t *p_i2s)
{
    __IO uint32_t *p_ldata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_L;
    __IO uint32_t *p_rdata_reg = &p_i2s->p_instance->I2S_CHANNEL[0].DATA_R;

    if (NULL != p_i2s->p_rx_buffer)
    {
        while (__HAL_I2S_GET_FLAG(p_i2s, I2S_FLAG_RXDA) && p_i2s->rx_xfer_count)
        {
            *(uint32_t *)p_i2s->p_rx_buffer = *p_ldata_reg;
            p_i2s->p_rx_buffer += 2;
            *(uint32_t *)p_i2s->p_rx_buffer = *p_rdata_reg;
            p_i2s->p_rx_buffer += 2;
            p_i2s->rx_xfer_count -= 2;
        }
    }
    else
    {
        while (__HAL_I2S_GET_FLAG(p_i2s, I2S_FLAG_RXDA) && p_i2s->rx_xfer_count)
        {
            ll_i2s_receive_ldata(p_i2s->p_instance, 0);
            ll_i2s_receive_rdata(p_i2s->p_instance, 0);
            p_i2s->rx_xfer_count -= 2;
        }
    }
}

static hal_status_t i2s_transmit(i2s_handle_t *p_i2s, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Send the first frame data, and then open clock */
    p_i2s->write_fifo(p_i2s);
    if (I2S_M == p_i2s->p_instance)
    {
        /* Enable clock */
        __HAL_I2S_ENABLE_CLOCK(p_i2s);
    }

    while (0U < p_i2s->tx_xfer_count)
    {
        p_i2s->write_fifo(p_i2s);

        if (HAL_MAX_DELAY != timeout)
        {
            if ((0 == timeout) || (timeout < (hal_get_tick() - tickstart)))
            {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    return status;
}

static hal_status_t i2s_receive(i2s_handle_t *p_i2s, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    if (I2S_M == p_i2s->p_instance)
    {
        /* Enable clock */
        __HAL_I2S_ENABLE_CLOCK(p_i2s);
    }

    while (0U < p_i2s->rx_xfer_count)
    {
        p_i2s->read_fifo(p_i2s);

        if (HAL_MAX_DELAY != timeout)
        {
            if ((0 == timeout) || (timeout < (hal_get_tick() - tickstart)))
            {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    return status;
}

static hal_status_t i2s_transmit_receive(i2s_handle_t *p_i2s, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Send the first frame data, and then open clock */
    p_i2s->write_fifo(p_i2s);
    if (I2S_M == p_i2s->p_instance)
    {
        /* Enable clock */
        __HAL_I2S_ENABLE_CLOCK(p_i2s);
    }
    p_i2s->read_fifo(p_i2s);

    while ((0U < p_i2s->tx_xfer_count) || (0U < p_i2s->rx_xfer_count))
    {
        p_i2s->write_fifo(p_i2s);
        p_i2s->read_fifo(p_i2s);

        if (HAL_MAX_DELAY != timeout)
        {
            if ((0 == timeout) || (timeout < (hal_get_tick() - tickstart)))
            {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    return status;
}

#endif /* HAL_I2S_MODULE_ENABLED */


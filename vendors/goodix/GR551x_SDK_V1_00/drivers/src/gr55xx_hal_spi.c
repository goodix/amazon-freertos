/**
  ****************************************************************************************
  * @file    gr55xx_hal_spi.c
  * @author  BLE Driver Team
  * @brief   SPI HAL module driver.
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

#ifdef  USE_GLOBLE_DISABLE
#define HAL_GLOBAL_EXCEPTION_DISABLE()  GLOBAL_EXCEPTION_DISABLE()
#define HAL_GLOBAL_EXCEPTION_ENABLE()   GLOBAL_EXCEPTION_ENABLE()
#else
#define HAL_GLOBAL_EXCEPTION_DISABLE()  ((void)0U)
#define HAL_GLOBAL_EXCEPTION_ENABLE()   ((void)0U)
#endif

#ifdef HAL_SPI_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void spi_dma_rx_cplt(dma_handle_t *p_dma);
static void spi_dma_tx_cplt(dma_handle_t *p_dma);
static void spi_dma_error(dma_handle_t *p_dma);
static void spi_dma_abort_cplt(dma_handle_t *p_dma);
static hal_status_t spi_wait_flag_state_until_timeout(spi_handle_t *p_spi, uint32_t flag, flag_status_t state, uint32_t tick_start, uint32_t timeout);
static void spi_config(spi_handle_t *p_spi, uint32_t length, uint32_t functions_mode);
static void spi_send_8bit(spi_handle_t *p_spi);
static void spi_send_16bit(spi_handle_t *p_spi);
static void spi_send_32bit(spi_handle_t *p_spi);
static void spi_receive_8bit(spi_handle_t *p_spi);
static void spi_receive_16bit(spi_handle_t *p_spi);
static void spi_receive_32bit(spi_handle_t *p_spi);
static hal_status_t spi_transmit(spi_handle_t *p_spi, uint32_t timeout);
static hal_status_t spi_receive(spi_handle_t *p_spi, uint32_t timeout);
static hal_status_t spi_transmit_receive(spi_handle_t *p_spi, uint32_t timeout);
static hal_status_t spi_read_eeprom(spi_handle_t *p_spi, uint32_t timeout);

__WEAK hal_status_t hal_spi_init(spi_handle_t *p_spi)
{
    hal_status_t   status    = HAL_ERROR;
    error_status_t err       = SUCCESS;
    uint32_t       tickstart = hal_get_tick();
    ll_spim_init_t spim_init = LL_SPIM_DEFAULT_CONFIG;
    ll_spis_init_t spis_init = LL_SPIS_DEFAULT_CONFIG;

    /* Check the SPI handle allocation */
    if (NULL == p_spi)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(p_spi->p_instance));
    if (SPIM == p_spi->p_instance)
    {
        gr_assert_param(IS_SPI_DATASIZE(p_spi->init.data_size));
        gr_assert_param(IS_SPI_CPOL(p_spi->init.clock_polarity));
        gr_assert_param(IS_SPI_CPHA(p_spi->init.clock_phase));
        gr_assert_param(IS_SPI_BAUDRATE_PRESCALER(p_spi->init.baudrate_prescaler));
        gr_assert_param(IS_SPI_TIMODE(p_spi->init.ti_mode));
        gr_assert_param(IS_SPI_SLAVE(p_spi->init.slave_select));
    }
    else
    {
        gr_assert_param(IS_SPI_DATASIZE(p_spi->init.data_size));
        gr_assert_param(IS_SPI_CPOL(p_spi->init.clock_polarity));
        gr_assert_param(IS_SPI_CPHA(p_spi->init.clock_phase));
    }

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_RESET == p_spi->state)
    {
        /* Allocate lock resource and initialize it */
        p_spi->lock = HAL_UNLOCKED;

        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable SPIM/SPIS Clock */
        if(p_spi->p_instance == SPIM)
        {
            ll_cgc_disable_force_off_spim_hclk();
        }
        else if(p_spi->p_instance == SPIS)
        {
            ll_cgc_disable_force_off_spis_hclk();
        }

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_spi_msp_init(p_spi);

        /* Configure the default timeout for the SPI memory access */
        hal_spi_set_timeout(p_spi, HAL_SPI_TIMEOUT_DEFAULT_VALUE);
    }

    /* Configure SPI FIFO Threshold */
    ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 0);
    ll_spi_set_rx_fifo_threshold(p_spi->p_instance, 0);
    ll_spi_set_dma_tx_fifo_threshold(p_spi->p_instance, 4);
    ll_spi_set_dma_rx_fifo_threshold(p_spi->p_instance, 0);

    /* Wait till BUSY flag reset */
    status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

    if (HAL_OK == status)
    {
        /* Disable the SPI peripheral */
        __HAL_SPI_DISABLE(p_spi);
        /* Configure SPI Clock Prescaler and Clock Mode */
        if (SPIM == p_spi->p_instance)
        {
            spim_init.data_size      = p_spi->init.data_size;
            spim_init.clock_polarity = p_spi->init.clock_polarity;
            spim_init.clock_phase    = p_spi->init.clock_phase;
            spim_init.slave_select   = p_spi->init.slave_select;
            spim_init.baud_rate      = p_spi->init.baudrate_prescaler;
            err = ll_spim_init(p_spi->p_instance, &spim_init);
        }
        else
        {
            spis_init.data_size = p_spi->init.data_size;
            spis_init.clock_polarity = p_spi->init.clock_polarity;
            spis_init.clock_phase = p_spi->init.clock_phase;
            err = ll_spis_init(p_spi->p_instance, &spis_init);
        }
        ll_spi_set_standard(p_spi->p_instance, p_spi->init.ti_mode);

        if (SUCCESS == err)
        {
            /* Enable the SPI peripheral */
            __HAL_SPI_ENABLE(p_spi);

            /* Set write/read fifo interface */
            if (p_spi->init.data_size <= SPI_DATASIZE_8BIT)
            {
                p_spi->write_fifo = spi_send_8bit;
                p_spi->read_fifo = spi_receive_8bit;
            }
            else if (p_spi->init.data_size <= SPI_DATASIZE_16BIT)
            {
                p_spi->write_fifo = spi_send_16bit;
                p_spi->read_fifo = spi_receive_16bit;
            }
            else
            {
                p_spi->write_fifo = spi_send_32bit;
                p_spi->read_fifo = spi_receive_32bit;
            }

            /* Set SPI error code to none */
            p_spi->error_code = HAL_SPI_ERROR_NONE;

            /* Initialize the SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
        }
        else
        {
            status = HAL_ERROR;
        }
    }

    /* Release Lock */
    __HAL_UNLOCK(p_spi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_deinit(spi_handle_t *p_spi)
{
    /* Check the SPI handle allocation */
    if (NULL == p_spi)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (p_spi->state != HAL_SPI_STATE_RESET)
    {
        /* Disable the SPI Peripheral Clock */
        if (SPIM == p_spi->p_instance)
            ll_spim_deinit(p_spi->p_instance);
        else
            ll_spis_deinit(p_spi->p_instance);

        /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
        hal_spi_msp_deinit(p_spi);

        /* Disable SPIM/SPIS Clock */
        if(p_spi->p_instance == SPIM)
        {
            ll_cgc_enable_force_off_spim_hclk();
        }
        else if(p_spi->p_instance == SPIS)
        {
            ll_cgc_enable_force_off_spis_hclk();
        }

        if((LL_CGC_FRC_I2S_S_HCLK & ll_cgc_get_force_off_hclk_0()) && 
          ((LL_CGC_FRC_SERIALS_HCLK2 & ll_cgc_get_force_off_hclk_2()) == LL_CGC_FRC_SERIALS_HCLK2))
        {
            /* Disable Clock for Serial blocks  */
            ll_cgc_enable_force_off_serial_hclk();
        }

        /* Set SPI error code to none */
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Initialize the SPI state */
        p_spi->state = HAL_SPI_STATE_RESET;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_spi);

    return HAL_OK;
}

__WEAK void hal_spi_msp_init(spi_handle_t *p_spi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_spi_msp_init can be implemented in the user file
     */
}

__WEAK void hal_spi_msp_deinit(spi_handle_t *p_spi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_spi_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_spi_irq_handler(spi_handle_t *p_spi)
{
    uint32_t itsource = READ_REG(p_spi->p_instance->INTSTAT);

    if (itsource & (SPI_IT_MST | SPI_IT_RXO | SPI_IT_RXU | SPI_IT_TXO))
    {
        ll_spi_clear_flag_all(p_spi->p_instance);

        /* Disable all the SPI Interrupts */
        __HAL_SPI_DISABLE_IT(p_spi, (SPI_IT_MST | SPI_IT_RXF | SPI_IT_RXO | SPI_IT_RXU | SPI_IT_TXO | SPI_IT_TXE));

        /* Set error code */
        p_spi->error_code |= HAL_SPI_ERROR_TRANSFER;

        if (0 != p_spi->p_instance->DMAC)
        {
            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_spi->p_instance->DMAC);

            /* Abort DMA channel */
            p_spi->p_dmatx->xfer_abort_callback = spi_dma_abort_cplt;
            p_spi->p_dmarx->xfer_abort_callback = spi_dma_abort_cplt;
            hal_dma_abort_it(p_spi->p_dmatx);
            hal_dma_abort_it(p_spi->p_dmarx);
        }
        else
        {
            /* Change state of SPI */
            p_spi->state = HAL_SPI_STATE_READY;

            /* Error callback */
            hal_spi_error_callback(p_spi);
        }
    }

    if (itsource & SPI_IT_RXF)
    {
        p_spi->read_fifo(p_spi);

        if (0 == p_spi->rx_xfer_count)
        {
            /* All data have been received for the transfer */
            /* Disable the SPI RX Interrupt */
            __HAL_SPI_DISABLE_IT(p_spi, SPI_IT_RXF);

            if (HAL_SPI_STATE_BUSY_RX == p_spi->state)
            {
                /* Change state of SPI */
                p_spi->state = HAL_SPI_STATE_READY;
                hal_spi_rx_cplt_callback(p_spi);
            }
            else if (HAL_SPI_STATE_BUSY_TX_RX == p_spi->state)
            {
                /* Change state of SPI */
                p_spi->state = HAL_SPI_STATE_READY;
                hal_spi_tx_rx_cplt_callback(p_spi);
            }
            else if (HAL_SPI_STATE_ABORT == p_spi->state)
            {
                /* Change state of SPI */
                p_spi->state = HAL_SPI_STATE_READY;
                __HAL_SPI_DISABLE(p_spi);
                __HAL_SPI_ENABLE(p_spi);
                hal_spi_abort_cplt_callback(p_spi);
            }
        }
    }

    if (itsource & SPI_IT_TXE)
    {
        if (0 == p_spi->tx_xfer_count)
        {
            /* All data have been sended for the transfer */
            /* Disable the SPI TX Interrupt */
            __HAL_SPI_DISABLE_IT(p_spi, SPI_IT_TXE);

            if (HAL_SPI_STATE_BUSY_TX == p_spi->state)
            {
                /* Change state of SPI */
                p_spi->state = HAL_SPI_STATE_READY;
                hal_spi_tx_cplt_callback(p_spi);
            }
            else if (HAL_SPI_STATE_ABORT == p_spi->state)
            {
                /* Change state of SPI */
                p_spi->state = HAL_SPI_STATE_READY;
                __HAL_SPI_DISABLE(p_spi);
                __HAL_SPI_ENABLE(p_spi);
                hal_spi_abort_cplt_callback(p_spi);
            }
        }
        else
        {
            p_spi->write_fifo(p_spi);

            if (0 == p_spi->tx_xfer_count)
                ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 0);
        }
    }
}

__WEAK hal_status_t hal_spi_transmit(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, length, SPI_DIRECTION_SIMPLEX_TX);

            /* Configure counters and size of the handle */
            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_data;

            p_spi->rx_xfer_count = 0;
            p_spi->rx_xfer_size  = 0;
            p_spi->p_rx_buffer   = NULL;

            p_spi->state = HAL_SPI_STATE_BUSY_TX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            status = spi_transmit(p_spi, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_spi->state = HAL_SPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_receive(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, length, SPI_DIRECTION_SIMPLEX_RX);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_data;

            p_spi->tx_xfer_count = 0;
            p_spi->tx_xfer_size  = 0;
            p_spi->p_tx_buffer   = NULL;

            p_spi->state = HAL_SPI_STATE_BUSY_RX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            /* Send a dummy byte to start transfer */
            if (SPIM == p_spi->p_instance)
                p_spi->p_instance->DATA = 0xFFFFFFFF;
            status = spi_receive(p_spi, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_spi->state = HAL_SPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_receive(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, length, SPI_DIRECTION_FULL_DUPLEX);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_tx_data;

            p_spi->state = HAL_SPI_STATE_BUSY_TX_RX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            status = spi_transmit_receive(p_spi, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_spi->state = HAL_SPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_read_eeprom(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* SPIS can't in this mode */
    if (SPIS == p_spi->p_instance)
    {
        p_spi->error_code = HAL_SPI_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, rx_number_data, SPI_DIRECTION_READ_EEPROM);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = rx_number_data;
            p_spi->rx_xfer_size  = rx_number_data;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = tx_number_data;
            p_spi->tx_xfer_size  = tx_number_data;
            p_spi->p_tx_buffer   = p_tx_data;

            p_spi->state = HAL_SPI_STATE_BUSY_TX_RX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            status = spi_read_eeprom(p_spi, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_spi->state = HAL_SPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_it(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, length, SPI_DIRECTION_SIMPLEX_TX);

            /* Configure counters and size of the handle */
            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_data;

            p_spi->rx_xfer_count = 0;
            p_spi->rx_xfer_size  = 0;
            p_spi->p_rx_buffer   = NULL;

            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_BUSY_TX;

            ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 4);
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_TXE);

            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_receive_it(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, length, SPI_DIRECTION_SIMPLEX_RX);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_data;

            p_spi->tx_xfer_count = 0;
            p_spi->tx_xfer_size  = 0;
            p_spi->p_tx_buffer   = NULL;

            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_BUSY_RX;

            /* Send a dummy byte to start transfer */
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF | SPI_IT_RXO);
            if (SPIM == p_spi->p_instance)
                p_spi->p_instance->DATA = 0xFFFFFFFF;

            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_receive_it(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, length, SPI_DIRECTION_FULL_DUPLEX);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = length;
            p_spi->rx_xfer_size  = length;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = length;
            p_spi->tx_xfer_size  = length;
            p_spi->p_tx_buffer   = p_tx_data;

            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_BUSY_TX_RX;

            ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 4);
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF | SPI_IT_RXO | SPI_IT_TXE);

            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_read_eeprom_it(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* SPIS can't in this mode */
    if (SPIS == p_spi->p_instance)
    {
        p_spi->error_code = HAL_SPI_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            spi_config(p_spi, rx_number_data, SPI_DIRECTION_READ_EEPROM);

            /* Configure counters and size of the handle */
            p_spi->rx_xfer_count = rx_number_data;
            p_spi->rx_xfer_size  = rx_number_data;
            p_spi->p_rx_buffer   = p_rx_data;

            p_spi->tx_xfer_count = tx_number_data;
            p_spi->tx_xfer_size  = tx_number_data;
            p_spi->p_tx_buffer   = p_tx_data;

            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_BUSY_TX_RX;

            ll_spi_set_tx_fifo_threshold(p_spi->p_instance, 4);
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF | SPI_IT_RXO | SPI_IT_TXE);

            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_dma(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = length;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0 != (length % 2))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0 != (length % 4))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                spi_config(p_spi, length, SPI_DIRECTION_SIMPLEX_TX);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer = p_data;

                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_BUSY_TX;

                /* Set the SPI DMA transfer complete callback */
                p_spi->p_dmatx->xfer_tfr_callback = spi_dma_tx_cplt;

                /* Set the DMA error callback */
                p_spi->p_dmatx->xfer_error_callback = spi_dma_error;

                /* Clear the DMA abort callback */
                p_spi->p_dmatx->xfer_abort_callback = NULL;

                /* Configure the destibation peripheral */
                if (SPIM == p_spi->p_instance)
                    ll_dma_set_destination_peripheral(DMA, p_spi->p_dmatx->instance, LL_DMA_PERIPH_SPIM_TX);
                else
                    ll_dma_set_destination_peripheral(DMA, p_spi->p_dmatx->instance, LL_DMA_PERIPH_SPIS_TX);

                /* Enable the SPI transmit DMA Channel */
                hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);

                /* Process unlocked */
                __HAL_UNLOCK(p_spi);

                __HAL_SPI_ENABLE_DMATX(p_spi);
            }
            else
            {
                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_spi);
            }
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_receive_dma(spi_handle_t *p_spi, uint8_t *p_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmarx->init.src_data_alignment)
            {
                p_spi->rx_xfer_count = length;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0 != (length % 2))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0 != (length % 4))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                spi_config(p_spi, length, SPI_DIRECTION_SIMPLEX_RX);

                /* Configure counters and size of the handle */
                p_spi->rx_xfer_size = p_spi->rx_xfer_count;
                p_spi->p_rx_buffer = p_data;

                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_BUSY_RX;

                /* Set the SPI DMA transfer complete callback */
                p_spi->p_dmarx->xfer_tfr_callback = spi_dma_rx_cplt;

                /* Set the DMA error callback */
                p_spi->p_dmarx->xfer_error_callback = spi_dma_error;

                /* Clear the DMA abort callback */
                p_spi->p_dmarx->xfer_abort_callback = NULL;

                /* Configure the source peripheral */
                if (SPIM == p_spi->p_instance)
                    ll_dma_set_source_peripheral(DMA, p_spi->p_dmarx->instance, LL_DMA_PERIPH_SPIM_RX);
                else
                    ll_dma_set_source_peripheral(DMA, p_spi->p_dmarx->instance, LL_DMA_PERIPH_SPIS_RX);

                /* Enable the SPI transmit DMA Channel */
                hal_dma_start_it(p_spi->p_dmarx, (uint32_t)&p_spi->p_instance->DATA, (uint32_t)p_data, p_spi->rx_xfer_size);

                /* Process unlocked */
                __HAL_UNLOCK(p_spi);

                /* Send a dummy byte to start transfer */
                __HAL_SPI_ENABLE_DMARX(p_spi);
                if (SPIM == p_spi->p_instance)
                    p_spi->p_instance->DATA = 0xFFFFFFFF;
            }
            else
            {
                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_spi);
            }
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_transmit_receive_dma(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = length;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0 != (length % 2))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0 != (length % 4))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (length >> 2);
                }
            }

            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmarx->init.src_data_alignment)
            {
                p_spi->rx_xfer_count = length;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0 != (length % 2))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0 != (length % 4))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (length >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                spi_config(p_spi, length, SPI_DIRECTION_FULL_DUPLEX);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer  = p_tx_data;
                p_spi->rx_xfer_size = p_spi->rx_xfer_count;
                p_spi->p_rx_buffer  = p_rx_data;

                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_BUSY_TX_RX;

                /* Set the SPI DMA transfer complete callback */
                p_spi->p_dmatx->xfer_tfr_callback = spi_dma_tx_cplt;

                /* Set the DMA error callback */
                p_spi->p_dmatx->xfer_error_callback = spi_dma_error;

                /* Clear the DMA abort callback */
                p_spi->p_dmatx->xfer_abort_callback = NULL;

                /* Set the SPI DMA transfer complete callback */
                p_spi->p_dmarx->xfer_tfr_callback = spi_dma_rx_cplt;

                /* Set the DMA error callback */
                p_spi->p_dmarx->xfer_error_callback = spi_dma_error;

                /* Clear the DMA abort callback */
                p_spi->p_dmarx->xfer_abort_callback = NULL;

                /* Configure the source peripheral */
                if (SPIM == p_spi->p_instance)
                {
                    ll_dma_set_destination_peripheral(DMA, p_spi->p_dmatx->instance, LL_DMA_PERIPH_SPIM_TX);
                    ll_dma_set_source_peripheral(DMA, p_spi->p_dmarx->instance, LL_DMA_PERIPH_SPIM_RX);
                }
                else
                {
                    ll_dma_set_destination_peripheral(DMA, p_spi->p_dmatx->instance, LL_DMA_PERIPH_SPIS_TX);
                    ll_dma_set_source_peripheral(DMA, p_spi->p_dmarx->instance, LL_DMA_PERIPH_SPIS_RX);
                }

                /* Enable the SPI transmit DMA Channel */
                hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_tx_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);
                hal_dma_start_it(p_spi->p_dmarx, (uint32_t)&p_spi->p_instance->DATA, (uint32_t)p_rx_data, p_spi->rx_xfer_size);

                /* Process unlocked */
                __HAL_UNLOCK(p_spi);

                __HAL_SPI_ENABLE_DMARX(p_spi);
                __HAL_SPI_ENABLE_DMATX(p_spi);
            }
            else
            {
                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_spi);
            }
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_read_eeprom_dma(spi_handle_t *p_spi, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t tx_number_data, uint32_t rx_number_data)
{
    hal_status_t status = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* SPIS can't in this mode */
    if (SPIS == p_spi->p_instance)
    {
        p_spi->error_code = HAL_SPI_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        p_spi->error_code = HAL_SPI_ERROR_NONE;

        /* Update SPI state */
        p_spi->state = HAL_SPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, p_spi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmatx->init.src_data_alignment)
            {
                p_spi->tx_xfer_count = tx_number_data;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0 != (tx_number_data % 2))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (tx_number_data >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmatx->init.src_data_alignment)
            {
                if (0 != (tx_number_data % 4))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->tx_xfer_count = (tx_number_data >> 2);
                }
            }

            if (DMA_SDATAALIGN_BYTE == p_spi->p_dmarx->init.src_data_alignment)
            {
                p_spi->rx_xfer_count = rx_number_data;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0 != (rx_number_data % 2))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (rx_number_data >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_spi->p_dmarx->init.src_data_alignment)
            {
                if (0 != (rx_number_data % 4))
                {
                    p_spi->error_code |= HAL_SPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_spi->rx_xfer_count = (rx_number_data >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                spi_config(p_spi, rx_number_data, SPI_DIRECTION_READ_EEPROM);

                /* Configure counters and size of the handle */
                p_spi->tx_xfer_size = p_spi->tx_xfer_count;
                p_spi->p_tx_buffer  = p_tx_data;
                p_spi->rx_xfer_size = p_spi->rx_xfer_count;
                p_spi->p_rx_buffer  = p_rx_data;

                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_BUSY_TX_RX;

                /* Set the SPI DMA transfer complete callback */
                p_spi->p_dmatx->xfer_tfr_callback = spi_dma_tx_cplt;

                /* Set the DMA error callback */
                p_spi->p_dmatx->xfer_error_callback = spi_dma_error;

                /* Clear the DMA abort callback */
                p_spi->p_dmatx->xfer_abort_callback = NULL;

                /* Set the SPI DMA transfer complete callback */
                p_spi->p_dmarx->xfer_tfr_callback = spi_dma_rx_cplt;

                /* Set the DMA error callback */
                p_spi->p_dmarx->xfer_error_callback = spi_dma_error;

                /* Clear the DMA abort callback */
                p_spi->p_dmarx->xfer_abort_callback = NULL;

                /* Configure the source peripheral */
                if (SPIM == p_spi->p_instance)
                {
                    ll_dma_set_destination_peripheral(DMA, p_spi->p_dmatx->instance, LL_DMA_PERIPH_SPIM_TX);
                    ll_dma_set_source_peripheral(DMA, p_spi->p_dmarx->instance, LL_DMA_PERIPH_SPIM_RX);
                }
                else
                {
                    ll_dma_set_destination_peripheral(DMA, p_spi->p_dmatx->instance, LL_DMA_PERIPH_SPIS_TX);
                    ll_dma_set_source_peripheral(DMA, p_spi->p_dmarx->instance, LL_DMA_PERIPH_SPIS_RX);
                }

                /* Enable the SPI transmit DMA Channel */
                hal_dma_start_it(p_spi->p_dmatx, (uint32_t)p_tx_data, (uint32_t)&p_spi->p_instance->DATA, p_spi->tx_xfer_size);
                hal_dma_start_it(p_spi->p_dmarx, (uint32_t)&p_spi->p_instance->DATA, (uint32_t)p_rx_data, p_spi->rx_xfer_size);

                /* Process unlocked */
                __HAL_UNLOCK(p_spi);

                __HAL_SPI_ENABLE_DMARX(p_spi);
                __HAL_SPI_ENABLE_DMATX(p_spi);
            }
            else
            {
                /* Update SPI state */
                p_spi->state = HAL_SPI_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_spi);
            }
        }
        else
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_spi);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_spi);
    }

    /* Return function status */
    return status;
}

__WEAK void hal_spi_error_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_abort_cplt_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_rx_cplt_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_tx_cplt_callback(spi_handle_t *p_spi)
{

}

__WEAK void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi)
{

}

__WEAK hal_spi_state_t hal_spi_get_state(spi_handle_t *p_spi)
{
    /* Return SPI handle state */
    return p_spi->state;
}

__WEAK uint32_t hal_spi_get_error(spi_handle_t *p_spi)
{
    return p_spi->error_code;
}

__WEAK hal_status_t hal_spi_abort(spi_handle_t *p_spi)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();
    uint32_t     tmp_dmac;

    /* Process locked */
    __HAL_LOCK(p_spi);

    /* Check if the state is in one of the busy states */
    if (0 != (p_spi->state & 0x2))
    {
        if (0 != p_spi->p_instance->DMAC)
        {
            tmp_dmac = p_spi->p_instance->DMAC;

            /* Abort DMA channel */
            if (tmp_dmac & SSI_DMAC_TDMAE)
                status = hal_dma_abort(p_spi->p_dmatx);
            if (tmp_dmac & SSI_DMAC_RDMAE)
                status = hal_dma_abort(p_spi->p_dmarx);
            if (HAL_OK != status)
            {
                p_spi->error_code |= HAL_SPI_ERROR_DMA;
            }

            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_spi->p_instance->DMAC);

            /* Flush FIFO */
            __HAL_SPI_DISABLE(p_spi);
            __HAL_SPI_ENABLE(p_spi);
        }
        else
        {
            /* Disable all interrupts */
            CLEAR_REG(p_spi->p_instance->INTMASK);

            /* Flush FIFO */
            __HAL_SPI_DISABLE(p_spi);
            __HAL_SPI_ENABLE(p_spi);
        }

        /* Update state */
        p_spi->state = HAL_SPI_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    return status;
}

__WEAK hal_status_t hal_spi_abort_it(spi_handle_t *p_spi)
{
    hal_status_t status = HAL_OK;
    uint32_t     tmp_dmac;

    /* Process locked */
    __HAL_LOCK(p_spi);

    /* Check if the state is in one of the busy states */
    if (0 != (p_spi->state & 0x2))
    {
        /* Disable all interrupts */
        CLEAR_REG(p_spi->p_instance->INTMASK);

        if (0 != p_spi->p_instance->DMAC)
        {
            tmp_dmac = p_spi->p_instance->DMAC;
            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_spi->p_instance->DMAC);

            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_ABORT;
            /* Abort DMA channel */
            if (tmp_dmac & SSI_DMAC_TDMAE)
            {
                p_spi->p_dmatx->xfer_abort_callback = spi_dma_abort_cplt;
                hal_dma_abort_it(p_spi->p_dmatx);
            }
            if (tmp_dmac & SSI_DMAC_RDMAE)
            {
                p_spi->p_dmarx->xfer_abort_callback = spi_dma_abort_cplt;
                hal_dma_abort_it(p_spi->p_dmarx);
            }
        }
        else if (0 != (p_spi->state & 0x10))
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_ABORT;
            p_spi->tx_xfer_count = 0;
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_TXE);
        }
        else if (HAL_SPI_STATE_BUSY_RX == p_spi->state)
        {
            /* Update SPI state */
            p_spi->state = HAL_SPI_STATE_ABORT;
            p_spi->rx_xfer_count = 0;
            __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_RXF);
        }
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    return status;
}

__WEAK void hal_spi_set_timeout(spi_handle_t *p_spi, uint32_t timeout)
{
    p_spi->timeout = timeout;
}

__WEAK hal_status_t hal_spi_set_tx_fifo_threshold(spi_handle_t *p_spi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        /* Configure SPI FIFO Threshold */
        ll_spi_set_tx_fifo_threshold(p_spi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_spi_set_rx_fifo_threshold(spi_handle_t *p_spi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_spi);

    if (HAL_SPI_STATE_READY == p_spi->state)
    {
        /* Configure SPI FIFO Threshold */
        ll_spi_set_rx_fifo_threshold(p_spi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_spi);

    /* Return function status */
    return status;
}

__WEAK uint32_t hal_spi_get_tx_fifo_threshold(spi_handle_t *p_spi)
{
    return ll_spi_get_tx_fifo_threshold(p_spi->p_instance);
}

__WEAK uint32_t hal_spi_get_rx_fifo_threshold(spi_handle_t *p_spi)
{
    return ll_spi_get_rx_fifo_threshold(p_spi->p_instance);
}

static void spi_dma_rx_cplt(dma_handle_t *p_dma)
{
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;
    p_spi->rx_xfer_count = 0;
    __HAL_SPI_DISABLE_DMARX(p_spi);

    if (HAL_SPI_STATE_BUSY_RX == p_spi->state)
    {
        /* Change state of SPI */
        p_spi->state = HAL_SPI_STATE_READY;
        hal_spi_rx_cplt_callback(p_spi);
    }
    if (HAL_SPI_STATE_BUSY_TX_RX == p_spi->state)
    {
        /* Change state of SPI */
        p_spi->state = HAL_SPI_STATE_READY;
        hal_spi_tx_rx_cplt_callback(p_spi);
    }
}

static void spi_dma_tx_cplt(dma_handle_t *p_dma)
{
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;
    p_spi->tx_xfer_count = 0;
    __HAL_SPI_DISABLE_DMATX(p_spi);

    __HAL_SPI_ENABLE_IT(p_spi, SPI_IT_TXE);
}

static void spi_dma_error(dma_handle_t *p_dma)
{
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;

    p_spi->error_code  |= HAL_SPI_ERROR_DMA;

    /* Abort the SPI */
    hal_spi_abort_it(p_spi);
}

static void spi_dma_abort_cplt(dma_handle_t *p_dma)
{
    spi_handle_t* p_spi = ( spi_handle_t* )p_dma->p_parent;

    p_spi->rx_xfer_count = 0;
    p_spi->tx_xfer_count = 0;

    __HAL_SPI_DISABLE(p_spi);
    __HAL_SPI_ENABLE(p_spi);

    if (HAL_SPI_STATE_ABORT == p_spi->state)
    {
        /* Change state of SPI */
        p_spi->state = HAL_SPI_STATE_READY;
        hal_spi_abort_cplt_callback(p_spi);
    }
    else
    {
        /* DMA Abort called due to a transfer error interrupt */
        /* Change state of SPI */
        p_spi->state = HAL_SPI_STATE_READY;
        /* Error callback */
        hal_spi_error_callback(p_spi);
    }
}

static hal_status_t spi_wait_flag_state_until_timeout(spi_handle_t *p_spi,
                                                      uint32_t      flag,
                                                      flag_status_t state,
                                                      uint32_t      tick_start,
                                                      uint32_t      timeout)
{
    /* Wait until flag is in expected state */
    while ((__HAL_SPI_GET_FLAG(p_spi, flag)) != state)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if ((0 == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                p_spi->state     = HAL_SPI_STATE_ERROR;
                p_spi->error_code |= HAL_SPI_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static void spi_config(spi_handle_t *p_spi, uint32_t length, uint32_t direction)
{
    gr_assert_param(IS_SPI_DIRECTION(direction));

    __HAL_SPI_DISABLE(p_spi);

    ll_spi_set_transfer_direction(p_spi->p_instance, direction);
    if ((SPI_DIRECTION_SIMPLEX_RX == direction) || (SPI_DIRECTION_READ_EEPROM == direction))
    {
        if (SPI_DATASIZE_8BIT >= p_spi->init.data_size)
            ll_spi_set_receive_size(p_spi->p_instance, length - 1);
        else if (SPI_DATASIZE_16BIT >= p_spi->init.data_size)
            ll_spi_set_receive_size(p_spi->p_instance, (length >> 1) - 1);
        else
            ll_spi_set_receive_size(p_spi->p_instance, (length >> 2) - 1);
    }

    __HAL_SPI_ENABLE(p_spi);
}

static void spi_send_8bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF) && p_spi->tx_xfer_count)
    {
        p_spi->p_instance->DATA = *p_spi->p_tx_buffer++;
        p_spi->tx_xfer_count--;
    }
}

static void spi_send_16bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF) && p_spi->tx_xfer_count)
    {
        p_spi->p_instance->DATA = *(uint16_t *)p_spi->p_tx_buffer;
        p_spi->p_tx_buffer += 2;
        p_spi->tx_xfer_count -= 2;
    }
}

static void spi_send_32bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_TFNF) && p_spi->tx_xfer_count)
    {
        p_spi->p_instance->DATA = *(uint32_t *)p_spi->p_tx_buffer;
        p_spi->p_tx_buffer += 4;
        p_spi->tx_xfer_count -= 4;
    }
}

static void spi_receive_8bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE) && p_spi->rx_xfer_count)
    {
        *p_spi->p_rx_buffer++ = p_spi->p_instance->DATA;
        p_spi->rx_xfer_count--;
    }
}

static void spi_receive_16bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE) && p_spi->rx_xfer_count)
    {
        *(uint16_t *)p_spi->p_rx_buffer = p_spi->p_instance->DATA;
        p_spi->p_rx_buffer += 2;
        p_spi->rx_xfer_count -= 2;
    }
}

static void spi_receive_32bit(spi_handle_t *p_spi)
{
    while (__HAL_SPI_GET_FLAG(p_spi, SPI_FLAG_RFNE) && p_spi->rx_xfer_count)
    {
        *(uint32_t *)p_spi->p_rx_buffer = p_spi->p_instance->DATA;
        p_spi->p_rx_buffer += 4;
        p_spi->rx_xfer_count -= 4;
    }
}

static hal_status_t spi_transmit(spi_handle_t *p_spi, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    while (0U < p_spi->tx_xfer_count)
    {
        p_spi->write_fifo(p_spi);

        if ((0 == timeout) || ((HAL_MAX_DELAY != timeout) && (timeout <= (hal_get_tick() - tickstart))))
        {
            status = HAL_TIMEOUT;
            break;
        }
    }

    if (HAL_OK == status)
    {
        /* Wait until TFE flag is set then to check BUSY flag */
        spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_TFE, SET, tickstart, timeout);
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);
    }

    return status;
}

static hal_status_t spi_receive(spi_handle_t *p_spi, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    while (0U < p_spi->rx_xfer_count)
    {
        p_spi->read_fifo(p_spi);

        if ((0 == timeout) || ((HAL_MAX_DELAY != timeout) && (timeout <= (hal_get_tick() - tickstart))))
        {
            status = HAL_TIMEOUT;
            break;
        }
    }

    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);
    }

    return status;
}

static hal_status_t spi_transmit_receive(spi_handle_t *p_spi, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    while ((0U < p_spi->rx_xfer_count) || (0U < p_spi->tx_xfer_count))
    {
        p_spi->write_fifo(p_spi);

        p_spi->read_fifo(p_spi);

        if ((0 == timeout) || ((HAL_MAX_DELAY != timeout) && (timeout <= (hal_get_tick() - tickstart))))
        {
            status = HAL_TIMEOUT;
            break;
        }
    }

    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);
    }

    return status;
}

static hal_status_t spi_read_eeprom(spi_handle_t *p_spi, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    while (0U < p_spi->tx_xfer_count)
    {
        p_spi->write_fifo(p_spi);

        if ((0 == timeout) || ((HAL_MAX_DELAY != timeout) && (timeout <= (hal_get_tick() - tickstart))))
        {
            status = HAL_TIMEOUT;
            goto error;
        }
    }

    p_spi->rx_xfer_size  = p_spi->rx_xfer_count;

    while (0U < p_spi->rx_xfer_count)
    {
        p_spi->read_fifo(p_spi);

        if ((0 == timeout) || ((HAL_MAX_DELAY != timeout) && (timeout <= (hal_get_tick() - tickstart))))
        {
            status = HAL_TIMEOUT;
            goto error;
        }
    }

error:
    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = spi_wait_flag_state_until_timeout(p_spi, SPI_FLAG_BUSY, RESET, tickstart, timeout);
    }

    return status;
}


#endif /* HAL_SPI_MODULE_ENABLED */


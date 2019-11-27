/**
  ****************************************************************************************
  * @file    gr55xx_hal_qspi.c
  * @author  BLE Driver Team
  * @brief   QSPI HAL module driver.
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

#ifdef HAL_QSPI_MODULE_ENABLED

#define QSPI_DIRECTION_FULL_DUPLEX      LL_SSI_FULL_DUPLEX
#define QSPI_DIRECTION_ONLY_WRITE       LL_SSI_SIMPLEX_TX
#define QSPI_DIRECTION_ONLY_READ        LL_SSI_SIMPLEX_RX
#define QSPI_DIRECTION_READ_EEPROM      LL_SSI_READ_EEPROM

#define IS_QSPI_DIRECTION(DIRECTION)    (((DIRECTION) == QSPI_DIRECTION_FULL_DUPLEX) || \
                                         ((DIRECTION) == QSPI_DIRECTION_ONLY_WRITE)  || \
                                         ((DIRECTION) == QSPI_DIRECTION_ONLY_READ)   || \
                                         ((DIRECTION) == QSPI_DIRECTION_READ_EEPROM))

static void qspi_dma_rx_cplt(dma_handle_t *p_dma);
static void qspi_dma_tx_cplt(dma_handle_t *p_dma);
static void qspi_dma_error(dma_handle_t *p_dma);
static void qspi_dma_abort_cplt(dma_handle_t *p_dma);
static hal_status_t qspi_wait_flag_state_until_timeout(qspi_handle_t *p_qspi, uint32_t flag, flag_status_t state, uint32_t tick_start, uint32_t timeout);
static void qspi_config(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint32_t functions_mode);
static hal_status_t qspi_transmit(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t timeout);
static hal_status_t qspi_receive(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t timeout);
static hal_status_t qspi_send_inst_addr(qspi_handle_t *p_qspi, qspi_command_t *p_cmd);

__WEAK hal_status_t hal_qspi_init(qspi_handle_t *p_qspi)
{
    hal_status_t status     = HAL_ERROR;
    uint32_t tickstart      = hal_get_tick();
    ll_qspi_init_t spi_init = LL_QSPI_DEFAULT_CONFIG;

    /* Check the QSPI handle allocation */
    if (NULL == p_qspi)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_QSPI_ALL_INSTANCE(p_qspi->p_instance));
    gr_assert_param(IS_QSPI_CLOCK_PRESCALER(p_qspi->init.clock_prescaler));
    gr_assert_param(IS_QSPI_CLOCK_MODE(p_qspi->init.clock_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_RESET == p_qspi->state)
    {
        /* Allocate lock resource and initialize it */
        p_qspi->lock = HAL_UNLOCKED;

        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable QSPIx Clock */
        if(p_qspi->p_instance == QSPI0)
        {
            ll_cgc_disable_force_off_qspi0_hclk();
        }
        else if(p_qspi->p_instance == QSPI1)
        {
            ll_cgc_disable_force_off_qspi1_hclk();
        }

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_qspi_msp_init(p_qspi);

        /* Configure the default timeout for the QSPI memory access */
        hal_qspi_set_timeout(p_qspi, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
    }

    /* Configure QSPI FIFO Threshold */
    ll_spi_set_tx_fifo_threshold(p_qspi->p_instance, 0);
    ll_spi_set_rx_fifo_threshold(p_qspi->p_instance, 0);
    ll_spi_set_dma_tx_fifo_threshold(p_qspi->p_instance, 4);
    ll_spi_set_dma_rx_fifo_threshold(p_qspi->p_instance, 0);

    /* Wait till BUSY flag reset */
    status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

    if (HAL_OK == status)
    {
        /* Configure QSPI Clock Prescaler and Clock Mode */
        spi_init.baud_rate = p_qspi->init.clock_prescaler;
        spi_init.clock_polarity = p_qspi->init.clock_mode & SSI_CTRL0_SCPOL;
        spi_init.clock_phase = p_qspi->init.clock_mode & SSI_CTRL0_SCPHA;

        /* Disable the QSPI peripheral */
        __HAL_QSPI_DISABLE(p_qspi);
        if (SUCCESS == ll_qspi_init(p_qspi->p_instance, &spi_init))
        {
            /* Enable the QSPI peripheral */
            __HAL_QSPI_ENABLE(p_qspi);

            /* Set QSPI error code to none */
            p_qspi->error_code = HAL_QSPI_ERROR_NONE;

            /* Initialize the QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
        else
        {
            status = HAL_ERROR;
        }
    }

    /* Release Lock */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_deinit(qspi_handle_t *p_qspi)
{
    /* Check the QSPI handle allocation */
    if (NULL == p_qspi)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (p_qspi->state != HAL_QSPI_STATE_RESET)
    {
        /* Disable the QSPI Peripheral Clock */
        ll_qspi_deinit(p_qspi->p_instance);

        /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
        hal_qspi_msp_deinit(p_qspi);

        /* Disable QSPIx Clock */
        if(p_qspi->p_instance == QSPI0)
        {
            ll_cgc_enable_force_off_qspi0_hclk();
        }
        else if(p_qspi->p_instance == QSPI1)
        {
            ll_cgc_enable_force_off_qspi1_hclk();
        }

        if((LL_CGC_FRC_I2S_S_HCLK & ll_cgc_get_force_off_hclk_0()) && 
          ((LL_CGC_FRC_SERIALS_HCLK2 & ll_cgc_get_force_off_hclk_2()) == LL_CGC_FRC_SERIALS_HCLK2))
        {
            /* Disable Clock for Serial blocks  */
            ll_cgc_enable_force_off_serial_hclk();
        }

        /* Set QSPI error code to none */
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Initialize the QSPI state */
        p_qspi->state = HAL_QSPI_STATE_RESET;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_qspi);

    return HAL_OK;
}

__WEAK void hal_qspi_msp_init(qspi_handle_t *p_qspi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_qspi_msp_init can be implemented in the user file
     */
}

__WEAK void hal_qspi_msp_deinit(qspi_handle_t *p_qspi)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_qspi_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_qspi_irq_handler(qspi_handle_t *p_qspi)
{
    __IO uint32_t *data_reg = &p_qspi->p_instance->DATA;
    uint32_t itsource       = READ_REG(p_qspi->p_instance->INTSTAT);

    if (itsource & (QSPI_IT_MST | QSPI_IT_RXO | QSPI_IT_RXU | QSPI_IT_TXO))
    {
        ll_spi_clear_flag_all(p_qspi->p_instance);

        /* Disable all the QSPI Interrupts */
        __HAL_QSPI_DISABLE_IT(p_qspi, (QSPI_IT_MST | QSPI_IT_RXF | QSPI_IT_RXO | QSPI_IT_RXU | QSPI_IT_TXO | QSPI_IT_TXE));

        /* Set error code */
        p_qspi->error_code |= HAL_QSPI_ERROR_TRANSFER;

        if (0 != p_qspi->p_instance->DMAC)
        {
            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_qspi->p_instance->DMAC);

            /* Abort DMA channel */
            p_qspi->p_dma->xfer_abort_callback = qspi_dma_abort_cplt;
            hal_dma_abort_it(p_qspi->p_dma);
        }
        else
        {
            /* Change state of QSPI */
            p_qspi->state = HAL_QSPI_STATE_READY;

            /* Error callback */
            hal_qspi_error_callback(p_qspi);
        }
    }

    if (itsource & QSPI_IT_RXF)
    {
        while (__HAL_QSPI_GET_FLAG(p_qspi, QSPI_FLAG_RFNE) && p_qspi->rx_xfer_count)
        {
            *p_qspi->p_rx_buffer++ = *(__IO uint8_t *)((__IO void *)data_reg);
            p_qspi->rx_xfer_count--;
        }

        if (0 == p_qspi->rx_xfer_count)
        {
            /* All data have been received for the transfer */
            /* Disable the QSPI RX Interrupt */
            __HAL_QSPI_DISABLE_IT(p_qspi, QSPI_IT_RXF | QSPI_IT_RXO);

            if (HAL_QSPI_STATE_BUSY_INDIRECT_RX == p_qspi->state)
            {
                /* Change state of QSPI */
                p_qspi->state = HAL_QSPI_STATE_READY;
                hal_qspi_rx_cplt_callback(p_qspi);
            }
            else if (HAL_QSPI_STATE_ABORT == p_qspi->state)
            {
                /* Change state of QSPI */
                p_qspi->state = HAL_QSPI_STATE_READY;
                __HAL_QSPI_DISABLE(p_qspi);
                __HAL_QSPI_ENABLE(p_qspi);
                hal_qspi_abort_cplt_callback(p_qspi);
            }
        }
    }

    if (itsource & QSPI_IT_TXE)
    {
        if (0 == p_qspi->tx_xfer_count)
        {
            /* All data have been sended for the transfer */
            /* Disable the QSPI TX Interrupt */
            __HAL_QSPI_DISABLE_IT(p_qspi, QSPI_IT_TXE);

            if (HAL_QSPI_STATE_BUSY_INDIRECT_TX == p_qspi->state)
            {
                /* Change state of QSPI */
                p_qspi->state = HAL_QSPI_STATE_READY;
                hal_qspi_tx_cplt_callback(p_qspi);
            }
            else if (HAL_QSPI_STATE_ABORT == p_qspi->state)
            {
                /* Change state of QSPI */
                p_qspi->state = HAL_QSPI_STATE_READY;
                __HAL_QSPI_DISABLE(p_qspi);
                __HAL_QSPI_ENABLE(p_qspi);
                hal_qspi_abort_cplt_callback(p_qspi);
            }
        }
        else
        {
            while (__HAL_QSPI_GET_FLAG(p_qspi, QSPI_FLAG_TFNF) && p_qspi->tx_xfer_count)
            {
                *(__IO uint8_t *)((__IO void *)data_reg) = *p_qspi->p_tx_buffer++;
                p_qspi->tx_xfer_count--;
            }
            if (0 == p_qspi->tx_xfer_count)
                ll_spi_set_tx_fifo_threshold(p_qspi->p_instance, 0);
        }
    }
}

__WEAK hal_status_t hal_qspi_command_transmit(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t status = HAL_ERROR;
    uint32_t tickstart  = hal_get_tick();

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DUMMY_CYCLES(p_cmd->dummy_cycles));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_ONLY_WRITE);

            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
            {
                if (QSPI_INSTSIZE_00_BITS != p_cmd->instruction_size)
                    p_qspi->p_instance->DATA = p_cmd->instruction;
                if (QSPI_ADDRSIZE_00_BITS != p_cmd->address_size)
                    p_qspi->p_instance->DATA = p_cmd->address;
            }
            else
            {
                qspi_send_inst_addr(p_qspi, p_cmd);
            }
            status = qspi_transmit(p_qspi, p_data, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_qspi->state = HAL_QSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command_receive(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t status = HAL_ERROR;
    uint32_t tickstart  = hal_get_tick();

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DUMMY_CYCLES(p_cmd->dummy_cycles));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
                qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_ONLY_READ);
            else
                qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_READ_EEPROM);

            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_RX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
            {
                if (QSPI_INSTSIZE_00_BITS != p_cmd->instruction_size)
                    p_qspi->p_instance->DATA = p_cmd->instruction;
                if (QSPI_ADDRSIZE_00_BITS != p_cmd->address_size)
                    p_qspi->p_instance->DATA = p_cmd->address;
            }
            else
            {
                qspi_send_inst_addr(p_qspi, p_cmd);
            }
            status = qspi_receive(p_qspi, p_data, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_qspi->state = HAL_QSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint32_t timeout)
{
    hal_status_t   status     = HAL_ERROR;
    uint32_t       tickstart  = hal_get_tick();
    qspi_command_t cmmand;

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DUMMY_CYCLES(p_cmd->dummy_cycles));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            cmmand.instruction      = p_cmd->instruction;
            cmmand.instruction_size = p_cmd->instruction_size;
            cmmand.address_size     = QSPI_ADDRSIZE_00_BITS;
            cmmand.dummy_cycles     = 0;
            cmmand.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPIFRF;
            cmmand.data_mode        = p_cmd->data_mode;
            cmmand.length           = 1;
            qspi_config(p_qspi, &cmmand, QSPI_DIRECTION_ONLY_WRITE);

            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
                p_qspi->p_instance->DATA = p_cmd->instruction;
            else
                qspi_send_inst_addr(p_qspi, &cmmand);
            HAL_GLOBAL_EXCEPTION_ENABLE();

            qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_TFE, SET, tickstart, timeout);
        }

        p_qspi->state = HAL_QSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_transmit(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t   status    = HAL_OK;
    uint32_t       tickstart = hal_get_tick();
    qspi_command_t cmd;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            cmd.instruction_size = QSPI_INSTSIZE_00_BITS;
            cmd.address_size     = QSPI_ADDRSIZE_00_BITS;
            cmd.dummy_cycles     = 0;
            cmd.data_mode        = QSPI_DATA_MODE_SPI;
            cmd.length           = length;
            qspi_config(p_qspi, &cmd, QSPI_DIRECTION_ONLY_WRITE);

            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            status = qspi_transmit(p_qspi, p_data, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_qspi->state = HAL_QSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_receive(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t   status     = HAL_OK;
    uint32_t       tickstart  = hal_get_tick();
    qspi_command_t cmd;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            cmd.instruction_size = QSPI_INSTSIZE_00_BITS;
            cmd.address_size     = QSPI_ADDRSIZE_00_BITS;
            cmd.dummy_cycles     = 0;
            cmd.data_mode        = QSPI_DATA_MODE_SPI;
            cmd.length           = length;
            qspi_config(p_qspi, &cmd, QSPI_DIRECTION_ONLY_READ);

            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_RX;

            HAL_GLOBAL_EXCEPTION_DISABLE();
            /* Send a dummy byte to start transfer */
            p_qspi->p_instance->DATA = 0xFF;
            status = qspi_receive(p_qspi, p_data, timeout);
            HAL_GLOBAL_EXCEPTION_ENABLE();
        }

        p_qspi->state = HAL_QSPI_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command_transmit_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t status    = HAL_ERROR;
    uint32_t     tickstart = hal_get_tick();

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_ONLY_WRITE);

            p_qspi->tx_xfer_count = p_cmd->length;
            p_qspi->tx_xfer_size  = p_cmd->length;
            p_qspi->p_tx_buffer   = p_data;

            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

            GLOBAL_EXCEPTION_DISABLE();
            if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
            {
                if (QSPI_INSTSIZE_00_BITS != p_cmd->instruction_size)
                    p_qspi->p_instance->DATA = p_cmd->instruction;
                if (QSPI_ADDRSIZE_00_BITS != p_cmd->address_size)
                    p_qspi->p_instance->DATA = p_cmd->address;
            }
            else
            {
                qspi_send_inst_addr(p_qspi, p_cmd);
            }
            ll_spi_set_tx_fifo_threshold(p_qspi->p_instance, 4);
            __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_TXE);
            GLOBAL_EXCEPTION_ENABLE();
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command_receive_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t status    = HAL_ERROR;
    uint32_t     tickstart = hal_get_tick();

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DUMMY_CYCLES(p_cmd->dummy_cycles));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
                qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_ONLY_READ);
            else
                qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_READ_EEPROM);

            p_qspi->rx_xfer_count = p_cmd->length;
            p_qspi->rx_xfer_size  = p_cmd->length;
            p_qspi->p_rx_buffer   = p_data;

            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_RX;

            GLOBAL_EXCEPTION_DISABLE();
            if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
            {
                if (QSPI_INSTSIZE_00_BITS != p_cmd->instruction_size)
                    p_qspi->p_instance->DATA = p_cmd->instruction;
                if (QSPI_ADDRSIZE_00_BITS != p_cmd->address_size)
                    p_qspi->p_instance->DATA = p_cmd->address;
            }
            else
            {
                qspi_send_inst_addr(p_qspi, p_cmd);
            }
            __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_RXF | QSPI_IT_RXO);
            GLOBAL_EXCEPTION_ENABLE();
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command_it(qspi_handle_t *p_qspi, qspi_command_t *p_cmd)
{
    hal_status_t   status    = HAL_ERROR;
    uint32_t       tickstart = hal_get_tick();
    uint8_t        buffer[2] = {0};
    qspi_command_t cmmand;

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            cmmand.instruction_size = p_cmd->instruction_size;
            cmmand.address_size     = QSPI_ADDRSIZE_00_BITS;
            cmmand.dummy_cycles     = 0;
            cmmand.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPIFRF;
            cmmand.data_mode        = p_cmd->data_mode;
            cmmand.length           = 1;
            qspi_config(p_qspi, &cmmand, QSPI_DIRECTION_ONLY_WRITE);

            p_qspi->tx_xfer_size = 0;

            if (QSPI_INSTSIZE_16_BITS == p_cmd->instruction_size)
            {
                buffer[p_qspi->tx_xfer_size++] = (p_cmd->instruction >> 8) & 0xFF;
                buffer[p_qspi->tx_xfer_size++] = p_cmd->instruction & 0xFF;
            }
            else
            {
                buffer[p_qspi->tx_xfer_size++] = p_cmd->instruction & 0xFF;
            }

            p_qspi->tx_xfer_count = p_qspi->tx_xfer_size;
            p_qspi->p_tx_buffer     = buffer;

            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

            __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_TXE);
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_transmit_it(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length)
{
    hal_status_t   status    = HAL_OK;
    uint32_t       tickstart = hal_get_tick();
    qspi_command_t cmd;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            cmd.instruction_size = QSPI_INSTSIZE_00_BITS;
            cmd.address_size     = QSPI_ADDRSIZE_00_BITS;
            cmd.dummy_cycles     = 0;
            cmd.data_mode        = QSPI_DATA_MODE_SPI;
            cmd.length           = length;
            qspi_config(p_qspi, &cmd, QSPI_DIRECTION_ONLY_WRITE);

            /* Configure counters and size of the handle */
            p_qspi->tx_xfer_count = length;
            p_qspi->tx_xfer_size  = length;
            p_qspi->p_tx_buffer   = p_data;

            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

            ll_spi_set_tx_fifo_threshold(p_qspi->p_instance, 4);
            __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_TXE);
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_receive_it(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length)
{
    hal_status_t   status    = HAL_OK;
    uint32_t       tickstart = hal_get_tick();
    qspi_command_t cmd;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            /* Call the configuration function */
            cmd.instruction_size = QSPI_INSTSIZE_00_BITS;
            cmd.address_size     = QSPI_ADDRSIZE_00_BITS;
            cmd.dummy_cycles     = 0;
            cmd.data_mode        = QSPI_DATA_MODE_SPI;
            cmd.length           = length;
            qspi_config(p_qspi, &cmd, QSPI_DIRECTION_ONLY_READ);

            /* Configure counters and size of the handle */
            p_qspi->rx_xfer_count = length;
            p_qspi->rx_xfer_size  = length;
            p_qspi->p_rx_buffer   = p_data;

            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_RX;

            /* Send a dummy byte to start transfer */
            __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_RXF | QSPI_IT_RXO);
            p_qspi->p_instance->DATA = 0xFF;
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command_transmit_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();
    uint32_t     data_size = p_cmd->length;

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_qspi->p_dma->init.src_data_alignment)
            {
                p_qspi->tx_xfer_count = data_size;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (data_size % 2))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->tx_xfer_count = (data_size >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (data_size % 4))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->tx_xfer_count = (data_size >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_ONLY_WRITE);

                p_qspi->tx_xfer_size = p_qspi->tx_xfer_count;
                p_qspi->p_tx_buffer  = p_data;

                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

                /* Set the QSPI DMA transfer complete callback */
                p_qspi->p_dma->xfer_tfr_callback = qspi_dma_tx_cplt;

                /* Set the DMA error callback */
                p_qspi->p_dma->xfer_error_callback = qspi_dma_error;

                /* Clear the DMA abort callback */
                p_qspi->p_dma->xfer_abort_callback = NULL;

                /* Configure the direction of the DMA */
                p_qspi->p_dma->init.direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;

                /* Configure the source address increment */
                ll_dma_set_source_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_SRC_INCREMENT);

                /* Configure the destination address increment */
                ll_dma_set_destination_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_DST_NO_CHANGE);

                /* Configure the destibation peripheral */
                if (QSPI0 == p_qspi->p_instance)
                    ll_dma_set_destination_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI0_TX);
                else if (QSPI1 == p_qspi->p_instance)
                    ll_dma_set_destination_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI1_TX);

                /* Enable the QSPI transmit DMA Channel */
                hal_dma_start_it(p_qspi->p_dma, (uint32_t)p_data, (uint32_t)&p_qspi->p_instance->DATA, p_qspi->tx_xfer_size);

                GLOBAL_EXCEPTION_DISABLE();
                if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
                {
                    if (QSPI_INSTSIZE_00_BITS != p_cmd->instruction_size)
                        p_qspi->p_instance->DATA = p_cmd->instruction;
                    if (QSPI_ADDRSIZE_00_BITS != p_cmd->address_size)
                        p_qspi->p_instance->DATA = p_cmd->address;
                }
                else
                {
                    qspi_send_inst_addr(p_qspi, p_cmd);
                }
                __HAL_QSPI_ENABLE_DMATX(p_qspi);
                GLOBAL_EXCEPTION_ENABLE();
            }
            else
            {
                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_READY;
            }
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command_receive_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();
    uint32_t     data_size = p_cmd->length;

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DUMMY_CYCLES(p_cmd->dummy_cycles));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_qspi->p_dma->init.src_data_alignment)
            {
                p_qspi->rx_xfer_count = data_size;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (data_size % 2))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->rx_xfer_count = (data_size >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (data_size % 4))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->rx_xfer_count = (data_size >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
                    qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_ONLY_READ);
                else
                    qspi_config(p_qspi, p_cmd, QSPI_DIRECTION_READ_EEPROM);

                p_qspi->rx_xfer_size = p_qspi->rx_xfer_count;
                p_qspi->p_rx_buffer    = p_data;

                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_RX;

                /* Set the QSPI DMA transfer complete callback */
                p_qspi->p_dma->xfer_tfr_callback = qspi_dma_rx_cplt;

                /* Set the DMA error callback */
                p_qspi->p_dma->xfer_error_callback = qspi_dma_error;

                /* Clear the DMA abort callback */
                p_qspi->p_dma->xfer_abort_callback = NULL;

                /* Configure the direction of the DMA */
                p_qspi->p_dma->init.direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;

                /* Configure the source address increment */
                ll_dma_set_source_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_SRC_NO_CHANGE);

                /* Configure the destination address increment */
                ll_dma_set_destination_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_DST_INCREMENT);

                /* Configure the source peripheral */
                if (QSPI0 == p_qspi->p_instance)
                    ll_dma_set_source_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI0_RX);
                else if (QSPI1 == p_qspi->p_instance)
                    ll_dma_set_source_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI1_RX);

                /* Enable the QSPI transmit DMA Channel */
                hal_dma_start_it(p_qspi->p_dma, (uint32_t)&p_qspi->p_instance->DATA, (uint32_t)p_data, p_qspi->rx_xfer_size);

                GLOBAL_EXCEPTION_DISABLE();
                if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
                {
                    if (QSPI_INSTSIZE_00_BITS != p_cmd->instruction_size)
                        p_qspi->p_instance->DATA = p_cmd->instruction;
                    if (QSPI_ADDRSIZE_00_BITS != p_cmd->address_size)
                        p_qspi->p_instance->DATA = p_cmd->address;
                }
                else
                {
                    qspi_send_inst_addr(p_qspi, p_cmd);
                }
                __HAL_QSPI_ENABLE_DMARX(p_qspi);
                GLOBAL_EXCEPTION_ENABLE();
            }
            else
            {
                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_READY;
            }
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_command_dma(qspi_handle_t *p_qspi, qspi_command_t *p_cmd)
{
    hal_status_t   status    = HAL_ERROR;
    uint32_t       tickstart = hal_get_tick();
    uint8_t        buffer[2] = {0};
    qspi_command_t cmmand;

    /* Check the parameters */
    gr_assert_param(IS_QSPI_INSTADDR_MODE(p_cmd->instruction_address_mode));
    gr_assert_param(IS_QSPI_INSTRUCTION_SIZE(p_cmd->instruction_size));
    gr_assert_param(IS_QSPI_ADDRESS_SIZE(p_cmd->address_size));
    gr_assert_param(IS_QSPI_DATA_MODE(p_cmd->data_mode));

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            p_qspi->tx_xfer_size = 0;
            if (QSPI_INSTSIZE_16_BITS == p_cmd->instruction_size)
            {
                buffer[p_qspi->tx_xfer_size++] = (p_cmd->instruction >> 8) & 0xFF;
                buffer[p_qspi->tx_xfer_size++] = p_cmd->instruction & 0xFF;
            }
            else
            {
                buffer[p_qspi->tx_xfer_size++] = p_cmd->instruction & 0xFF;
            }

            if (DMA_SDATAALIGN_BYTE == p_qspi->p_dma->init.src_data_alignment)
            {
                p_qspi->tx_xfer_count = p_qspi->tx_xfer_size;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (p_qspi->tx_xfer_size % 2))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->tx_xfer_count = (p_qspi->tx_xfer_size >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (p_qspi->tx_xfer_size % 4))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->tx_xfer_count = (p_qspi->tx_xfer_size >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                cmmand.instruction_size = p_cmd->instruction_size;
                cmmand.address_size     = QSPI_ADDRSIZE_00_BITS;
                cmmand.dummy_cycles     = 0;
                cmmand.instruction_address_mode = QSPI_INST_ADDR_ALL_IN_SPIFRF;
                cmmand.data_mode        = p_cmd->data_mode;
                cmmand.length           = 1;
                qspi_config(p_qspi, &cmmand, QSPI_DIRECTION_ONLY_WRITE);

                p_qspi->p_tx_buffer = buffer;

                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

                /* Set the QSPI DMA transfer complete callback */
                p_qspi->p_dma->xfer_tfr_callback = qspi_dma_tx_cplt;

                /* Set the DMA error callback */
                p_qspi->p_dma->xfer_error_callback = qspi_dma_error;

                /* Clear the DMA abort callback */
                p_qspi->p_dma->xfer_abort_callback = NULL;

                /* Configure the direction of the DMA */
                p_qspi->p_dma->init.direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;

                /* Configure the source address increment */
                ll_dma_set_source_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_SRC_INCREMENT);

                /* Configure the destination address increment */
                ll_dma_set_destination_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_DST_NO_CHANGE);

                /* Configure the destibation peripheral */
                if (QSPI0 == p_qspi->p_instance)
                    ll_dma_set_destination_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI0_TX);
                else if (QSPI1 == p_qspi->p_instance)
                    ll_dma_set_destination_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI1_TX);

                /* Enable the QSPI transmit DMA Channel */
                hal_dma_start_it(p_qspi->p_dma, (uint32_t)buffer, (uint32_t)&p_qspi->p_instance->DATA, p_qspi->tx_xfer_size);

                __HAL_QSPI_ENABLE_DMATX(p_qspi);
            }
            else
            {
                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_READY;
            }
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_transmit_dma(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length)
{
    hal_status_t   status    = HAL_OK;
    uint32_t       tickstart = hal_get_tick();
    qspi_command_t cmd;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_qspi->p_dma->init.src_data_alignment)
            {
                p_qspi->tx_xfer_count = length;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (length % 2))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->tx_xfer_count = (length >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (length % 4))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->tx_xfer_count = (length >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                cmd.instruction_size = QSPI_INSTSIZE_00_BITS;
                cmd.address_size     = QSPI_ADDRSIZE_00_BITS;
                cmd.dummy_cycles     = 0;
                cmd.data_mode        = QSPI_DATA_MODE_SPI;
                cmd.length           = length;
                qspi_config(p_qspi, &cmd, QSPI_DIRECTION_ONLY_WRITE);

                /* Configure counters and size of the handle */
                p_qspi->tx_xfer_size = p_qspi->tx_xfer_count;
                p_qspi->p_tx_buffer  = p_data;

                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_TX;

                /* Set the QSPI DMA transfer complete callback */
                p_qspi->p_dma->xfer_tfr_callback = qspi_dma_tx_cplt;

                /* Set the DMA error callback */
                p_qspi->p_dma->xfer_error_callback = qspi_dma_error;

                /* Clear the DMA abort callback */
                p_qspi->p_dma->xfer_abort_callback = NULL;

                /* Configure the direction of the DMA */
                p_qspi->p_dma->init.direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;

                /* Configure the source address increment */
                ll_dma_set_source_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_SRC_INCREMENT);

                /* Configure the destination address increment */
                ll_dma_set_destination_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_DST_NO_CHANGE);

                /* Configure the destibation peripheral */
                if (QSPI0 == p_qspi->p_instance)
                    ll_dma_set_destination_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI0_TX);
                else if (QSPI1 == p_qspi->p_instance)
                    ll_dma_set_destination_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI1_TX);

                /* Enable the QSPI transmit DMA Channel */
                hal_dma_start_it(p_qspi->p_dma, (uint32_t)p_data, (uint32_t)&p_qspi->p_instance->DATA, p_qspi->tx_xfer_size);

                __HAL_QSPI_ENABLE_DMATX(p_qspi);
            }
            else
            {
                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_READY;
            }
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_receive_dma(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t length)
{
    hal_status_t   status    = HAL_OK;
    uint32_t       tickstart = hal_get_tick();
    qspi_command_t cmd;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        p_qspi->error_code = HAL_QSPI_ERROR_NONE;

        /* Update QSPI state */
        p_qspi->state = HAL_QSPI_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, p_qspi->timeout);

        if (HAL_OK == status)
        {
            if (DMA_SDATAALIGN_BYTE == p_qspi->p_dma->init.src_data_alignment)
            {
                p_qspi->rx_xfer_count = length;
            }
            else if (DMA_SDATAALIGN_HALFWORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (length % 2))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->rx_xfer_count = (length >> 1);
                }
            }
            else if (DMA_SDATAALIGN_WORD == p_qspi->p_dma->init.src_data_alignment)
            {
                if (0 != (length % 4))
                {
                    p_qspi->error_code |= HAL_QSPI_ERROR_INVALID_PARAM;
                    status = HAL_ERROR;
                }
                else
                {
                    p_qspi->rx_xfer_count = (length >> 2);
                }
            }

            if (HAL_OK == status)
            {
                /* Call the configuration function */
                cmd.instruction_size = QSPI_INSTSIZE_00_BITS;
                cmd.address_size     = QSPI_ADDRSIZE_00_BITS;
                cmd.dummy_cycles     = 0;
                cmd.data_mode        = QSPI_DATA_MODE_SPI;
                cmd.length           = length;
                qspi_config(p_qspi, &cmd, QSPI_DIRECTION_ONLY_READ);

                /* Configure counters and size of the handle */
                p_qspi->rx_xfer_size = p_qspi->rx_xfer_count;
                p_qspi->p_rx_buffer  = p_data;

                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_BUSY_INDIRECT_RX;

                /* Set the QSPI DMA transfer complete callback */
                p_qspi->p_dma->xfer_tfr_callback = qspi_dma_rx_cplt;

                /* Set the DMA error callback */
                p_qspi->p_dma->xfer_error_callback = qspi_dma_error;

                /* Clear the DMA abort callback */
                p_qspi->p_dma->xfer_abort_callback = NULL;

                /* Configure the direction of the DMA */
                p_qspi->p_dma->init.direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;

                /* Configure the source address increment */
                ll_dma_set_source_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_SRC_NO_CHANGE);

                /* Configure the destination address increment */
                ll_dma_set_destination_increment_mode(DMA, p_qspi->p_dma->instance, LL_DMA_DST_INCREMENT);

                /* Configure the source peripheral */
                if (QSPI0 == p_qspi->p_instance)
                    ll_dma_set_source_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI0_RX);
                else if (QSPI1 == p_qspi->p_instance)
                    ll_dma_set_source_peripheral(DMA, p_qspi->p_dma->instance, LL_DMA_PERIPH_QSPI1_RX);

                /* Enable the QSPI transmit DMA Channel */
                hal_dma_start_it(p_qspi->p_dma, (uint32_t)&p_qspi->p_instance->DATA, (uint32_t)p_data, p_qspi->rx_xfer_size);

                /* Send a dummy byte to start transfer */
                __HAL_QSPI_ENABLE_DMARX(p_qspi);
                p_qspi->p_instance->DATA = 0xFF;
            }
            else
            {
                /* Update QSPI state */
                p_qspi->state = HAL_QSPI_STATE_READY;
            }
        }
        else
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK void hal_qspi_error_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_abort_cplt_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_fifo_threshold_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_rx_cplt_callback(qspi_handle_t *p_qspi)
{

}

__WEAK void hal_qspi_tx_cplt_callback(qspi_handle_t *p_qspi)
{

}

__WEAK hal_qspi_state_t hal_qspi_get_state(qspi_handle_t *p_qspi)
{
    /* Return QSPI handle state */
    return p_qspi->state;
}

__WEAK uint32_t hal_qspi_get_error(qspi_handle_t *p_qspi)
{
    return p_qspi->error_code;
}

__WEAK hal_status_t hal_qspi_abort(qspi_handle_t *p_qspi)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    /* Check if the state is in one of the busy states */
    if (0 != (p_qspi->state & 0x2))
    {
        /* Process unlocked */
        __HAL_UNLOCK(p_qspi);

        /* Disable all interrupts */
        CLEAR_REG(p_qspi->p_instance->INTMASK);

        if (0 != p_qspi->p_instance->DMAC)
        {
            /* Abort DMA channel */
            status = hal_dma_abort(p_qspi->p_dma);
            if (HAL_OK != status)
            {
                p_qspi->error_code |= HAL_QSPI_ERROR_DMA;
            }

            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_qspi->p_instance->DMAC);
        }

        /* Flush FIFO */
        __HAL_QSPI_DISABLE(p_qspi);
        __HAL_QSPI_ENABLE(p_qspi);

        /* Wait till BUSY flag reset */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            /* Update state */
            p_qspi->state = HAL_QSPI_STATE_READY;
        }
    }

    return status;
}

__WEAK hal_status_t hal_qspi_abort_it(qspi_handle_t *p_qspi)
{
    hal_status_t status = HAL_OK;

    /* Check if the state is in one of the busy states */
    if (0 != (p_qspi->state & 0x2))
    {
        /* Process unlocked */
        __HAL_UNLOCK(p_qspi);

        /* Disable all interrupts */
        CLEAR_REG(p_qspi->p_instance->INTMASK);

        if (0 != p_qspi->p_instance->DMAC)
        {
            /* Disable the DMA transfer by clearing the DMAEN bit in the SSI DMAC register */
            CLEAR_REG(p_qspi->p_instance->DMAC);

            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_ABORT;
            /* Abort DMA channel */
            p_qspi->p_dma->xfer_abort_callback = qspi_dma_abort_cplt;
            hal_dma_abort_it(p_qspi->p_dma);
        }
        else if (HAL_QSPI_STATE_BUSY_INDIRECT_TX == p_qspi->state)
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_ABORT;
            p_qspi->tx_xfer_count = 0;
            __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_TXE);
        }
        else if (HAL_QSPI_STATE_BUSY_INDIRECT_RX == p_qspi->state)
        {
            /* Update QSPI state */
            p_qspi->state = HAL_QSPI_STATE_ABORT;
            p_qspi->rx_xfer_count = 0;
            __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_RXF);
        }
    }

    return status;
}

__WEAK void hal_qspi_set_timeout(qspi_handle_t *p_qspi, uint32_t timeout)
{
    p_qspi->timeout = timeout;
}

__WEAK hal_status_t hal_qspi_set_tx_fifo_threshold(qspi_handle_t *p_qspi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        /* Configure QSPI FIFO Threshold */
        ll_spi_set_tx_fifo_threshold(p_qspi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_qspi_set_rx_fifo_threshold(qspi_handle_t *p_qspi, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_qspi);

    if (HAL_QSPI_STATE_READY == p_qspi->state)
    {
        /* Configure QSPI FIFO Threshold */
        ll_spi_set_rx_fifo_threshold(p_qspi->p_instance, threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_qspi);

    /* Return function status */
    return status;
}

__WEAK uint32_t hal_qspi_get_tx_fifo_threshold(qspi_handle_t *p_qspi)
{
    return ll_spi_get_tx_fifo_threshold(p_qspi->p_instance);
}

__WEAK uint32_t hal_qspi_get_rx_fifo_threshold(qspi_handle_t *p_qspi)
{
    return ll_spi_get_rx_fifo_threshold(p_qspi->p_instance);
}

static void qspi_dma_rx_cplt(dma_handle_t *p_dma)
{
    qspi_handle_t *p_qspi = ( qspi_handle_t* )((dma_handle_t* )p_dma)->p_parent;

    p_qspi->rx_xfer_count = 0;
    __HAL_QSPI_DISABLE_DMARX(p_qspi);

    if (HAL_QSPI_STATE_BUSY_INDIRECT_RX == p_qspi->state)
    {
        /* Change state of QSPI */
        p_qspi->state = HAL_QSPI_STATE_READY;
        hal_qspi_rx_cplt_callback(p_qspi);
    }
}

static void qspi_dma_tx_cplt(dma_handle_t *p_dma)
{
    qspi_handle_t *p_qspi = ( qspi_handle_t* )((dma_handle_t* )p_dma)->p_parent;

    p_qspi->tx_xfer_count = 0;
    __HAL_QSPI_DISABLE_DMATX(p_qspi);

    __HAL_QSPI_ENABLE_IT(p_qspi, QSPI_IT_TXE);
}

static void qspi_dma_error(dma_handle_t *p_dma)
{
    qspi_handle_t *p_qspi = ( qspi_handle_t* )((dma_handle_t* )p_dma)->p_parent;

    p_qspi->error_code  |= HAL_QSPI_ERROR_DMA;

    /* Abort the QSPI */
    hal_qspi_abort_it(p_qspi);
}

static void qspi_dma_abort_cplt(dma_handle_t *p_dma)
{
    qspi_handle_t *p_qspi = ( qspi_handle_t* )((dma_handle_t* )p_dma)->p_parent;

    p_qspi->rx_xfer_count = 0;
    p_qspi->tx_xfer_count = 0;

    __HAL_QSPI_DISABLE(p_qspi);
    __HAL_QSPI_ENABLE(p_qspi);

    if (HAL_QSPI_STATE_ABORT == p_qspi->state)
    {
        /* Change state of QSPI */
        p_qspi->state = HAL_QSPI_STATE_READY;
        hal_qspi_abort_cplt_callback(p_qspi);
    }
    else
    {
        /* DMA Abort called due to a transfer error interrupt */
        /* Change state of QSPI */
        p_qspi->state = HAL_QSPI_STATE_READY;
        /* Error callback */
        hal_qspi_error_callback(p_qspi);
    }
}

static hal_status_t qspi_wait_flag_state_until_timeout(qspi_handle_t *p_qspi,
                                                       uint32_t       flag,
                                                       flag_status_t  state,
                                                       uint32_t       tick_start,
                                                       uint32_t       timeout)
{
    /* Wait until flag is in expected state */
    while ((__HAL_QSPI_GET_FLAG(p_qspi, flag)) != state)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if ((0 == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                p_qspi->state     = HAL_QSPI_STATE_ERROR;
                p_qspi->error_code |= HAL_QSPI_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static void qspi_config(qspi_handle_t *p_qspi, qspi_command_t *p_cmd, uint32_t direction)
{
    gr_assert_param(IS_QSPI_DIRECTION(direction));

    __HAL_QSPI_DISABLE(p_qspi);

    ll_spi_set_transfer_direction(p_qspi->p_instance, direction);
    ll_spi_set_receive_size(p_qspi->p_instance, p_cmd->length - 1);

    if (QSPI_DATA_MODE_SPI != p_cmd->data_mode)
    {
        ll_spi_set_instruction_size(p_qspi->p_instance, p_cmd->instruction_size);
        ll_spi_set_address_size(p_qspi->p_instance, p_cmd->address_size);
        ll_spi_set_add_inst_transfer_format(p_qspi->p_instance, p_cmd->instruction_address_mode);
        ll_spi_set_wait_cycles(p_qspi->p_instance, p_cmd->dummy_cycles);
    }
    ll_spi_set_frame_format(p_qspi->p_instance, p_cmd->data_mode);

    __HAL_QSPI_ENABLE(p_qspi);
}

static hal_status_t qspi_transmit(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t   status    = HAL_OK;
    uint32_t       tickstart = hal_get_tick();
    __IO uint32_t *data_reg  = &p_qspi->p_instance->DATA;

    /* Configure counters and size of the handle */
    p_qspi->tx_xfer_count = (p_qspi->p_instance->CTRL1 & SSI_CTRL1_NDF_Msk) + 1;
    p_qspi->tx_xfer_size  = (p_qspi->p_instance->CTRL1 & SSI_CTRL1_NDF_Msk) + 1;
    p_qspi->p_tx_buffer   = p_data;

    while (0U < p_qspi->tx_xfer_count)
    {
        /* Wait until TFNF flag is set to send data */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_TFNF, SET, tickstart, timeout);

        if (HAL_OK != status)
        {
            break;
        }

        *(__IO uint8_t *)((__IO void *)data_reg) = *p_qspi->p_tx_buffer++;
        p_qspi->tx_xfer_count--;
    }

    if (HAL_OK == status)
    {
        /* Wait until TFE flag is set then to check BUSY flag */
        qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_TFE, SET, tickstart, timeout);
        /* Wait until BUSY flag is reset to go back in idle state */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, timeout);
    }

    return status;
}

static hal_status_t qspi_receive(qspi_handle_t *p_qspi, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t   status    = HAL_OK;
    uint32_t       tickstart = hal_get_tick();
    __IO uint32_t *data_reg  = &p_qspi->p_instance->DATA;

    /* Configure counters and size of the handle */
    p_qspi->rx_xfer_count = (p_qspi->p_instance->CTRL1 & SSI_CTRL1_NDF_Msk) + 1;
    p_qspi->rx_xfer_size  = (p_qspi->p_instance->CTRL1 & SSI_CTRL1_NDF_Msk) + 1;
    p_qspi->p_rx_buffer   = p_data;

    while (0U < p_qspi->rx_xfer_count)
    {
        /* Wait until RFNE flag is set to receive data */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_RFNE, SET, tickstart, timeout);

        if (HAL_OK != status)
        {
            break;
        }

        *p_qspi->p_rx_buffer++ = *(__IO uint8_t *)((__IO void *)data_reg);
        p_qspi->rx_xfer_count--;
    }

    if (HAL_OK == status)
    {
        /* Wait until BUSY flag is reset to go back in idle state */
        status = qspi_wait_flag_state_until_timeout(p_qspi, QSPI_FLAG_BUSY, RESET, tickstart, timeout);
    }

    return status;
}

static hal_status_t qspi_send_inst_addr(qspi_handle_t *p_qspi, qspi_command_t *p_cmd)
{
    uint8_t data_bit;
    uint8_t dummy_dec = 8 >> (p_cmd->data_mode >> SSI_CTRL0_SPIFRF_Pos);

    if (QSPI_INSTSIZE_00_BITS != p_cmd->instruction_size)
    {
        if (QSPI_INSTSIZE_16_BITS == p_cmd->instruction_size)
            data_bit = 16;
        else
            data_bit = 8;
        for (; data_bit > 0; data_bit -= 8)
        {
            p_qspi->p_instance->DATA = (p_cmd->instruction >> (data_bit - 8)) & 0xFF;
        }
    }
    if (QSPI_ADDRSIZE_00_BITS != p_cmd->address_size)
    {
        for (data_bit = p_cmd->address_size; data_bit > 0; data_bit -= 8)
        {
            p_qspi->p_instance->DATA = (p_cmd->address >> (data_bit - 8)) & 0xFF;
        }
    }
    if (0 != p_cmd->dummy_cycles)
    {
        for (data_bit = p_cmd->dummy_cycles; data_bit > 0; data_bit -= dummy_dec)
        {
            p_qspi->p_instance->DATA = 0xFF;
        }
    }

    return HAL_OK;
}



#endif /* HAL_QSPI_MODULE_ENABLED */


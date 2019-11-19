/**
  ****************************************************************************************
  * @file    gr55xx_hal_uart.c
  * @author  BLE Driver Team
  * @brief   UART HAL module driver.
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

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_UART_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup UART_Private_Constants UART Private Constants
  * @{
  */

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup UART_Flags     UART Status Flags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the USR register
  * @{
  */
#define UART_FLAG_LINE_TEMT                 LL_UART_LSR_TEMT

#define UART_FLAG_FIFO_RFF                  LL_UART_USR_RFF     /*!< Rx FIFO Full flag */
#define UART_FLAG_FIFO_RFNE                 LL_UART_USR_RFNE    /*!< Rx FIFO Not Empty flag */
#define UART_FLAG_FIFO_TFE                  LL_UART_USR_TFE     /*!< Tx FIFO Empty flag */
#define UART_FLAG_FIFO_TFNF                 LL_UART_USR_TFNF    /*!< Tx FIFO Not Full flag */
/** @} */

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @addtogroup UART_Private_Functions
  * @{
  */
__STATIC_INLINE void uart_end_tx_transfer(uart_handle_t *p_uart);
__STATIC_INLINE void uart_end_rx_transfer(uart_handle_t *p_uart);
__STATIC_INLINE void uart_stop_dma_tx(uart_handle_t *p_uart);
__STATIC_INLINE void uart_stop_dma_rx(uart_handle_t *p_uart);

static void uart_dma_transmit_cplt(dma_handle_t *p_dma);
static void uart_dma_receive_cplt(dma_handle_t *p_dma);

static void uart_dma_error(dma_handle_t *p_dma);
static void uart_dma_abort_on_error(dma_handle_t *p_dma);

static void uart_dma_tx_abort_callback(dma_handle_t *p_dma);
static void uart_dma_rx_abort_callback(dma_handle_t *p_dma);
static void uart_dma_tx_only_abort_callback(dma_handle_t *p_dma);
static void uart_dma_rx_only_abort_callback(dma_handle_t *p_dma);

static hal_status_t uart_transmit_it(uart_handle_t *p_uart);
static hal_status_t uart_receive_it(uart_handle_t *p_uart, flag_status_t cto_status);

static hal_status_t uart_wait_line_flag_until_timeout(uart_handle_t *p_uart, uint32_t flag, flag_status_t status, uint32_t tick_start, uint32_t timeout);
static hal_status_t uart_wait_fifo_flag_until_timeout(uart_handle_t *p_uart, uint32_t flag, flag_status_t status, uint32_t tick_start, uint32_t timeout);

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup UART_Exported_Functions UART Exported Functions
  * @{
  */

/** @defgroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  * @{
  */

__WEAK hal_status_t hal_uart_init(uart_handle_t *p_uart)
{
    ll_uart_init_t InitStruct;

    /* Check the UART handle allocation */
    if (NULL == p_uart)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));

    if (HAL_UART_STATE_RESET == p_uart->g_state)
    {
        /* Allocate lock resource and initialize it */
        p_uart->lock = HAL_UNLOCKED;

        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable UARTx Clock */
        if(p_uart->p_instance == UART0)
        {
            ll_cgc_disable_force_off_uart0_hclk();
        }
        else if(p_uart->p_instance == UART1)
        {
            ll_cgc_disable_force_off_uart1_hclk();
        }

        /* init the low level hardware : GPIO, CLOCK */
        hal_uart_msp_init(p_uart);
    }

    p_uart->g_state = HAL_UART_STATE_BUSY;

    /* Set the UART Communication parameters */
    InitStruct.baud_rate    = p_uart->init.baud_rate;
    InitStruct.data_bits    = p_uart->init.data_bits;
    InitStruct.parity       = p_uart->init.parity;
    InitStruct.stop_bits    = p_uart->init.stop_bits;
    InitStruct.hw_flow_ctrl = p_uart->init.hw_flow_ctrl;

    ll_uart_init(p_uart->p_instance, &InitStruct);

    /* Initialize the UART ErrorCode */
    p_uart->error_code = HAL_UART_ERROR_NONE;

    /* Initialize the UART State */
    p_uart->g_state  = HAL_UART_STATE_READY;
    p_uart->rx_state = HAL_UART_STATE_READY;

    /* Process Unlocked */
    __HAL_UNLOCK(p_uart);

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_deinit(uart_handle_t *p_uart)
{
    uint32_t tickstart = hal_get_tick();

    /* Check the UART handle allocation */
    if (NULL == p_uart)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));

    if (p_uart->g_state != HAL_UART_STATE_RESET)
    {
        p_uart->g_state = HAL_UART_STATE_BUSY;

        /* Check TEMT to make sure all data was sent out */
        ll_uart_disable_break_sending(p_uart->p_instance);
        uart_wait_line_flag_until_timeout(p_uart, UART_FLAG_LINE_TEMT, RESET, tickstart, HAL_UART_TIMEOUT_DEFAULT_VALUE);

        /* Set the UART registers to default value */
        ll_uart_deinit(p_uart->p_instance);

        /* DeInit the low level hardware */
        hal_uart_msp_deinit(p_uart);

        /* Disable UARTx Clock */
        if(p_uart->p_instance == UART0)
        {
            ll_cgc_enable_force_off_uart0_hclk();
        }
        else if(p_uart->p_instance == UART1)
        {
            ll_cgc_enable_force_off_uart1_hclk();
        }

        if((LL_CGC_FRC_I2S_S_HCLK & ll_cgc_get_force_off_hclk_0()) && 
          ((LL_CGC_FRC_SERIALS_HCLK2 & ll_cgc_get_force_off_hclk_2()) == LL_CGC_FRC_SERIALS_HCLK2))
        {
            /* Disable Clock for Serial blocks  */
            ll_cgc_enable_force_off_serial_hclk();
        }

        p_uart->error_code = HAL_UART_ERROR_NONE;
        p_uart->g_state    = HAL_UART_STATE_RESET;
        p_uart->rx_state   = HAL_UART_STATE_RESET;
    }

    /* Process Unlock */
    __HAL_UNLOCK(p_uart);

    return HAL_OK;
}

__WEAK void hal_uart_msp_init(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_msp_deinit(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

/** @} */

/** @defgroup UART_Exported_Functions_Group2 IO operation functions
  * @brief UART Transmit/Receive functions
  * @{
  */

__WEAK hal_status_t hal_uart_transmit(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Tx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->g_state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        //__HAL_LOCK(p_uart);

        p_uart->error_code = HAL_UART_ERROR_NONE;
        p_uart->g_state    = HAL_UART_STATE_BUSY_TX;

        /* init tickstart for timeout managment*/
        tickstart = hal_get_tick();

        p_uart->tx_xfer_size  = size;
        p_uart->tx_xfer_count = size;
        while (0U < p_uart->tx_xfer_count)
        {
            if (HAL_OK != uart_wait_fifo_flag_until_timeout(p_uart, UART_FLAG_FIFO_TFNF, RESET, tickstart, timeout))
            {
                return HAL_TIMEOUT;
            }
            p_uart->tx_xfer_count--;
            ll_uart_transmit_data8(p_uart->p_instance, *p_data++);

        }

        if (HAL_OK != uart_wait_line_flag_until_timeout(p_uart, UART_FLAG_LINE_TEMT, RESET, tickstart, timeout))
        {
            return HAL_TIMEOUT;
        }

        /* At end of Tx process, restore p_uart->gState to Ready */
        p_uart->g_state = HAL_UART_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(p_uart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_uart_receive(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    /* Check that a Rx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->rx_state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        //__HAL_LOCK(p_uart);

        p_uart->error_code = HAL_UART_ERROR_NONE;
        p_uart->rx_state   = HAL_UART_STATE_BUSY_RX;

        /* init tickstart for timeout managment*/
        tickstart = hal_get_tick();

        p_uart->rx_xfer_size  = size;
        p_uart->rx_xfer_count = size;

        /* as long as data have to be received */
        while (0U < p_uart->rx_xfer_count)
        {
            if (HAL_OK != uart_wait_fifo_flag_until_timeout(p_uart, UART_FLAG_FIFO_RFNE, RESET, tickstart, timeout))
            {
                return HAL_TIMEOUT;
            }
            p_uart->rx_xfer_count--;
            *p_data++ = ll_uart_receive_data8(p_uart->p_instance);
        }

        /* At end of Rx process, restore p_uart->RxState to Ready */
        p_uart->rx_state = HAL_UART_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(p_uart);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_uart_transmit_it(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
    /* Check that a Tx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->g_state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        //__HAL_LOCK(p_uart);

        p_uart->p_tx_buffer   = p_data;
        p_uart->tx_xfer_size  = size;
        p_uart->tx_xfer_count = size;

        p_uart->error_code = HAL_UART_ERROR_NONE;
        p_uart->g_state = HAL_UART_STATE_BUSY_TX;

        ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_CHAR_2);

        /* Process Unlocked */
        __HAL_UNLOCK(p_uart);

        /* Note : The UART interrupt transfer must be enabled after unlocking current process
                to avoid the risk of UART interrupt handle execution before current
                process unlock */

        /* Enable the UART Transmit Data Register Empty Interrupt */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_THRE);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_uart_receive_it(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
    /* Check that a Rx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->rx_state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        //__HAL_LOCK(p_uart);

        p_uart->p_rx_buffer   = p_data;
        p_uart->rx_xfer_size  = size;
        p_uart->rx_xfer_count = size;

        p_uart->error_code = HAL_UART_ERROR_NONE;
        p_uart->rx_state = HAL_UART_STATE_BUSY_RX;

        ll_uart_set_rx_fifo_threshold(p_uart->p_instance, LL_UART_RX_FIFO_TH_HALF_FULL);

        /* If Overrun error occurs. */
        if(ll_uart_get_line_status_flag(p_uart->p_instance) & HAL_UART_ERROR_OE)
        {
            ll_uart_flush_rx_fifo(p_uart->p_instance);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(p_uart);

        /* Note : The UART interrupt transfer must be enabled after unlocking current process
                to avoid the risk of UART interrupt handle execution before current
                process unlock */

        /* Enable the UART Line Status Interrupt */
        /* Enable the UART Received Data Available and Character Timeout Interrupts */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_uart_transmit_dma(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
    /* Check if UART p_instance supports continuous communication using DMA */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));

    /* Check that a Tx process is not already ongoing */
    if (HAL_UART_STATE_READY == p_uart->g_state)
    {
        if ((NULL == p_data) || (0U == size) || (NULL == p_uart->p_dmatx))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        //__HAL_LOCK(p_uart);

        /* Set UART TX FIFO Threshold and DMA Burst Length for DMA transfer */
        ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_CHAR_2);
        ll_dma_set_destination_burst_length(DMA, p_uart->p_dmatx->instance, LL_DMA_DST_BURST_LENGTH_8);

        p_uart->p_tx_buffer   = p_data;
        p_uart->tx_xfer_size  = size;
        p_uart->tx_xfer_count = size;

        p_uart->error_code  = HAL_UART_ERROR_NONE;
        p_uart->g_state     = HAL_UART_STATE_BUSY_TX;
        p_uart->dma_tx_mode = ENABLE;

        /* Set the UART DMA transfer complete callback */
        p_uart->p_dmatx->xfer_tfr_callback = uart_dma_transmit_cplt;

        /* Set the DMA error callback */
        p_uart->p_dmatx->xfer_error_callback = uart_dma_error;

        /* Set the DMA abort callback */
        p_uart->p_dmatx->xfer_abort_callback = NULL;

        /* Process Unlocked */
        __HAL_UNLOCK(p_uart);

        /* Note : The UART DMA transfer must be enabled after unlocking current process
                to avoid the risk of UART/DMA interrupt handle execution before current
                process unlock */

        /* Enable the UART transmit DMA channel */
        hal_dma_start_it(p_uart->p_dmatx, (uint32_t)p_uart->p_tx_buffer, ll_uart_dma_get_register_address(p_uart->p_instance), size);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_uart_receive_dma(uart_handle_t *p_uart, uint8_t *p_data, uint16_t size)
{
    /* Check if UART p_instance supports continuous communication using DMA */
    gr_assert_param(IS_UART_ALL_INSTANCE(p_uart->p_instance));

    /* Check that a Rx process is not already ongoing */

    if (HAL_UART_STATE_READY == p_uart->rx_state)
    {
        if ((NULL == p_data) || (0U == size) || (NULL == p_uart->p_dmarx))
        {
            return HAL_ERROR;
        }

        /* Process Locked */
        //__HAL_LOCK(p_uart);

        /* Set UART RX FIFO Threshold and DMA Burst Length for DMA transfer */
        if (UART_RECEIVER_TIMEOUT_DISABLE == p_uart->init.rx_timeout_mode)
        {
            ll_uart_set_rx_fifo_threshold(p_uart->p_instance, LL_UART_RX_FIFO_TH_CHAR_1);
            ll_dma_set_source_burst_length(DMA, p_uart->p_dmarx->instance, LL_DMA_SRC_BURST_LENGTH_1);
        }
        else
        {
            ll_uart_set_rx_fifo_threshold(p_uart->p_instance, LL_UART_RX_FIFO_TH_QUARTER_FULL);
            ll_dma_set_source_burst_length(DMA, p_uart->p_dmarx->instance, LL_DMA_SRC_BURST_LENGTH_8);
        }

        p_uart->p_rx_buffer   = p_data;
        p_uart->rx_xfer_size  = size;
        p_uart->rx_xfer_count = size;

        p_uart->error_code  = HAL_UART_ERROR_NONE;
        p_uart->rx_state    = HAL_UART_STATE_BUSY_RX;
        p_uart->dma_rx_mode = ENABLE;

        /* Set the UART DMA transfer complete callback */
        p_uart->p_dmarx->xfer_tfr_callback = uart_dma_receive_cplt;

        /* Set the DMA error callback */
        p_uart->p_dmarx->xfer_error_callback = uart_dma_error;

        /* Set the DMA abort callback */
        p_uart->p_dmarx->xfer_abort_callback = NULL;

        /* If Overrun error occurs. */
        if(ll_uart_get_line_status_flag(p_uart->p_instance) & HAL_UART_ERROR_OE)
        {
            ll_uart_flush_rx_fifo(p_uart->p_instance);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(p_uart);

        /* Note : The UART DMA transfer must be enabled after unlocking current process
                to avoid the risk of UART/DMA interrupt handle execution before current
                process unlock */

        /* Enable the DMA channel */
        hal_dma_start_it(p_uart->p_dmarx, ll_uart_dma_get_register_address(p_uart->p_instance), (uint32_t)p_uart->p_rx_buffer, size);

        /* Enable RLS Interrupt */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_RLS);
        if (UART_RECEIVER_TIMEOUT_ENABLE == p_uart->init.rx_timeout_mode)
        {
            /* Enable Received Data Available Interrupt and Character Timeout Interrupt */
            __HAL_UART_ENABLE_IT(p_uart, UART_IT_RDA);
        }

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_uart_dma_pause(uart_handle_t *p_uart)
{
    /* Process Locked */
    //__HAL_LOCK(p_uart);

    if ((HAL_UART_STATE_BUSY_TX == p_uart->g_state) && (ENABLE == p_uart->dma_tx_mode))
    {
        /* Suspend the DMA channel to disable the UART DMA Tx request */
        ll_dma_suspend_channel(DMA, p_uart->p_dmatx->instance);
    }

    if ((HAL_UART_STATE_BUSY_RX == p_uart->rx_state) && (ENABLE == p_uart->dma_rx_mode))
    {
        /* Disable RLS interrupt */
        __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS);

        /* Suspend the DMA channel to disable the UART DMA Rx request */
        ll_dma_suspend_channel(DMA, p_uart->p_dmarx->instance);

    }

    /* Process Unlocked */
    __HAL_UNLOCK(p_uart);

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_dma_resume(uart_handle_t *p_uart)
{
    /* Process Locked */
    //__HAL_LOCK(p_uart);

    if (HAL_UART_STATE_BUSY_TX == p_uart->g_state)
    {
        /* Enable the UART DMA Tx request */
        ll_dma_resume_channel(DMA, p_uart->p_dmatx->instance);
    }
    if (HAL_UART_STATE_BUSY_RX == p_uart->rx_state)
    {
        /* Clear receive line error status */
        ll_uart_clear_line_status_flag(p_uart->p_instance);

        /* Enable RLS interrupt */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_RLS);

        /* Enable the UART DMA Rx request */
        ll_dma_resume_channel(DMA, p_uart->p_dmarx->instance);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(p_uart);

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_dma_stop(uart_handle_t *p_uart)
{
    /* The Lock is not implemented on this API to allow the user application
       to call the HAL UART API under callbacks hal_uart_tx_cplt_callback() / hal_uart_rx_cplt_callback():
       indeed, when hal_dma_abort() API is called, the DMA TX/RX Transfer complete interrupt is generated
       if the DMA transfer interruption occurs at the middle or at the end of the stream and the
       corresponding call back is executed. */

    /* Stop UART DMA Tx request if ongoing */
    uart_stop_dma_tx(p_uart);

    /* Stop UART DMA Rx request if ongoing */
    uart_stop_dma_rx(p_uart);

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort(uart_handle_t *p_uart)
{
    /* Disable THRE, RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmatx);
        }
        p_uart->dma_tx_mode = DISABLE;
    }

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmarx);
        }
        p_uart->dma_rx_mode = DISABLE;
    }

    /* Reset Tx and Rx transfer counters */
    p_uart->tx_xfer_count = 0U;
    p_uart->rx_xfer_count = 0U;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Flush the whole TX FIFO and Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_TXRXDATA_FLUSH_REQUEST);

    /* Restore p_uart->gState and p_uart->RxState to Ready */
    p_uart->g_state  = HAL_UART_STATE_READY;
    p_uart->rx_state = HAL_UART_STATE_READY;

    /* Reset Handle ErrorCode to No Error */
    p_uart->error_code = HAL_UART_ERROR_NONE;

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_transmit(uart_handle_t *p_uart)
{
    /* Disable THRE interrupt */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmatx);
        }
        p_uart->dma_tx_mode = DISABLE;
    }

    /* Reset Tx transfer counter */
    p_uart->tx_xfer_count = 0U;

    /* Flush the whole TX FIFO */
    __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

    /* Restore p_uart->gState to Ready */
    p_uart->g_state = HAL_UART_STATE_READY;

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_receive(uart_handle_t *p_uart)
{
    /* Disable RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use blocking DMA Abort API (no callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* Set the UART DMA Abort callback to Null.
                No call back execution at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = NULL;

            hal_dma_abort(p_uart->p_dmarx);
        }
        p_uart->dma_rx_mode = DISABLE;
    }

    /* Reset Rx transfer counter */
    p_uart->rx_xfer_count = 0U;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

    /* Restore p_uart->RxState to Ready */
    p_uart->rx_state = HAL_UART_STATE_READY;

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_it(uart_handle_t *p_uart)
{
    uint32_t abortcplt = 1U;

    /* Disable THRE, RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* UART Tx DMA Abort callback will lead to call hal_uart_abort_cplt_callback()
               at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = uart_dma_tx_abort_callback;

            /* Abort DMA TX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmatx))
            {
                p_uart->p_dmatx->xfer_abort_callback = NULL;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* UART Rx DMA Abort callback will lead to call hal_uart_abort_cplt_callback()
               at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = uart_dma_rx_abort_callback;

            /* Abort DMA RX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmarx))
            {
                p_uart->p_dmarx->xfer_abort_callback = NULL;
                abortcplt = 1U;
            }
            else
            {
                abortcplt = 0U;
            }
        }
    }

    /* if no DMA abort complete callback execution is required => call user Abort Complete callback */
    if (1U == abortcplt)
    {
        /* Reset Tx and Rx transfer counters */
        p_uart->tx_xfer_count = 0U;
        p_uart->rx_xfer_count = 0U;

        /* Reset errorCode */
        p_uart->error_code = HAL_UART_ERROR_NONE;

        /* Clear the Error flags in the LSR register */
        ll_uart_clear_line_status_flag(p_uart->p_instance);

        /* Flush the whole TX FIFO and Discard the received data */
        __HAL_UART_SEND_REQ(p_uart, UART_TXRXDATA_FLUSH_REQUEST);

        /* Restore p_uart->gState and p_uart->RxState to Ready */
        p_uart->g_state  = HAL_UART_STATE_READY;
        p_uart->rx_state = HAL_UART_STATE_READY;

        /* As no DMA to be aborted, call directly user Abort complete callback */
        hal_uart_abort_cplt_callback(p_uart);

        p_uart->dma_tx_mode = DISABLE;
        p_uart->dma_rx_mode = DISABLE;
    }
    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_transmit_it(uart_handle_t *p_uart)
{
    /* Disable THRE interrupt */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE);

    /* Disable the UART DMA Tx request if enabled */
    if (ENABLE == p_uart->dma_tx_mode)
    {
        /* Abort the UART DMA Tx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmatx)
        {
            /* Set the UART DMA Abort callback :
                will lead to call hal_uart_abort_cplt_callback() at end of DMA abort procedure */
            p_uart->p_dmatx->xfer_abort_callback = uart_dma_tx_only_abort_callback;

            /* Abort DMA TX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmatx))
            {
                /* Call Directly p_uart->p_dmatx->XferAbortCallback function in case of error */
                p_uart->p_dmatx->xfer_abort_callback(p_uart->p_dmatx);
            }
        }
        else
        {
            /* Reset Tx transfer counter */
            p_uart->tx_xfer_count = 0U;

            /* Restore p_uart->gState to Ready */
            p_uart->g_state = HAL_UART_STATE_READY;

            /* As no DMA to be aborted, call directly user Abort complete callback */
            hal_uart_abort_tx_cplt_callback(p_uart);
        }
        p_uart->dma_tx_mode = DISABLE;
    }
    else
    {
        /* Reset Tx transfer counter */
        p_uart->tx_xfer_count = 0U;

        /* Flush the whole TX FIFO */
        __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

        /* Restore p_uart->gState to Ready */
        p_uart->g_state = HAL_UART_STATE_READY;

        /* As no DMA to be aborted, call directly user Abort complete callback */
        hal_uart_abort_tx_cplt_callback(p_uart);
    }

    return HAL_OK;
}

__WEAK hal_status_t hal_uart_abort_receive_it(uart_handle_t *p_uart)
{
    /* Disable RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

    /* Disable the UART DMA Rx request if enabled */
    if (ENABLE == p_uart->dma_rx_mode)
    {
        /* Abort the UART DMA Rx channel : use non blocking DMA Abort API (callback) */
        if (NULL != p_uart->p_dmarx)
        {
            /* Set the UART DMA Abort callback :
                will lead to call hal_uart_abort_cplt_callback() at end of DMA abort procedure */
            p_uart->p_dmarx->xfer_abort_callback = uart_dma_rx_only_abort_callback;

            /* Abort DMA RX */
            if (HAL_OK != hal_dma_abort_it(p_uart->p_dmarx))
            {
                /* Call Directly p_uart->p_dmarx->XferAbortCallback function in case of error */
                p_uart->p_dmarx->xfer_abort_callback(p_uart->p_dmarx);
            }
        }
        else
        {
            /* Reset Rx transfer counter */
            p_uart->rx_xfer_count = 0U;

            /* Clear the Error flags in the LSR register */
            ll_uart_clear_line_status_flag(p_uart->p_instance);

            /* Discard the received data */
            __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

            /* Restore p_uart->RxState to Ready */
            p_uart->rx_state = HAL_UART_STATE_READY;

            /* As no DMA to be aborted, call directly user Abort complete callback */
            hal_uart_abort_rx_cplt_callback(p_uart);
        }
        p_uart->dma_rx_mode = DISABLE;
    }
    else
    {
        /* Reset Rx transfer counter */
        p_uart->rx_xfer_count = 0U;

        /* Clear the Error flags in the LSR register */
        ll_uart_clear_line_status_flag(p_uart->p_instance);

        /* Restore p_uart->RxState to Ready */
        p_uart->rx_state = HAL_UART_STATE_READY;

        /* As no DMA to be aborted, call directly user Abort complete callback */
        hal_uart_abort_rx_cplt_callback(p_uart);
    }

    return HAL_OK;
}

__WEAK void hal_uart_irq_handler(uart_handle_t *p_uart)
{
    uint32_t isrflag      = ll_uart_get_it_flag(p_uart->p_instance);
    uint32_t linestat     = 0;
    flag_status_t ctostat = RESET;

    switch (isrflag)
    {
    /* UART transmit hold register empty */
    case LL_UART_IIR_THRE:
        uart_transmit_it(p_uart);
        break;

    /* UART receive data avialble */
    case LL_UART_IIR_RDA:
        uart_receive_it(p_uart, RESET);
        break;

    /* UART character timeout */
    case LL_UART_IIR_CTO:
        /* In rx_timeout_mode, rx timeout will also be treated as receive complete. */
        if (UART_RECEIVER_TIMEOUT_ENABLE == p_uart->init.rx_timeout_mode)
        {
            ctostat = SET;
        }
        uart_receive_it(p_uart, ctostat);
        break;

    /* UART receive line error */
    case LL_UART_IIR_RLS:
        linestat = ll_uart_get_line_status_flag(p_uart->p_instance);
        p_uart->error_code |= (linestat & UART_LINE_ERROR_MASK);

        if (SET == ll_uart_is_active_flag_rfne(p_uart->p_instance))
        {
            /* Method 1: call uart_receive_it() to continue receive error data,
               Method 2: do nothing, user need to decide whether flush RX FIFO
               or continue to receive data */
            uart_receive_it(p_uart, RESET);
        }

        /* Call UART Error Call back function if need be ----------------------------*/
        /* If Overrun error occurs, or if any error occurs in DMA mode reception,
            consider error as blocking */
        if ((RESET != (p_uart->error_code & HAL_UART_ERROR_OE))/* ||
                (HAL_IS_BIT_SET(p_uart->p_instance->CR3, USART_CR3_DMAR))*/)
        {
            /* Blocking error : transfer is aborted
                Set the UART state ready to be able to start again the process,
                Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
            uart_end_rx_transfer(p_uart);

            /* Disable the UART DMA Rx request if enabled */
            if (ENABLE == p_uart->dma_rx_mode)
            {
                /* Abort the UART DMA Rx channel */
                if (NULL != p_uart->p_dmarx)
                {
                    /* Set the UART DMA Abort callback :
                        will lead to call hal_uart_error_callback() at end of DMA abort procedure */
                    p_uart->p_dmarx->xfer_abort_callback = uart_dma_abort_on_error;

                    /* Abort DMA RX */
                    if (HAL_OK != hal_dma_abort_it(p_uart->p_dmarx))
                    {
                        /* Call Directly p_uart->p_dmarx->XferAbortCallback function in case of error */
                        p_uart->p_dmarx->xfer_abort_callback(p_uart->p_dmarx);
                    }
                }
                else
                {
                    /* Call user error callback */
                    hal_uart_error_callback(p_uart);
                }
                p_uart->dma_rx_mode = DISABLE;
            }
            else
            {
                /* Call user error callback */
                hal_uart_error_callback(p_uart);
            }
        }
        else
        {
            /* Non Blocking error : transfer could go on.
                Error is notified to user through user error callback */
            hal_uart_error_callback(p_uart);
            p_uart->error_code = HAL_UART_ERROR_NONE;
        }
        break;

    default:
        break;
    }
    return;
}

__WEAK void hal_uart_tx_cplt_callback(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_rx_cplt_callback(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_error_callback(uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_abort_cplt_callback (uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_abort_tx_cplt_callback (uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

__WEAK void hal_uart_abort_rx_cplt_callback (uart_handle_t *p_uart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_uart);
}

/** @} */


/** @defgroup UART_Exported_Functions_Group3 Peripheral Control and State functions
 *  @brief   UART Peripheral State functions
  * @{
  */

__WEAK hal_uart_state_t hal_uart_get_state(uart_handle_t *p_uart)
{
    uint32_t temp1 = 0x00U;
    uint32_t temp2 = 0x00U;

    temp1 = p_uart->g_state;
    temp2 = p_uart->rx_state;

    return (hal_uart_state_t)(temp1 | temp2);
}

__WEAK uint32_t hal_uart_get_error(uart_handle_t *p_uart)
{
    return p_uart->error_code;
}

// HAL_StatusTypeDef HAL_UART_SetTxFifoThreshold(UART_HandleTypeDef *p_uart, uint32_t Threshold)
// {
//     HAL_StatusTypeDef status = HAL_OK;

//     /* Process locked */
//     //__HAL_LOCK(p_uart);

//     if (HAL_UART_STATE_READY == p_uart->gState)
//     {
//         /* Configure UART TX FIFO Threshold */
//         LL_UART_SetTxFIFOThreshold(p_uart->p_instance, Threshold);
//     }
//     else
//     {
//         status = HAL_BUSY;
//     }

//     /* Process unlocked */
//     __HAL_UNLOCK(p_uart);

//     /* Return function status */
//     return status;
// }

// HAL_StatusTypeDef HAL_UART_SetRxFifoThreshold(UART_HandleTypeDef *p_uart, uint32_t Threshold)
// {
//     HAL_StatusTypeDef status = HAL_OK;

//     /* Process locked */
//     //__HAL_LOCK(p_uart);

//     if (HAL_UART_STATE_READY == p_uart->RxState)
//     {
//         /* Configure UART RX FIFO Threshold */
//         LL_UART_SetRxFIFOThreshold(p_uart->p_instance, Threshold);
//     }
//     else
//     {
//         status = HAL_BUSY;
//     }

//     /* Process unlocked */
//     __HAL_UNLOCK(p_uart);

//     /* Return function status */
//     return status;
// }

// uint32_t HAL_UART_GetTxFifoThreshold(UART_HandleTypeDef *p_uart)
// {
//     return LL_UART_GetTxFIFOThreshold(p_uart->p_instance);
// }

// uint32_t HAL_UART_GetRxFifoThreshold(UART_HandleTypeDef *p_uart)
// {
//     return LL_UART_GetTxFIFOThreshold(p_uart->p_instance);
// }

/** @} */

/** @} */

/** @defgroup UART_Private_Functions UART Private Functions
  * @{
  */

/**
  * @brief  Handle UART Receive Line Communication Timeout.
  * @param p_uart UART handle.
  * @param  Flag Specifies the UART line flag to check
  * @param  Status Flag status (SET or RESET)
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
hal_status_t uart_wait_line_flag_until_timeout(uart_handle_t *p_uart,
                                               uint32_t       flag,
                                               flag_status_t  Status,
                                               uint32_t       tick_start,
                                               uint32_t       timeout)
{
    /* Wait until flag is set */
    while ((HAL_IS_BIT_SET(p_uart->p_instance->LSR, flag) ? SET : RESET) == Status)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if ((0U == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                /* Disable THRE, RLS, RDA interrupts */
                __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);

                p_uart->g_state  = HAL_UART_STATE_READY;
                p_uart->rx_state = HAL_UART_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(p_uart);
                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
}

/**
  * @brief  Handle UART FIFO Communication Timeout.
  * @param p_uart UART handle.
  * @param  Flag Specifies the UART FIFO flag to check
  * @param  Status Flag status (SET or RESET)
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
hal_status_t uart_wait_fifo_flag_until_timeout(uart_handle_t *p_uart,
                                               uint32_t       flag,
                                               flag_status_t  Status,
                                               uint32_t       tick_start,
                                               uint32_t       timeout)
{
    /* Wait until flag is set */
    while ((HAL_IS_BIT_SET(p_uart->p_instance->USR, flag) ? SET : RESET) == Status)
    {
        /* Check for the Timeout */
        if(HAL_MAX_DELAY != timeout)
        {
            if ((0U == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                /* Disable THRE, RLS, RDA interrupts */
                __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE | UART_IT_RLS | UART_IT_RDA);

                p_uart->g_state  = HAL_UART_STATE_READY;
                p_uart->rx_state = HAL_UART_STATE_READY;

                /* Process Unlocked */
                __HAL_UNLOCK(p_uart);
                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
}


__STATIC_INLINE void uart_stop_dma_tx(uart_handle_t *p_uart)
{
    /* Stop UART DMA Tx request if ongoing */
    if ((HAL_UART_STATE_BUSY_TX == p_uart->g_state) && (ENABLE == p_uart->dma_tx_mode))
    {
        /* Abort the UART DMA Tx channel */
        if (NULL != p_uart->p_dmatx)
        {
            hal_dma_abort(p_uart->p_dmatx);
        }

        p_uart->dma_tx_mode = DISABLE;
        uart_end_tx_transfer(p_uart);
    }
}

__STATIC_INLINE void uart_stop_dma_rx(uart_handle_t *p_uart)
{
    /* Stop UART DMA Rx request if ongoing */
    if ((HAL_UART_STATE_BUSY_RX == p_uart->rx_state) && (ENABLE == p_uart->dma_rx_mode))
    {
        /* Abort the UART DMA Rx channel */
        if (NULL != p_uart->p_dmarx)
        {
            hal_dma_abort(p_uart->p_dmarx);
        }

        p_uart->dma_rx_mode = DISABLE;
        uart_end_rx_transfer(p_uart);
    }
}

/**
  * @brief  End ongoing Tx transfer on UART peripheral (following error detection or Transmit completion).
  * @param  p_uart UART handle.
  * @retval None
  */
__STATIC_INLINE void uart_end_tx_transfer(uart_handle_t *p_uart)
{
    /* Disable THRE interrupt */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE);

    /* At end of Tx process, restore p_uart->gState to Ready */
    p_uart->g_state = HAL_UART_STATE_READY;
}


/**
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  p_uart UART handle.
  * @retval None
  */
__STATIC_INLINE void uart_end_rx_transfer(uart_handle_t *p_uart)
{
    /* Disable RLS, RDA interrupts */
    __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS | UART_IT_RDA);

    /* At end of Rx process, restore p_uart->RxState to Ready */
    p_uart->rx_state = HAL_UART_STATE_READY;
}

/**
  * @brief DMA UART transmit process complete callback.
  * @param p_dma DMA handle.
  * @retval None
  */
static void uart_dma_transmit_cplt(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

    /* DMA Normal mode */
    if (DMA_NORMAL == ll_dma_get_mode(DMA, p_dma->instance))
    {
        p_uart->tx_xfer_count = 0U;

        /* Disable the DMA transfer mode */
        p_uart->dma_tx_mode = DISABLE;

        /* Set UART TX FIFO Threshold to EMPTY to make sure all data were sent out */
        ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_EMPTY);

        /* Enable the UART THRE interrupt, wait all data was sent out */
        __HAL_UART_ENABLE_IT(p_uart, UART_IT_THRE);
    }
    /* DMA Circular mode */
    else
    {
        hal_uart_tx_cplt_callback(p_uart);
    }
}


/**
  * @brief DMA UART receive process complete callback.
  * @param p_dma DMA handle.
  * @retval None
  */
static void uart_dma_receive_cplt(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

    /* DMA Normal mode */
    if (DMA_NORMAL == ll_dma_get_mode(DMA, p_dma->instance))
    {
        p_uart->rx_xfer_count = 0U;

        /* Disable RLS interrupt */
        __HAL_UART_DISABLE_IT(p_uart, UART_IT_RLS);
        if (UART_RECEIVER_TIMEOUT_ENABLE == p_uart->init.rx_timeout_mode)
        {
            /* Disable Received Data Available Interrupt and Character Timeout Interrupt */
            __HAL_UART_DISABLE_IT(p_uart, UART_IT_RDA);
        }

        /* Disable the DMA transfer mode */
        p_uart->dma_rx_mode = DISABLE;

        /* At end of Rx process, restore p_uart->RxState to Ready */
        p_uart->rx_state = HAL_UART_STATE_READY;
    }

    hal_uart_rx_cplt_callback(p_uart);
}

/**
  * @brief DMA UART communication error callback.
  * @param p_dma DMA handle.
  * @retval None
  */
static void uart_dma_error(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

    /* Stop UART DMA Tx request if ongoing */
    if ((HAL_UART_STATE_BUSY_TX == p_uart->g_state) && (ENABLE == p_uart->dma_tx_mode))
    {
        p_uart->tx_xfer_count = 0U;
        uart_end_tx_transfer(p_uart);
    }

    /* Stop UART DMA Rx request if ongoing */
    if ((HAL_UART_STATE_BUSY_RX == p_uart->rx_state) && (ENABLE == p_uart->dma_rx_mode) )
    {
        p_uart->rx_xfer_count = 0U;
        uart_end_rx_transfer(p_uart);
    }

    p_uart->error_code |= HAL_UART_ERROR_DMA;
    hal_uart_error_callback(p_uart);
}

/**
  * @brief  DMA UART communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_abort_on_error(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);
    p_uart->rx_xfer_count = 0U;
    p_uart->tx_xfer_count = 0U;

    hal_uart_error_callback(p_uart);
}

/**
  * @brief  DMA UART Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_tx_abort_callback(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t* )(p_dma->p_parent);

    p_uart->p_dmatx->xfer_abort_callback = NULL;

    /* Check if an Abort process is still ongoing */
    if (NULL != p_uart->p_dmarx)
    {
        if (NULL != p_uart->p_dmarx->xfer_abort_callback)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
    p_uart->tx_xfer_count = 0U;
    p_uart->rx_xfer_count = 0U;

    /* Reset errorCode */
    p_uart->error_code = HAL_UART_ERROR_NONE;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Flush the whole TX FIFO (if needed) */
    __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

    /* Restore p_uart->gState and p_uart->RxState to Ready */
    p_uart->g_state  = HAL_UART_STATE_READY;
    p_uart->rx_state = HAL_UART_STATE_READY;

    /* Call user Abort complete callback */
    hal_uart_abort_cplt_callback(p_uart);
}


/**
  * @brief  DMA UART Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_rx_abort_callback(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t* )(p_dma->p_parent);

    p_uart->p_dmarx->xfer_abort_callback = NULL;

    /* Check if an Abort process is still ongoing */
    if (NULL != p_uart->p_dmatx)
    {
        if (NULL != p_uart->p_dmatx->xfer_abort_callback)
        {
            return;
        }
    }

    /* No Abort process still ongoing : All DMA channels are aborted, call user Abort Complete callback */
    p_uart->tx_xfer_count = 0U;
    p_uart->rx_xfer_count = 0U;

    /* Reset errorCode */
    p_uart->error_code = HAL_UART_ERROR_NONE;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

    /* Restore p_uart->gState and p_uart->RxState to Ready */
    p_uart->g_state  = HAL_UART_STATE_READY;
    p_uart->rx_state = HAL_UART_STATE_READY;

    /* Call user Abort complete callback */
    hal_uart_abort_cplt_callback(p_uart);
}


/**
  * @brief  DMA UART Tx communication abort callback, when initiated by user by a call to
  *         hal_uart_abort_transmit_it API (Abort only Tx transfer)
  *         (This callback is executed at end of DMA Tx Abort procedure following user abort request,
  *         and leads to user Tx Abort Complete callback execution).
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_tx_only_abort_callback(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = (uart_handle_t*)(p_dma->p_parent);

    p_uart->tx_xfer_count = 0U;

    /* Flush the whole TX FIFO (if needed) */
    __HAL_UART_SEND_REQ(p_uart, UART_TXDATA_FLUSH_REQUEST);

    /* Restore p_uart->gState to Ready */
    p_uart->g_state = HAL_UART_STATE_READY;

    /* Call user Abort complete callback */
    hal_uart_abort_tx_cplt_callback(p_uart);
}

/**
  * @brief  DMA UART Rx communication abort callback, when initiated by user by a call to
  *         hal_uart_abort_receive_it API (Abort only Rx transfer)
  *         (This callback is executed at end of DMA Rx Abort procedure following user abort request,
  *         and leads to user Rx Abort Complete callback execution).
  * @param  p_dma DMA handle.
  * @retval None
  */
static void uart_dma_rx_only_abort_callback(dma_handle_t *p_dma)
{
    uart_handle_t *p_uart = ( uart_handle_t* )((dma_handle_t* )p_dma)->p_parent;

    p_uart->rx_xfer_count = 0U;

    /* Clear the Error flags in the LSR register */
    ll_uart_clear_line_status_flag(p_uart->p_instance);

    /* Discard the received data */
    __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);

    /* Restore p_uart->RxState to Ready */
    p_uart->rx_state = HAL_UART_STATE_READY;

    /* Call user Abort complete callback */
    hal_uart_abort_rx_cplt_callback(p_uart);
}

/**
  * @brief  Send an amount of data in interrupt mode.
  * @note   Function is called under interruption only, once
  *         interruptions have been enabled by hal_uart_transmit_it().
  * @param  p_uart UART handle.
  * @retval HAL status
  */
hal_status_t uart_transmit_it(uart_handle_t *p_uart)
{
    uint8_t curxfercnt = UART_TXFIFO_SIZE - ll_uart_get_tx_fifo_level(p_uart->p_instance);

    /* Check that a Tx process is ongoing */
    if (HAL_UART_STATE_BUSY_TX == p_uart->g_state)
    {
        if (0U == p_uart->tx_xfer_count)
        {
            uart_end_tx_transfer(p_uart);
            hal_uart_tx_cplt_callback(p_uart);
        }
        else
        {
            while ((0U != curxfercnt) && (0U != p_uart->tx_xfer_count))
            {
                ll_uart_transmit_data8(p_uart->p_instance, *p_uart->p_tx_buffer++);
                curxfercnt--;
                p_uart->tx_xfer_count--;
            }

            if (0U == p_uart->tx_xfer_count)
            {
                /* Set UART TX FIFO Threshold to EMPTY to make sure all data were sent out */
                ll_uart_set_tx_fifo_threshold(p_uart->p_instance, LL_UART_TX_FIFO_TH_EMPTY);
            }
        }
        return HAL_OK;
    }
    else
    {
        uart_end_tx_transfer(p_uart);
        return HAL_BUSY;
    }
}

/**
  * @brief  Receive an amount of data in interrupt mode.
  * @note   Function is called under interruption only, once
  *         interruptions have been enabled by hal_uart_receive_it()
  * @param  p_uart UART handle.
  * @param  cto_status Character Timeout status.
  * @retval HAL status
  */
hal_status_t uart_receive_it(uart_handle_t *p_uart, flag_status_t cto_status)
{
    uint8_t curxfercnt = ll_uart_get_rx_fifo_level(p_uart->p_instance);

    if ((ENABLE == p_uart->dma_rx_mode) && (RESET == cto_status))
        return HAL_OK;

    /* Check that a Rx process is ongoing */
    if (HAL_UART_STATE_BUSY_RX == p_uart->rx_state)
    {
        if ((RESET == cto_status) && (1 < curxfercnt))
            curxfercnt--;

        if (ENABLE == p_uart->dma_rx_mode)
        {
            uint32_t count = ll_dma_get_block_size(DMA, p_uart->p_dmarx->instance);
            p_uart->p_rx_buffer += count;
            p_uart->rx_xfer_count -= count;
        }

        while ((0U != curxfercnt) && (0U != p_uart->rx_xfer_count))
        {
            *p_uart->p_rx_buffer++ = ll_uart_receive_data8(p_uart->p_instance);
            curxfercnt--;
            p_uart->rx_xfer_count--;
        }

        if ((0U == p_uart->rx_xfer_count) || (SET == cto_status))
        {
            if (ENABLE == p_uart->dma_rx_mode)
            {
                uart_stop_dma_rx(p_uart);
            }
            else
            {
                uart_end_rx_transfer(p_uart);
            }

            hal_uart_rx_cplt_callback(p_uart);
        }

        return HAL_OK;
    }
    else
    {
        // /* Not in Rx state, clear RDA interrupt flag */
        // __HAL_UART_SEND_REQ(p_uart, UART_RXDATA_FLUSH_REQUEST);
        return HAL_BUSY;
    }
}

/** @} */

#endif /* HAL_UART_MODULE_ENABLED */

/** @} */

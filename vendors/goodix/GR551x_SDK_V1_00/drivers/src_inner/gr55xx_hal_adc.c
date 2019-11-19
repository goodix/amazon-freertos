/**
  ****************************************************************************************
  * @file    gr55xx_hal_adc.c
  * @author  BLE Driver Team
  * @brief   ADC HAL module driver.
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

#ifdef HAL_ADC_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void adc_dma_cplt(dma_handle_t *p_dma);
static void adc_dma_error(dma_handle_t *p_dma);
static hal_status_t adc_wait_notempty_until_timeout(adc_handle_t *p_adc, uint32_t tick_start, uint32_t timeout);

__WEAK hal_status_t hal_adc_init(adc_handle_t *p_adc)
{
    hal_status_t   status = HAL_OK;
    error_status_t err    = SUCCESS;

    /* Check the ADC handle allocation */
    if (NULL == p_adc)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_adc);

    if (HAL_ADC_STATE_RESET == p_adc->state)
    {
        /* Allocate lock resource and initialize it */
        p_adc->lock = HAL_UNLOCKED;

        /* Enable ADC Clock and Automatic turn off ADC clock during WFI. */
        ll_cgc_disable_force_off_snsadc_hclk();
        ll_cgc_disable_wfi_off_snsadc_hclk();

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_adc_msp_init(p_adc);
    }

    /* Disable the ADC peripheral */
    __HAL_ADC_DISABLE(p_adc);

    /* Configure ADC peripheral */
    err = ll_adc_init(&p_adc->init);

    if (SUCCESS == err)
    {
        /* Enable the ADC peripheral */
        __HAL_ADC_ENABLE(p_adc);

        /* Set ADC error code to none */
        p_adc->error_code = HAL_ADC_ERROR_NONE;

        /* Initialize the ADC state */
        p_adc->state = HAL_ADC_STATE_READY;
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_adc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_adc_deinit(adc_handle_t *p_adc)
{
    /* Check the ADC handle allocation */
    if (NULL == p_adc)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_adc);

    /* Reset ADC Peripheral */
    ll_adc_deinit();

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    hal_adc_msp_deinit(p_adc);

    /* Disable ADC Clock. */
    ll_cgc_enable_force_off_snsadc_hclk();
    ll_cgc_disable_wfi_off_snsadc_hclk();

    /* Set ADC error code to none */
    p_adc->error_code = HAL_ADC_ERROR_NONE;

    /* Initialize the ADC state */
    p_adc->state = HAL_ADC_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_adc);

    return HAL_OK;
}

__WEAK void hal_adc_msp_init(adc_handle_t *p_adc)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_adc_msp_init can be implemented in the user file
     */
}

__WEAK void hal_adc_msp_deinit(adc_handle_t *p_adc)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_adc_msp_deinit can be implemented in the user file
     */
}

__WEAK hal_status_t hal_adc_conversion(adc_handle_t *p_adc, uint16_t *p_data, uint32_t length)
{
    hal_status_t status = HAL_OK;
    uint32_t     wcont  = length >> 1;
    uint32_t    *p_buf  = (uint32_t *)p_data;
    uint32_t     fifo_cnt;

    /* Process locked */
    __HAL_LOCK(p_adc);

    if (HAL_ADC_STATE_READY == p_adc->state)
    {
        p_adc->error_code = HAL_ADC_ERROR_NONE;

        /* Update ADC state */
        p_adc->state = HAL_ADC_STATE_BUSY;

        /* Flush FIFO and then enable adc clock */
        __HAL_ADC_FLUSH_FIFO(p_adc);
        __HAL_ADC_ENABLE_CLOCK(p_adc);

        while (wcont)
        {
            status = adc_wait_notempty_until_timeout(p_adc, hal_get_tick(), 1000);
            if (HAL_OK != status)
                break;

            fifo_cnt = ll_adc_get_fifo_count();
            fifo_cnt = fifo_cnt > wcont ? wcont : fifo_cnt;
            for (uint32_t i = 0; i < fifo_cnt; i++)
            {
                *p_buf++ = ll_adc_read_fifo();
            }
            wcont -= fifo_cnt;
        }

        if ((0 == wcont) && (length & 0x1))
        {
            if (HAL_OK == adc_wait_notempty_until_timeout(p_adc, hal_get_tick(), 1000))
            {
                *(uint16_t *)p_buf = ll_adc_read_fifo() & 0xFFFF;
            }
        }

        /* Disable ADC clock to stop conversion */
        __HAL_ADC_DISABLE_CLOCK(p_adc);

        p_adc->state = HAL_ADC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_adc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_adc_conversion_dma(adc_handle_t *p_adc, uint16_t *p_data, uint32_t length)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_adc);

    if (HAL_ADC_STATE_READY == p_adc->state)
    {
        p_adc->error_code = HAL_ADC_ERROR_NONE;

        /* Update ADC state */
        p_adc->state = HAL_ADC_STATE_BUSY;

        if (DMA_SDATAALIGN_WORD != p_adc->p_dma->init.src_data_alignment)
        {
            p_adc->error_code |= HAL_ADC_ERROR_INVALID_PARAM;
            status = HAL_ERROR;
        }

        if (HAL_OK == status)
        {
            /* Config FIFO thresh */
            ll_adc_set_thresh(16);
            ll_dma_set_source_burst_length(DMA, p_adc->p_dma->instance, LL_DMA_SRC_BURST_LENGTH_8);

            /* Configure counters and size of buffer */
            p_adc->buff_count = length;
            p_adc->buff_size  = p_adc->buff_count;
            p_adc->p_buffer   = p_data;

            /* Set the ADC DMA transfer complete callback */
            p_adc->p_dma->xfer_tfr_callback = adc_dma_cplt;

            /* Set the DMA error callback */
            p_adc->p_dma->xfer_error_callback = adc_dma_error;

            /* Clear the DMA abort callback */
            p_adc->p_dma->xfer_abort_callback = NULL;

            /* Flush FIFO and then enable adc clock */
            __HAL_ADC_FLUSH_FIFO(p_adc);
            __HAL_ADC_ENABLE_CLOCK(p_adc);

            /* Process unlocked */
            __HAL_UNLOCK(p_adc);

            /* Enable the ADC transmit DMA Channel */
            status = hal_dma_start_it(p_adc->p_dma, (uint32_t)&MCU_SUB->SENSE_ADC_FIFO, (uint32_t)p_adc->p_buffer, p_adc->buff_count >> 1);
        }
        else
        {
            /* Update ADC state */
            p_adc->state = HAL_ADC_STATE_READY;
            /* Process unlocked */
            __HAL_UNLOCK(p_adc);
        }
    }
    else
    {
        status = HAL_BUSY;
        /* Process unlocked */
        __HAL_UNLOCK(p_adc);
    }

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_adc_conversion_abort(adc_handle_t *p_adc)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_adc);

    /* Check if the state is in busy states */
    if (HAL_ADC_STATE_BUSY == p_adc->state)
    {
        /* Disable ADC clock to stop conversion */
        __HAL_ADC_DISABLE_CLOCK(p_adc);
        if (HAL_DMA_STATE_BUSY == p_adc->p_dma->state)
        {
            /* Abort DMA channel */
            status = hal_dma_abort(p_adc->p_dma);
            if (HAL_OK != status)
            {
                p_adc->error_code |= HAL_ADC_ERROR_DMA;
            }
        }

        /* Update state */
        p_adc->state = HAL_ADC_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_adc);

    return status;
}

__WEAK void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
    return;
}

__WEAK hal_adc_state_t hal_adc_get_state(adc_handle_t *p_adc)
{
    /* Return ADC handle state */
    return p_adc->state;
}

__WEAK uint32_t hal_adc_get_error(adc_handle_t *p_adc)
{
    return p_adc->error_code;
}

__WEAK hal_status_t hal_adc_set_dma_threshold(adc_handle_t *p_adc, uint32_t threshold)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_adc);

    if (HAL_ADC_STATE_READY == p_adc->state)
    {
        /* Configure ADC FIFO Threshold */
        ll_adc_set_thresh(threshold);
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_adc);

    /* Return function status */
    return status;
}

__WEAK uint32_t hal_adc_get_dma_threshold(adc_handle_t *p_adc)
{
    return ll_adc_get_thresh();
}

static void adc_dma_cplt(dma_handle_t *p_dma)
{
    adc_handle_t *p_adc = (adc_handle_t *)p_dma->p_parent;

    if (p_adc->buff_size & 0x1)
    {
        if (HAL_OK == adc_wait_notempty_until_timeout(p_adc, hal_get_tick(), 1000))
        {
            p_adc->p_buffer[p_adc->buff_size - 1] = ll_adc_read_fifo() & 0xFFFF;
        }
    }

    p_adc->buff_count = 0;
    /* Disable ADC clock */
    __HAL_ADC_DISABLE_CLOCK(p_adc);

    if (HAL_ADC_STATE_BUSY == p_adc->state)
    {
        /* Change state of ADC */
        p_adc->state = HAL_ADC_STATE_READY;
        hal_adc_conv_cplt_callback(p_adc);
    }
}

static void adc_dma_error(dma_handle_t *p_dma)
{
    adc_handle_t *p_adc = (adc_handle_t *)p_dma->p_parent;

    p_adc->error_code  |= HAL_ADC_ERROR_DMA;

    /* Abort the ADC */
    hal_adc_conversion_abort(p_adc);
}

static hal_status_t adc_wait_notempty_until_timeout(adc_handle_t *p_adc, uint32_t tick_start, uint32_t timeout)
{
    /* Wait until notempty flag is set */
    while (SET != (__HAL_ADC_GET_FLAG_NOTEMPTY(p_adc)))
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if ((0 == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                p_adc->state       = HAL_ADC_STATE_ERROR;
                p_adc->error_code |= HAL_ADC_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

#endif /* HAL_ADC_MODULE_ENABLED */


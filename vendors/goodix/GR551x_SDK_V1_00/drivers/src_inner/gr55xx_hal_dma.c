/**
  ****************************************************************************************
  * @file    gr55xx_hal_dma.c
  * @author  BLE Driver Team
  * @brief   DMA HAL module driver.
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

#ifdef HAL_DMA_MODULE_ENABLED

#define HAL_TIMEOUT_DMA_ABORT    ((uint32_t)1000U)  /* 1s  */

__WEAK hal_status_t hal_dma_init(dma_handle_t *p_dma)
{
    ll_dma_init_t dma_init;

    /* Check the DMA peripheral state */
    if (NULL == p_dma)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_DMA_ALL_INSTANCE(p_dma->instance));
    gr_assert_param(IS_DMA_ALL_REQUEST(p_dma->init.src_request));
    gr_assert_param(IS_DMA_ALL_REQUEST(p_dma->init.dst_request));
    gr_assert_param(IS_DMA_DIRECTION(p_dma->init.direction));
    gr_assert_param(IS_DMA_SOURCE_INC_STATE(p_dma->init.src_increment));
    gr_assert_param(IS_DMA_DESTINATION_INC_STATE(p_dma->init.dst_increment));
    gr_assert_param(IS_DMA_SOURCE_DATA_SIZE(p_dma->init.src_data_alignment));
    gr_assert_param(IS_DMA_DESTINATION_DATA_SIZE(p_dma->init.dst_data_alignment));
    gr_assert_param(IS_DMA_MODE(p_dma->init.mode));
    gr_assert_param(IS_DMA_PRIORITY(p_dma->init.priority));

    if (HAL_DMA_STATE_RESET == p_dma->state)
    {
        /* Allocate lock resource and initialize it */
        p_dma->lock = HAL_UNLOCKED;

        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_dma_hclk();
        ll_cgc_disable_wfi_off_dma_hclk();
    }

    /* Change DMA peripheral state */
    p_dma->state = HAL_DMA_STATE_BUSY;

    /* Set each dma_init_t field to default value. */
    ll_dma_struct_init(&dma_init);

    /* Prepare the DMA Channel configuration */
    dma_init.src_peripheral     = p_dma->init.src_request;
    dma_init.dst_peripheral     = p_dma->init.dst_request;
    dma_init.src_increment_mode = p_dma->init.src_increment;
    dma_init.dst_increment_mode =  p_dma->init.dst_increment;
    dma_init.src_data_width     = p_dma->init.src_data_alignment;
    dma_init.dst_data_width     = p_dma->init.dst_data_alignment;
    dma_init.direction          = p_dma->init.direction;
    dma_init.mode               = p_dma->init.mode;
    dma_init.priority           = p_dma->init.priority;

    /* Initialize the DMA registers according to the specified parameters in dma_init_t. */
    ll_dma_init(DMA, p_dma->instance, &dma_init);

    /* Initialise the error code */
    p_dma->error_code = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state*/
    p_dma->state  = HAL_DMA_STATE_READY;

    /* Allocate lock resource and initialize it */
    p_dma->lock = HAL_UNLOCKED;

    return HAL_OK;
}

__WEAK hal_status_t hal_dma_deinit(dma_handle_t *p_dma)
{
    /* Check the DMA peripheral state */
    if (NULL == p_dma)
    {
        return HAL_ERROR;
    }

    /* Check the DMA peripheral state */
    if (HAL_DMA_STATE_BUSY == p_dma->state)
    {
        return HAL_ERROR;
    }

    ll_dma_deinit(DMA, p_dma->instance);

    if(!ll_dma_is_enable(DMA))
    {
        /* Disable DMA clock. */
        ll_cgc_enable_force_off_dma_hclk();
        ll_cgc_disable_wfi_off_dma_hclk();
    }

    /* Initialise the error code */
    p_dma->error_code = HAL_DMA_ERROR_NONE;

    /* Initialize the DMA state */
    p_dma->state = HAL_DMA_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_dma);

    return HAL_OK;
}

__WEAK hal_status_t hal_dma_start(dma_handle_t *p_dma, uint32_t src_address, uint32_t dst_address, uint32_t data_length)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dma);

    if(HAL_DMA_STATE_READY == p_dma->state)
    {
        /* Change DMA peripheral state */
        p_dma->state = HAL_DMA_STATE_BUSY;

        /* Check the parameters */
        gr_assert_param(IS_DMA_BUFFER_SIZE(data_length));

        /* Clear the transfer complete status */
        ll_dma_clear_flag_tfr(DMA, p_dma->instance);
        /* Clear the block status */
        ll_dma_clear_flag_blk(DMA, p_dma->instance);
        /* Clear the transfer error status */
        ll_dma_clear_flag_err(DMA, p_dma->instance);

        /* Configure the source, destination address and the data length */
        ll_dma_config_address(DMA, p_dma->instance, src_address, dst_address, p_dma->init.direction);
        ll_dma_set_block_size(DMA, p_dma->instance, data_length);

        /* Enable the channle */
        ll_dma_enable_channel(DMA, p_dma->instance);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(p_dma);
        status = HAL_BUSY;
    }

    return status;
}

__WEAK hal_status_t hal_dma_start_it(dma_handle_t *p_dma, uint32_t src_address, uint32_t dst_address, uint32_t data_length)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dma);

    if(HAL_DMA_STATE_READY == p_dma->state)
    {
        /* Change DMA peripheral state */
        p_dma->state = HAL_DMA_STATE_BUSY;

        /* Check the parameters */
        gr_assert_param(IS_DMA_BUFFER_SIZE(data_length));

        /* Disable DMA channel interrupt */
        ll_dma_disable_it(DMA, p_dma->instance);

        /* Configure the source, destination address and the data length */
        ll_dma_config_address(DMA, p_dma->instance, src_address, dst_address, p_dma->init.direction);
        ll_dma_set_block_size(DMA, p_dma->instance, data_length);

        /* Enable the transfer complete interrupt */
        ll_dma_clear_flag_tfr(DMA, p_dma->instance);
        ll_dma_enable_it_tfr(DMA, p_dma->instance);

        /* Enable the block complete interrupt */
        ll_dma_clear_flag_blk(DMA, p_dma->instance);
        ll_dma_enable_it_blk(DMA, p_dma->instance);

        /* Enable the transfer Error interrupt */
        ll_dma_clear_flag_err(DMA, p_dma->instance);
        ll_dma_enable_it_err(DMA, p_dma->instance);

        /* Enable DMA channel interrupt */
        ll_dma_enable_it(DMA, p_dma->instance);

        /* Enable the channle */
        ll_dma_enable_channel(DMA, p_dma->instance);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(p_dma);

        /* Remain BUSY */
        status = HAL_BUSY;
    }
    return status;
}

__WEAK hal_status_t hal_dma_abort(dma_handle_t *p_dma)
{
    uint32_t tick_start = 0U;

    /* Suspend the channel */
    ll_dma_suspend_channel(DMA, p_dma->instance);

    /* Get timeout */
    tick_start = hal_get_tick();

    while (0 == ll_dma_is_empty_fifo(DMA, p_dma->instance))
    {
        /* Check for the Timeout */
        if (HAL_TIMEOUT_DMA_ABORT < (hal_get_tick()  - tick_start))
        {
            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(p_dma);

            /* Change the DMA state */
            p_dma->state = HAL_DMA_STATE_TIMEOUT;

            return HAL_TIMEOUT;
        }
    }

    /* Disable the channel */
    ll_dma_disable_channel(DMA, p_dma->instance);

    /* Get timeout */
    tick_start = hal_get_tick();

    /* Check if the DMA Channel is effectively disabled */
    while (ll_dma_is_enabled_channel(DMA, p_dma->instance))
    {
        /* Check for the Timeout */
        if (HAL_TIMEOUT_DMA_ABORT < (hal_get_tick()  - tick_start))
        {
            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_TIMEOUT;

            /* Process Unlocked */
            __HAL_UNLOCK(p_dma);

            /* Change the DMA state */
            p_dma->state = HAL_DMA_STATE_TIMEOUT;

            return HAL_TIMEOUT;
        }
    }

    /* Resume the channel */
    ll_dma_resume_channel(DMA, p_dma->instance);

    /* Process Unlocked */
    __HAL_UNLOCK(p_dma);

    /* Change the DMA state*/
    p_dma->state = HAL_DMA_STATE_READY;

    return HAL_OK;
}

__WEAK hal_status_t hal_dma_abort_it(dma_handle_t *p_dma)
{
    hal_status_t status = HAL_OK;

    if (HAL_DMA_STATE_BUSY != p_dma->state)
    {
        /* no transfer ongoing */
        p_dma->error_code = HAL_DMA_ERROR_NO_XFER;

        status = HAL_ERROR;
    }
    else
    {
        /* Disable the channle */
        ll_dma_disable_channel(DMA, p_dma->instance);
        while (ll_dma_is_enabled_channel(DMA, p_dma->instance));

        /* Mask interrupt bits for DMAx_Channely */
        ll_dma_disable_it_tfr(DMA, p_dma->instance);
        ll_dma_disable_it_blk(DMA, p_dma->instance);
        ll_dma_disable_it_srct(DMA, p_dma->instance);
        ll_dma_disable_it_dstt(DMA, p_dma->instance);
        ll_dma_disable_it_err(DMA, p_dma->instance);

        /* Reset interrupt pending bits for DMAx_Channely */
        ll_dma_clear_flag_tfr(DMA, p_dma->instance);
        ll_dma_clear_flag_blk(DMA, p_dma->instance);
        ll_dma_clear_flag_srct(DMA, p_dma->instance);
        ll_dma_clear_flag_dstt(DMA, p_dma->instance);
        ll_dma_clear_flag_err(DMA, p_dma->instance);

        /* Disable DMA channel interrupt */
        ll_dma_disable_it(DMA, p_dma->instance);

        /* Change the DMA state */
        p_dma->state = HAL_DMA_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(p_dma);

        /* Call User Abort callback */
        if (NULL != p_dma->xfer_abort_callback)
        {
            p_dma->xfer_abort_callback(p_dma);
        }
    }
    return status;
}

__WEAK hal_status_t hal_dma_poll_for_transfer(dma_handle_t *p_dma, uint32_t timeout)
{
    uint32_t tick_start = 0U;

    if (HAL_DMA_STATE_BUSY != p_dma->state)
    {
        /* no transfer ongoing */
        p_dma->error_code = HAL_DMA_ERROR_NO_XFER;
        __HAL_UNLOCK(p_dma);

        return HAL_ERROR;
    }
    /* Get timeout */
    tick_start = hal_get_tick();

    while ((0 == ll_dma_is_active_flag_rtfr(DMA, p_dma->instance)) && (0 == ll_dma_is_active_flag_rblk(DMA, p_dma->instance)))
    {
        if (ll_dma_is_active_flag_rerr(DMA, p_dma->instance))
        {
            /* Clear the transfer error flags */
            ll_dma_clear_flag_err(DMA, p_dma->instance);

            /* Update error code */
            SET_BITS(p_dma->error_code, HAL_DMA_ERROR_TE);

            /* Change the DMA state */
            p_dma->state= HAL_DMA_STATE_ERROR;

            /* Process Unlocked */
            __HAL_UNLOCK(p_dma);

            return HAL_ERROR;
        }

        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if ((0U == timeout)||(timeout < (hal_get_tick() - tick_start)))
            {
                /* Update error code */
                SET_BITS(p_dma->error_code, HAL_DMA_ERROR_TIMEOUT);

                /* Change the DMA state */
                p_dma->state= HAL_DMA_STATE_TIMEOUT;

                /* Process Unlocked */
                __HAL_UNLOCK(p_dma);

                return HAL_TIMEOUT;
            }
        }
    }

    /* Clear the transfer complete flag */
    ll_dma_clear_flag_tfr(DMA, p_dma->instance);

    /* Clear the block flag */
    ll_dma_clear_flag_blk(DMA, p_dma->instance);

    /* The selected Channelx EN bit is cleared (DMA is disabled and all transfers are complete) */
    p_dma->state = HAL_DMA_STATE_READY;

    /* Process unlocked */
    __HAL_UNLOCK(p_dma);

    return HAL_OK;
}

__WEAK void hal_dma_irq_handler(dma_handle_t *p_dma)
{
    /* Check dma handle state */
    if (HAL_DMA_STATE_RESET == p_dma->state)
    {
        return;
    }

    /* Transfer Error Interrupt management ***************************************/
    if (ll_dma_is_active_flag_err(DMA, p_dma->instance))
    {
        if (ll_dma_is_enable_it_err(DMA, p_dma->instance))
        {
            if (DMA_NORMAL == p_dma->init.mode)
            {
                /* Disable the transfer error interrupt */
                ll_dma_disable_it_err(DMA, p_dma->instance);
            }
            /* Clear the transfer error flag */
            ll_dma_clear_flag_err(DMA, p_dma->instance);

            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_TE;

            /* Change the DMA state */
            p_dma->state = HAL_DMA_STATE_ERROR;

            /* Process Unlocked */
            __HAL_UNLOCK(p_dma);

            if (NULL != p_dma->xfer_error_callback)
            {
                /* Transfer error callback */
                p_dma->xfer_error_callback(p_dma);
            }
        }
    }

    /* Transfer Complete Interrupt management ******************************/
    if (ll_dma_is_active_flag_tfr(DMA, p_dma->instance))
    {
        if (ll_dma_is_enable_it_tfr(DMA, p_dma->instance))
        {
            if (DMA_NORMAL == p_dma->init.mode)
            {
                /* Disable the transfer Complete interrupt */
                ll_dma_disable_it_tfr(DMA, p_dma->instance);
            }

            /* Clear the transfer Complete flag */
            ll_dma_clear_flag_tfr(DMA, p_dma->instance);

            /* Clear the transfer Complete flag */
            ll_dma_clear_flag_tfr(DMA, p_dma->instance);

            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_NONE;

            /* Change the DMA state */
            p_dma->state = HAL_DMA_STATE_READY;

            /* Process Unlocked */
            __HAL_UNLOCK(p_dma);

            if (NULL != p_dma->xfer_tfr_callback)
            {
                /* Transfer Complete callback */
                p_dma->xfer_tfr_callback(p_dma);
            }
        }
    }

    /* Block Complete Interrupt management ******************************/
    if (ll_dma_is_active_flag_blk(DMA, p_dma->instance))
    {
        if (ll_dma_is_enable_it_blk(DMA, p_dma->instance))
        {
            if (DMA_NORMAL == p_dma->init.mode)
            {
                /* Disable the block interrupt */
                ll_dma_disable_it_blk(DMA, p_dma->instance);

                /* Clear the block flag */
                ll_dma_clear_flag_blk(DMA, p_dma->instance);

                /* Change the DMA state */
                p_dma->state = HAL_DMA_STATE_READY;
            }

            /* Update error code */
            p_dma->error_code |= HAL_DMA_ERROR_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(p_dma);

            if (NULL != p_dma->xfer_blk_callback)
            {
                /* Transfer Complete callback */
                p_dma->xfer_blk_callback(p_dma);
            }

            if (DMA_NORMAL != p_dma->init.mode)
            {
                /* Clear the block flag */
                ll_dma_clear_flag_blk(DMA, p_dma->instance);
            }
        }
    }
}

__WEAK hal_status_t hal_dma_register_callback(dma_handle_t *p_dma, hal_dma_callback_id_t id, void (* callback)( dma_handle_t *p_dma))
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dma);

    if (HAL_DMA_STATE_READY == p_dma->state)
    {
        switch (id)
        {
        case  HAL_DMA_XFER_TFR_CB_ID:
            p_dma->xfer_tfr_callback = callback;
            break;

        case  HAL_DMA_XFER_BLK_CB_ID:
            p_dma->xfer_blk_callback = callback;
            break;

        case  HAL_DMA_XFER_ERROR_CB_ID:
            p_dma->xfer_error_callback = callback;
            break;

        case  HAL_DMA_XFER_ABORT_CB_ID:
            p_dma->xfer_abort_callback = callback;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_dma);

    return status;
}

__WEAK hal_status_t hal_dma_unregister_callback(dma_handle_t *p_dma, hal_dma_callback_id_t id)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_dma);

    if (HAL_DMA_STATE_READY == p_dma->state)
    {
        switch (id)
        {
        case  HAL_DMA_XFER_TFR_CB_ID:
            p_dma->xfer_tfr_callback = NULL;
            break;

        case  HAL_DMA_XFER_BLK_CB_ID:
            p_dma->xfer_blk_callback = NULL;
            break;

        case  HAL_DMA_XFER_ERROR_CB_ID:
            p_dma->xfer_error_callback = NULL;
            break;

        case  HAL_DMA_XFER_ABORT_CB_ID:
            p_dma->xfer_abort_callback = NULL;
            break;

        case   HAL_DMA_XFER_ALL_CB_ID:
            p_dma->xfer_tfr_callback   = NULL;
            p_dma->xfer_blk_callback   = NULL;
            p_dma->xfer_error_callback = NULL;
            p_dma->xfer_abort_callback = NULL;
            break;

        default:
            status = HAL_ERROR;
            break;
        }
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_dma);

    return status;
}

__WEAK hal_dma_state_t hal_dma_get_state(dma_handle_t *p_dma)
{
    return p_dma->state;
}

__WEAK uint32_t hal_dma_get_error(dma_handle_t *p_dma)
{
    return p_dma->error_code;
}

#endif


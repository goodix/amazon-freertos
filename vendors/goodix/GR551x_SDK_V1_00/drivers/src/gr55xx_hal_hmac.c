/**
  ****************************************************************************************
  * @file    gr55xx_hal_hmac.c
  * @author  BLE Driver Team
  * @brief   HMAC HAL module driver.
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

#include "gr55xx_hal.h"

#ifdef HAL_HMAC_MODULE_ENABLED

static hal_status_t hmac_wait_flag_state_until_timeout(hmac_handle_t *p_hmac, uint32_t flag, \
        flag_status_t state, uint32_t tick_start, uint32_t timeout);
static hal_status_t hmac_config(hmac_handle_t *p_hmac);
static void hmac_read_data(hmac_handle_t *p_hmac, uint32_t *p_data);
static void hmac_write_data(hmac_handle_t *p_hmac, uint32_t *p_data);
static hal_status_t hmac_mcu_process(hmac_handle_t *p_hmac, uint32_t timeout);
static hal_status_t hmac_dma_process(hmac_handle_t *p_hmac, uint32_t timeout);


__WEAK hal_status_t hal_hmac_init(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    /* Check the HMAC handle allocation */
    if (NULL == p_hmac)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_HMAC_ALL_INSTANCE(p_hmac->p_instance));

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_RESET == p_hmac->state)
    {
        /* Allocate lock resource and initialize it */
        p_hmac->lock = HAL_UNLOCKED;

        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
        ll_cgc_disable_wfi_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* Init the low level hardware : CLOCK, NVIC */
        hal_hmac_msp_init(p_hmac);

        /* Configure the default timeout for the encryption or decryption */
        hal_hmac_set_timeout(p_hmac, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
    }

    status = hmac_config(p_hmac);

    if (HAL_OK == status)
    {
        /* Set HMAC error code to none */
        p_hmac->error_code = HAL_HMAC_ERROR_NONE;

        /* Initialize the HMAC state */
        p_hmac->state = HAL_HMAC_STATE_READY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_hmac);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_hmac_deinit(hmac_handle_t *p_hmac)
{
    /* Check the HMAC handle allocation */
    if (NULL == p_hmac)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_hmac);

    /* Disable the HMAC Peripheral Clock */
    ll_hmac_deinit(p_hmac->p_instance);

    /* DeInit the low level hardware: CLOCK, NVIC... */
    hal_hmac_msp_deinit(p_hmac);

    /* Set HMAC error code to none */
    p_hmac->error_code = HAL_HMAC_ERROR_NONE;

    /* Initialize the HMAC state */
    p_hmac->state = HAL_HMAC_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_hmac);

    return HAL_OK;
}

__WEAK void hal_hmac_msp_init(hmac_handle_t *p_hmac)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_hmac_msp_init can be implemented in the user file
     */
}

__WEAK void hal_hmac_msp_deinit(hmac_handle_t *p_hmac)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_hmac_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_hmac_irq_handler(hmac_handle_t *p_hmac)
{
    __HAL_HMAC_CLEAR_FLAG_IT(p_hmac, HMAC_IT_DONE);
    /* Check if DMA transfer error occurred */
    if (RESET != __HAL_HMAC_GET_FLAG(p_hmac, HMAC_FLAG_DMA_ERR))
    {
        __HAL_HMAC_DISABLE(p_hmac);
        ll_hmac_disable_dma_start(p_hmac->p_instance);

        __HAL_HMAC_DISABLE_IT(p_hmac);

        if (HAL_HMAC_STATE_BUSY == p_hmac->state)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_TRANSFER;
            p_hmac->state = HAL_HMAC_STATE_READY;

            hal_hmac_error_callback(p_hmac);
        }

        return;
    }

    /* Done in SHA/HMAC mode */
    if ((RESET != __HAL_HMAC_GET_FLAG(p_hmac, HMAC_FLAG_DATAREADY_SHA)) || \
            (RESET != __HAL_HMAC_GET_FLAG(p_hmac, HMAC_FLAG_DATAREADY_HMAC)))
    {
        if (ll_hmac_is_enabled_dma_start(p_hmac->p_instance))
        {
            /* DMA mode */
            if (1 != p_hmac->is_last_trans)
            {
                p_hmac->block_count = 0;
                ll_hmac_disable_dma_start(p_hmac->p_instance);
                __HAL_HMAC_DISABLE_IT(p_hmac);

                if (HAL_HMAC_STATE_BUSY == p_hmac->state)
                {
                    p_hmac->state = HAL_HMAC_STATE_SUSPENDED;
                    hal_hmac_done_callback(p_hmac);
                }
            }
            else if (1 < p_hmac->block_count)
            {
                p_hmac->block_count = 1;
                ll_hmac_disable_dma_start(p_hmac->p_instance);

                ll_hmac_set_dma_transfer_block(p_hmac->p_instance, 1);
                ll_hmac_set_dma_read_address(p_hmac->p_instance, (uint32_t)&p_hmac->p_message[(p_hmac->block_size - 1) << 4]);
                p_hmac->p_instance->CTRL |= 0xA;
            }
            else
            {
                p_hmac->block_count = 0;
                __HAL_HMAC_DISABLE(p_hmac);
                ll_hmac_disable_dma_start(p_hmac->p_instance);
                __HAL_HMAC_DISABLE_IT(p_hmac);

                if (HAL_HMAC_STATE_BUSY == p_hmac->state)
                {
                    p_hmac->state = HAL_HMAC_STATE_READY;
                    hal_hmac_done_callback(p_hmac);
                }
            }
        }
        else
        {
            /* MCU mode */
            p_hmac->block_count--;
            if (0 < p_hmac->block_count)
            {
                if ((1 == p_hmac->block_count) && (1 == p_hmac->is_last_trans))
                    ll_hmac_enable_last_transfer(p_hmac->p_instance);

                hmac_write_data(p_hmac, p_hmac->p_message);
                p_hmac->p_message += HMAC_BLOCKSIZE_WORDS;
            }
            else
            {
                if (1 == p_hmac->is_last_trans)
                    __HAL_HMAC_DISABLE(p_hmac);
                __HAL_HMAC_DISABLE_IT(p_hmac);

                if (HAL_HMAC_STATE_BUSY == p_hmac->state)
                {
                    if (1 == p_hmac->is_last_trans)
                    {
                        p_hmac->state = HAL_HMAC_STATE_READY;
                        hmac_read_data(p_hmac, p_hmac->p_digest);
                    }
                    else
                    {
                        p_hmac->state = HAL_HMAC_STATE_SUSPENDED;
                    }
                    hal_hmac_done_callback(p_hmac);
                }
            }
        }
    }
}

__WEAK hal_status_t hal_sha256_digest(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_SHA;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_mcu_process(p_hmac, ((timeout > 0) ? timeout : 1));
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_start(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_HMAC;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = NULL;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 0;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_mcu_process(p_hmac, ((timeout > 0) ? timeout : 1));
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_continue(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t timeout)
{
    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_SUSPENDED == p_hmac->state)
    {
        p_hmac->p_message     = p_message;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 0;

        return hmac_mcu_process(p_hmac, ((timeout > 0) ? timeout : 1));
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_finish(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *digest, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_SUSPENDED == p_hmac->state)
    {
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;

        return hmac_mcu_process(p_hmac, timeout);
    }
    else if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_HMAC;
        p_hmac->p_message       = p_message;
        p_hmac->p_digest        = digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_mcu_process(p_hmac, ((timeout > 0) ? timeout : 1));
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_sha256_digest_it(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_SHA;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_mcu_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_start_it(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_HMAC;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = NULL;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 0;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_mcu_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_continue_it(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number)
{
    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_SUSPENDED == p_hmac->state)
    {
        p_hmac->p_message     = p_message;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 0;

        return hmac_mcu_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_finish_it(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_SUSPENDED == p_hmac->state)
    {
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;

        return hmac_mcu_process(p_hmac, 0);
    }
    else if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_HMAC;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_mcu_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_sha256_digest_dma(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *digest)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_SHA;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_dma_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_start_dma(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_HMAC;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = NULL;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 0;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_dma_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_continue_dma(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number)
{
    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_SUSPENDED == p_hmac->state)
    {
        p_hmac->p_message     = p_message;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 0;

        return hmac_dma_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_hmac_sha256_digest_finish_dma(hmac_handle_t *p_hmac, uint32_t *p_message, uint32_t number, uint32_t *p_digest)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_hmac);

    if (HAL_HMAC_STATE_SUSPENDED == p_hmac->state)
    {
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;

        return hmac_dma_process(p_hmac, 0);
    }
    else if (HAL_HMAC_STATE_READY == p_hmac->state)
    {
        p_hmac->init.mode     = HMAC_MODE_HMAC;
        p_hmac->p_message     = p_message;
        p_hmac->p_digest      = p_digest;
        p_hmac->block_size    = number >> 6;
        p_hmac->block_count   = number >> 6;
        p_hmac->is_last_trans = 1;
        status = hmac_config(p_hmac);

        if (HAL_OK != status)
        {
            p_hmac->error_code = HAL_HMAC_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }

        return hmac_dma_process(p_hmac, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return HAL_BUSY;
}

__WEAK void hal_hmac_done_callback(hmac_handle_t *p_hmac)
{

}

__WEAK void hal_hmac_error_callback(hmac_handle_t *p_hmac)
{

}

__WEAK void hal_hmac_abort_cplt_callback(hmac_handle_t *p_hmac)
{

}

__WEAK hal_hmac_state_t hal_hmac_get_state(hmac_handle_t *p_hmac)
{
    return p_hmac->state;
}

__WEAK uint32_t hal_hmac_get_error(hmac_handle_t *p_hmac)
{
    return p_hmac->error_code;
}

__WEAK hal_status_t hal_hmac_abort(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    return status;
}

__WEAK hal_status_t hal_hmac_abort_it(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    return status;
}

__WEAK void hal_hmac_set_timeout(hmac_handle_t *p_hmac, uint32_t timeout)
{
    p_hmac->timeout = timeout;
}

static hal_status_t hmac_wait_flag_state_until_timeout(hmac_handle_t *p_hmac, uint32_t flag, \
        flag_status_t state, uint32_t tick_start, uint32_t timeout)
{
    /* Wait until flag is in expected state */
    while ((__HAL_HMAC_GET_FLAG(p_hmac, flag)) != state)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if((0 == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                p_hmac->state      = HAL_HMAC_STATE_ERROR;
                p_hmac->error_code |= HAL_HMAC_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static hal_status_t hmac_config(hmac_handle_t *p_hmac)
{
    hal_status_t status = HAL_OK;

    ll_hmac_disable(p_hmac->p_instance);
    ll_hmac_disable_dma_start(p_hmac->p_instance);

    do {
        /* Set SHA or HMAC */
        if (HMAC_MODE_SHA == p_hmac->init.mode)
        {
            ll_hmac_enable_sha(p_hmac->p_instance);
        }
        else
        {
            ll_hmac_disable_sha(p_hmac->p_instance);
            /* Set key */
            if (NULL == p_hmac->init.p_key)
            {
                status = HAL_ERROR;
                break;
            }

            ll_hmac_set_key0(p_hmac->p_instance, p_hmac->init.p_key[0]);
            ll_hmac_set_key1(p_hmac->p_instance, p_hmac->init.p_key[1]);
            ll_hmac_set_key2(p_hmac->p_instance, p_hmac->init.p_key[2]);
            ll_hmac_set_key3(p_hmac->p_instance, p_hmac->init.p_key[3]);
            ll_hmac_set_key4(p_hmac->p_instance, p_hmac->init.p_key[4]);
            ll_hmac_set_key5(p_hmac->p_instance, p_hmac->init.p_key[5]);
            ll_hmac_set_key6(p_hmac->p_instance, p_hmac->init.p_key[6]);
            ll_hmac_set_key7(p_hmac->p_instance, p_hmac->init.p_key[7]);
        }
        /* Set initial HASH in user HASH mode */
        if (NULL != p_hmac->init.p_user_hash)
        {
            ll_hmac_enable_user_hash(p_hmac->p_instance);
            ll_hmac_set_user_hash_255_224(p_hmac->p_instance, p_hmac->init.p_user_hash[0]);
            ll_hmac_set_user_hash_223_192(p_hmac->p_instance, p_hmac->init.p_user_hash[1]);
            ll_hmac_set_user_hash_191_160(p_hmac->p_instance, p_hmac->init.p_user_hash[2]);
            ll_hmac_set_user_hash_159_128(p_hmac->p_instance, p_hmac->init.p_user_hash[3]);
            ll_hmac_set_user_hash_127_96 (p_hmac->p_instance, p_hmac->init.p_user_hash[4]);
            ll_hmac_set_user_hash_95_64  (p_hmac->p_instance, p_hmac->init.p_user_hash[5]);
            ll_hmac_set_user_hash_63_32  (p_hmac->p_instance, p_hmac->init.p_user_hash[6]);
            ll_hmac_set_user_hash_31_0   (p_hmac->p_instance, p_hmac->init.p_user_hash[7]);
        }
        else
        {
            ll_hmac_disable_user_hash(p_hmac->p_instance);
        }
        /* Set DPA Resistence */
        if (ENABLE == p_hmac->init.dpa_mode)
        {
            ll_hmac_enable_private(p_hmac->p_instance);
        }
        else
        {
            ll_hmac_disable_private(p_hmac->p_instance);
        }
        /* Set little endian */
        ll_hmac_enable_little_endian(p_hmac->p_instance);
        /* Set fetch key from MCU */
        ll_hmac_set_key_type(p_hmac->p_instance, LL_HMAC_KEYTYPE_MCU);
        /* Disable interrupt and clear flag */
        p_hmac->p_instance->INTERRUPT = 1;
    } while(0);

    return status;
}

static void hmac_read_data(hmac_handle_t *p_hmac, uint32_t *p_data)
{
    for (uint32_t i = 0; i < HMAC_DIGESTSIZE_WORDS; i++)
    {
        p_data[i] = p_hmac->p_instance->FIFO_OUT;
    }
}

static void hmac_write_data(hmac_handle_t *p_hmac, uint32_t *p_data)
{
    for (uint32_t i = 0; i < HMAC_BLOCKSIZE_WORDS; i++)
    {
        p_hmac->p_instance->MESSAGE_FIFO = p_data[i];
    }
}

static hal_status_t hmac_mcu_process(hmac_handle_t *p_hmac, uint32_t Timeout)
{
    hal_status_t status      = HAL_OK;
    uint32_t     tickstart   = hal_get_tick();
    uint32_t     last_status = p_hmac->state;

    p_hmac->state = HAL_HMAC_STATE_BUSY;

    ll_hmac_set_dma_read_address(p_hmac->p_instance, (uint32_t)p_hmac->p_message & 0x3);
    ll_hmac_set_dma_write_address(p_hmac->p_instance, (uint32_t)p_hmac->p_digest & 0x3);

    if (HAL_HMAC_STATE_READY == last_status)
        __HAL_HMAC_ENABLE(p_hmac);

    if ((HMAC_MODE_HMAC == p_hmac->init.mode) && (HAL_HMAC_STATE_READY == last_status))
    {
        status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_SHA, SET, tickstart, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
        if (HAL_OK != status)
        {
            __HAL_HMAC_DISABLE(p_hmac);

            p_hmac->error_code = HAL_HMAC_ERROR_TIMEOUT;
            p_hmac->state = HAL_HMAC_STATE_READY;

            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }
    }

    if (0 < Timeout)
    {
        while (0 < p_hmac->block_count)
        {
            if ((1 == p_hmac->block_count) && (1 == p_hmac->is_last_trans))
                ll_hmac_enable_last_transfer(p_hmac->p_instance);

            hmac_write_data(p_hmac, p_hmac->p_message);
            p_hmac->p_message += HMAC_BLOCKSIZE_WORDS;

            if ((1 == p_hmac->block_count) && (1 == p_hmac->is_last_trans))
                status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_HMAC, SET, tickstart, Timeout);
            else
                status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_SHA, SET, tickstart, Timeout);

            if (HAL_OK != status)
                break;

            p_hmac->block_count--;
        }

        if (HAL_OK != status)
        {
            __HAL_HMAC_DISABLE(p_hmac);

            p_hmac->error_code = HAL_HMAC_ERROR_TIMEOUT;
            p_hmac->state      = HAL_HMAC_STATE_READY;
        }
        else if (1 == p_hmac->is_last_trans)
        {
            hmac_read_data(p_hmac, p_hmac->p_digest);

            __HAL_HMAC_DISABLE(p_hmac);
            p_hmac->state = HAL_HMAC_STATE_READY;
        }
        else
        {
            p_hmac->state = HAL_HMAC_STATE_SUSPENDED;
        }
    }
    else
    {
        __HAL_HMAC_ENABLE_IT(p_hmac);

        if ((1 == p_hmac->block_count) && (1 == p_hmac->is_last_trans))
            ll_hmac_enable_last_transfer(p_hmac->p_instance);

        hmac_write_data(p_hmac, p_hmac->p_message);
        p_hmac->p_message += HMAC_BLOCKSIZE_WORDS;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return status;
}

static hal_status_t hmac_dma_process(hmac_handle_t *p_hmac, uint32_t Timeout)
{
    hal_status_t status      = HAL_OK;
    uint32_t     tickstart   = hal_get_tick();
    uint32_t     last_status = p_hmac->state;

    p_hmac->state = HAL_HMAC_STATE_BUSY;

    if ((1 == p_hmac->block_size) || (1 != p_hmac->is_last_trans))
        ll_hmac_set_dma_transfer_block(p_hmac->p_instance, p_hmac->block_size);
    else
        ll_hmac_set_dma_transfer_block(p_hmac->p_instance, p_hmac->block_size - 1);
    ll_hmac_set_dma_read_address(p_hmac->p_instance, (uint32_t)p_hmac->p_message);
    ll_hmac_set_dma_write_address(p_hmac->p_instance, (uint32_t)p_hmac->p_digest);

    if (HAL_HMAC_STATE_READY == last_status)
        __HAL_HMAC_ENABLE(p_hmac);

    if ((HMAC_MODE_HMAC == p_hmac->init.mode) && (HAL_HMAC_STATE_READY == last_status))
    {
        status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DATAREADY_SHA, SET, tickstart, HAL_HMAC_TIMEOUT_DEFAULT_VALUE);
        if (HAL_OK != status)
        {
            __HAL_HMAC_DISABLE(p_hmac);

            p_hmac->error_code = HAL_HMAC_ERROR_TIMEOUT;
            p_hmac->state = HAL_HMAC_STATE_READY;

            /* Process unlocked */
            __HAL_UNLOCK(p_hmac);

            return status;
        }
    }

    if (0 < Timeout)
    {
        while (0 < p_hmac->block_count)
        {
            if ((1 == p_hmac->block_count) && (1 == p_hmac->is_last_trans))
                p_hmac->p_instance->CTRL |= 0xA;
            else
                ll_hmac_enable_dma_start(p_hmac->p_instance);

            if (1 != p_hmac->is_last_trans)
            {
                status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DMA_MESSAGEDONE, SET, tickstart, Timeout);
                p_hmac->block_count = 0;
                ll_hmac_disable_dma_start(p_hmac->p_instance);
            }
            else if (1 == p_hmac->block_count)
            {
                status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DMA_DONE, SET, tickstart, Timeout);
                p_hmac->block_count--;
            }
            else
            {
                status = hmac_wait_flag_state_until_timeout(p_hmac, HMAC_FLAG_DMA_MESSAGEDONE, SET, tickstart, Timeout);
                p_hmac->block_count = 1;
                ll_hmac_disable_dma_start(p_hmac->p_instance);

                ll_hmac_set_dma_transfer_block(p_hmac->p_instance, 1);
                ll_hmac_set_dma_read_address(p_hmac->p_instance, (uint32_t)&p_hmac->p_message[(p_hmac->block_size - 1) << 4]);
            }

            if (HAL_OK != status)
                break;
        }

        if (HAL_OK != status)
        {
            __HAL_HMAC_DISABLE(p_hmac);
            ll_hmac_disable_dma_start(p_hmac->p_instance);

            p_hmac->error_code = HAL_HMAC_ERROR_TIMEOUT;
            p_hmac->state = HAL_HMAC_STATE_READY;
        }
        else if (1 == p_hmac->is_last_trans)
        {
            __HAL_HMAC_DISABLE(p_hmac);
            ll_hmac_disable_dma_start(p_hmac->p_instance);

            p_hmac->state = HAL_HMAC_STATE_READY;
        }
        else
        {
            p_hmac->state = HAL_HMAC_STATE_SUSPENDED;
        }
    }
    else
    {
        __HAL_HMAC_ENABLE_IT(p_hmac);

        if ((1 == p_hmac->block_count) && (1 == p_hmac->is_last_trans))
            p_hmac->p_instance->CTRL |= 0xA;
        else
            ll_hmac_enable_dma_start(p_hmac->p_instance);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_hmac);

    return status;
}

#endif /* HAL_HMAC_MODULE_ENABLED */


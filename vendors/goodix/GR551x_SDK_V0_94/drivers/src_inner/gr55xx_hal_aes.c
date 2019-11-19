/**
  ****************************************************************************************
  * @file    gr55xx_hal_aes.c
  * @author  BLE Driver Team
  * @brief   AES HAL module driver.
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

#ifdef HAL_AES_MODULE_ENABLED

static hal_status_t aes_wait_flag_state_until_timeout(aes_handle_t *p_aes,
                                                      uint32_t      flag,
                                                      flag_status_t state,
                                                      uint32_t      tick_start,
                                                      uint32_t      timeout);
static hal_status_t aes_config(aes_handle_t *p_aes);
static void aes_read_data(aes_handle_t *p_aes, uint32_t *p_data);
static void aes_write_data(aes_handle_t *p_aes, uint32_t *p_data);
static hal_status_t aes_mcu_process(aes_handle_t *p_aes, uint32_t timeout);
static hal_status_t aes_dma_process(aes_handle_t *p_aes, uint32_t timeout);

__WEAK hal_status_t hal_aes_init(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_AES_ALL_INSTANCE(p_aes->p_instance));
    gr_assert_param(IS_AES_KEY_SIZE(p_aes->init.key_size));
    gr_assert_param(IS_AES_OPERATION_MODE(p_aes->init.operation_mode));
    gr_assert_param(IS_AES_CHAININGMODE(p_aes->init.chaining_mode));

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_RESET == p_aes->state)
    {
        /* Allocate lock resource and initialize it */
        p_aes->lock = HAL_UNLOCKED;

        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
        ll_cgc_disable_wfi_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* Init the low level hardware : CLOCK, NVIC */
        hal_aes_msp_init(p_aes);

        /* Configure the default timeout for the encryption or decryption */
        hal_aes_set_timeout(p_aes, HAL_AES_TIMEOUT_DEFAULT_VALUE);
    }

    /* Enable AES */
    __HAL_AES_ENABLE(p_aes);

    status = aes_config(p_aes);

    if (HAL_OK == status)
    {
        /* Set AES error code to none */
        p_aes->error_code = HAL_AES_ERROR_NONE;

        /* Initialize the AES state */
        p_aes->state = HAL_AES_STATE_READY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_aes);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_aes_deinit(aes_handle_t *p_aes)
{
    /* Check the AES handle allocation */
    if (NULL == p_aes)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_aes);

    /* Disable the AES Peripheral Clock */
    ll_aes_deinit(p_aes->p_instance);

    /* DeInit the low level hardware: CLOCK, NVIC... */
    hal_aes_msp_deinit(p_aes);

    /* Set AES error code to none */
    p_aes->error_code = HAL_AES_ERROR_NONE;

    /* Initialize the AES state */
    p_aes->state = HAL_AES_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_aes);

    return HAL_OK;
}

__WEAK void hal_aes_msp_init(aes_handle_t *p_aes)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_aes_msp_init can be implemented in the user file
     */
}

__WEAK void hal_aes_msp_deinit(aes_handle_t *p_aes)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_aes_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_aes_irq_handler(aes_handle_t *p_aes)
{
    __HAL_AES_CLEAR_FLAG_IT(p_aes, AES_IT_DONE);
    /* Check if DMA transfer error occurred */
    if (RESET != __HAL_AES_GET_FLAG(p_aes, AES_FLAG_DMA_ERR))
    {
        ll_aes_disable_dma_start(p_aes->p_instance);

        __HAL_AES_DISABLE_IT(p_aes);

        if (HAL_AES_STATE_BUSY == p_aes->state)
        {
            p_aes->error_code = HAL_AES_ERROR_TRANSFER;
            p_aes->state = HAL_AES_STATE_READY;

            hal_aes_error_callback(p_aes);
        }

        return;
    }

    /* DMA Mode done */
    if (RESET != __HAL_AES_GET_FLAG(p_aes, AES_FLAG_DMA_DONE))
    {
        ll_aes_disable_dma_start(p_aes->p_instance);

        __HAL_AES_DISABLE_IT(p_aes);

        if (HAL_AES_STATE_BUSY == p_aes->state)
        {
            p_aes->state = HAL_AES_STATE_READY;
            hal_aes_done_callback(p_aes);
        }
    }

    if (RESET != __HAL_AES_GET_FLAG(p_aes, AES_FLAG_DATAREADY))
    {
        aes_read_data(p_aes, p_aes->p_cryp_output_buffer);
        p_aes->p_cryp_output_buffer += AES_BLOCKSIZE_WORDS;

        p_aes->block_count--;
        ll_aes_disable_start(p_aes->p_instance);

        if (0 < p_aes->block_count)
        {
            aes_write_data(p_aes, p_aes->p_cryp_input_buffer);
            p_aes->p_cryp_input_buffer += AES_BLOCKSIZE_WORDS;

            ll_aes_enable_start(p_aes->p_instance);
        }
        else
        {
            __HAL_AES_DISABLE_IT(p_aes);

            if (HAL_AES_STATE_BUSY == p_aes->state)
            {
                p_aes->state = HAL_AES_STATE_READY;
                hal_aes_done_callback(p_aes);
            }
        }
    }
}

__WEAK hal_status_t hal_aes_ecb_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_ENCRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, timeout);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_ecb_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_DECRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, timeout);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_cbc_encrypt(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_ENCRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, timeout);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_cbc_decrypt(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_DECRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, timeout);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_ecb_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_ENCRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_ecb_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_DECRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_cbc_encrypt_it(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_ENCRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_cbc_decrypt_it(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_DECRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_mcu_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_ecb_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_ENCRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_dma_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_ecb_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_DECRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_ECB;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_dma_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_cbc_encrypt_dma(aes_handle_t *p_aes, uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_ENCRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_plain_data;
        p_aes->p_cryp_output_buffer = p_cypher_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_dma_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK hal_status_t hal_aes_cbc_decrypt_dma(aes_handle_t *p_aes, uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_aes);

    if (HAL_AES_STATE_READY == p_aes->state)
    {
        p_aes->init.operation_mode  = AES_OPERATION_MODE_DECRYPT;
        p_aes->init.chaining_mode   = AES_CHAININGMODE_CBC;
        p_aes->p_cryp_input_buffer  = p_cypher_data;
        p_aes->p_cryp_output_buffer = p_plain_data;
        p_aes->block_size           = number >> 4;
        p_aes->block_count          = number >> 4;
        status = aes_config(p_aes);

        if (HAL_OK != status)
        {
            p_aes->error_code = HAL_AES_ERROR_INVALID_PARAM;
            /* Process unlocked */
            __HAL_UNLOCK(p_aes);

            return status;
        }

        return aes_dma_process(p_aes, 0);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return HAL_BUSY;
}

__WEAK void hal_aes_done_callback(aes_handle_t *p_aes)
{

}

__WEAK void hal_aes_error_callback(aes_handle_t *p_aes)
{

}

__WEAK void hal_aes_abort_cplt_callback(aes_handle_t *p_aes)
{

}

__WEAK hal_aes_state_t hal_aes_get_state(aes_handle_t *p_aes)
{
    return p_aes->state;
}

__WEAK uint32_t hal_aes_get_error(aes_handle_t *p_aes)
{
    return p_aes->error_code;
}

__WEAK hal_status_t hal_aes_abort(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    return status;
}

__WEAK hal_status_t hal_aes_abort_it(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    return status;
}

__WEAK void hal_aes_set_timeout(aes_handle_t *p_aes, uint32_t timeout)
{
    p_aes->timeout = timeout;
}

static hal_status_t aes_wait_flag_state_until_timeout(aes_handle_t *p_aes,
                                                      uint32_t      flag,
                                                      flag_status_t state,
                                                      uint32_t      tick_start,
                                                      uint32_t      timeout)
{
    /* Wait until flag is in expected state */
    while ((__HAL_AES_GET_FLAG(p_aes, flag)) != state)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if((0U == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                p_aes->state     = HAL_AES_STATE_ERROR;
                p_aes->error_code |= HAL_AES_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static hal_status_t aes_config(aes_handle_t *p_aes)
{
    hal_status_t status = HAL_OK;

    ll_aes_disable_start(p_aes->p_instance);
    ll_aes_disable_dma_start(p_aes->p_instance);

    do {
        /* Set key */
        ll_aes_set_key_size(p_aes->p_instance, p_aes->init.key_size);
        if (NULL == p_aes->init.p_key)
        {
            status = HAL_ERROR;
            break;
        }
        switch (p_aes->init.key_size)
        {
        case AES_KEYSIZE_256BITS:
            ll_aes_set_key_31_0   (p_aes->p_instance, p_aes->init.p_key[7]);
            ll_aes_set_key_63_32  (p_aes->p_instance, p_aes->init.p_key[6]);
        case AES_KEYSIZE_192BITS:
            ll_aes_set_key_95_64  (p_aes->p_instance, p_aes->init.p_key[5]);
            ll_aes_set_key_127_96 (p_aes->p_instance, p_aes->init.p_key[4]);
        case AES_KEYSIZE_128BITS:
            ll_aes_set_key_159_128(p_aes->p_instance, p_aes->init.p_key[3]);
            ll_aes_set_key_191_160(p_aes->p_instance, p_aes->init.p_key[2]);
            ll_aes_set_key_223_192(p_aes->p_instance, p_aes->init.p_key[1]);
            ll_aes_set_key_255_224(p_aes->p_instance, p_aes->init.p_key[0]);
            break;
        default:
            break;
        }
        /* Set enctyption or decryption */
        if (AES_OPERATION_MODE_ENCRYPT == p_aes->init.operation_mode)
            ll_aes_enable_encryption(p_aes->p_instance);
        else
            ll_aes_disable_encryption(p_aes->p_instance);
        /* Set ECB or CBC */
        ll_aes_set_operation_mode(p_aes->p_instance, p_aes->init.chaining_mode);
        /* Set initial vector in CBC mode */
        if (AES_CHAININGMODE_CBC == p_aes->init.chaining_mode)
        {
            if (NULL == p_aes->init.p_init_vector)
            {
                status = HAL_ERROR;
                break;
            }
            ll_aes_set_vector_127_96(p_aes->p_instance, p_aes->init.p_init_vector[0]);
            ll_aes_set_vector_95_64 (p_aes->p_instance, p_aes->init.p_init_vector[1]);
            ll_aes_set_vector_63_32 (p_aes->p_instance, p_aes->init.p_init_vector[2]);
            ll_aes_set_vector_31_0  (p_aes->p_instance, p_aes->init.p_init_vector[3]);
        }
        /* Set DPA Resistence */
        if (ENABLE == p_aes->init.dpa_mode)
        {
            if (NULL == p_aes->init.p_seed)
            {
                status = HAL_ERROR;
                break;
            }
            ll_aes_enable_full_mask(p_aes->p_instance);
            ll_aes_set_seed_in (p_aes->p_instance, p_aes->init.p_seed[0]);
            ll_aes_set_seed_out(p_aes->p_instance, p_aes->init.p_seed[1]);
            ll_aes_set_seed_Imask(p_aes->p_instance, p_aes->init.p_seed[2]);
            ll_aes_set_seed_Osbox(p_aes->p_instance, p_aes->init.p_seed[3]);
        }
        else
        {
            ll_aes_disable_full_mask(p_aes->p_instance);
        }
        /* Set little endian */
        ll_aes_enable_little_endian(p_aes->p_instance);
        /* Set fetch key from MCU */
        ll_aes_set_key_type(p_aes->p_instance, LL_AES_KEYTYPE_MCU);
        /* Disable interrupt and clear flag */
        p_aes->p_instance->INTERRUPT = 1;
    } while(0);

    return status;
}

static void aes_read_data(aes_handle_t *p_aes, uint32_t *p_data)
{
    for (uint32_t i = 0; i < AES_BLOCKSIZE_WORDS; i++)
    {
        p_data[i] = p_aes->p_instance->DATA_OUT[i];
    }
}

static void aes_write_data(aes_handle_t *p_aes, uint32_t *p_data)
{
    for (uint32_t i = 0; i < AES_BLOCKSIZE_WORDS; i++)
    {
        p_aes->p_instance->DATA_IN[i] = p_data[i];
    }
}

static hal_status_t aes_mcu_process(aes_handle_t *p_aes, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    p_aes->state = HAL_AES_STATE_BUSY;

    if (AES_CHAININGMODE_CBC == p_aes->init.chaining_mode)
    {
        ll_aes_set_first_block(p_aes->p_instance);
    }
    if (ENABLE == p_aes->init.dpa_mode)
    {
        ll_aes_set_load_seed(p_aes->p_instance);
    }

    if (0 < timeout)
    {
        while (p_aes->block_count)
        {
            aes_write_data(p_aes, p_aes->p_cryp_input_buffer);
            p_aes->p_cryp_input_buffer += AES_BLOCKSIZE_WORDS;

            ll_aes_enable_start(p_aes->p_instance);

            status = aes_wait_flag_state_until_timeout(p_aes, AES_FLAG_DATAREADY, SET, tickstart, timeout);
            if (HAL_OK != status)
                break;

            aes_read_data(p_aes, p_aes->p_cryp_output_buffer);
            p_aes->p_cryp_output_buffer += AES_BLOCKSIZE_WORDS;

            p_aes->block_count--;

            ll_aes_disable_start(p_aes->p_instance);
        }

        if (HAL_OK != status)
            p_aes->error_code = HAL_AES_ERROR_TIMEOUT;

        p_aes->state = HAL_AES_STATE_READY;
    }
    else
    {
        aes_write_data(p_aes, p_aes->p_cryp_input_buffer);
        p_aes->p_cryp_input_buffer += AES_BLOCKSIZE_WORDS;

        __HAL_AES_ENABLE_IT(p_aes);
        ll_aes_enable_start(p_aes->p_instance);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return status;
}

static hal_status_t aes_dma_process(aes_handle_t *p_aes, uint32_t timeout)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    p_aes->state = HAL_AES_STATE_BUSY;

    ll_aes_set_dma_transfer_block(p_aes->p_instance, p_aes->block_size);
    ll_aes_set_dma_read_address(p_aes->p_instance, (uint32_t)p_aes->p_cryp_input_buffer);
    ll_aes_set_dma_write_address(p_aes->p_instance, (uint32_t)p_aes->p_cryp_output_buffer);

    if (AES_CHAININGMODE_CBC == p_aes->init.chaining_mode)
    {
        ll_aes_set_first_block(p_aes->p_instance);
    }
    if (ENABLE == p_aes->init.dpa_mode)
    {
        ll_aes_set_load_seed(p_aes->p_instance);
    }

    if (0 < timeout)
    {
        ll_aes_enable_dma_start(p_aes->p_instance);

        status = aes_wait_flag_state_until_timeout(p_aes, AES_FLAG_DMA_DONE, SET, tickstart, timeout);
        if (HAL_OK != status)
            p_aes->error_code = HAL_AES_ERROR_TIMEOUT;

        if (RESET != __HAL_AES_GET_FLAG(p_aes, AES_FLAG_DMA_ERR))
        {
            status = HAL_ERROR;
            p_aes->error_code = HAL_AES_ERROR_TRANSFER;
        }

        ll_aes_disable_dma_start(p_aes->p_instance);

        p_aes->state = HAL_AES_STATE_READY;
    }
    else
    {
        __HAL_AES_ENABLE_IT(p_aes);
        ll_aes_enable_dma_start(p_aes->p_instance);
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_aes);

    return status;
}

#endif /* HAL_AES_MODULE_ENABLED */


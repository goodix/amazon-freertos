/**
  ****************************************************************************************
  * @file    gr55xx_hal_efuse.c
  * @author  BLE Driver Team
  * @brief   EFUSE HAL module driver.
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
#include <string.h>

#ifdef HAL_EFUSE_MODULE_ENABLED

#define EFUSE_TIMEOUT_RETRY         400000

static hal_status_t efuse_wait_flag_state_until_timeout(efuse_handle_t *p_efuse, uint32_t flag, flag_status_t state);

__WEAK hal_status_t hal_efuse_init(efuse_handle_t *p_efuse)
{
    hal_status_t status = HAL_OK;

    if ((NULL == p_efuse) || (EFUSE != p_efuse->p_instance))
        return HAL_ERROR;

    /* Process locked */
    __HAL_LOCK(p_efuse);

    if (HAL_EFUSE_STATE_RESET == p_efuse->state)
    {
        /* Allocate lock resource and initialize it */
        p_efuse->lock = HAL_UNLOCKED;

        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
        ll_cgc_disable_wfi_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* init the low level hardware : GPIO, CLOCK */
        hal_efuse_msp_init(p_efuse);
    }

    p_efuse->state = HAL_EFUSE_STATE_BUSY;
 
    ll_efuse_enable_power(p_efuse->p_instance);

    /* Configure EFUSE info mode */
    if (ENABLE == p_efuse->init.info_mode)
        __HAL_EFUSE_ENABLE_MAIN_BACKUP(p_efuse);
    else
        __HAL_EFUSE_DISABLE_MAIN_BACKUP(p_efuse);

    /* Set EFUSE error code to none */
    p_efuse->error_code = HAL_EFUSE_ERROR_NONE;

    /* Initialize the EFUSE state */
    p_efuse->state = HAL_EFUSE_STATE_READY;

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_efuse_deinit(efuse_handle_t *p_efuse)
{
    /* Check the EFUSE handle allocation */
    if (NULL == p_efuse)
        return HAL_ERROR;

    /* Process locked */
    __HAL_LOCK(p_efuse);

    /* DeInit the low level hardware: GPIO, CLOCK... */
    hal_efuse_msp_deinit(p_efuse);

    ll_efuse_disable_power(p_efuse->p_instance);

    /* Set EFUSE error code to none */
    p_efuse->error_code = HAL_EFUSE_ERROR_NONE;

    /* Initialize the EFUSE state */
    p_efuse->state = HAL_EFUSE_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    return HAL_OK;
}

__WEAK void hal_efuse_msp_init(efuse_handle_t *p_efuse)
{
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK void hal_efuse_msp_deinit(efuse_handle_t *p_efuse)
{
    /* Prevent unused argument(s) compilation warning */
    return;
}

__WEAK hal_status_t hal_efuse_write(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t *p_data, uint32_t nword)
{
    hal_status_t status = HAL_OK;
    volatile uint32_t *pefuse = (volatile uint32_t *)(EFUSE_STORAGE_BASE);
    uint32_t count = 0;

    if (((word_offset + nword) > 0x80) || (NULL == p_data) || (0 == nword))
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    pefuse += word_offset;
    /* Process locked */
    __HAL_LOCK(p_efuse);
    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state = HAL_EFUSE_STATE_BUSY;

        /* Set program time, and config PGENB to 0 */
        ll_efuse_set_tpro(p_efuse->p_instance, 0x50);
        ll_efuse_disable_pgenb(p_efuse->p_instance);

        /* Open VDD */
        ll_efuse_set_controller_power_timing(p_efuse->p_instance, 0x10, 0x13, 0x16);
        ll_efuse_enable_controller_power_begin(p_efuse->p_instance);
        while(ll_efuse_is_controller_power_flag(p_efuse->p_instance, LL_EFUSE_PWR_CTL_EN_DONE));
        /* Wait for more than 1ms */
        for (count = 0; count < 8000; count++)
            __asm("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n");
        for (count = 0; count < nword; count++)
        {
            pefuse[count] = p_data[count];
            while(ll_efuse_is_active_flag(p_efuse->p_instance, LL_EFUSE_WRITE_DONE) == 0);
        }

        ll_efuse_enable_controller_power_stop(p_efuse->p_instance);
        while(ll_efuse_is_controller_power_flag(p_efuse->p_instance, LL_EFUSE_PWR_CTL_DIS_DONE));
        ll_efuse_disable_controller_power(p_efuse->p_instance);

        /* Config PGENB to 1 */
        ll_efuse_enable_pgenb(p_efuse->p_instance);

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    return status;
}

__WEAK hal_status_t hal_efuse_read(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t *p_data, uint32_t nword)
{
    hal_status_t status = HAL_OK;
    volatile uint32_t *pefuse = (volatile uint32_t *)(EFUSE_STORAGE_BASE);
    uint32_t count = 0;

    if (((word_offset + nword) > 0x80) || (NULL == p_data) || (0 == nword))
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    pefuse += word_offset;
    /* Process locked */
    __HAL_LOCK(p_efuse);
    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state = HAL_EFUSE_STATE_BUSY;

        __HAL_EFUSE_ENABLE_PGENB(p_efuse);

        for (count = 0; count < nword; count++)
        {
            p_data[count] = pefuse[count];
        }

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    return status;
}

__WEAK hal_status_t hal_efuse_write_keyram(efuse_handle_t *p_efuse, keyram_mask_t *p_mask)
{
    hal_status_t status = HAL_OK;
    keyram_mask_t *keyram_mask = (keyram_mask_t*)(KRAM_BASE + 0x200UL);

    /* Process locked */
    __HAL_LOCK(p_efuse);

    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state = HAL_EFUSE_STATE_BUSY;

        memcpy(keyram_mask, p_mask, sizeof(keyram_mask_t));

        ll_efuse_set_key_mask(p_efuse->p_instance, keyram_mask->efuse_mask);

        ll_efuse_set_operation(p_efuse->p_instance, LL_EFUSE_WRITE_KEYRAM);
        status = efuse_wait_flag_state_until_timeout(p_efuse, EFUSE_FLAG_WRITE_KEYRAM_BUSY, RESET);

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    return status;
}

__WEAK hal_status_t hal_efuse_initial_value_check(efuse_handle_t *p_efuse)
{
    hal_status_t status = HAL_OK;
    uint32_t flag, retry = EFUSE_TIMEOUT_RETRY;

    /* Process locked */
    __HAL_LOCK(p_efuse);

    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state      = HAL_EFUSE_STATE_BUSY;

        ll_efuse_set_operation(p_efuse->p_instance, LL_EFUSE_INIT_CHECK);
        do {
            if (0 == retry--)
            {
                status = HAL_ERROR;
                p_efuse->error_code = HAL_EFUSE_ERROR_TIMEOUT;
                break;
            }
            flag = p_efuse->p_instance->STAT;
        } while(!(flag & EFUSE_FLAG_INIT_CHECK_DONE));

        if (HAL_OK == status)
        {
            if (!(flag & EFUSE_FLAG_INIT_CHECK_SUCCESS))
                status = HAL_ERROR;
        }

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    return status;
}

__WEAK hal_status_t hal_efuse_crc_calculate(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t nword, uint32_t *p_result)
{
    hal_status_t status = HAL_OK;

    if (((word_offset + nword) > 0x80) || (NULL == p_result) || (0 == nword))
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_efuse);

    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state      = HAL_EFUSE_STATE_BUSY;

        ll_efuse_set_crc_check_addr(p_efuse->p_instance, EFUSE_STORAGE_BASE + (word_offset << 2));
        ll_efuse_set_crc_check_len(p_efuse->p_instance, nword);
        ll_efuse_set_operation(p_efuse->p_instance, LL_EFUSE_CRC_CHECK);
        status = efuse_wait_flag_state_until_timeout(p_efuse, EFUSE_FLAG_CRC_CHECK_DONE, SET);
        if (HAL_OK == status)
            *p_result = ll_efuse_get_crc_check_result(p_efuse->p_instance);

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    return status;
}

__WEAK hal_status_t hal_efuse_read_trim(efuse_handle_t *p_efuse, uint32_t word_offset, uint32_t *p_data, uint32_t nword)
{
    hal_status_t status = HAL_OK;

    if (((word_offset + nword) > 0x80) || (NULL == p_data) || (0 == nword))
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_efuse);

    if (HAL_EFUSE_STATE_READY == p_efuse->state)
    {
        p_efuse->error_code = HAL_EFUSE_ERROR_NONE;
        p_efuse->state      = HAL_EFUSE_STATE_BUSY;

        ll_efuse_set_trim_addr(p_efuse->p_instance, EFUSE_STORAGE_BASE + (word_offset << 2));
        ll_efuse_set_trim_length(p_efuse->p_instance, nword);
        ll_efuse_set_operation(p_efuse->p_instance, LL_EFUSE_READ_TRIM);
        status = efuse_wait_flag_state_until_timeout(p_efuse, EFUSE_FLAG_READ_TRIM_DONE, SET);
        if (HAL_OK == status)
        {
            for (uint32_t i = 0; i < nword; i++)
                p_data[i] = ll_efuse_get_trim_value(p_efuse->p_instance, i);
        }

        p_efuse->state = HAL_EFUSE_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_efuse);

    return status;
}

__WEAK hal_status_t hal_efuse_set_main_backup(efuse_handle_t *p_efuse)
{
    if (p_efuse->init.info_mode)
        __HAL_EFUSE_ENABLE_MAIN_BACKUP(p_efuse);
    else
        __HAL_EFUSE_DISABLE_MAIN_BACKUP(p_efuse);

    return HAL_OK;
}

static hal_status_t efuse_wait_flag_state_until_timeout(efuse_handle_t *p_efuse, uint32_t flag, flag_status_t state)
{
    uint32_t retry = EFUSE_TIMEOUT_RETRY;
    /* Wait until flag is in expected state */
    while ((__HAL_EFUSE_GET_FLAG(p_efuse, flag)) != state)
    {
        /* Check for the Timeout */
        if (0 == retry--)
        {
            p_efuse->state       = HAL_EFUSE_STATE_ERROR;
            p_efuse->error_code |= HAL_EFUSE_ERROR_TIMEOUT;

            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

#endif /* HAL_EFUSE_MODULE_ENABLED */


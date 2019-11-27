/**
  ****************************************************************************************
  * @file    app_hmac.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_hmac.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include "string.h"

#ifdef HAL_HMAC_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App hmac state types. */
typedef enum
{
   APP_HMAC_INVALID = 0,
   APP_HMAC_ACTIVITY,
} app_hmac_state_t;

struct hmac_env_t
{
    app_hmac_evt_handler_t   evt_handler;
    hmac_handle_t            handle;
    app_hmac_type_t          ues_type;
    app_hmac_state_t         hmac_state;
    bool                     start_flag;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

static bool hmac_prepare_for_sleep(void);
static void hmac_sleep_canceled(void);
static void hmac_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
struct hmac_env_t s_hmac_env;
static bool s_sleep_cb_registered_flag = false;
static pwr_id_t s_hmac_pwr_id;

const static app_sleep_callbacks_t hmac_sleep_cb =
{
    .app_prepare_for_sleep = hmac_prepare_for_sleep,
    .app_sleep_canceled = hmac_sleep_canceled,
    .app_wake_up_ind = hmac_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool hmac_prepare_for_sleep(void)
{
    if (s_hmac_env.hmac_state == APP_HMAC_ACTIVITY)
    {
        if (hal_hmac_get_state(&s_hmac_env.handle) == HAL_HMAC_STATE_BUSY)
        {
            return false;
        }
    }

    return true;
}

static void hmac_sleep_canceled(void)
{
}

static void hmac_wake_up_ind(void)
{
    if (s_hmac_env.hmac_state == APP_HMAC_ACTIVITY)
    {
        if (s_hmac_env.ues_type != APP_HMAC_TYPE_POLLING)
        {
            hal_nvic_clear_pending_irq(HMAC_IRQn);
            hal_nvic_enable_irq(HMAC_IRQn);
        }

        hal_hmac_deinit(&s_hmac_env.handle);
        hal_hmac_init(&s_hmac_env.handle);
    }
}

static void app_hmac_event_call(hmac_handle_t *p_hmac, app_hmac_evt_type_t evt_type)
{
    app_hmac_evt_t hmac_evt = { APP_HMAC_EVT_ERROR, 0};

    hmac_evt.type = evt_type;
    s_hmac_env.start_flag = false;
    if (evt_type == APP_HMAC_EVT_ERROR)
    {
        hmac_evt.error_code = p_hmac->error_code;
    }

    if (s_hmac_env.evt_handler != NULL)
    {
        s_hmac_env.evt_handler(&hmac_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_hmac_init(app_hmac_params_t *p_params, app_hmac_evt_handler_t evt_handler)
{
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    app_systick_init();

    if (p_params->use_type != APP_HMAC_TYPE_POLLING)
    {
        hal_nvic_clear_pending_irq(HMAC_IRQn);
        hal_nvic_enable_irq(HMAC_IRQn);
    }

    s_hmac_env.ues_type = p_params->use_type;
    s_hmac_env.evt_handler = evt_handler;

    memcpy(&s_hmac_env.handle.init, &p_params->init, sizeof(hmac_init_t));
    s_hmac_env.handle.p_instance = HMAC;
    hal_err_code = hal_hmac_deinit(&s_hmac_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_hmac_init(&s_hmac_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    if(s_sleep_cb_registered_flag == false)    // register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_hmac_pwr_id = pwr_register_sleep_cb(&hmac_sleep_cb);
        if (s_hmac_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_hmac_env.hmac_state = APP_HMAC_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_deinit(void)
{
    hal_status_t  hal_err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    hal_nvic_disable_irq(HMAC_IRQn);
    s_hmac_env.hmac_state = APP_HMAC_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    pwr_unregister_sleep_cb(s_hmac_pwr_id);
    s_sleep_cb_registered_flag = false;
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_err_code =  hal_hmac_deinit(&s_hmac_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_user_hash(uint32_t *p_user_hash)
{
    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        s_hmac_env.start_flag == true)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    s_hmac_env.handle.init.p_user_hash = p_user_hash;

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_sha256_sync(uint32_t *p_message, uint32_t number, uint32_t *p_digest, uint32_t timeout)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0 ||
        p_digest == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_sha256_digest(&s_hmac_env.handle, p_message, number, p_digest, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_sha256_async(uint32_t *p_message, uint32_t number, uint32_t *p_digest)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0 ||
        p_digest == NULL ||
        s_hmac_env.ues_type == APP_HMAC_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_hmac_env.start_flag == true)
    {
        return APP_DRV_ERR_BUSY;
    }

    s_hmac_env.start_flag = true;
    switch(s_hmac_env.ues_type)
    {
        case APP_HMAC_TYPE_INTERRUPT:
            err_code = hal_sha256_digest_it(&s_hmac_env.handle, p_message, number, p_digest);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_HMAC_TYPE_DMA:
            err_code = hal_sha256_digest_dma(&s_hmac_env.handle, p_message, number, p_digest);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_sha256_hmac_start_sync(uint32_t *p_message, uint32_t number, uint32_t timeout)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_hmac_sha256_digest_start(&s_hmac_env.handle, p_message, number, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_sha256_hmac_continue_sync(uint32_t *p_message, uint32_t number, uint32_t timeout)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_hmac_sha256_digest_continue(&s_hmac_env.handle, p_message, number, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_sha256_hmac_finish_sync(uint32_t *p_message, uint32_t number, uint32_t *digest, uint32_t timeout)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0 ||
        digest == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_hmac_sha256_digest_finish(&s_hmac_env.handle, p_message, number, digest, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_sha256_hmac_start_async(uint32_t *p_message, uint32_t number)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0 ||
        s_hmac_env.ues_type == APP_HMAC_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_hmac_env.start_flag == true)
    {
        return APP_DRV_ERR_BUSY;
    }

    s_hmac_env.start_flag = true;
    switch(s_hmac_env.ues_type)
    {
        case APP_HMAC_TYPE_INTERRUPT:
            err_code = hal_hmac_sha256_digest_start_it(&s_hmac_env.handle, p_message, number);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_HMAC_TYPE_DMA:
            err_code = hal_hmac_sha256_digest_start_dma(&s_hmac_env.handle, p_message, number);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}


uint16_t app_hmac_sha256_hmac_continue_async(uint32_t *p_message, uint32_t number)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0 ||
        s_hmac_env.ues_type == APP_HMAC_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_hmac_env.start_flag == true)
    {
        return APP_DRV_ERR_BUSY;
    }

    s_hmac_env.start_flag = true;
    switch(s_hmac_env.ues_type)
    {
        case APP_HMAC_TYPE_INTERRUPT:
            err_code = hal_hmac_sha256_digest_continue_it(&s_hmac_env.handle, p_message, number);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_HMAC_TYPE_DMA:
            err_code = hal_hmac_sha256_digest_continue_dma(&s_hmac_env.handle, p_message, number);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_hmac_sha256_hmac_finish_async(uint32_t *p_message, uint32_t number, uint32_t *digest)
{
    hal_status_t err_code;

    if (s_hmac_env.hmac_state == APP_HMAC_INVALID ||
        p_message == NULL ||
        number == 0 ||
        digest == NULL ||
        s_hmac_env.ues_type == APP_HMAC_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_hmac_env.start_flag == true)
    {
        return APP_DRV_ERR_BUSY;
    }

    s_hmac_env.start_flag = true;
    switch(s_hmac_env.ues_type)
    {
        case APP_HMAC_TYPE_INTERRUPT:
            err_code = hal_hmac_sha256_digest_finish_it(&s_hmac_env.handle, p_message, number, digest);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_HMAC_TYPE_DMA:
            err_code = hal_hmac_sha256_digest_finish_dma(&s_hmac_env.handle, p_message, number, digest);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

void hal_hmac_done_callback(hmac_handle_t *p_hmac)
{
    app_hmac_event_call(p_hmac, APP_HMAC_EVT_DONE);
}

void hal_hmac_error_callback(hmac_handle_t *p_hmac)
{
    app_hmac_event_call(p_hmac, APP_HMAC_EVT_ERROR);
}

void HMAC_IRQHandler(void)
{
    hal_hmac_irq_handler(&s_hmac_env.handle);
}

#endif

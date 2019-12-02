/**
  ****************************************************************************************
  * @file    app_aes.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.
  ****************************************************************************************
  */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_aes.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include "string.h"

#ifdef HAL_AES_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App aes state types. */
typedef enum
{
   APP_AES_INVALID = 0,
   APP_AES_ACTIVITY,
} app_aes_state_t;

struct aes_env_t
{
    app_aes_evt_handler_t   evt_handler;
    aes_handle_t            handle;
    app_aes_mode_t          use_mode;
    app_aes_type_t          ues_type;
    app_aes_state_t         aes_state;
    bool                    start_flag;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */

static bool aes_prepare_for_sleep(void);
static void aes_sleep_canceled(void);
static void aes_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
struct aes_env_t s_aes_env;
static bool s_sleep_cb_registered_flag = false;
static pwr_id_t s_aes_pwr_id;

const static app_sleep_callbacks_t aes_sleep_cb =
{
    .app_prepare_for_sleep = aes_prepare_for_sleep,
    .app_sleep_canceled = aes_sleep_canceled,
    .app_wake_up_ind = aes_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool aes_prepare_for_sleep(void)
{
    if (s_aes_env.aes_state == APP_AES_ACTIVITY)
    {
        if (hal_aes_get_state(&s_aes_env.handle) == HAL_AES_STATE_BUSY)
        {
            return false;
        }
    }

    return true;
}

static void aes_sleep_canceled(void)
{
}

static void aes_wake_up_ind(void)
{
    if (s_aes_env.aes_state == APP_AES_ACTIVITY)
    {
        if (s_aes_env.ues_type != APP_AES_TYPE_POLLING)
        {
            hal_nvic_clear_pending_irq(AES_IRQn);
            hal_nvic_enable_irq(AES_IRQn);
        }

        hal_aes_deinit(&s_aes_env.handle);
        hal_aes_init(&s_aes_env.handle);
    }
}

static void app_aes_event_call(aes_handle_t *p_aes, app_aes_evt_type_t evt_type)
{
    app_aes_evt_t aes_evt = { APP_AES_EVT_ERROR, 0};

    aes_evt.type = evt_type;
    s_aes_env.start_flag = false;
    if(evt_type == APP_AES_EVT_ERROR)
    {
        aes_evt.error_code = p_aes->error_code;
    }

    if (s_aes_env.evt_handler != NULL)
    {
        s_aes_env.evt_handler(&aes_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_aes_init(app_aes_params_t *p_params, app_aes_evt_handler_t evt_handler)
{
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    app_systick_init();

    if (p_params->use_type != APP_AES_TYPE_POLLING)
    {
        hal_nvic_clear_pending_irq(AES_IRQn);
        hal_nvic_enable_irq(AES_IRQn);
    }

    s_aes_env.ues_type = p_params->use_type;
    s_aes_env.use_mode = p_params->use_mode;
    s_aes_env.evt_handler = evt_handler;

    memcpy(&s_aes_env.handle.init, &p_params->init, sizeof(aes_init_t));
    s_aes_env.handle.p_instance = AES;
    hal_err_code = hal_aes_deinit(&s_aes_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_aes_init(&s_aes_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    if(s_sleep_cb_registered_flag == false)    // register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_aes_pwr_id = pwr_register_sleep_cb(&aes_sleep_cb);
        if (s_aes_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_aes_env.aes_state = APP_AES_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_aes_deinit(void)
{
    hal_status_t  hal_err_code;

    if (s_aes_env.aes_state == APP_AES_INVALID)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    hal_nvic_disable_irq(AES_IRQn);
    s_aes_env.aes_state = APP_AES_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    pwr_unregister_sleep_cb(s_aes_pwr_id);
    s_sleep_cb_registered_flag = false;
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_err_code =  hal_aes_deinit(&s_aes_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_aes_encrypt_sync(uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout)
{
    hal_status_t err_code;

    if (s_aes_env.aes_state == APP_AES_INVALID ||
        p_plain_data == NULL ||
        number == 0 ||
        p_cypher_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(s_aes_env.use_mode)
    {
        case APP_AES_MODE_ECB:
            err_code = hal_aes_ecb_encrypt(&s_aes_env.handle, p_plain_data, number, p_cypher_data, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_AES_MODE_CBC:
            err_code = hal_aes_cbc_encrypt(&s_aes_env.handle, p_plain_data, number, p_cypher_data, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;
        
        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_aes_decrypt_sync(uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout)
{
    hal_status_t err_code;

    if (s_aes_env.aes_state == APP_AES_INVALID ||
        p_plain_data == NULL ||
        number == 0 ||
        p_cypher_data == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(s_aes_env.use_mode)
    {
        case APP_AES_MODE_ECB:
            err_code = hal_aes_ecb_decrypt(&s_aes_env.handle, p_cypher_data, number, p_plain_data, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_AES_MODE_CBC:
            err_code = hal_aes_cbc_decrypt(&s_aes_env.handle, p_cypher_data, number, p_plain_data, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;
        
        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_aes_encrypt_async(uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data)
{
    hal_status_t err_code;

    if (s_aes_env.aes_state == APP_AES_INVALID ||
        p_plain_data == NULL ||
        number == 0 ||
        p_cypher_data == NULL ||
        s_aes_env.ues_type == APP_AES_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if(s_aes_env.start_flag == true)
    {
        return APP_DRV_ERR_BUSY;
    }

    s_aes_env.start_flag = true;
    switch(s_aes_env.use_mode)
    {
        case APP_AES_MODE_ECB:
            switch(s_aes_env.ues_type)
            {
                case APP_AES_TYPE_INTERRUPT:
                    err_code = hal_aes_ecb_encrypt_it(&s_aes_env.handle, p_plain_data, number, p_cypher_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                case APP_AES_TYPE_DMA:
                    err_code = hal_aes_ecb_encrypt_dma(&s_aes_env.handle, p_plain_data, number, p_cypher_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                default:
                    break;
            }
            break;

        case APP_AES_MODE_CBC:
            switch(s_aes_env.ues_type)
            {
                case APP_AES_TYPE_INTERRUPT:
                    err_code = hal_aes_cbc_encrypt_it(&s_aes_env.handle, p_plain_data, number, p_cypher_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                case APP_AES_TYPE_DMA:
                    err_code = hal_aes_cbc_encrypt_dma(&s_aes_env.handle, p_plain_data, number, p_cypher_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                default:
                    break;
            }
            break;
        
        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_aes_decrypt_async(uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data)
{
    hal_status_t err_code;

    if (s_aes_env.aes_state == APP_AES_INVALID ||
        p_plain_data == NULL ||
        number == 0 ||
        p_cypher_data == NULL ||
        s_aes_env.ues_type == APP_AES_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if(s_aes_env.start_flag == true)
    {
        return APP_DRV_ERR_BUSY;
    }

    s_aes_env.start_flag = true;
    switch(s_aes_env.use_mode)
    {
        case APP_AES_MODE_ECB:
            switch(s_aes_env.ues_type)
            {
                case APP_AES_TYPE_INTERRUPT:
                    err_code = hal_aes_ecb_decrypt_it(&s_aes_env.handle, p_cypher_data, number, p_plain_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                case APP_AES_TYPE_DMA:
                    err_code = hal_aes_ecb_decrypt_dma(&s_aes_env.handle, p_cypher_data, number, p_plain_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                default:
                    break;
            }
            break;

        case APP_AES_MODE_CBC:
            switch(s_aes_env.ues_type)
            {
                case APP_AES_TYPE_INTERRUPT:
                    err_code = hal_aes_cbc_decrypt_it(&s_aes_env.handle, p_cypher_data, number, p_plain_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                case APP_AES_TYPE_DMA:
                    err_code = hal_aes_cbc_decrypt_dma(&s_aes_env.handle, p_cypher_data, number, p_plain_data);
                    HAL_ERR_CODE_CHECK(err_code);
                    break;
                default:
                    break;
            }
            break;
        
        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

void hal_aes_done_callback(aes_handle_t *p_aes)
{
    app_aes_event_call(p_aes, APP_AES_EVT_DONE);
}

void hal_aes_error_callback(aes_handle_t *p_aes)
{
    app_aes_event_call(p_aes, APP_AES_EVT_ERROR);
}

void AES_IRQHandler(void)
{
    hal_aes_irq_handler(&s_aes_env.handle);
}

#endif

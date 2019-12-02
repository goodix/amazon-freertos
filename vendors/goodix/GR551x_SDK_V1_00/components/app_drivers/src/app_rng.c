/**
  ****************************************************************************************
  * @file    app_rng.c
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
#include "app_rng.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include "string.h"

#ifdef HAL_RNG_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App rng state types. */
typedef enum
{
   APP_RNG_INVALID = 0,
   APP_RNG_ACTIVITY,
} app_rng_state_t;

struct rng_env_t
{
    app_rng_evt_handler_t   evt_handler;
    rng_handle_t            handle;
    app_rng_type_t          ues_type;
    app_rng_state_t         rng_state;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool rng_prepare_for_sleep(void);
static void rng_sleep_canceled(void);
static void rng_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
struct rng_env_t s_rng_env;
static bool s_sleep_cb_registered_flag = false;
static pwr_id_t s_rng_pwr_id;

const static app_sleep_callbacks_t rng_sleep_cb =
{
    .app_prepare_for_sleep = rng_prepare_for_sleep,
    .app_sleep_canceled = rng_sleep_canceled,
    .app_wake_up_ind = rng_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool rng_prepare_for_sleep(void)
{
    if (s_rng_env.rng_state == APP_RNG_ACTIVITY)
    {
        if (hal_rng_get_state(&s_rng_env.handle) == HAL_RNG_STATE_BUSY)
        {
            return false;
        }
    }

    return true;
}

static void rng_sleep_canceled(void)
{

}

static void rng_wake_up_ind(void)
{
    if (s_rng_env.rng_state == APP_RNG_ACTIVITY)
    {
        if (s_rng_env.ues_type == APP_RNG_TYPE_INTERRUPT)
        {
            hal_nvic_clear_pending_irq(RNG_IRQn);
            hal_nvic_enable_irq(RNG_IRQn);
        }

        hal_rng_deinit(&s_rng_env.handle);
        hal_rng_init(&s_rng_env.handle);
    }
}

static void app_rng_event_call(rng_handle_t *p_rng, app_rng_evt_type_t evt_type, uint32_t random32bit)
{
    app_rng_evt_t rng_evt = {APP_RNG_EVT_ERROR, 0x0};

    if (p_rng->p_instance == RNG)
    {
        rng_evt.type = evt_type;
        rng_evt.random_data = random32bit;
    }

    if (s_rng_env.evt_handler != NULL)
    {
        s_rng_env.evt_handler(&rng_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_rng_init(app_rng_params_t *p_params, app_rng_evt_handler_t evt_handler)
{
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    app_systick_init();

    if (p_params->use_type == APP_RNG_TYPE_INTERRUPT)
    {
        hal_nvic_clear_pending_irq(RNG_IRQn);
        hal_nvic_enable_irq(RNG_IRQn);
    }

    s_rng_env.ues_type = p_params->use_type;
    s_rng_env.evt_handler = evt_handler;

    memcpy(&s_rng_env.handle.init, &p_params->init, sizeof(rng_init_t));
    s_rng_env.handle.p_instance = RNG;
    hal_err_code = hal_rng_deinit(&s_rng_env.handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_rng_init(&s_rng_env.handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    if(s_sleep_cb_registered_flag == false)    // register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_rng_pwr_id = pwr_register_sleep_cb(&rng_sleep_cb);
        if (s_rng_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_rng_env.rng_state = APP_RNG_ACTIVITY;

    return 0;
}

uint16_t app_rng_deinit(void)
{
    hal_status_t  hal_err_code;

    if (s_rng_env.rng_state == APP_RNG_INVALID)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    hal_nvic_disable_irq(RNG_IRQn);
    s_rng_env.rng_state = APP_RNG_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    pwr_unregister_sleep_cb(s_rng_pwr_id);
    s_sleep_cb_registered_flag = false;
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_err_code = hal_rng_deinit(&s_rng_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}


uint16_t app_rng_generate_number_sync(uint16_t *p_seed, uint32_t *p_random32bit)
{
    hal_status_t err_code;

    if (s_rng_env.rng_state == APP_RNG_INVALID ||
        p_seed == NULL ||
        p_random32bit == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_rng_generate_random_number(&s_rng_env.handle, p_seed, p_random32bit);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_rng_generate_number_async(uint16_t *p_seed)
{
    hal_status_t err_code;

    if (s_rng_env.rng_state == APP_RNG_INVALID ||
        s_rng_env.ues_type == APP_RNG_TYPE_POLLING ||
        p_seed == NULL)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_rng_generate_random_number_it(&s_rng_env.handle, p_seed);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

void hal_rng_ready_data_callback(rng_handle_t *p_rng, uint32_t random32bit)
{
    app_rng_event_call(p_rng, APP_RNG_EVT_DONE, random32bit);
}

void RNG_IRQHandler(void)
{
    hal_rng_irq_handler(&s_rng_env.handle);
}

#endif

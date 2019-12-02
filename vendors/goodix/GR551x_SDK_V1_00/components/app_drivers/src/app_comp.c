/**
  ****************************************************************************************
  * @file    app_comp.c
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
#include "app_comp.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include "string.h"

#ifdef HAL_COMP_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App comp state types. */
typedef enum
{
   APP_COMP_INVALID = 0,
   APP_COMP_ACTIVITY,
} app_comp_state_t;

struct comp_env_t
{
    app_comp_evt_handler_t  evt_handler;
    comp_handle_t           handle;
    app_comp_pin_cfg_t      pin_cfg;
    app_comp_state_t        comp_state;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool comp_prepare_for_sleep(void);
static void comp_sleep_canceled(void);
static void comp_wake_up_ind(void);
static uint16_t comp_config_gpio(uint32_t ref_source, app_comp_pin_cfg_t pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
struct comp_env_t s_comp_env;
static bool s_sleep_cb_registered_flag = false;
static pwr_id_t s_comp_pwr_id;

const static app_sleep_callbacks_t comp_sleep_cb =
{
    .app_prepare_for_sleep = comp_prepare_for_sleep,
    .app_sleep_canceled = comp_sleep_canceled,
    .app_wake_up_ind = comp_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static bool comp_prepare_for_sleep(void)
{
    if (s_comp_env.comp_state == APP_COMP_ACTIVITY)
    {
        if (hal_comp_get_state(&s_comp_env.handle) == HAL_COMP_STATE_BUSY)
        {
            return false;
        }
    }

    return true;
}

static void comp_sleep_canceled(void)
{
}

static void comp_wake_up_ind(void)
{
    if (s_comp_env.comp_state == APP_COMP_ACTIVITY)
    {
        hal_nvic_clear_pending_irq(COMP_EXT_IRQn);
        hal_nvic_enable_irq(COMP_EXT_IRQn);

        comp_config_gpio(s_comp_env.handle.init.ref_source, s_comp_env.pin_cfg);

        hal_comp_deinit(&s_comp_env.handle);
        hal_comp_init(&s_comp_env.handle);
    }
}

static uint16_t comp_config_gpio(uint32_t ref_source, app_comp_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.mode = APP_IO_MODE_ANALOG;
    io_init.pin  = pin_cfg.input.pin;
    io_init.mux  = pin_cfg.input.mux;
    err_code = app_io_init(pin_cfg.input.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (ref_source != COMP_REF_SRC_VBAT && ref_source == COMP_REF_SRC_VREF)
    {
        io_init.pin  = pin_cfg.vref.pin;
        io_init.mux  = pin_cfg.vref.mux;
        err_code = app_io_init(pin_cfg.vref.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

static void app_comp_event_call(comp_handle_t *p_comp, app_comp_evt_t evt_type)
{
    app_comp_evt_t comp_evt = APP_COMP_EVT_ERROR;

    if (evt_type == APP_COMP_EVT_DONE)
    {
        comp_evt = APP_COMP_EVT_DONE;
    }

    if (s_comp_env.evt_handler != NULL)
    {
        s_comp_env.evt_handler(&comp_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_comp_init(app_comp_params_t *p_params, app_comp_evt_handler_t evt_handler)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    app_err_code = comp_config_gpio(p_params->init.ref_source, p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    hal_nvic_clear_pending_irq(COMP_EXT_IRQn);
    hal_nvic_enable_irq(COMP_EXT_IRQn);

    memcpy(&s_comp_env.pin_cfg, &p_params->pin_cfg, sizeof(app_comp_pin_cfg_t));
    s_comp_env.evt_handler = evt_handler;

    memcpy(&s_comp_env.handle.init, &p_params->init, sizeof(comp_init_t));
    hal_err_code = hal_comp_deinit(&s_comp_env.handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_comp_init(&s_comp_env.handle);
    APP_DRV_ERR_CODE_CHECK(hal_err_code);

    if(s_sleep_cb_registered_flag == false)    // register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_comp_pwr_id = pwr_register_sleep_cb(&comp_sleep_cb);
        if (s_comp_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_comp_env.comp_state = APP_COMP_ACTIVITY;

    return 0;
}

uint16_t app_comp_deinit(void)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (s_comp_env.comp_state == APP_COMP_INVALID)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_err_code = app_io_deinit(s_comp_env.pin_cfg.input.type, s_comp_env.pin_cfg.input.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    if (s_comp_env.handle.init.ref_source != COMP_REF_SRC_VBAT &&
        s_comp_env.handle.init.ref_source != COMP_REF_SRC_VREF)
    {
        app_err_code = app_io_deinit(s_comp_env.pin_cfg.vref.type, s_comp_env.pin_cfg.vref.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    hal_nvic_disable_irq(COMP_EXT_IRQn);
    s_comp_env.comp_state = APP_COMP_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    pwr_unregister_sleep_cb(s_comp_pwr_id);
    s_sleep_cb_registered_flag = false;
    GLOBAL_EXCEPTION_ENABLE();

    hal_err_code = hal_comp_deinit(&s_comp_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_comp_start(void)
{
    hal_status_t  hal_err_code;

    if (s_comp_env.comp_state == APP_COMP_INVALID)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    hal_err_code = hal_comp_start(&s_comp_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_comp_stop(void)
{
    hal_status_t  hal_err_code;

    if (s_comp_env.comp_state == APP_COMP_INVALID)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    hal_err_code = hal_comp_stop(&s_comp_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

void hal_comp_trigger_callback(comp_handle_t *p_comp)
{
    app_comp_event_call(p_comp, APP_COMP_EVT_DONE);
}

void COMP_IRQHandler(void)
{
    hal_comp_irq_handler(&s_comp_env.handle);
}

#endif

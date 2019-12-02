/**
  ****************************************************************************************
  * @file    app_adc.c
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
#include "app_adc.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include "gr551x_adc_voltage_api.h"
#include <string.h>

#ifdef HAL_ADC_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App adc state types. */
typedef enum
{
   APP_ADC_INVALID = 0,
   APP_ADC_ACTIVITY,
} app_adc_state_t;

struct adc_env_t
{
    app_adc_evt_handler_t   evt_handler;
    adc_handle_t            handle;
    app_adc_mode_t          use_mode;
    app_adc_pin_cfg_t       pin_cfg;
    dma_id_t                dma_id;
    app_adc_state_t         adc_state;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool adc_prepare_for_sleep(void);
static void adc_sleep_canceled(void);
static void adc_wake_up_ind(void);
static uint16_t adc_config_gpio(uint32_t input_mode, app_adc_pin_cfg_t pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
struct adc_env_t s_adc_env;
static bool      s_sleep_cb_registered_flag = false;
static pwr_id_t  s_adc_pwr_id;

const static app_sleep_callbacks_t adc_sleep_cb =
{
    .app_prepare_for_sleep = adc_prepare_for_sleep,
    .app_sleep_canceled    = adc_sleep_canceled,
    .app_wake_up_ind       = adc_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool adc_prepare_for_sleep(void)
{
    if (s_adc_env.adc_state == APP_ADC_ACTIVITY)
    {
        if (hal_adc_get_state(&s_adc_env.handle) == HAL_ADC_STATE_BUSY)
        {
            return false;
        }
    }

    return true;
}


static void adc_sleep_canceled(void)
{
}

static void adc_wake_up_ind(void)
{
    if (s_adc_env.adc_state == APP_ADC_ACTIVITY)
    {
        adc_config_gpio(s_adc_env.handle.init.input_mode, s_adc_env.pin_cfg);

        hal_adc_deinit(&s_adc_env.handle);
        hal_adc_init(&s_adc_env.handle);
    }
}

static uint16_t adc_config_gpio(uint32_t input_mode, app_adc_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.mode = APP_IO_MODE_ANALOG;
    if (input_mode == LL_ADC_INPUT_DIFFERENTIAL)
    {
        io_init.pin  = pin_cfg.channel_p.pin;
        io_init.mux  = pin_cfg.channel_p.mux;
        err_code = app_io_init(pin_cfg.channel_p.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    io_init.pin  = pin_cfg.channel_n.pin;
    io_init.mux  = pin_cfg.channel_n.mux;
    err_code = app_io_init(pin_cfg.channel_n.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static uint16_t adc_config_dma(app_adc_params_t *p_params)
{
    app_dma_params_t dma_params;

    dma_params.channel_number            = p_params->use_mode.dma_channel;
    dma_params.init.src_request          = DMA_REQUEST_SNSADC;
    dma_params.init.direction            = DMA_PERIPH_TO_MEMORY;
    dma_params.init.src_increment        = DMA_SRC_NO_CHANGE;
    dma_params.init.dst_increment        = DMA_DST_INCREMENT;
    dma_params.init.src_data_alignment   = DMA_SDATAALIGN_WORD;
    dma_params.init.dst_data_alignment   = DMA_DDATAALIGN_WORD;
    dma_params.init.mode                 = DMA_NORMAL;
    dma_params.init.priority             = DMA_PRIORITY_LOW;

    s_adc_env.dma_id = app_dma_init(&dma_params, NULL);
    if (s_adc_env.dma_id < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_adc_env.handle.p_dma = app_dma_get_handle(s_adc_env.dma_id);
    s_adc_env.handle.p_dma->p_parent = (void*)&s_adc_env.handle;

    return APP_DRV_SUCCESS;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_adc_init(app_adc_params_t *p_params, app_adc_evt_handler_t evt_handler)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    app_systick_init();

    app_err_code = adc_config_gpio(p_params->init.input_mode, p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    if(p_params->use_mode.type == APP_ADC_TYPE_DMA)
    {
        app_err_code = adc_config_dma(p_params);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    s_adc_env.use_mode.type = p_params->use_mode.type;
    s_adc_env.use_mode.dma_channel = p_params->use_mode.dma_channel;
    memcpy(&s_adc_env.pin_cfg, &p_params->pin_cfg, sizeof(app_adc_pin_cfg_t));
    s_adc_env.evt_handler = evt_handler;

    memcpy(&s_adc_env.handle.init, &p_params->init, sizeof(adc_init_t));
    hal_err_code = hal_adc_deinit(&s_adc_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    hal_err_code = hal_adc_init(&s_adc_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    if(!s_sleep_cb_registered_flag)    // register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_adc_pwr_id = pwr_register_sleep_cb(&adc_sleep_cb);
        if (s_adc_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_adc_env.adc_state = APP_ADC_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_adc_deinit(void)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if(s_adc_env.handle.init.input_mode == LL_ADC_INPUT_DIFFERENTIAL)
    {
        app_err_code = app_io_deinit(s_adc_env.pin_cfg.channel_p.type, s_adc_env.pin_cfg.channel_p.pin);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    app_err_code = app_io_deinit(s_adc_env.pin_cfg.channel_n.type, s_adc_env.pin_cfg.channel_n.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    if(s_adc_env.use_mode.type == APP_ADC_TYPE_DMA)
    {
        app_err_code = app_dma_deinit(s_adc_env.dma_id);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }
    s_adc_env.adc_state = APP_ADC_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    pwr_unregister_sleep_cb(s_adc_pwr_id);
    s_sleep_cb_registered_flag = false;
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_err_code = hal_adc_deinit(&s_adc_env.handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t adc_conversion_sync(uint16_t *p_data, uint32_t length)
{
    hal_status_t err_code;

    if (p_data == NULL ||
        length == 0 ||
        s_adc_env.adc_state == APP_ADC_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_adc_poll_for_conversion(&s_adc_env.handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t adc_conversion_async(uint16_t *p_data, uint32_t length)
{
    hal_status_t err_code;

    if (p_data == NULL ||
        length == 0 ||
        s_adc_env.adc_state == APP_ADC_INVALID ||
        s_adc_env.use_mode.type != APP_ADC_TYPE_DMA)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_adc_start_dma(&s_adc_env.handle, p_data, length);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t adc_voltage_intern(uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    if (inbuf == NULL || outbuf == NULL || buflen == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_gr551x_adc_voltage_intern(&s_adc_env.handle, inbuf, outbuf, buflen);

    return APP_DRV_SUCCESS;
}

uint16_t adc_voltage_extern(double ref, uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    if (inbuf == NULL || outbuf == NULL || buflen == 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    hal_gr551x_adc_voltage_extern(&s_adc_env.handle, ref, inbuf, outbuf, buflen);

    return APP_DRV_SUCCESS;
}

void hal_adc_conv_cplt_callback(adc_handle_t *p_adc)
{
    app_adc_evt_t evt;

    evt.type = APP_ADC_EVT_CONV_CPLT;

    if (s_adc_env.evt_handler != NULL)
    {
        s_adc_env.evt_handler(&evt);
    }
}

#endif

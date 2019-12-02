/**
  ****************************************************************************************
  * @file    app_i2s.c
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
#include "app_i2s.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include <string.h>

#ifdef HAL_I2S_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/**@brief App i2s state types. */
typedef enum
{
   APP_I2S_INVALID = 0,
   APP_I2S_ACTIVITY,
} app_i2s_state_t;

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
struct i2s_env_t
{
    app_i2s_evt_handler_t   evt_handler;
    i2s_handle_t            handle;
    app_i2s_mode_t          use_mode;
    app_i2s_pin_cfg_t       pin_cfg;
    dma_id_t                dma_id[2];
    app_i2s_state_t         i2s_state;
    bool                    start_flag;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool     i2s_prepare_for_sleep(void);
static void     i2s_sleep_canceled(void);
static void     i2s_wake_up_ind(void);
static uint16_t i2s_gpio_config(app_i2s_id_t id, app_i2s_pin_cfg_t pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type s_i2s_irq[APP_I2S_ID_MAX] = {I2S_S_IRQn, I2S_M_IRQn};
static const uint32_t  s_i2s_instance[APP_I2S_ID_MAX] = {I2S_S_BASE, I2S_M_BASE};

struct i2s_env_t s_i2s_env[APP_I2S_ID_MAX];
static bool      s_sleep_cb_registered_flag = false;
static pwr_id_t  s_i2s_pwr_id;

const static app_sleep_callbacks_t i2s_sleep_cb =
{
    .app_prepare_for_sleep = i2s_prepare_for_sleep,
    .app_sleep_canceled    = i2s_sleep_canceled,
    .app_wake_up_ind       = i2s_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool i2s_prepare_for_sleep(void)
{
    hal_i2s_state_t state;
    uint8_t i;

    for (i = 0; i < APP_I2S_ID_MAX; i++)
    {
        if (s_i2s_env[i].i2s_state == APP_I2S_ACTIVITY)
        {
            state = hal_i2s_get_state(&s_i2s_env[i].handle);
            if (state > HAL_I2S_STATE_READY && state < HAL_I2S_STATE_ABORT)
            {
                return false;
            }
        }
    }

    return true;
}

static void i2s_sleep_canceled(void)
{
}

static void i2s_wake_up_ind(void)
{
    uint8_t i;

    for (i = 0; i < APP_I2S_ID_MAX; i++)
    {
        if (s_i2s_env[i].i2s_state == APP_I2S_ACTIVITY)
        {
            i2s_gpio_config((app_i2s_id_t)i, s_i2s_env[i].pin_cfg);

            if(s_i2s_env[i].use_mode.type == APP_I2S_TYPE_INTERRUPT)
            {
                hal_nvic_clear_pending_irq(s_i2s_irq[i]);
                hal_nvic_enable_irq(s_i2s_irq[i]);
            }

            hal_i2s_deinit(&s_i2s_env[i].handle);
            hal_i2s_init(&s_i2s_env[i].handle);
        }
    }
}

static uint16_t i2s_gpio_config(app_i2s_id_t id, app_i2s_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = pin_cfg.ws.pin;
    io_init.mux  = pin_cfg.ws.mux;
    err_code = app_io_init(pin_cfg.ws.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pin  = pin_cfg.sdo.pin;
    io_init.mux  = pin_cfg.sdo.mux;
    err_code = app_io_init(pin_cfg.sdo.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pin  = pin_cfg.sdi.pin;
    io_init.mux  = pin_cfg.sdi.mux;
    err_code = app_io_init(pin_cfg.sdo.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pin  = pin_cfg.sclk.pin;
    io_init.mux  = pin_cfg.sclk.mux;
    err_code = app_io_init(pin_cfg.sclk.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static uint16_t app_i2s_config_dma(app_i2s_params_t *p_params)
{
    app_dma_params_t tx_dma_params = {DMA_Channel0, {0}};
    app_dma_params_t rx_dma_params = {DMA_Channel0, {0}};

    tx_dma_params.channel_number             = p_params->use_mode.tx_dma_channel;
    tx_dma_params.init.src_request           = DMA_REQUEST_MEM;
    tx_dma_params.init.dst_request           = (p_params->id == APP_I2S_ID_SLAVE) ? DMA_REQUEST_I2S_S_TX : DMA_REQUEST_I2S_M_TX;
    tx_dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    if (p_params->init.data_size <= I2S_DATASIZE_16BIT)
    {
        tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_HALFWORD;
        tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
        tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    }
    tx_dma_params.init.mode                  = DMA_NORMAL;
    tx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    s_i2s_env[p_params->id].dma_id[0] = app_dma_init(&tx_dma_params, NULL);
    if (s_i2s_env[p_params->id].dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_i2s_env[p_params->id].handle.p_dmatx = app_dma_get_handle(s_i2s_env[p_params->id].dma_id[0]);
    s_i2s_env[p_params->id].handle.p_dmatx->p_parent = (void*)&s_i2s_env[p_params->id].handle;

    rx_dma_params.channel_number             = p_params->use_mode.rx_dma_channel;
    rx_dma_params.init.src_request           = (p_params->id == APP_I2S_ID_SLAVE) ? DMA_REQUEST_I2S_S_RX : DMA_REQUEST_I2S_M_RX;
    rx_dma_params.init.dst_request           = DMA_REQUEST_MEM;
    rx_dma_params.init.direction             = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment         = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    if (p_params->init.data_size <= I2S_DATASIZE_16BIT)
    {
        rx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_HALFWORD;
        rx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_HALFWORD;
    }
    else
    {
        rx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_WORD;
        rx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_WORD;
    }
    rx_dma_params.init.mode                  = DMA_NORMAL;
    rx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    s_i2s_env[p_params->id].dma_id[1] = app_dma_init(&rx_dma_params, NULL);
    if (s_i2s_env[p_params->id].dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_i2s_env[p_params->id].handle.p_dmarx = app_dma_get_handle(s_i2s_env[p_params->id].dma_id[1]);
    s_i2s_env[p_params->id].handle.p_dmarx->p_parent = (void*)&s_i2s_env[p_params->id].handle;

    return APP_DRV_SUCCESS;
}

static void app_i2s_event_call(i2s_handle_t *p_i2s, app_i2s_evt_type_t evt_type)
{
    app_i2s_evt_t i2s_evt;
    app_i2s_id_t id;

    if (p_i2s->p_instance == I2S_S)
    {
        id = APP_I2S_ID_SLAVE;
    }
    else if (p_i2s->p_instance == I2S_M)
    {
        id = APP_I2S_ID_MASTER;
    }

    i2s_evt.type = evt_type;
    if (evt_type == APP_I2S_EVT_ERROR)
    {
        i2s_evt.data.error_code = p_i2s->error_code;
    }
    else if (evt_type == APP_I2S_EVT_TX_CPLT)
    {
        i2s_evt.data.size = p_i2s->tx_xfer_size - p_i2s->tx_xfer_count;
    }
    else if (evt_type == APP_I2S_EVT_RX_DATA)
    {
        i2s_evt.data.size = p_i2s->rx_xfer_size - p_i2s->rx_xfer_count;
    }

    s_i2s_env[id].start_flag = false;
    if(s_i2s_env[id].evt_handler != NULL)
    {
        s_i2s_env[id].evt_handler(&i2s_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_i2s_init(app_i2s_params_t *p_params, app_i2s_evt_handler_t evt_handler)
{
    uint8_t id = p_params->id;
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (p_params == NULL)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_I2S_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_systick_init();

    app_err_code = i2s_gpio_config(p_params->id, p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    if(p_params->use_mode.type == APP_I2S_TYPE_DMA)
    {
        app_err_code = app_i2s_config_dma(p_params);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    if(p_params->use_mode.type != APP_I2S_TYPE_POLLING)
    {
        hal_nvic_clear_pending_irq(s_i2s_irq[id]);
        hal_nvic_enable_irq(s_i2s_irq[id]);
    }

    s_i2s_env[id].use_mode.type = p_params->use_mode.type;
    s_i2s_env[id].use_mode.rx_dma_channel = p_params->use_mode.rx_dma_channel;
    s_i2s_env[id].use_mode.tx_dma_channel = p_params->use_mode.tx_dma_channel;
    memcpy(&s_i2s_env[id].pin_cfg, &p_params->pin_cfg, sizeof(app_i2s_pin_cfg_t));
    s_i2s_env[id].evt_handler = evt_handler;

    memcpy(&s_i2s_env[id].handle.init, &p_params->init, sizeof(i2s_init_t));
    s_i2s_env[id].handle.p_instance = (i2s_regs_t *)s_i2s_instance[id];

    hal_err_code = hal_i2s_deinit(&s_i2s_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);
    hal_err_code = hal_i2s_init(&s_i2s_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    if (s_sleep_cb_registered_flag == false)// register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_i2s_pwr_id = pwr_register_sleep_cb(&i2s_sleep_cb);
        if (s_i2s_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_i2s_env[id].i2s_state = APP_I2S_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_deinit(app_i2s_id_t id)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if ((id >= APP_I2S_ID_MAX) || (s_i2s_env[id].i2s_state == APP_I2S_INVALID))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_err_code = app_io_deinit(s_i2s_env[id].pin_cfg.ws.type, s_i2s_env[id].pin_cfg.ws.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_i2s_env[id].pin_cfg.sdo.type, s_i2s_env[id].pin_cfg.sdo.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_i2s_env[id].pin_cfg.sdi.type, s_i2s_env[id].pin_cfg.sdi.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_i2s_env[id].pin_cfg.sclk.type, s_i2s_env[id].pin_cfg.sclk.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    hal_nvic_disable_irq(s_i2s_irq[id]);
    if(s_i2s_env[id].use_mode.type == APP_I2S_TYPE_DMA)
    {
        app_dma_deinit(s_i2s_env[id].dma_id[0]);
        app_dma_deinit(s_i2s_env[id].dma_id[1]);
    }
    s_i2s_env[id].i2s_state = APP_I2S_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    if(s_i2s_env[APP_I2S_ID_SLAVE].i2s_state == APP_I2S_INVALID && 
        s_i2s_env[APP_I2S_ID_MASTER].i2s_state == APP_I2S_INVALID)
    {
         pwr_unregister_sleep_cb(s_i2s_pwr_id);
         s_sleep_cb_registered_flag = false;
    }
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_err_code = hal_i2s_deinit(&s_i2s_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_receive_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_I2S_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2s_env[id].i2s_state == APP_I2S_INVALID ||
        s_i2s_env[id].use_mode.type == APP_I2S_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_i2s_env[id].start_flag == false)
    {
        s_i2s_env[id].start_flag = true;
        switch (s_i2s_env[id].use_mode.type)
        {
            case APP_I2S_TYPE_INTERRUPT:
                err_code = hal_i2s_receive_it(&s_i2s_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_I2S_TYPE_DMA:
                err_code = hal_i2s_receive_dma(&s_i2s_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_receive_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_I2S_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2s_env[id].i2s_state == APP_I2S_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_i2s_receive(&s_i2s_env[id].handle, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_transmit_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_I2S_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2s_env[id].i2s_state == APP_I2S_INVALID ||
        s_i2s_env[id].use_mode.type == APP_I2S_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_i2s_env[id].start_flag == false)
    {
        s_i2s_env[id].start_flag = true;
        switch (s_i2s_env[id].use_mode.type)
        {
            case APP_I2S_TYPE_INTERRUPT:
                err_code = hal_i2s_transmit_it(&s_i2s_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_I2S_TYPE_DMA:
                err_code =  hal_i2s_transmit_dma(&s_i2s_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_transmit_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_I2S_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2s_env[id].i2s_state == APP_I2S_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code =  hal_i2s_transmit(&s_i2s_env[id].handle, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_enable_clock(app_i2s_id_t id)
{
    if (id != APP_I2S_ID_MASTER || s_i2s_env[id].i2s_state == APP_I2S_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    __HAL_I2S_ENABLE_CLOCK(&s_i2s_env[id].handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_disable_clock(app_i2s_id_t id)
{
    if (id != APP_I2S_ID_MASTER  || s_i2s_env[id].i2s_state == APP_I2S_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    __HAL_I2S_DISABLE_CLOCK(&s_i2s_env[id].handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_flush_tx_fifo(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX  || s_i2s_env[id].i2s_state == APP_I2S_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    __HAL_I2S_FLUSH_TX_FIFO(&s_i2s_env[id].handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2s_flush_rx_fifo(app_i2s_id_t id)
{
    if (id >= APP_I2S_ID_MAX  || s_i2s_env[id].i2s_state == APP_I2S_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    __HAL_I2S_FLUSH_RX_FIFO(&s_i2s_env[id].handle);

    return APP_DRV_SUCCESS;
}

void hal_i2s_tx_cplt_callback(i2s_handle_t *p_i2s)
{
    app_i2s_event_call(p_i2s, APP_I2S_EVT_TX_CPLT); 
}

void hal_i2s_rx_cplt_callback(i2s_handle_t *p_i2s)
{
    app_i2s_event_call(p_i2s, APP_I2S_EVT_RX_DATA);
}

void hal_i2s_error_callback(i2s_handle_t *p_i2s)
{
    app_i2s_event_call(p_i2s, APP_I2S_EVT_ERROR);
}

void I2S_S_IRQHandler(void)
{
    hal_i2s_irq_handler(&s_i2s_env[APP_I2S_ID_SLAVE].handle);
}

void I2S_M_IRQHandler(void)
{
    hal_i2s_irq_handler(&s_i2s_env[APP_I2S_ID_MASTER].handle);
}

#endif

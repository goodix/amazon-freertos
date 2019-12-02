/**
  ****************************************************************************************
  * @file    app_i2c.c
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
#include "app_i2c.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include <string.h>

#ifdef HAL_I2C_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App i2c state types. */
typedef enum
{
   APP_I2C_INVALID = 0,
   APP_I2C_ACTIVITY,
} app_i2c_state_t;

struct i2c_env_t
{
    app_i2c_evt_handler_t   evt_handler;
    i2c_handle_t            handle;
    app_i2c_mode_t          use_mode;
    app_i2c_role_t          role;
    app_i2c_pin_cfg_t       pin_cfg;
    dma_id_t                dma_id[2];
    app_i2c_state_t         i2c_state;
    bool                    start_flag;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool i2c_prepare_for_sleep(void);
static void i2c_sleep_canceled(void);
static void i2c_wake_up_ind(void);
static uint16_t i2c_gpio_config(app_i2c_pin_cfg_t pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type   s_i2c_irq[APP_I2C_ID_MAX]      = { I2C0_IRQn, I2C1_IRQn };
static const uint32_t    s_i2c_instance[APP_I2C_ID_MAX] = { I2C0_BASE, I2C1_BASE };

struct i2c_env_t s_i2c_env[APP_I2C_ID_MAX];
static bool      s_sleep_cb_registered_flag = false;
static pwr_id_t  s_i2c_pwr_id;

static const app_sleep_callbacks_t i2c_sleep_cb =
{
    .app_prepare_for_sleep = i2c_prepare_for_sleep,
    .app_sleep_canceled    = i2c_sleep_canceled,
    .app_wake_up_ind       = i2c_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool i2c_prepare_for_sleep(void)
{
    hal_i2c_state_t state;
    uint8_t i;

    for (i = 0; i < APP_I2C_ID_MAX; i++)
    {
        if (s_i2c_env[i].i2c_state == APP_I2C_ACTIVITY)
        {
            state = hal_i2c_get_state(&s_i2c_env[i].handle);
            if (state > HAL_I2C_STATE_READY && state < HAL_I2C_STATE_LISTEN)
            {
                return false;
            }
        }
    }

    return true;
}


static void i2c_sleep_canceled(void)
{
}

static void i2c_wake_up_ind(void)
{
    uint8_t i;

    for (i = 0; i < APP_I2C_ID_MAX; i++)
    {
        if (s_i2c_env[i].i2c_state == APP_I2C_ACTIVITY)
        {
            i2c_gpio_config(s_i2c_env[i].pin_cfg);

            if(s_i2c_env[i].use_mode.type == APP_I2C_TYPE_INTERRUPT)
            {
                hal_nvic_clear_pending_irq(s_i2c_irq[i]);
                hal_nvic_enable_irq(s_i2c_irq[i]);
            }

            hal_i2c_deinit(&s_i2c_env[i].handle);
            hal_i2c_init(&s_i2c_env[i].handle);
        }
    }
}

static uint16_t i2c_gpio_config(app_i2c_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = pin_cfg.scl.pin;
    io_init.mux  = pin_cfg.scl.mux;
    err_code = app_io_init(pin_cfg.scl.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);
    
    io_init.pin  = pin_cfg.sda.pin;
    io_init.mux  = pin_cfg.sda.mux;
    err_code = app_io_init(pin_cfg.sda.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static uint16_t app_i2c_config_dma(app_i2c_params_t *p_params)
{
    app_dma_params_t tx_dma_params;
    app_dma_params_t rx_dma_params;

    tx_dma_params.channel_number             = p_params->use_mode.tx_dma_channel;
    tx_dma_params.init.src_request           = DMA_REQUEST_MEM;
    tx_dma_params.init.dst_request           = (p_params->id == APP_I2C_ID_0) ? DMA_REQUEST_I2C0_TX : DMA_REQUEST_I2C1_TX;
    tx_dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
    tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    tx_dma_params.init.mode                  = DMA_NORMAL;
    tx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    s_i2c_env[p_params->id].dma_id[0] = app_dma_init(&tx_dma_params, NULL);
    if (s_i2c_env[p_params->id].dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_i2c_env[p_params->id].handle.p_dmatx = app_dma_get_handle(s_i2c_env[p_params->id].dma_id[0]);
    s_i2c_env[p_params->id].handle.p_dmatx->p_parent = (void*)&s_i2c_env[p_params->id].handle;

    rx_dma_params.channel_number             = p_params->use_mode.rx_dma_channel;
    rx_dma_params.init.src_request           = (p_params->id == APP_I2C_ID_0) ? DMA_REQUEST_I2C0_RX : DMA_REQUEST_I2C1_RX;
    rx_dma_params.init.dst_request           = DMA_REQUEST_MEM;
    rx_dma_params.init.direction             = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment         = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    rx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
    rx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    rx_dma_params.init.mode                  = DMA_NORMAL;
    rx_dma_params.init.priority              = DMA_PRIORITY_LOW;

    s_i2c_env[p_params->id].dma_id[1] = app_dma_init(&rx_dma_params, NULL);
    if (s_i2c_env[p_params->id].dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_i2c_env[p_params->id].handle.p_dmarx = app_dma_get_handle(s_i2c_env[p_params->id].dma_id[1]);
    s_i2c_env[p_params->id].handle.p_dmarx->p_parent = (void*)&s_i2c_env[p_params->id].handle;

    return APP_DRV_SUCCESS;
}

static void app_i2c_event_call(i2c_handle_t *p_i2c, app_i2c_evt_type_t evt_type)
{
    app_i2c_evt_t i2c_evt;
    app_i2c_id_t id;
    if(p_i2c->p_instance == I2C0)
    {
        id = APP_I2C_ID_0;
    }
    else if(p_i2c->p_instance == I2C1)
    {
        id = APP_I2C_ID_1;
    }
    i2c_evt.type = evt_type;
    if(evt_type == APP_I2C_EVT_ERROR)
    {
        i2c_evt.data.error_code = p_i2c->error_code;
    }
    else if(evt_type == APP_I2C_EVT_TX_CPLT)
    {
        i2c_evt.data.size = p_i2c->xfer_size - p_i2c->xfer_count;
    }
    else if(evt_type == APP_I2C_EVT_RX_DATA)
    {
        i2c_evt.data.size = p_i2c->xfer_size - p_i2c->xfer_count;
    }

    s_i2c_env[id].start_flag = false;
    if (s_i2c_env[id].evt_handler != NULL)
    {
        s_i2c_env[id].evt_handler(&i2c_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_i2c_init(app_i2c_params_t *p_params, app_i2c_evt_handler_t evt_handler)
{
    uint8_t       id       = p_params->id;
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_I2C_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_systick_init();

    app_err_code = i2c_gpio_config(p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    if(p_params->use_mode.type == APP_I2C_TYPE_DMA)
    {
        app_err_code = app_i2c_config_dma(p_params);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    if(p_params->use_mode.type != APP_I2C_TYPE_POLLING)
    {
        hal_nvic_clear_pending_irq(s_i2c_irq[id]);
        hal_nvic_enable_irq(s_i2c_irq[id]);
    }

    s_i2c_env[id].use_mode.type = p_params->use_mode.type;
    s_i2c_env[id].use_mode.rx_dma_channel = p_params->use_mode.rx_dma_channel;
    s_i2c_env[id].use_mode.tx_dma_channel = p_params->use_mode.tx_dma_channel;
    s_i2c_env[id].role = p_params->role;
    memcpy(&s_i2c_env[id].pin_cfg, &p_params->pin_cfg, sizeof(app_i2c_pin_cfg_t));
    s_i2c_env[id].evt_handler = evt_handler;

    memcpy(&s_i2c_env[id].handle.init, &p_params->init, sizeof(i2c_init_t));
    s_i2c_env[id].handle.p_instance = (i2c_regs_t *)s_i2c_instance[id];
    hal_err_code = hal_i2c_deinit(&s_i2c_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code = hal_i2c_init(&s_i2c_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    if(s_sleep_cb_registered_flag == false)// register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_i2c_pwr_id = pwr_register_sleep_cb(&i2c_sleep_cb);

        if (s_i2c_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }
    s_i2c_env[id].i2c_state = APP_I2C_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_deinit(app_i2c_id_t id)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if ((id >= APP_I2C_ID_MAX) || (s_i2c_env[id].i2c_state == APP_I2C_INVALID))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_err_code = app_io_deinit(s_i2c_env[id].pin_cfg.scl.type, s_i2c_env[id].pin_cfg.scl.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_i2c_env[id].pin_cfg.sda.type, s_i2c_env[id].pin_cfg.sda.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    hal_nvic_disable_irq(s_i2c_irq[id]);
    if(s_i2c_env[id].use_mode.type == APP_I2C_TYPE_DMA)
    {
        app_dma_deinit(s_i2c_env[id].dma_id[0]);
        app_dma_deinit(s_i2c_env[id].dma_id[1]);
    }
    s_i2c_env[id].i2c_state = APP_I2C_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    if(s_i2c_env[APP_I2C_ID_0].i2c_state == APP_I2C_INVALID && 
        s_i2c_env[APP_I2C_ID_1].i2c_state == APP_I2C_INVALID)
    {
         pwr_unregister_sleep_cb(s_i2c_pwr_id);
         s_sleep_cb_registered_flag = false;
    }
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_err_code = hal_i2c_deinit(&s_i2c_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_receive_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(s_i2c_env[id].role)
    {
        case APP_I2C_ROLE_MASTER:
            err_code = hal_i2c_master_receive(&s_i2c_env[id].handle, target_address, p_data, size, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_I2C_ROLE_SLAVE:
            err_code = hal_i2c_slave_receive(&s_i2c_env[id].handle, p_data, size, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_receive_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID ||
        s_i2c_env[id].use_mode.type == APP_I2C_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if(s_i2c_env[id].start_flag == false)
    {
        s_i2c_env[id].start_flag = true;
        switch(s_i2c_env[id].role)
        {
            case APP_I2C_ROLE_MASTER:
                switch(s_i2c_env[id].use_mode.type)
                {
                    case APP_I2C_TYPE_INTERRUPT:
                        err_code = hal_i2c_master_receive_it(&s_i2c_env[id].handle, target_address, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    case APP_I2C_TYPE_DMA:
                        err_code = hal_i2c_master_receive_dma(&s_i2c_env[id].handle, target_address, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    default:
                        break;
                }
                break;

            case APP_I2C_ROLE_SLAVE:
                switch(s_i2c_env[id].use_mode.type)
                {
                    case APP_I2C_TYPE_INTERRUPT:
                        err_code = hal_i2c_slave_receive_it(&s_i2c_env[id].handle, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    case APP_I2C_TYPE_DMA:
                        err_code = hal_i2c_slave_receive_dma(&s_i2c_env[id].handle, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_transmit_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch(s_i2c_env[id].role)
    {
        case APP_I2C_ROLE_MASTER:
            err_code = hal_i2c_master_transmit(&s_i2c_env[id].handle, target_address, p_data, size, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_I2C_ROLE_SLAVE:
            err_code = hal_i2c_slave_transmit(&s_i2c_env[id].handle, p_data, size, timeout);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_transmit_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID ||
        s_i2c_env[id].use_mode.type == APP_I2C_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if(s_i2c_env[id].start_flag == false)
    {
        s_i2c_env[id].start_flag = true;
        switch(s_i2c_env[id].role)
        {
            case APP_I2C_ROLE_MASTER:
                switch(s_i2c_env[id].use_mode.type)
                {
                    case APP_I2C_TYPE_INTERRUPT:
                        err_code = hal_i2c_master_transmit_it(&s_i2c_env[id].handle, target_address, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    case APP_I2C_TYPE_DMA:
                        err_code = hal_i2c_master_transmit_dma(&s_i2c_env[id].handle, target_address, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    default:
                        break;
                }
                break;

            case APP_I2C_ROLE_SLAVE:
                switch(s_i2c_env[id].use_mode.type)
                {
                    case APP_I2C_TYPE_INTERRUPT:
                        err_code = hal_i2c_slave_transmit_it(&s_i2c_env[id].handle, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    case APP_I2C_TYPE_DMA:
                        err_code = hal_i2c_slave_transmit_dma(&s_i2c_env[id].handle, p_data, size);
                        HAL_ERR_CODE_CHECK(err_code);
                        break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}


uint16_t app_i2c_mem_read_sync(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_i2c_mem_read(&s_i2c_env[id].handle, dev_address, mem_address, mem_addr_size, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_mem_read_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID ||
        s_i2c_env[id].use_mode.type == APP_I2C_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if(s_i2c_env[id].start_flag == false)
    {
        s_i2c_env[id].start_flag = true;

        switch(s_i2c_env[id].use_mode.type)
        {
            case APP_I2C_TYPE_INTERRUPT:
                err_code = hal_i2c_mem_read_it(&s_i2c_env[id].handle, dev_address, mem_address, mem_addr_size, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_I2C_TYPE_DMA:
                err_code = hal_i2c_mem_read_dma(&s_i2c_env[id].handle, dev_address, mem_address, mem_addr_size, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_mem_write_sync(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_i2c_mem_write(&s_i2c_env[id].handle, dev_address, mem_address, mem_addr_size, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_i2c_mem_write_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_I2C_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_i2c_env[id].i2c_state == APP_I2C_INVALID ||
        s_i2c_env[id].use_mode.type == APP_I2C_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if(s_i2c_env[id].start_flag == false)
    {
        s_i2c_env[id].start_flag = true;

        switch(s_i2c_env[id].use_mode.type)
        {
            case APP_I2C_TYPE_INTERRUPT:
                err_code = hal_i2c_mem_write_it(&s_i2c_env[id].handle, dev_address, mem_address, mem_addr_size, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_I2C_TYPE_DMA:
                err_code = hal_i2c_mem_write_dma(&s_i2c_env[id].handle, dev_address, mem_address, mem_addr_size, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

void hal_i2c_master_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_TX_CPLT);
}

void hal_i2c_master_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_RX_DATA);
}

void hal_i2c_slave_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_TX_CPLT);
}

void hal_i2c_slave_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_RX_DATA);
}

void hal_i2c_mem_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_TX_CPLT);
}

void hal_i2c_mem_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_RX_DATA);
}

void hal_i2c_error_callback(i2c_handle_t *p_i2c)
{
    app_i2c_event_call(p_i2c, APP_I2C_EVT_ERROR);
}

void I2C0_IRQHandler(void)
{
    hal_i2c_irq_handler(&s_i2c_env[APP_I2C_ID_0].handle);
}

void I2C1_IRQHandler(void)
{
    hal_i2c_irq_handler(&s_i2c_env[APP_I2C_ID_1].handle);
}

#endif

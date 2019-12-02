/**
  ****************************************************************************************
  * @file    app_spi.c
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
#include "app_spi.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include <string.h>

#ifdef HAL_SPI_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

#define SPI_SMART_CS_LOW(id)                                      \
    do {                                                          \
            if(APP_SPI_ID_SLAVE != id)                            \
            {                                                     \
                app_io_write_pin(s_spi_env[id].pin_cfg.cs.type,   \
                                s_spi_env[id].pin_cfg.cs.pin,     \
                                APP_IO_PIN_RESET);                \
            }\
        } while(0)

#define SPI_SMART_CS_HIGH(id)                                     \
    do {                                                          \
            if(APP_SPI_ID_SLAVE != id)                            \
            {                                                     \
                app_io_write_pin(s_spi_env[id].pin_cfg.cs.type,   \
                                 s_spi_env[id].pin_cfg.cs.pin,    \
                                 APP_IO_PIN_SET);                 \
            }                                                     \
    } while(0)

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App spi state types. */
typedef enum
{
   APP_SPI_INVALID = 0,
   APP_SPI_ACTIVITY,
} app_spi_state_t;

struct spi_env_t
{
    app_spi_evt_handler_t   evt_handler;
    spi_handle_t            handle;
    app_spi_mode_t          use_mode;
    app_spi_pin_cfg_t       pin_cfg;
    dma_id_t                dma_id[2];
    app_spi_state_t         spi_state;
    bool                    start_flag;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool spi_prepare_for_sleep(void);
static void spi_sleep_canceled(void);
static void spi_wake_up_ind(void);
static uint16_t spi_gpio_config(app_spi_id_t id, app_spi_pin_cfg_t pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type s_spi_irq[APP_SPI_ID_MAX]      = {SPI_S_IRQn, SPI_M_IRQn};
static const uint32_t  s_spi_instance[APP_SPI_ID_MAX] = {SPIS_BASE, SPIM_BASE};

struct spi_env_t  s_spi_env[APP_SPI_ID_MAX];
static bool       s_sleep_cb_registered_flag = false;
static pwr_id_t   s_spi_pwr_id;

static const app_sleep_callbacks_t spi_sleep_cb =
{
    .app_prepare_for_sleep = spi_prepare_for_sleep,
    .app_sleep_canceled    = spi_sleep_canceled,
    .app_wake_up_ind       = spi_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool spi_prepare_for_sleep(void)
{
    hal_spi_state_t state;
    uint8_t i;

    for (i = 0; i < APP_SPI_ID_MAX; i++)
    {
        if (s_spi_env[i].spi_state == APP_SPI_ACTIVITY)
        {
            state = hal_spi_get_state(&s_spi_env[i].handle);
            if (state > HAL_SPI_STATE_READY && state < HAL_SPI_STATE_ABORT)
            {
                return false;
            }
        }
    }

    return true;
}

static void spi_sleep_canceled(void)
{
}

static void spi_wake_up_ind(void)
{
    uint8_t i;

    for (i = 0; i < APP_SPI_ID_MAX; i++)
    {
        if (s_spi_env[i].spi_state == APP_SPI_ACTIVITY)
        {
            spi_gpio_config((app_spi_id_t)i, s_spi_env[i].pin_cfg);
            if(s_spi_env[i].use_mode.type == APP_SPI_TYPE_INTERRUPT || \
               s_spi_env[i].use_mode.type == APP_SPI_TYPE_DMA)
            {
                hal_nvic_clear_pending_irq(s_spi_irq[i]);
                hal_nvic_enable_irq(s_spi_irq[i]);
            }

            hal_spi_deinit(&s_spi_env[i].handle);
            hal_spi_init(&s_spi_env[i].handle);
        }
    }
}

static uint16_t spi_gpio_config(app_spi_id_t id, app_spi_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if(id == APP_SPI_ID_SLAVE)
    {
        io_init.pull = APP_IO_PULLUP;
        io_init.mode = APP_IO_MODE_MUX;
        io_init.pin  = pin_cfg.cs.pin;
        io_init.mux  = pin_cfg.cs.mux;
        err_code = app_io_init(pin_cfg.cs.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }
    else
    {
        io_init.pull = APP_IO_PULLUP;
        io_init.mode = APP_IO_MODE_OUT_PUT;
        io_init.pin  = pin_cfg.cs.pin;
        io_init.mux  = APP_IO_MUX_7;
        err_code = app_io_init(pin_cfg.cs.type, &io_init);
        app_io_write_pin(pin_cfg.cs.type, pin_cfg.cs.pin, APP_IO_PIN_SET);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = pin_cfg.clk.pin;
    io_init.mux  = pin_cfg.clk.mux;
    err_code = app_io_init(pin_cfg.clk.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);
    
    io_init.pin  = pin_cfg.mosi.pin;
    io_init.mux  = pin_cfg.mosi.mux;
    err_code = app_io_init(pin_cfg.mosi.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pin  = pin_cfg.miso.pin;
    io_init.mux  = pin_cfg.miso.mux;
    err_code = app_io_init(pin_cfg.miso.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static uint16_t app_spi_config_dma(app_spi_params_t *p_params)
{
    app_dma_params_t tx_dma_params;
    app_dma_params_t rx_dma_params;

    tx_dma_params.channel_number             = p_params->use_mode.tx_dma_channel;
    tx_dma_params.init.src_request           = DMA_REQUEST_MEM;
    tx_dma_params.init.dst_request           = (p_params->id == APP_SPI_ID_SLAVE) ? DMA_REQUEST_SPIS_TX : DMA_REQUEST_SPIM_TX;
    tx_dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    if (p_params->init.data_size <= SPI_DATASIZE_8BIT)
    {
        tx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
        tx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    }
    else if (p_params->init.data_size <= SPI_DATASIZE_16BIT)
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

    s_spi_env[p_params->id].dma_id[0] = app_dma_init(&tx_dma_params, NULL);

    if (s_spi_env[p_params->id].dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_spi_env[p_params->id].handle.p_dmatx = app_dma_get_handle(s_spi_env[p_params->id].dma_id[0]);
    s_spi_env[p_params->id].handle.p_dmatx->p_parent = (void*)&s_spi_env[p_params->id].handle;

    rx_dma_params.channel_number             = p_params->use_mode.rx_dma_channel;
    rx_dma_params.init.src_request           = (p_params->id == APP_SPI_ID_SLAVE) ? DMA_REQUEST_SPIS_RX : DMA_REQUEST_SPIM_TX;
    rx_dma_params.init.dst_request           = DMA_REQUEST_MEM;
    rx_dma_params.init.direction             = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment         = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment         = DMA_DST_INCREMENT;
    if (p_params->init.data_size <= SPI_DATASIZE_8BIT)
    {
        rx_dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
        rx_dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    }
    else if (p_params->init.data_size <= SPI_DATASIZE_16BIT)
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

    s_spi_env[p_params->id].dma_id[1] = app_dma_init(&rx_dma_params, NULL);

    if (s_spi_env[p_params->id].dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_spi_env[p_params->id].handle.p_dmarx = app_dma_get_handle(s_spi_env[p_params->id].dma_id[1]);
    s_spi_env[p_params->id].handle.p_dmarx->p_parent = (void*)&s_spi_env[p_params->id].handle;

    return APP_DRV_SUCCESS;
}

static void app_spi_event_call(spi_handle_t *p_spi, app_spi_evt_type_t evt_type)
{
    app_spi_evt_t spi_evt;
    app_spi_id_t id;

    if (p_spi->p_instance == SPIS)
    {
        id = APP_SPI_ID_SLAVE;
    }
    else if (p_spi->p_instance == SPIM)
    {
        id = APP_SPI_ID_MASTER;
    }

    spi_evt.type = evt_type;
    if (evt_type == APP_SPI_EVT_ERROR)
    {
        spi_evt.data.error_code = p_spi->error_code;
    }
    else if (evt_type == APP_SPI_EVT_TX_CPLT)
    {
        spi_evt.data.size = p_spi->tx_xfer_size - p_spi->tx_xfer_count;
    }
    else if (evt_type == APP_SPI_EVT_RX_DATA)
    {
        spi_evt.data.size = p_spi->rx_xfer_size - p_spi->rx_xfer_count;
    }

    SPI_SMART_CS_HIGH(id);
    s_spi_env[id].start_flag = false;
    if(s_spi_env[id].evt_handler != NULL)
    {
        s_spi_env[id].evt_handler(&spi_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_spi_init(app_spi_params_t *p_params, app_spi_evt_handler_t evt_handler)
{
    uint8_t       id       = p_params->id;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_SPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_systick_init();

    err_code = spi_gpio_config(p_params->id, p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if(p_params->use_mode.type == APP_SPI_TYPE_DMA)
    {
        err_code = app_spi_config_dma(p_params);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    if(p_params->use_mode.type != APP_SPI_TYPE_POLLING)
    {
        hal_nvic_clear_pending_irq(s_spi_irq[id]);
        hal_nvic_enable_irq(s_spi_irq[id]);
    }

    s_spi_env[id].use_mode.type = p_params->use_mode.type;
    s_spi_env[id].use_mode.rx_dma_channel = p_params->use_mode.rx_dma_channel;
    s_spi_env[id].use_mode.tx_dma_channel = p_params->use_mode.tx_dma_channel;
    memcpy(&s_spi_env[id].pin_cfg, &p_params->pin_cfg, sizeof(app_spi_pin_cfg_t));
    s_spi_env[id].evt_handler = evt_handler;

    memcpy(&s_spi_env[id].handle.init, &p_params->init, sizeof(spi_init_t));
    s_spi_env[id].handle.p_instance = (ssi_regs_t *)s_spi_instance[id];

    hal_spi_deinit(&s_spi_env[id].handle);
    hal_spi_init(&s_spi_env[id].handle);

    if (s_sleep_cb_registered_flag == false)// register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_spi_pwr_id = pwr_register_sleep_cb(&spi_sleep_cb);

        if (s_spi_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_spi_env[id].spi_state = APP_SPI_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_deinit(app_spi_id_t id)
{
    if ((id >= APP_SPI_ID_MAX) || (s_spi_env[id].spi_state == APP_SPI_INVALID))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_io_deinit(s_spi_env[id].pin_cfg.cs.type, s_spi_env[id].pin_cfg.cs.pin);
    app_io_deinit(s_spi_env[id].pin_cfg.clk.type, s_spi_env[id].pin_cfg.clk.pin);
    app_io_deinit(s_spi_env[id].pin_cfg.mosi.type, s_spi_env[id].pin_cfg.mosi.pin);
    app_io_deinit(s_spi_env[id].pin_cfg.miso.type, s_spi_env[id].pin_cfg.miso.pin);

    hal_nvic_disable_irq(s_spi_irq[id]);
    if(s_spi_env[id].use_mode.type == APP_SPI_TYPE_DMA)
    {
        app_dma_deinit(s_spi_env[id].dma_id[0]);
        app_dma_deinit(s_spi_env[id].dma_id[1]);
    }
    s_spi_env[id].spi_state = APP_SPI_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    if(s_spi_env[APP_SPI_ID_SLAVE].spi_state == APP_SPI_INVALID && 
       s_spi_env[APP_SPI_ID_MASTER].spi_state == APP_SPI_INVALID)
    {
         pwr_unregister_sleep_cb(s_spi_pwr_id);
         s_sleep_cb_registered_flag = false;
    }
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_spi_deinit(&s_spi_env[id].handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_receive_async(app_spi_id_t id, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_spi_env[id].spi_state == APP_SPI_INVALID ||
        s_spi_env[id].use_mode.type == APP_SPI_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_spi_env[id].start_flag == false)
    {
        s_spi_env[id].start_flag = true;
        switch (s_spi_env[id].use_mode.type)
        {
            case APP_SPI_TYPE_INTERRUPT:
                SPI_SMART_CS_LOW(id);
                err_code = hal_spi_receive_it(&s_spi_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_SPI_TYPE_DMA:
                SPI_SMART_CS_LOW(id);
                err_code = hal_spi_receive_dma(&s_spi_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_receive_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_spi_env[id].spi_state == APP_SPI_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    SPI_SMART_CS_LOW(id);
    err_code = hal_spi_receive(&s_spi_env[id].handle, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);
    SPI_SMART_CS_HIGH(id);

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_transmit_async(app_spi_id_t id, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_spi_env[id].spi_state == APP_SPI_INVALID ||
        s_spi_env[id].use_mode.type == APP_SPI_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_spi_env[id].start_flag == false)
    {
        s_spi_env[id].start_flag = true;
        switch (s_spi_env[id].use_mode.type)
        {
            case APP_SPI_TYPE_INTERRUPT:
                SPI_SMART_CS_LOW(id);
                err_code = hal_spi_transmit_it(&s_spi_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_SPI_TYPE_DMA:
                SPI_SMART_CS_LOW(id);
                err_code = hal_spi_transmit_dma(&s_spi_env[id].handle, p_data, size);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_spi_transmit_sync(app_spi_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_SPI_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_spi_env[id].spi_state == APP_SPI_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    SPI_SMART_CS_LOW(id);
    err_code = hal_spi_transmit(&s_spi_env[id].handle, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);
    SPI_SMART_CS_HIGH(id);

    return APP_DRV_SUCCESS;
}

void hal_spi_tx_cplt_callback(spi_handle_t *p_spi)
{
    app_spi_event_call(p_spi, APP_SPI_EVT_TX_CPLT); 
}

void hal_spi_rx_cplt_callback(spi_handle_t *p_spi)
{
    app_spi_event_call(p_spi, APP_SPI_EVT_RX_DATA);
}

void hal_spi_tx_rx_cplt_callback(spi_handle_t *p_spi)
{
    app_spi_event_call(p_spi, APP_SPI_EVT_TX_RX);
}

void hal_spi_error_callback(spi_handle_t *p_spi)
{
    app_spi_event_call(p_spi, APP_SPI_EVT_ERROR);
}

void SPI_S_IRQHandler(void)
{
    hal_spi_irq_handler(&s_spi_env[APP_SPI_ID_SLAVE].handle);
}

void SPI_M_IRQHandler(void)
{
    hal_spi_irq_handler(&s_spi_env[APP_SPI_ID_MASTER].handle);
}

#endif


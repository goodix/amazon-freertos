/**
  ****************************************************************************************
  * @file    app_uart.c
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
#include "app_uart.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include <string.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define TX_ONCE_MAX_SIZE     128

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App uart state types. */
typedef enum
{
   APP_UART_INVALID = 0,
   APP_UART_ACTIVITY,
} app_uart_state_t;

struct uart_env_t
{
    app_uart_evt_handler_t evt_handler;
    uart_handle_t          handle;
    app_uart_mode_t        use_mode;
    app_uart_pin_cfg_t     pin_cfg;
    dma_id_t               dma_id[2];
    app_uart_state_t       uart_state;
    ring_buffer_t          tx_ring_buffer;
    bool                   start_tx_flag;
    bool                   start_flush_flag;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool uart_prepare_for_sleep(void);
static void uart_sleep_canceled(void);
static void uart_wake_up_ind(void);
static uint16_t uart_gpio_config(uint32_t hw_flow_ctrl, app_uart_pin_cfg_t pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type s_uart_irq[APP_UART_ID_MAX]      = {UART0_IRQn, UART1_IRQn};
static const uint32_t  s_uart_instance[APP_UART_ID_MAX] = {UART0_BASE, UART1_BASE};

struct uart_env_t s_uart_env[APP_UART_ID_MAX];
static bool       s_sleep_cb_registered_flag = false;
static pwr_id_t   s_uart_pwr_id;
static uint8_t    s_tx_send_buf[TX_ONCE_MAX_SIZE];

static const app_sleep_callbacks_t uart_sleep_cb =
{
    .app_prepare_for_sleep = uart_prepare_for_sleep,
    .app_sleep_canceled    = uart_sleep_canceled,
    .app_wake_up_ind       = uart_wake_up_ind,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

static bool uart_prepare_for_sleep(void)
{
    hal_uart_state_t state;

    for (uint8_t i = 0; i < APP_UART_ID_MAX; i++)
    {
        if (s_uart_env[i].uart_state == APP_UART_ACTIVITY)
        {
            state = hal_uart_get_state(&s_uart_env[i].handle);
            if (state > HAL_UART_STATE_READY && state < HAL_UART_STATE_TIMEOUT)
            {
                return false;
            }
        }
    }

    return true;
}


static void uart_sleep_canceled(void)
{

}

static void uart_wake_up_ind(void)
{
    uint8_t i;

    for (i = 0; i < APP_UART_ID_MAX; i++)
    {
        if (s_uart_env[i].uart_state == APP_UART_ACTIVITY)
        {
            uart_gpio_config(s_uart_env[i].handle.init.hw_flow_ctrl, s_uart_env[i].pin_cfg);

            if (s_uart_env[i].use_mode.type == APP_UART_TYPE_INTERRUPT)
            {
                hal_nvic_clear_pending_irq(s_uart_irq[i]);
                hal_nvic_enable_irq(s_uart_irq[i]);
            }

            hal_uart_deinit(&s_uart_env[i].handle);
            hal_uart_init(&s_uart_env[i].handle);
        }
    }
}

static uint16_t uart_gpio_config(uint32_t hw_flow_ctrl, app_uart_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init  = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = pin_cfg.tx.pin;
    io_init.mux  = pin_cfg.tx.mux;
    err_code = app_io_init(pin_cfg.tx.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pin  = pin_cfg.rx.pin;
    io_init.mux  = pin_cfg.rx.mux;
    err_code = app_io_init(pin_cfg.rx.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (UART_HWCONTROL_RTS_CTS == hw_flow_ctrl)
    {
        io_init.pin  = pin_cfg.cts.pin;
        io_init.mux  = pin_cfg.cts.mux;
        err_code = app_io_init(pin_cfg.cts.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);

        io_init.pin  = pin_cfg.rts.pin;
        io_init.mux  = pin_cfg.rts.mux;
        err_code = app_io_init(pin_cfg.rts.type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    return err_code;
}

static uint16_t app_uart_config_dma(app_uart_params_t *p_params)
{
    app_dma_params_t tx_dma_params;
    app_dma_params_t rx_dma_params;

    tx_dma_params.channel_number           = p_params->use_mode.tx_dma_channel;
    tx_dma_params.init.src_request         = DMA_REQUEST_MEM;
    tx_dma_params.init.dst_request         = DMA_REQUEST_UART0_TX;
    tx_dma_params.init.direction           = DMA_MEMORY_TO_PERIPH;
    tx_dma_params.init.src_increment       = DMA_SRC_INCREMENT;
    tx_dma_params.init.dst_increment       = DMA_DST_NO_CHANGE;
    tx_dma_params.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    tx_dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    tx_dma_params.init.mode                = DMA_NORMAL;
    tx_dma_params.init.priority            = DMA_PRIORITY_LOW;
    s_uart_env[p_params->id].dma_id[0]     = app_dma_init(&tx_dma_params, NULL);

    if (s_uart_env[p_params->id].dma_id[0] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    /* Associate the initialized DMA handle to the UART handle */
    s_uart_env[p_params->id].handle.p_dmatx = app_dma_get_handle(s_uart_env[p_params->id].dma_id[0]);
    s_uart_env[p_params->id].handle.p_dmatx->p_parent = (void*)&s_uart_env[p_params->id].handle;

    rx_dma_params.channel_number           = p_params->use_mode.rx_dma_channel;
    rx_dma_params.init.src_request         = DMA_REQUEST_UART0_RX;
    rx_dma_params.init.dst_request         = DMA_REQUEST_MEM;
    rx_dma_params.init.direction           = DMA_PERIPH_TO_MEMORY;
    rx_dma_params.init.src_increment       = DMA_SRC_NO_CHANGE;
    rx_dma_params.init.dst_increment       = DMA_DST_INCREMENT;
    rx_dma_params.init.src_data_alignment  = DMA_SDATAALIGN_BYTE;
    rx_dma_params.init.dst_data_alignment  = DMA_DDATAALIGN_BYTE;
    rx_dma_params.init.mode                = DMA_NORMAL;
    rx_dma_params.init.priority            = DMA_PRIORITY_HIGH;
    s_uart_env[p_params->id].dma_id[1] = app_dma_init(&rx_dma_params, NULL);

    if (s_uart_env[p_params->id].dma_id[1] < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    /* Associate the initialized DMA handle to the the UART handle */
    s_uart_env[p_params->id].handle.p_dmarx = app_dma_get_handle(s_uart_env[p_params->id].dma_id[1]);
    s_uart_env[p_params->id].handle.p_dmarx->p_parent = (void*)&s_uart_env[p_params->id].handle;

    return APP_DRV_SUCCESS;
}

static uint16_t app_uart_start_transmit_async(app_uart_id_t id)
{
    uint16_t items_count = ring_buffer_items_count_get(&s_uart_env[id].tx_ring_buffer);
    uint16_t send_size   = items_count;
    hal_status_t err_code;

    if (items_count == 0)
    {
        s_uart_env[id].start_tx_flag = false;
        return APP_DRV_SUCCESS;
    }

    if (items_count >= TX_ONCE_MAX_SIZE)
    {
        ring_buffer_read(&s_uart_env[id].tx_ring_buffer, s_tx_send_buf, TX_ONCE_MAX_SIZE);
        send_size = TX_ONCE_MAX_SIZE;
    }
    else 
    {
        ring_buffer_read(&s_uart_env[id].tx_ring_buffer, s_tx_send_buf, items_count);
    }

    switch (s_uart_env[id].use_mode.type)
    {
        case APP_UART_TYPE_INTERRUPT:
            err_code = hal_uart_transmit_it(&s_uart_env[id].handle, s_tx_send_buf, send_size);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_UART_TYPE_DMA:
            err_code = hal_uart_transmit_dma(&s_uart_env[id].handle, s_tx_send_buf, send_size);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}


static void app_uart_event_call(uart_handle_t *p_uart, app_uart_evt_type_t evt_type)
{
    app_uart_evt_t uart_evt;
    app_uart_id_t id;

    if (p_uart->p_instance == UART0)
    {
        id = APP_UART_ID_0;
    }
    else if (p_uart->p_instance == UART1)
    {
        id = APP_UART_ID_1;
    }

    uart_evt.type = evt_type;
    if (evt_type == APP_UART_EVT_ERROR)
    {
        uart_evt.data.error_code = p_uart->error_code;
        if (s_uart_env[id].evt_handler != NULL)
        {
            s_uart_env[id].evt_handler(&uart_evt);
        }
    }
    else if (evt_type == APP_UART_EVT_TX_CPLT)
    {
        uart_evt.data.size = p_uart->tx_xfer_size - p_uart->tx_xfer_count;
        app_uart_start_transmit_async(id);
        if(s_uart_env[id].start_tx_flag == false && s_uart_env[id].evt_handler != NULL)
        {
            s_uart_env[id].evt_handler(&uart_evt);
        }
    }
    else if (evt_type == APP_UART_EVT_RX_DATA)
    {
        uart_evt.data.size = p_uart->rx_xfer_size - p_uart->rx_xfer_count;
        if (s_uart_env[id].evt_handler != NULL)
        {
            s_uart_env[id].evt_handler(&uart_evt);
        }
    }
    else if (evt_type == APP_UART_EVT_ABORT_TX)
    {
        s_uart_env[id].start_tx_flag = false;
    }
    else if (evt_type == APP_UART_EVT_ABORT_RX)
    {

    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_uart_init(app_uart_params_t *p_params, app_uart_evt_handler_t evt_handler, app_uart_tx_buf_t *tx_buffer)
{
    uint8_t       id       = p_params->id;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_UART_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    if (APP_UART_TYPE_POLLING != p_params->use_mode.type && NULL == tx_buffer)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    app_systick_init();

    err_code = uart_gpio_config(p_params->init.hw_flow_ctrl, p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (APP_UART_TYPE_DMA == p_params->use_mode.type)
    {
        if (id != APP_UART_ID_0)
        {
            return APP_DRV_ERR_INVALID_ID;
        }
        else
        {
            err_code = app_uart_config_dma(p_params);
            APP_DRV_ERR_CODE_CHECK(err_code);
        }
    }

    if (p_params->use_mode.type != APP_UART_TYPE_POLLING)
    {
        ring_buffer_init(&s_uart_env[id].tx_ring_buffer, tx_buffer->tx_buf, tx_buffer->tx_buf_size);
        hal_nvic_clear_pending_irq(s_uart_irq[id]);
        hal_nvic_enable_irq(s_uart_irq[id]);
    }

    s_uart_env[id].use_mode.type = p_params->use_mode.type;
    s_uart_env[id].use_mode.rx_dma_channel = p_params->use_mode.rx_dma_channel;
    s_uart_env[id].use_mode.tx_dma_channel = p_params->use_mode.tx_dma_channel;
    memcpy(&s_uart_env[id].pin_cfg, &p_params->pin_cfg, sizeof(app_uart_pin_cfg_t));
    s_uart_env[id].evt_handler = evt_handler;

    memcpy(&s_uart_env[id].handle.init, &p_params->init, sizeof(uart_init_t));
    s_uart_env[id].handle.p_instance = (uart_regs_t *)s_uart_instance[id];

    hal_uart_deinit(&s_uart_env[id].handle);
    hal_uart_init(&s_uart_env[id].handle);

    if (s_sleep_cb_registered_flag == false)// register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_uart_pwr_id = pwr_register_sleep_cb(&uart_sleep_cb);

        if (s_uart_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_uart_env[id].uart_state = APP_UART_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_deinit(app_uart_id_t id)
{
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    if ((id >= APP_UART_ID_MAX) || (s_uart_env[id].uart_state == APP_UART_INVALID))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    err_code = app_io_deinit(s_uart_env[id].pin_cfg.tx.type, s_uart_env[id].pin_cfg.tx.pin);
    APP_DRV_ERR_CODE_CHECK(err_code);

    err_code = app_io_deinit(s_uart_env[id].pin_cfg.rx.type, s_uart_env[id].pin_cfg.rx.pin);
    APP_DRV_ERR_CODE_CHECK(err_code);

    if (UART_HWCONTROL_RTS_CTS == s_uart_env[id].handle.init.hw_flow_ctrl)
    {
        err_code = app_io_deinit(s_uart_env[id].pin_cfg.rts.type, s_uart_env[id].pin_cfg.rts.pin);
        APP_DRV_ERR_CODE_CHECK(err_code);

        err_code = app_io_deinit(s_uart_env[id].pin_cfg.cts.type, s_uart_env[id].pin_cfg.cts.pin);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    hal_nvic_disable_irq(s_uart_irq[id]);

    if (s_uart_env[id].use_mode.type == APP_UART_TYPE_DMA)
    {
        err_code = app_dma_deinit(s_uart_env[id].dma_id[0]);
        APP_DRV_ERR_CODE_CHECK(err_code);

        err_code = app_dma_deinit(s_uart_env[id].dma_id[1]);
        APP_DRV_ERR_CODE_CHECK(err_code);
    }

    s_uart_env[id].uart_state = APP_UART_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    if (s_uart_env[APP_UART_ID_0].uart_state == APP_UART_INVALID && 
        s_uart_env[APP_UART_ID_1].uart_state == APP_UART_INVALID)
    {
         pwr_unregister_sleep_cb(s_uart_pwr_id);
         s_sleep_cb_registered_flag = false;
    }
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();


    hal_uart_deinit(&s_uart_env[id].handle);

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_receive_async(app_uart_id_t id, uint8_t *p_data, uint16_t size)
{
    hal_status_t err_code;

    if (id >= APP_UART_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_uart_env[id].uart_state == APP_UART_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    switch (s_uart_env[id].use_mode.type)
    {
        case APP_UART_TYPE_INTERRUPT:
            err_code = hal_uart_receive_it(&s_uart_env[id].handle, p_data, size);
            HAL_ERR_CODE_CHECK(err_code);
            break;

        case APP_UART_TYPE_DMA:
            err_code = hal_uart_receive_dma(&s_uart_env[id].handle, p_data, size);
            HAL_ERR_CODE_CHECK(err_code);
            break;
        case APP_UART_TYPE_POLLING:
            break;

        default:
            break;
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_receive_sync(app_uart_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_UART_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_uart_env[id].uart_state == APP_UART_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_uart_receive(&s_uart_env[id].handle, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}



uint16_t app_uart_transmit_async(app_uart_id_t id, uint8_t *p_data, uint16_t size)
{
    if (id >= APP_UART_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_uart_env[id].uart_state == APP_UART_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    ring_buffer_write(&s_uart_env[id].tx_ring_buffer, p_data, size);

    if ((s_uart_env[id].start_tx_flag == false) && (s_uart_env[id].start_flush_flag == false) &&
        (s_uart_env[id].uart_state == APP_UART_ACTIVITY) && ll_uart_is_enabled_fifo(s_uart_env[id].handle.p_instance))
    {
        s_uart_env[id].start_tx_flag = true;
        return app_uart_start_transmit_async(id);
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_uart_transmit_sync(app_uart_id_t id, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_UART_ID_MAX ||
        p_data == NULL ||
        size == 0 ||
        s_uart_env[id].uart_state == APP_UART_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_uart_transmit(&s_uart_env[id].handle, p_data, size, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uart_handle_t *app_uart_get_handle(app_uart_id_t id)
{
    if (id >= APP_UART_ID_MAX)
    {
        return NULL;
    }
    return &s_uart_env[id].handle;
}

void app_uart_flush(app_uart_id_t id)
{
    uint16_t items_count;
    uart_handle_t *p_uart = &s_uart_env[id].handle;

    if (APP_UART_ID_MAX <= id)
    {
        return;
    }

    app_systick_init();

    if (s_uart_env[id].uart_state == APP_UART_ACTIVITY)
    {
        s_uart_env[id].start_flush_flag = true;

        if (APP_UART_TYPE_POLLING != s_uart_env[id].use_mode.type)
        {
            uint16_t tx_xfer_size;
            uint16_t tx_xfer_count;

            /* Disable THRE interrupt */
            __HAL_UART_DISABLE_IT(p_uart, UART_IT_THRE);

            while(!ll_uart_is_active_flag_tfe(s_uart_env[id].handle.p_instance));

            tx_xfer_size  = s_uart_env[id].handle.tx_xfer_size;
            tx_xfer_count = s_uart_env[id].handle.tx_xfer_count;

            hal_uart_abort_transmit_it(&s_uart_env[id].handle);

            hal_uart_transmit(&s_uart_env[id].handle,
                              s_tx_send_buf + tx_xfer_size - tx_xfer_count,
                              tx_xfer_count,
                              5000);

            do{
                items_count = ring_buffer_items_count_get(&s_uart_env[id].tx_ring_buffer);
                while(items_count)
                {
                    uint8_t send_char;

                    ring_buffer_read(&s_uart_env[id].tx_ring_buffer, &send_char, 1);

                    while(!ll_uart_is_active_flag_tfnf(s_uart_env[id].handle.p_instance));

                    ll_uart_transmit_data8(s_uart_env[id].handle.p_instance, send_char);

                    items_count--;
                }
            } while(ring_buffer_items_count_get(&s_uart_env[id].tx_ring_buffer));
        }

        while(!ll_uart_is_active_flag_tfe(s_uart_env[id].handle.p_instance));

        if (APP_UART_TYPE_POLLING != s_uart_env[id].use_mode.type)
        {
            /* Enable the UART Transmit Data Register Empty Interrupt */
            __HAL_UART_ENABLE_IT(p_uart, UART_IT_THRE);
        }

        s_uart_env[id].start_flush_flag = false;
    }
}

void hal_uart_tx_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_event_call(p_uart, APP_UART_EVT_TX_CPLT); 
}

void hal_uart_rx_cplt_callback(uart_handle_t *p_uart)
{
     app_uart_event_call(p_uart, APP_UART_EVT_RX_DATA);
}

void hal_uart_error_callback(uart_handle_t *p_uart)
{
    app_uart_event_call(p_uart, APP_UART_EVT_ERROR);
}

void hal_uart_abort_tx_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_event_call(p_uart, APP_UART_EVT_ABORT_TX);
}

void hal_uart_abort_rx_cplt_callback(uart_handle_t *p_uart)
{
    app_uart_event_call(p_uart, APP_UART_EVT_ABORT_RX);
}

void UART0_IRQHandler(void)
{
    hal_uart_irq_handler(&s_uart_env[APP_UART_ID_0].handle);
}

void UART1_IRQHandler(void)
{
    hal_uart_irq_handler(&s_uart_env[APP_UART_ID_1].handle);
}


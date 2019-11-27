/**
  ****************************************************************************************
  * @file    app_qspi.c
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
#include "app_qspi.h"
#include "app_io.h"
#include "app_dma.h"
#include "app_pwr_mgmt.h"
#include "app_systick.h"
#include <string.h>

#ifdef HAL_QSPI_MODULE_ENABLED

/*
 * DEFINES
 *****************************************************************************************
 */

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */

/**@brief App qspi state types. */
typedef enum
{
   APP_QSPI_INVALID = 0,
   APP_QSPI_ACTIVITY,
} app_qspi_state_t;

struct qspi_env_t
{
    app_qspi_evt_handler_t  evt_handler;
    qspi_handle_t           handle;
    app_qspi_mode_t         use_mode;
    app_qspi_pin_cfg_t      pin_cfg;
    dma_id_t                dma_id;
    app_qspi_state_t        qspi_state;
    bool                    start_flag;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool qspi_prepare_for_sleep(void);
static void qspi_sleep_canceled(void);
static void qspi_wake_up_ind(void);
static uint16_t qspi_gpio_config(app_qspi_pin_cfg_t pin_cfg);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const IRQn_Type   s_qspi_irq[APP_QSPI_ID_MAX] = { QSPI0_IRQn, QSPI1_IRQn };
static const uint32_t    s_qspi_instance[APP_QSPI_ID_MAX] = { QSPI0_BASE, QSPI1_BASE };

struct qspi_env_t s_qspi_env[APP_QSPI_ID_MAX];
static bool       s_sleep_cb_registered_flag = false;
static pwr_id_t   s_qspi_pwr_id;

static const app_sleep_callbacks_t qspi_sleep_cb =
{
    .app_prepare_for_sleep = qspi_prepare_for_sleep,
    .app_sleep_canceled    = qspi_sleep_canceled,
    .app_wake_up_ind       = qspi_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool qspi_prepare_for_sleep(void)
{
    hal_qspi_state_t state;
    uint8_t i;

    for (i = 0; i < APP_QSPI_ID_MAX; i++)
    {
        if (s_qspi_env[i].qspi_state == APP_QSPI_ACTIVITY)
        {
            state = hal_qspi_get_state(&s_qspi_env[i].handle);
            if (state > HAL_QSPI_STATE_READY && state < HAL_QSPI_STATE_ABORT)
            {
                return false;
            }
        }
    }

    return true;
}

static void qspi_sleep_canceled(void)
{
}

static void qspi_wake_up_ind(void)
{
    uint8_t i;

    for (i = 0; i < APP_QSPI_ID_MAX; i++)
    {
        if (s_qspi_env[i].qspi_state == APP_QSPI_ACTIVITY)
        {
            qspi_gpio_config(s_qspi_env[i].pin_cfg);

            if(s_qspi_env[i].use_mode.type == APP_QSPI_TYPE_INTERRUPT)
            {
                hal_nvic_clear_pending_irq(s_qspi_irq[i]);
                hal_nvic_enable_irq(s_qspi_irq[i]);
            }

            hal_qspi_deinit(&s_qspi_env[i].handle);
            hal_qspi_init(&s_qspi_env[i].handle);
        }
    }
}

static uint16_t qspi_gpio_config(app_qspi_pin_cfg_t pin_cfg)
{
    app_io_init_t io_init = APP_IO_DEFAULT_CONFIG;
    app_drv_err_t err_code = APP_DRV_SUCCESS;

    io_init.pull = APP_IO_PULLUP;
    io_init.mode = APP_IO_MODE_MUX;
    io_init.pin  = pin_cfg.cs.pin;
    io_init.mux  = pin_cfg.cs.mux;
    err_code = app_io_init(pin_cfg.cs.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);
    
    io_init.pin  = pin_cfg.clk.pin;
    io_init.mux  = pin_cfg.clk.mux;
    err_code = app_io_init(pin_cfg.clk.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    io_init.pin  = pin_cfg.io_0.pin;
    io_init.mux  = pin_cfg.io_0.mux;
    err_code = app_io_init(pin_cfg.io_0.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);
    
    io_init.pin  = pin_cfg.io_1.pin;
    io_init.mux  = pin_cfg.io_1.mux;
    err_code = app_io_init(pin_cfg.io_1.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);
    
    io_init.pin  = pin_cfg.io_2.pin;
    io_init.mux  = pin_cfg.io_2.mux;
    err_code = app_io_init(pin_cfg.io_2.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);
    
    io_init.pin  = pin_cfg.io_3.pin;
    io_init.mux  = pin_cfg.io_3.mux;
    err_code = app_io_init(pin_cfg.io_3.type, &io_init);
    APP_DRV_ERR_CODE_CHECK(err_code);

    return err_code;
}

static uint16_t app_qspi_config_dma(app_qspi_params_t *p_params)
{
    app_dma_params_t dma_params = {DMA_Channel0, {0}};

    dma_params.channel_number             = p_params->use_mode.dma_channel;
    dma_params.init.direction             = DMA_MEMORY_TO_PERIPH;
    dma_params.init.src_increment         = DMA_SRC_INCREMENT;
    dma_params.init.dst_increment         = DMA_DST_NO_CHANGE;
    dma_params.init.src_data_alignment    = DMA_SDATAALIGN_BYTE;
    dma_params.init.dst_data_alignment    = DMA_DDATAALIGN_BYTE;
    dma_params.init.mode                  = DMA_NORMAL;
    dma_params.init.priority              = DMA_PRIORITY_LOW;

    s_qspi_env[p_params->id].dma_id = app_dma_init(&dma_params, NULL);
    if (s_qspi_env[p_params->id].dma_id < 0)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }
    s_qspi_env[p_params->id].handle.p_dma = app_dma_get_handle(s_qspi_env[p_params->id].dma_id);
    s_qspi_env[p_params->id].handle.p_dma->p_parent = (void*)&s_qspi_env[p_params->id].handle;

    return APP_DRV_SUCCESS;
}

static void app_qspi_event_call(qspi_handle_t *p_qspi, app_qspi_evt_type_t evt_type)
{
    app_qspi_evt_t qspi_evt;
    app_qspi_id_t id;

    if (p_qspi->p_instance == QSPI0)
    {
        id = APP_QSPI_ID_0;
    }
    else if (p_qspi->p_instance == QSPI1)
    {
        id = APP_QSPI_ID_1;
    }

    qspi_evt.type = evt_type;
    if (evt_type == APP_QSPI_EVT_ERROR)
    {
        qspi_evt.data.error_code = p_qspi->error_code;
    }
    else if (evt_type == APP_QSPI_EVT_TX_CPLT)
    {
        qspi_evt.data.size = p_qspi->tx_xfer_size - p_qspi->tx_xfer_count;
    }
    else if (evt_type == APP_QSPI_EVT_RX_DATA)
    {
        qspi_evt.data.size = p_qspi->rx_xfer_size - p_qspi->rx_xfer_count;
    }

    s_qspi_env[id].start_flag = false;
    if(s_qspi_env[id].evt_handler != NULL)
    {
        s_qspi_env[id].evt_handler(&qspi_evt);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_qspi_init(app_qspi_params_t *p_params, app_qspi_evt_handler_t evt_handler)
{
    uint8_t id = p_params->id;
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }

    if (id >= APP_QSPI_ID_MAX)
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_systick_init();

    app_err_code = qspi_gpio_config(p_params->pin_cfg);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    if(p_params->use_mode.type == APP_QSPI_TYPE_DMA)
    {
        app_err_code = app_qspi_config_dma(p_params);
        APP_DRV_ERR_CODE_CHECK(app_err_code);
    }

    if(p_params->use_mode.type != APP_QSPI_TYPE_POLLING)
    {
        hal_nvic_clear_pending_irq(s_qspi_irq[id]);
        hal_nvic_enable_irq(s_qspi_irq[id]);
    }

    s_qspi_env[id].use_mode.type = p_params->use_mode.type;
    s_qspi_env[id].use_mode.dma_channel = p_params->use_mode.dma_channel;
    memcpy(&s_qspi_env[id].pin_cfg, &p_params->pin_cfg, sizeof(app_qspi_pin_cfg_t));
    s_qspi_env[id].evt_handler = evt_handler;

    memcpy(&s_qspi_env[id].handle.init, &p_params->init, sizeof(qspi_init_t));
    s_qspi_env[id].handle.p_instance = (ssi_regs_t *)s_qspi_instance[id];
    hal_err_code = hal_qspi_deinit(&s_qspi_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    hal_err_code =hal_qspi_init(&s_qspi_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    if(!s_sleep_cb_registered_flag)// register sleep callback
    {
        s_sleep_cb_registered_flag = true;
        s_qspi_pwr_id = pwr_register_sleep_cb(&qspi_sleep_cb);
        if (s_qspi_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
    }

    s_qspi_env[id].qspi_state = APP_QSPI_ACTIVITY;

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_deinit(app_qspi_id_t id)
{
    app_drv_err_t app_err_code;
    hal_status_t  hal_err_code;

    if ((id >= APP_QSPI_ID_MAX) || (s_qspi_env[id].qspi_state == APP_QSPI_INVALID))
    {
        return APP_DRV_ERR_INVALID_ID;
    }

    app_err_code = app_io_deinit(s_qspi_env[id].pin_cfg.cs.type, s_qspi_env[id].pin_cfg.cs.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_qspi_env[id].pin_cfg.clk.type, s_qspi_env[id].pin_cfg.clk.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_qspi_env[id].pin_cfg.io_0.type, s_qspi_env[id].pin_cfg.io_0.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_qspi_env[id].pin_cfg.io_1.type, s_qspi_env[id].pin_cfg.io_1.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_qspi_env[id].pin_cfg.io_2.type, s_qspi_env[id].pin_cfg.io_2.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);

    app_err_code = app_io_deinit(s_qspi_env[id].pin_cfg.io_3.type, s_qspi_env[id].pin_cfg.io_3.pin);
    APP_DRV_ERR_CODE_CHECK(app_err_code);


    hal_nvic_disable_irq(s_qspi_irq[id]);
    if(s_qspi_env[id].use_mode.type == APP_QSPI_TYPE_DMA)
    {
        app_dma_deinit(s_qspi_env[id].dma_id);
    }
    s_qspi_env[id].qspi_state = APP_QSPI_INVALID;

    GLOBAL_EXCEPTION_DISABLE();
    if(s_qspi_env[APP_QSPI_ID_0].qspi_state == APP_QSPI_INVALID && 
        s_qspi_env[APP_QSPI_ID_1].qspi_state == APP_QSPI_INVALID)
    {
         pwr_unregister_sleep_cb(s_qspi_pwr_id);
         s_sleep_cb_registered_flag = false;
    }
    GLOBAL_EXCEPTION_ENABLE();

    app_systick_deinit();

    hal_err_code = hal_qspi_deinit(&s_qspi_env[id].handle);
    HAL_ERR_CODE_CHECK(hal_err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_receive_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_cmd == NULL ||
        p_data == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_qspi_command_receive(&s_qspi_env[id].handle, p_cmd, p_data, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_receive_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_cmd == NULL ||
        p_data == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID ||
        s_qspi_env[id].use_mode.type == APP_QSPI_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_qspi_env[id].start_flag == false)
    {
        s_qspi_env[id].start_flag = true;
        switch (s_qspi_env[id].use_mode.type)
        {
            case APP_QSPI_TYPE_INTERRUPT:
                err_code = hal_qspi_command_receive_it(&s_qspi_env[id].handle, p_cmd, p_data);
                HAL_ERR_CODE_CHECK(err_code);
                break;
            case APP_QSPI_TYPE_DMA:
                err_code = hal_qspi_command_receive_dma(&s_qspi_env[id].handle, p_cmd, p_data);
                HAL_ERR_CODE_CHECK(err_code);
                break;
            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_transmit_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_cmd == NULL ||
        p_data == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_qspi_command_transmit(&s_qspi_env[id].handle, p_cmd, p_data, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_transmit_async(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint8_t *p_data)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_cmd == NULL ||
        p_data == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID ||
        s_qspi_env[id].use_mode.type == APP_QSPI_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_qspi_env[id].start_flag == false)
    {
        s_qspi_env[id].start_flag = true;
        switch (s_qspi_env[id].use_mode.type)
        {
            case APP_QSPI_TYPE_INTERRUPT:
                err_code = hal_qspi_command_transmit_it(&s_qspi_env[id].handle, p_cmd, p_data);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_QSPI_TYPE_DMA:
                err_code = hal_qspi_command_transmit_dma(&s_qspi_env[id].handle, p_cmd, p_data);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_sync(app_qspi_id_t id, app_qspi_command_t *p_cmd, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_cmd == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_qspi_command(&s_qspi_env[id].handle, p_cmd, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_command_async(app_qspi_id_t id, app_qspi_command_t *p_cmd)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_cmd == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID ||
        s_qspi_env[id].use_mode.type == APP_QSPI_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_qspi_env[id].start_flag == false)
    {
        s_qspi_env[id].start_flag = true;
        switch (s_qspi_env[id].use_mode.type)
        {
            case APP_QSPI_TYPE_INTERRUPT:
                err_code = hal_qspi_command_it(&s_qspi_env[id].handle, p_cmd);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_QSPI_TYPE_DMA:
                err_code = hal_qspi_command_dma(&s_qspi_env[id].handle, p_cmd);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_transmit_sync(app_qspi_id_t id, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_data == NULL ||
        length == 0 ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_qspi_transmit(&s_qspi_env[id].handle, p_data, length, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_transmit_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        p_data == NULL ||
        length == 0 ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID ||
        s_qspi_env[id].use_mode.type == APP_QSPI_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_qspi_env[id].start_flag == false)
    {
        s_qspi_env[id].start_flag = true;
        switch (s_qspi_env[id].use_mode.type)
        {
            case APP_QSPI_TYPE_INTERRUPT:
                err_code = hal_qspi_transmit_it(&s_qspi_env[id].handle, p_data, length);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_QSPI_TYPE_DMA:
                err_code = hal_qspi_transmit_dma(&s_qspi_env[id].handle, p_data, length);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_receive_sync(app_qspi_id_t id, uint8_t *p_data, uint32_t length, uint32_t timeout)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        length == 0 ||
        p_data == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    err_code = hal_qspi_receive(&s_qspi_env[id].handle, p_data, length, timeout);
    HAL_ERR_CODE_CHECK(err_code);

    return APP_DRV_SUCCESS;
}

uint16_t app_qspi_receive_async(app_qspi_id_t id, uint8_t *p_data, uint32_t length)
{
    hal_status_t err_code;

    if (id >= APP_QSPI_ID_MAX ||
        length == 0 ||
        p_data == NULL ||
        s_qspi_env[id].qspi_state == APP_QSPI_INVALID ||
        s_qspi_env[id].use_mode.type == APP_QSPI_TYPE_POLLING)
    {
        return APP_DRV_ERR_INVALID_PARAM;
    }

    if (s_qspi_env[id].start_flag == false)
    {
        s_qspi_env[id].start_flag = true;
        switch (s_qspi_env[id].use_mode.type)
        {
            case APP_QSPI_TYPE_INTERRUPT:
                err_code = hal_qspi_receive_it(&s_qspi_env[id].handle, p_data, length);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            case APP_QSPI_TYPE_DMA:
                err_code = hal_qspi_receive_dma(&s_qspi_env[id].handle, p_data, length);
                HAL_ERR_CODE_CHECK(err_code);
                break;

            default:
                break;
        }
    }

    return APP_DRV_SUCCESS;
}

void hal_qspi_error_callback(qspi_handle_t *p_qspi)
{
    app_qspi_event_call(p_qspi, APP_QSPI_EVT_ERROR); 
}

void hal_qspi_rx_cplt_callback(qspi_handle_t *p_qspi)
{
    app_qspi_event_call(p_qspi, APP_QSPI_EVT_RX_DATA); 
}

void hal_qspi_tx_cplt_callback(qspi_handle_t *p_qspi)
{
    app_qspi_event_call(p_qspi, APP_QSPI_EVT_TX_CPLT); 
}

void QSPI0_IRQHandler(void)
{
    hal_qspi_irq_handler(&s_qspi_env[APP_QSPI_ID_0].handle);
}

void QSPI1_IRQHandler(void)
{
    hal_qspi_irq_handler(&s_qspi_env[APP_QSPI_ID_1].handle);
}

#endif

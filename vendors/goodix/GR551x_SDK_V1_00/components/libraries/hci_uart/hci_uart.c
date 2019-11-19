/**
  ****************************************************************************************
  * @file    hci_uart.c
  * @author  BLE SDK Team
  * @brief   H4TL UART driver.
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
 ****************************************************************************************
 */
#include <stddef.h>     // standard definition
#include "gr55xx_hal.h"
#include "hci_uart.h"       // uart definition
#include "boards.h"

/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */


/* TX and RX channel class holding data used for asynchronous read and write data
 * transactions
 */
/// UART TX RX Channel
typedef struct
{
    /// call back function pointer
    void (*callback) (void*, uint8_t);
    /// dummy data pointer returned to callback when operation is over.
    void *p_dummy;
} uart_txrxchannel_t;

/// UART environment structure
typedef struct
{
    /// tx channel
    uart_txrxchannel_t tx;
    /// rx channel
    uart_txrxchannel_t rx;
    /// error detect
    uint8_t errordetect;
    /// external wakeup
    bool ext_wakeup;
} uart_env_tag_t;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// uart environment structure
static uart_env_tag_t uart_env;

static void hci_uart_flow_on(uart_handle_t *p_uart);

/// uart handle
uart_handle_t hci_uart0_handle;
uart_handle_t hci_uart1_handle;

/*
 * PRIVATED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void hci_uart_tx_cplt_callback(uart_handle_t *p_uart)
{
    void *data = NULL;
    void (*callback) (void*, uint8_t) = NULL;

    // Retrieve callback pointer
    callback = uart_env.tx.callback;
    data     = uart_env.tx.p_dummy;

    if(callback != NULL)
    {
        // Clear callback pointer
        uart_env.tx.callback = NULL;
        uart_env.tx.p_dummy    = NULL;

        // Call handler
        callback(data, 0);
    }
    return;
}

void hci_uart_rx_cplt_callback(uart_handle_t *p_uart)
{
    void *data = NULL;
    void (*callback) (void*, uint8_t) = NULL;

    // Retrieve callback pointer
    callback = uart_env.rx.callback;
    data     = uart_env.rx.p_dummy;

    if(callback != NULL)
    {
        // Clear callback pointer
        uart_env.rx.callback = NULL;
        uart_env.rx.p_dummy    = NULL;
    }
    // Call handler
    callback(data, 0);
    return;
}

void hci_uart_error_callback(uart_handle_t *p_uart)
{
    uint8_t tmp;
    /* dummy read to clear RLS interrupt */
    tmp = ll_uart_receive_data8(p_uart->p_instance);
    (void) tmp;

    /* Flush RX_FIFO */
    ll_uart_flush_rx_fifo(p_uart->p_instance);

    /* If Overrun error occurs. */
    if(p_uart->error_code & HAL_UART_ERROR_OE)
    {
        hci_uart_rx_cplt_callback(p_uart);
    }

    return;
}

static void hci_uart_init(uart_handle_t *p_uart)
{
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    IRQn_Type uart_irq_num = ((p_uart->p_instance == UART0) ? UART0_IRQn : UART1_IRQn);

    /// Auto flow control will be controlled in hci_uart_flow_on() and hci_uart_flow_off()
    p_uart->init.baud_rate       = HCI_UART_BAUDRATE;
    p_uart->init.data_bits       = UART_DATABITS_8;
    p_uart->init.stop_bits       = UART_STOPBITS_1;
    p_uart->init.parity          = UART_PARITY_NONE;
    p_uart->init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    p_uart->init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_DISABLE;

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pull = GPIO_PULLUP;
    gpio_config.pin = HCI_UART_TX_PIN;
    gpio_config.mux = HCI_UART_TX_PINMUX;
    hal_gpio_init(HCI_UART_TRN_PORT, &gpio_config);
    gpio_config.pin = HCI_UART_RX_PIN;
    gpio_config.mux = HCI_UART_RX_PINMUX;
    hal_gpio_init(HCI_UART_TRN_PORT, &gpio_config);

    hal_uart_deinit(p_uart);
    hal_uart_init(p_uart);

    NVIC_ClearPendingIRQ(uart_irq_num);
    NVIC_EnableIRQ(uart_irq_num);
    return;
}

static void hci_uart_write(uart_handle_t *p_uart, uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    if(NULL == uart_env.tx.callback)
    {
        uart_env.tx.callback = callback;
        uart_env.tx.p_dummy    = p_dummy;
    }

    hal_uart_transmit_it(p_uart, p_buffer, (uint16_t)size);
    return;
}

static void hci_uart_read(uart_handle_t *p_uart, uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    if(NULL == uart_env.rx.callback)
    {
        uart_env.rx.callback = callback;
        uart_env.rx.p_dummy    = p_dummy;
    }

    hal_uart_receive_it(p_uart, p_buffer, (uint16_t)size);
    return;
}

static void hci_uart_flow_on(uart_handle_t *p_uart)
{
    //Define HCI_UART_FLOW_ON to Open UART Flow Control
    #if HCI_UART_FLOW_ON == 1  //To define macro this way for auto test script to locate and modify, don't change this line
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pull = GPIO_PULLUP;
    gpio_config.pin = HCI_UART_CTS_PIN;
    gpio_config.mux = HCI_UART_CTS_PINMUX;
    hal_gpio_init(HCI_UART_FLOW_PORT, &gpio_config);
    gpio_config.pin = HCI_UART_RTS_PIN;
    gpio_config.mux = HCI_UART_RTS_PINMUX;
    hal_gpio_init(HCI_UART_FLOW_PORT, &gpio_config);

    /* Disable flow control default for .bit version < R02_27a_B0228a_k7 because CTS issue */
    ll_uart_set_hw_flow_ctrl(p_uart->p_instance, LL_UART_HWCONTROL_RTS_CTS);
    #endif //HCI_UART_FLOW_ON
    return;
}

static bool hci_uart_flow_off(uart_handle_t *p_uart)
{
    bool flow_off = true;
    hal_uart_state_t state = hal_uart_get_state(p_uart);

    do {
        /* Check if sleep is allowed by Host and if no transmission is ongoing */
        if ((state == HAL_UART_STATE_BUSY_TX) || (state == HAL_UART_STATE_BUSY_RX))
        {
            flow_off = false;
            break;
        }

        /* Force RTS to 'flow off' */
        ll_uart_set_hw_flow_ctrl(p_uart->p_instance, LL_UART_HWCONTROL_NONE);

        state = hal_uart_get_state(p_uart);
        /* Check if data has been received during wait time */
        if (state == HAL_UART_STATE_BUSY_RX)
        {
            /* Re-enable UART flow */
            hci_uart_flow_on(p_uart);

            flow_off = false;
        }
    } while(0);

    return flow_off;
}

static void hci_uart_finish_transfers(uart_handle_t *p_uart)
{
    return;
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void hci_uart0_init(void)
{
    hci_uart0_handle.p_instance = UART0;
    hci_uart_init(&hci_uart0_handle);
    return;
}

void hci_uart1_init(void)
{
    hci_uart1_handle.p_instance = UART1;
    hci_uart_init(&hci_uart1_handle);
    return;
}

void hci_uart0_flow_on(void)
{
    hci_uart_flow_on(&hci_uart0_handle);
    return;
}
void hci_uart1_flow_on(void)
{
    hci_uart_flow_on(&hci_uart1_handle);
    return;
}

bool hci_uart0_flow_off(void)
{
    return hci_uart_flow_off(&hci_uart0_handle);
}

bool hci_uart1_flow_off(void)
{
    return hci_uart_flow_off(&hci_uart1_handle);
}

void hci_uart0_finish_transfers(void)
{
    hci_uart_finish_transfers(&hci_uart0_handle);
    return;
}

void hci_uart1_finish_transfers(void)
{
    hci_uart_finish_transfers(&hci_uart1_handle);
    return;
}

void hci_uart0_read(uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    hci_uart_read(&hci_uart0_handle, p_buffer, size, callback, p_dummy);
    return;
}

void hci_uart1_read(uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    hci_uart_read(&hci_uart1_handle, p_buffer, size, callback, p_dummy);
    return;
}

void hci_uart0_write(uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    hci_uart_write(&hci_uart0_handle, p_buffer, size, callback, p_dummy);
    return;
}
void hci_uart1_write(uint8_t *p_buffer, uint32_t size, void (*callback) (void*, uint8_t), void* p_dummy)
{
    hci_uart_write(&hci_uart1_handle, p_buffer, size, callback, p_dummy);
    return;
}

void hci_uart0_irq_handler(void)
{
    hal_uart_irq_handler(&hci_uart0_handle);
}

void hci_uart1_irq_handler(void)
{
    hal_uart_irq_handler(&hci_uart1_handle);
}


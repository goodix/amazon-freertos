/**
 *****************************************************************************************
 *
 * @file app_log_port.c
 *
 * @brief App Log Port Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_log_port.h"
#include "app_log_cfg.h"
#include "boards.h"
#include "gr55xx_hal.h"
#include "SEGGER_RTT.h"
#if APP_LOG_ASYNC_OUTPUT_ENABLE > 0u
#include "ring_buffer.h"
#endif

#include "gr_porting.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#if APP_LOG_ASYNC_OUTPUT_ENABLE > 0u
static uint8_t     s_log_data_buf[APP_LOG_CACHE_BUF_SIZE];   /**< App log data buffer. */

static ring_buffer_t s_asyn_log_buffer =                     /**< Ring buffer for asynchronous log save. */
{
    .buffer_size        = APP_LOG_CACHE_BUF_SIZE,
    .p_buffer           = s_log_data_buf,
    .write_index        = 0,
    .read_index         = 0,
};
#endif

static gr_mutex_t   s_log_mutex = {
        .is_valid       = false,
        .is_actived     = false,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief App log char send via uart.
 *
 * @param[in] ch: App log char.
 *****************************************************************************************
 */
static void app_log_uart_send(int ch)
{
#if APP_LOG_ASYNC_OUTPUT_ENABLE > 0u
    uint16_t  surplus_space;

    // Fill ring buffer to use interrupt send.
    APP_LOG_LOCK();

    surplus_space = ring_buffer_surplus_space_get(&s_asyn_log_buffer);

    if (0 != surplus_space)
    {
        ring_buffer_write(&s_asyn_log_buffer, (uint8_t *)&ch, 1);

        // Enable TXE interrupt.
        ll_uart_enable_it(LOG_UART_GRP, LL_UART_IER_THRE);
    }

    APP_LOG_UNLOCK();
#else
    while(!ll_uart_is_active_flag_tfnf(LOG_UART_GRP));
    ll_uart_transmit_data8(LOG_UART_GRP, ch);
#endif
    return;
}

#if APP_LOG_ASYNC_OUTPUT_ENABLE > 0u
/**
 *****************************************************************************************
 * @brief App log char send callback.
 *****************************************************************************************
 */
static void app_log_uart_send_callback(uart_regs_t *UARTx)
{
    uint32_t isrflag     = LL_UART_IIR_THRE;//ll_uart_get_it_flag(UARTx);
    uint16_t items_avail = ring_buffer_items_count_get(&s_asyn_log_buffer);
    
    if (LL_UART_IIR_THRE == isrflag)
    {
        if (items_avail)
        {
            uint8_t curxfercnt = UART_TXFIFO_SIZE - ll_uart_get_tx_fifo_level(UARTx);
            uint8_t send_char;

            while(curxfercnt && items_avail)
            {
                ring_buffer_read(&s_asyn_log_buffer, &send_char, 1);
                ll_uart_transmit_data8(UARTx, send_char);
                items_avail = ring_buffer_items_count_get(&s_asyn_log_buffer);
                curxfercnt--;
            }
        }
        else
        {
            ll_uart_disable_it(UARTx, LL_UART_IER_THRE);
        }
    }
}
#endif

/**
 *****************************************************************************************
 * @brief App log char send.
 *
 * @param[in] ch: App log char.
 *****************************************************************************************
 */
static int app_log_char_send(int ch)
{
#if (APP_LOG_PROT_MODE == APP_LOG_UART_PORT)
    app_log_uart_send((uint8_t)ch);
#elif (APP_LOG_PROT_MODE == APP_LOG_RTT_PORT)
    SEGGER_RTT_Write(0, (void*)&ch, 1);
#elif (APP_LOG_PROT_MODE == APP_LOG_ITM_PORT)
    ITM_SendChar(ch);
#endif

    return ch;
}


/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void app_log_port_init(void)
{
    gr_mutex_init(&s_log_mutex);
    
#if (APP_LOG_PROT_MODE == APP_LOG_UART_PORT)

    uint32_t baud_rate;
    uint32_t uart_pclk;

    uart_pclk = SystemCoreClock;
    baud_rate = APP_LOG_UART_BAUDRATE;
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    // Enable serial module clock.
    ll_cgc_disable_force_off_serial_hclk();
    ll_cgc_disable_wfi_off_serial_hclk();

    if (UART0 == LOG_UART_GRP)
    {
        // Enable UART0 clock.
        ll_cgc_disable_force_off_uart0_hclk();
    }
    else if (UART1 == LOG_UART_GRP)
    {
        // Enable UART1 clock.
        ll_cgc_disable_force_off_uart1_hclk();
    }

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = LOG_UART_TX_PIN;
    gpio_config.mux  = LOG_UART_TX_PINMUX;
    hal_gpio_init(LOG_UART_PORT, &gpio_config);

    gpio_config.pin  = LOG_UART_RX_PIN;
    gpio_config.mux  = LOG_UART_RX_PINMUX;
    hal_gpio_init(LOG_UART_PORT, &gpio_config);

    ll_uart_set_baud_rate(LOG_UART_GRP, uart_pclk, baud_rate);
    ll_uart_config_character(LOG_UART_GRP, LL_UART_DATABITS_8B, LL_UART_PARITY_NONE, LL_UART_STOPBITS_1);
    
    ll_uart_set_rx_fifo_threshold(LOG_UART_GRP, LL_UART_RX_FIFO_TH_CHAR_1);
    ll_uart_enable_fifo(LOG_UART_GRP);
    
    ll_uart_enable_it(LOG_UART_GRP, LL_UART_IER_THRE | LL_UART_IER_RDA);

#if APP_LOG_ASYNC_OUTPUT_ENABLE > 0u
    // Set tx fifo threshold
    ll_uart_set_tx_fifo_threshold(LOG_UART_GRP, LL_UART_TX_FIFO_TH_EMPTY);

    // Enable NVIC uart interrupt */
    IRQn_Type uart_irq_num = ((LOG_UART_GRP == UART0) ? UART0_IRQn : UART1_IRQn);
    NVIC_SetPriority(uart_irq_num, 0xF0);
    NVIC_ClearPendingIRQ(uart_irq_num);
    NVIC_EnableIRQ(uart_irq_num);
#endif

#elif (APP_LOG_PROT_MODE == APP_LOG_RTT_PORT)

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

#elif (APP_LOG_PROT_MODE == APP_LOG_ITM_PORT)

    // Set MUX2 of GPIO_PIN_2 to enable SWV. 
    ll_gpio_set_mux_pin_0_7(GPIO0, LL_GPIO_PIN_2, LL_GPIO_MUX_2);

#endif
}

void app_log_activate_thread_safe(void){
    gr_mutex_set_activate(&s_log_mutex, true);
}

void app_log_port_output(const uint8_t *p_log, uint16_t length)
{
    gr_mutex_lock(&s_log_mutex);
    printf("%.*s", length, p_log);
    app_log_port_flush();
    gr_mutex_unlock(&s_log_mutex);
}

void app_log_port_flush(void)
{
#if (APP_LOG_PROT_MODE == APP_LOG_UART_PORT)

#if APP_LOG_ASYNC_OUTPUT_ENABLE > 0u
    IRQn_Type uart_irq_num = ((LOG_UART_GRP == UART0) ? UART0_IRQn : UART1_IRQn);
    uint16_t items_avail   = ring_buffer_items_count_get(&s_asyn_log_buffer);

    NVIC_DisableIRQ(uart_irq_num);
    NVIC_ClearPendingIRQ(uart_irq_num);

    while (items_avail)
    {
        uint8_t send_char;

        ring_buffer_read(&s_asyn_log_buffer, &send_char, 1);
        // Wait untill TX FIFO is not full.
        while(!ll_uart_is_active_flag_tfnf(LOG_UART_GRP));
        ll_uart_transmit_data8(LOG_UART_GRP, send_char);
        items_avail--;
    }
    // Wait untill TX FIFO is empty.
    while(!ll_uart_is_active_flag_tfe(LOG_UART_GRP));
    NVIC_EnableIRQ(uart_irq_num);
#else
    while(!ll_uart_is_active_flag_tfe(LOG_UART_GRP));
#endif

#endif
}

#if APP_LOG_ASYNC_OUTPUT_ENABLE > 0u

__WEAK void app_log_transfer_rx_char(char ch){    
    ch = ch;
}

void UART0_IRQHandler(void)
{
    uart_regs_t * UARTx     = UART0;
    uint32_t irq_flag       = ll_uart_get_it_flag(UARTx);

    switch(irq_flag){
        case LL_UART_IIR_THRE:
            app_log_uart_send_callback(UARTx);
            break;
        
        case LL_UART_IIR_RDA:
        {
            char ch;
            if(ll_uart_is_active_flag_rfne(UARTx)) {
                ch = ll_uart_receive_data8(UARTx);
                app_log_transfer_rx_char(ch);
            }
        }
        break;
        
        default:
            break;
    }
    
}

void UART1_IRQHandler(void)
{
    app_log_uart_send_callback(UART1);
}
#endif

#if defined(__CC_ARM)

struct __FILE
{
    int handle;
};

FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *file)
{
    return (app_log_char_send(ch));
}

#elif defined(__GNUC__)

int _write(int file, const char *buf, int len)
{
    int tx_len = 0;

    while (tx_len < len)
    {
        app_log_char_send(*buf);
        buf++;
        tx_len++;
    }
    return tx_len;
}

#elif defined(__ICCARM__)

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    size_t len = 0;

    while (len < size)
    {
        app_log_char_send(*buf);
        buf++;
        len++;
    }
    return len;
}

#endif /* defined(__CC_ARM) */

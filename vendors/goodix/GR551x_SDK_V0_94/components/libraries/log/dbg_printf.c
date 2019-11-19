#include <stdio.h>
#include <string.h>
#include "dbg_printf.h"
#include "SEGGER_RTT.h"
#include "boards.h"

typedef struct
{
    char     send_buf[DBG_RING_BUF_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t valid;
} ring_buf_t;

static ring_buf_t dbg_ring_buf;
static dbg_printf_mode_t dbg_printf_mode = DBG_PRINTF_NONE;
volatile int32_t ITM_RxBuffer = 0x5AA55AA5U;

__WEAK int SEGGER_RTT_ConfigUpBuffer(unsigned BufferIndex, const char* sName, void* pBuffer, unsigned BufferSize, unsigned Flags)
{
    return 0;
}

__WEAK unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes)
{
    return 0;
}

__WEAK int SEGGER_RTT_WaitKey(void)
{
    return 0;
}

static uint8_t push_char(ring_buf_t *p_ring_buf, uint8_t ch)
{
    uint8_t ret = 0;

    GLOBAL_EXCEPTION_DISABLE();
    if (p_ring_buf->valid < DBG_RING_BUF_SIZE)
    {
        p_ring_buf->send_buf[p_ring_buf->tail++] = ch;
        if (p_ring_buf->tail == DBG_RING_BUF_SIZE)
            p_ring_buf->tail = 0;
        p_ring_buf->valid++;
        ret = 1;
    }
    else
    {
        ret = 0;
    }
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

static uint8_t pop_char(ring_buf_t *p_ring_buf)
{
    uint8_t ret = 0;

    GLOBAL_EXCEPTION_DISABLE();
    if (p_ring_buf->valid)
    {
        p_ring_buf->valid--;
        ret =  p_ring_buf->send_buf[p_ring_buf->head++];
        if (p_ring_buf->head == DBG_RING_BUF_SIZE)
            p_ring_buf->head = 0;
    }
    else
    {
        ret = 0;
    }
    GLOBAL_EXCEPTION_ENABLE();

    return ret;
}

static void dbg_uart_init(uart_regs_t *UARTx, dbg_printf_mode_t mode)
{
    uint32_t baud;
    uint32_t uart_pclk = SystemCoreClock;
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    /* Initialize ring buffer */
    memset(&dbg_ring_buf, 0, sizeof(dbg_ring_buf));

    /* Enable serial module clock */
    ll_cgc_disable_force_off_serial_hclk();
    ll_cgc_disable_wfi_off_serial_hclk();

    if(UARTx == UART0)
    {
        /* Enable UART0 clock */
        ll_cgc_disable_force_off_uart0_hclk();
    }
    else if(UARTx == UART1)
    {
        /* Enable UART1 clock */
        ll_cgc_disable_force_off_uart1_hclk();
    }

    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = LOG_UART_TX_PIN;
    gpio_config.mux  = LOG_UART_TX_PINMUX;
    hal_gpio_init(LOG_UART_PORT, &gpio_config);

    gpio_config.pin  = LOG_UART_RX_PIN;
    gpio_config.mux  = LOG_UART_RX_PINMUX;
    hal_gpio_init(LOG_UART_PORT, &gpio_config);

    /* Set baudrate */
    baud = DBG_UART_BAUDRATE;
    ll_uart_set_baud_rate(UARTx, uart_pclk, baud);
    /* Set data bit = 8; 
       Set stop bit = 1; 
       Set parity = none;
       Set fifo enable;	*/
    ll_uart_config_character(UARTx, LL_UART_DATABITS_8B, LL_UART_PARITY_NONE, LL_UART_STOPBITS_1);
    /* Set fifo enable */
    ll_uart_enable_fifo(UARTx);

    if((DBG_PRINTF_INT_UART0 == mode) || (DBG_PRINTF_INT_UART1 == mode))
    {
        /* Set tx fifo threshold */
        ll_uart_set_tx_fifo_threshold(UARTx, LL_UART_TX_FIFO_TH_EMPTY);

        /* Enable NVIC uart interrupt */
        IRQn_Type uart_irq_num = ((UARTx == UART0) ? UART0_IRQn : UART1_IRQn);
        NVIC_SetPriority(uart_irq_num, 0xF0);
        NVIC_ClearPendingIRQ(uart_irq_num);
        NVIC_EnableIRQ(uart_irq_num);
    }
    return;
}

static void dbg_uart_send_char(uart_regs_t *UARTx, uint8_t ch)
{
    if((DBG_PRINTF_UART0 == dbg_printf_mode) || (DBG_PRINTF_UART1 == dbg_printf_mode))
    {
        /* Wait untill TX FIFO is not full */
        while(!ll_uart_is_active_flag_tfnf(UARTx));
        ll_uart_transmit_data8(UARTx, ch);
    }
    else if((DBG_PRINTF_INT_UART0 == dbg_printf_mode) || (DBG_PRINTF_INT_UART1 == dbg_printf_mode))
    {
        /* Fill ring buffer to use interrupt send */
        push_char(&dbg_ring_buf, ch);
        /* Enable TXE interrupt */
        ll_uart_enable_it(UARTx, LL_UART_IER_THRE);
    }
    return;
}

static uint8_t dbg_uart_get_char(uart_regs_t *UARTx)
{
    uint8_t ch;

    while(!ll_uart_is_active_flag_rfne(UARTx));
    /* UART receive byte */
    ch = ll_uart_receive_data8(UARTx);
    return ch;
}

static int dbg_send_char(int ch)
{
    switch(dbg_printf_mode)
    {
        case DBG_PRINTF_UART0:
        case DBG_PRINTF_INT_UART0:
            dbg_uart_send_char(UART0, (uint8_t)ch);
            break;
        case DBG_PRINTF_UART1:
        case DBG_PRINTF_INT_UART1:
            dbg_uart_send_char(UART1, (uint8_t)ch);
            break;
        case DBG_PRINTF_ITM:
            ITM_SendChar(ch);
            break;
        case DBG_PRINTF_RTT:
            SEGGER_RTT_Write(0, (void*)&ch, 1);
            break;
        default: break;
    }
    return ch;
}

static int dbg_get_char(void)
{
    int32_t ch = -1;

    switch(dbg_printf_mode)
    {
        case DBG_PRINTF_UART0:
        case DBG_PRINTF_INT_UART0:
            ch = dbg_uart_get_char(UART0);
            break;
        case DBG_PRINTF_UART1:
        case DBG_PRINTF_INT_UART1:
            ch = dbg_uart_get_char(UART1);
            break;
        case DBG_PRINTF_ITM:
            while (ITM_CheckChar() == 0);
            ch = ITM_ReceiveChar();
            break;
        case DBG_PRINTF_RTT:
            ch = SEGGER_RTT_WaitKey();
            break;
        default: break;
    }
    return ch;
}

void dbg_printf_set_mode(dbg_printf_mode_t mode)
{
    switch(mode)
    {
        case DBG_PRINTF_UART0:
        case DBG_PRINTF_INT_UART0:
            dbg_uart_init(UART0, mode);
            break;
        case DBG_PRINTF_UART1:
        case DBG_PRINTF_INT_UART1:
            dbg_uart_init(UART1, mode);
            break;
        case DBG_PRINTF_ITM:
            /* Set MUX2 of GPIO_PIN_2 to enable SWV. */
            ll_gpio_set_mux_pin_0_7(GPIO0, LL_GPIO_PIN_2, LL_GPIO_MUX_2);
            break;
        case DBG_PRINTF_RTT:
            SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
            break;
        default: break;
    }

    dbg_printf_mode = mode;
    return;
}

uint8_t dbg_printf_uart_callback(uart_regs_t *UARTx)
{
    uint32_t isrflag = ll_uart_get_it_flag(UARTx);
    
    if (LL_UART_IIR_THRE == isrflag)
    {
        if (0 == dbg_ring_buf.valid)
        {
            /* Disable TXE interrupt */
            ll_uart_disable_it(UARTx, LL_UART_IER_THRE);
        }
        else
        {
            uint8_t curxfercnt = UART_TXFIFO_SIZE - ll_uart_get_tx_fifo_level(UARTx);
            while(curxfercnt && dbg_ring_buf.valid)
            {
                ll_uart_transmit_data8(UARTx, pop_char(&dbg_ring_buf));
                curxfercnt--;
            }
        }
        return 1;
    }
    else
    {
        return 0;
    }
}

uint32_t dbg_printf_uart_flush(void)
{
    uint32_t count = 0;
    if((DBG_PRINTF_INT_UART0 == dbg_printf_mode) || (DBG_PRINTF_INT_UART1 == dbg_printf_mode))
    {
        uart_regs_t *UARTx = ((DBG_PRINTF_INT_UART0 == dbg_printf_mode)? UART0: UART1);
        IRQn_Type uart_irq_num = ((DBG_PRINTF_INT_UART0 == dbg_printf_mode) ? UART0_IRQn : UART1_IRQn);

        NVIC_DisableIRQ(uart_irq_num);
        NVIC_ClearPendingIRQ(uart_irq_num);
        count = dbg_ring_buf.valid;
        while(dbg_ring_buf.valid)
        {
            /* Wait untill TX FIFO is not full */
            while(!ll_uart_is_active_flag_tfnf(UARTx));
                ll_uart_transmit_data8(UARTx, pop_char(&dbg_ring_buf));
        }
        /* Wait untill TX FIFO is empty. */
        while(!ll_uart_is_active_flag_tfe(UARTx));
        NVIC_EnableIRQ(uart_irq_num);
    }
    else if ((DBG_PRINTF_UART0 == dbg_printf_mode) || (DBG_PRINTF_UART1 == dbg_printf_mode))
    {
        uart_regs_t *UARTx = ((DBG_PRINTF_UART0 == dbg_printf_mode)? UART0: UART1);
        /* Wait untill TX FIFO is empty. */
        while(!ll_uart_is_active_flag_tfe(UARTx));
    }

    return count;
}

#if defined(__CC_ARM)

struct __FILE
{
    int handle;
};

FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE * file)
{
    return (dbg_send_char(ch));
}

int fgetc(FILE * file)
{
    return dbg_get_char();
}

void _sys_exit(int return_code)
{
    while (1)
    {
        ;
    }
}

#elif defined(__GNUC__)

int _write(int file, const char * buf, int len)
{
    int tx_len = 0;

    while (tx_len < len)
    {
        dbg_send_char(*buf);
        buf++;
        tx_len++;
    }
    return tx_len;
}

int _read(int file, char * buf, int len)
{
    int rx_len = 0;

    while (rx_len < len)
    {
        *buf = dbg_get_char();
        buf++;
        rx_len++;
    }
    return rx_len;
}


#elif defined(__ICCARM__)

size_t __write(int handle, const unsigned char * buf, size_t size)
{
    size_t len = 0;

    while (len < size)
    {
        dbg_send_char(*buf);
        buf++;
        len++;
    }
    return len;
}

size_t __read(int handle, unsigned char * buf, size_t size)
{
    int rx_len = 0;

    while (rx_len < size)
    {
        *buf = dbg_get_char();
        buf++;
        rx_len++;
    }
    return rx_len;
}

#endif /* defined(__CC_ARM) */

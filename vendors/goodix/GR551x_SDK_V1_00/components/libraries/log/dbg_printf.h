#ifndef DBG_PRINTF_H_
#define DBG_PRINTF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "gr55xx_hal.h"

/* Ring buffer for interrupt send */
#define DBG_RING_BUF_SIZE       (8192ul)

/* The UART0/1 communication baud rate. */
#define DBG_UART_BAUDRATE       (115200)

typedef enum
{
    DBG_PRINTF_NONE = 0,
    DBG_PRINTF_UART0,
    DBG_PRINTF_UART1,
    DBG_PRINTF_INT_UART0,
    DBG_PRINTF_INT_UART1,
    DBG_PRINTF_ITM,
    DBG_PRINTF_RTT,
}dbg_printf_mode_t;

void dbg_printf_set_mode(dbg_printf_mode_t mode);
uint8_t dbg_printf_uart_callback(uart_regs_t *UARTx);
uint32_t dbg_printf_uart_flush(void);

#ifdef __cplusplus
}
#endif

#endif /* DBG_PRINTF_H_ */

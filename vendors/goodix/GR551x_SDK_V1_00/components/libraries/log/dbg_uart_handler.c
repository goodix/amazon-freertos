#include "dbg_printf.h"

void UART0_IRQHandler(void)
{
    dbg_printf_uart_callback(UART0);
}

void UART1_IRQHandler(void)
{
    dbg_printf_uart_callback(UART1);
}

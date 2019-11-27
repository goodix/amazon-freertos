/**
  ****************************************************************************************
  * @file    gr55xx_ll_uart.c
  * @author  BLE Driver Team
  * @brief   UART LL module driver.
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

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_ll_uart.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */

#if defined (UART0) || defined (UART1)

/** @addtogroup UART_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/** @addtogroup UART_LL_Private_Constants
  * @{
  */

/** @} */


/* Private macros ------------------------------------------------------------*/
/** @addtogroup UART_LL_Private_Macros
  * @{
  */

/* __BAUDRATE__ The maximum Baud Rate is derived from the maximum clock available
 *              divided by the smallest oversampling used on the UART (i.e. 8)    */
#define IS_LL_UART_BAUDRATE(__BAUDRATE__) ((__BAUDRATE__) <= 9000000U)

#define IS_LL_UART_PARITY(__VALUE__) (((__VALUE__) == LL_UART_PARITY_NONE) \
                                    || ((__VALUE__) == LL_UART_PARITY_EVEN) \
                                    || ((__VALUE__) == LL_UART_PARITY_ODD)) \
                                    || ((__VALUE__) == LL_UART_PARITY_SP0)) \
                                    || ((__VALUE__) == LL_UART_PARITY_SP1))

#define IS_LL_UART_DATABITS(__VALUE__) (((__VALUE__) == LL_UART_DATABITS_5B) \
                                       || ((__VALUE__) == LL_UART_DATABITS_6B) \
                                       || ((__VALUE__) == LL_UART_DATABITS_7B)) \
                                       || ((__VALUE__) == LL_UART_DATABITS_8B))

#define IS_LL_UART_STOPBITS(__VALUE__) (((__VALUE__) == LL_UART_STOPBITS_1) \
                                      || ((__VALUE__) == LL_UART_STOPBITS_1_5) \
                                      || ((__VALUE__) == LL_UART_STOPBITS_2))

#define IS_LL_UART_HWCONTROL(__VALUE__) (((__VALUE__) == LL_UART_HWCONTROL_NONE) \
                                       || ((__VALUE__) == LL_UART_HWCONTROL_RTS_CTS))

/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup UART_LL_Exported_Functions
  * @{
  */

/** @addtogroup UART_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize UART registers (Registers restored to their default values).
  * @param  UARTx UART instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: UART registers are de-initialized
  *          - ERROR: UART registers are not de-initialized
  */
__WEAK error_status_t ll_uart_deinit(uart_regs_t *UARTx)
{
    error_status_t status = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(UARTx));

    ll_uart_reset(UARTx);

    return (status);
}

/**
  * @brief  Initialize UART registers according to the specified
  *         parameters in UART_InitStruct.
  * @note   As some bits in UART configuration registers can only be written when the UART is disabled (UART_CR1_UE bit =0),
  *         UART IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @note   Baud rate value stored in p_uart_init BaudRate field, should be valid (different from 0).
  * @param  UARTx UART instance
  * @param  p_uart_init pointer to a ll_uart_init_t structure
  *         that contains the configuration information for the specified UART peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: UART registers are initialized according to p_uart_init content
  *          - ERROR: Problem occurred during UART Registers initialization
  */
__WEAK error_status_t ll_uart_init(uart_regs_t *UARTx, ll_uart_init_t *p_uart_init)
{
    error_status_t status    = ERROR;

    /* Check the parameters */
    gr_assert_param(IS_UART_ALL_INSTANCE(UARTx));
    gr_assert_param(IS_LL_UART_BAUDRATE(p_uart_init->baud_rate));
    gr_assert_param(IS_LL_UART_DATABITS(p_uart_init->data_bits));
    gr_assert_param(IS_LL_UART_STOPBITS(p_uart_init->stop_bits));
    gr_assert_param(IS_LL_UART_PARITY(p_uart_init->parity));
    gr_assert_param(IS_LL_UART_HWCONTROL(p_uart_init->hw_flow_ctrl));

    /* Update SystemCoreClock */
    SystemCoreUpdateClock();

    ll_uart_config_character(UARTx, p_uart_init->data_bits, p_uart_init->parity, p_uart_init->stop_bits);

    ll_uart_set_baud_rate(UARTx, SystemCoreClock, p_uart_init->baud_rate);

    ll_uart_set_hw_flow_ctrl(UARTx, p_uart_init->hw_flow_ctrl);

    /* Flush FIFO */
    ll_uart_flush_tx_fifo(UARTx);
    ll_uart_flush_rx_fifo(UARTx);

    /* Enable FIFO and set threshold */
    ll_uart_enable_fifo(UARTx);
    ll_uart_set_tx_fifo_threshold(UARTx, LL_UART_TX_FIFO_TH_CHAR_2);
    ll_uart_set_rx_fifo_threshold(UARTx, LL_UART_RX_FIFO_TH_CHAR_1);

    return (status);
}

/**
  * @brief Set each @ref ll_uart_init_t field to default value.
  * @param p_uart_init pointer to a @ref ll_uart_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_uart_struct_init(ll_uart_init_t *p_uart_init)
{
    /* Set UART_InitStruct fields to default values */
    p_uart_init->baud_rate    = 9600U;
    p_uart_init->data_bits    = LL_UART_DATABITS_8B;
    p_uart_init->stop_bits    = LL_UART_STOPBITS_1;
    p_uart_init->parity       = LL_UART_PARITY_NONE ;
    p_uart_init->hw_flow_ctrl = LL_UART_HWCONTROL_NONE;
}

/** @} */

/** @} */

/** @} */

#endif /* UART0 || UART1 */

/** @} */

/**
 *****************************************************************************************
 *
 * @file dfu_port.h
 *
 * @brief  DFU port API.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */
#ifndef _DFU_PORT_H__
#define _DFU_PORT_H__

#include <stdint.h>
#include "gr55xx_dfu.h"

/**@brief DFU uart send data function definition. */
typedef void (*dfu_uart_send_data)(uint8_t *p_data, uint16_t length);

/**@brief DFU enter callback definition. */
typedef void (*dfu_enter_callback)(void);

/**
 *****************************************************************************************
 * @brief DFU BLE service init.
 * @details If dfu enter function is not used, dfu_enter can set NULL.
 *
 * @param[in] dfu_enter: DFU enter callback.
 *****************************************************************************************
 */
void dfu_service_init(dfu_enter_callback dfu_enter);

/**
 *****************************************************************************************
 * @brief DFU port init.
 * @details If not using serial port update function, uart_send_data can be set NULL.
            if the user doesn't care about the upgrade status,p_dfu_callback can set NULL.
 *
 * @param[in] uart_send_data: Function is used to send data to master by UART.
 * @param[in] p_dfu_callback: DFU program state callback functions.
 *****************************************************************************************
 */
void dfu_port_init(dfu_uart_send_data uart_send_data, dfu_pro_callback_t *p_dfu_callback);

#endif


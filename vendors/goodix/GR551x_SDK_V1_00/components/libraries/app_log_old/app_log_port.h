/**
 ****************************************************************************************
 *
 * @file app_log_port.h
 *
 * @brief App Log Port API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __APP_LOG_PORT_H__
#define __APP_LOG_PORT_H__

#include <stdint.h>

/**
 * @defgroup APP_LOG_PORT_MAROC Defines
 * @{
 */
#define APP_LOG_UART_BAUDRATE           115200              /**< The app log uart communication baud rate. */
/** @} */

/**
 * @defgroup APP_LOG_PORT_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize app log port.
 *****************************************************************************************
 */
void app_log_port_init(void);

/**
 *****************************************************************************************
 * @brief Output app log via uart.
 *
 * @param[in] p_log:  Pointer to log.
 * @param[in] length: Length of log data.
 *****************************************************************************************
 */
void app_log_port_output(const uint8_t *p_log, uint16_t length);

void app_log_activate_thread_safe(void);

/**
 *****************************************************************************************
 * @brief Flush all log entries from the buffer
 *****************************************************************************************
 */
void app_log_port_flush(void);
/** @} */

#endif



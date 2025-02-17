/**
 ****************************************************************************************
 *
 * @file app_log_cfg.h
 *
 * @brief App Log Config API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __APP_LOG_CFG_H__
#define __APP_LOG_CFG_H__

#include  "custom_config.h"

/**
 * @defgroup APP_LOG_CFG_MAROC Defines
 * @{
 */
#ifndef APP_LOG_PRINTF_ENABLE
#define APP_LOG_PRINTF_ENABLE           APP_LOG_ENABLE             /**< Enable app log module.*/
#endif

#ifndef APP_LOG_COLOR_ENABLE
#define APP_LOG_COLOR_ENABLE            0                          /**< Enable text color format. */
#endif

#ifndef APP_LOG_OUTPUT_LOCK_ENABLE
#define APP_LOG_OUTPUT_LOCK_ENABLE      1                          /**< Enable app log output lock. */
#endif

#ifndef APP_LOG_TAG_ENABLE
#define APP_LOG_TAG_ENABLE              0                          /**< Enable app log tag. */
#endif

#ifndef APP_LOG_ASYNC_OUTPUT_ENABLE
#define APP_LOG_ASYNC_OUTPUT_ENABLE     1                          /**< Enable app log asynchronous output mode. */
#endif

#if APP_LOG_OUTPUT_LOCK_ENABLE
    #define APP_LOG_LOCK()              LOCAL_INT_DISABLE(BLE_IRQn) /**< App log lock. */
    #define APP_LOG_UNLOCK()            LOCAL_INT_RESTORE()         /**< APP log unlock. */
#else
    #define APP_LOG_LOCK()                                         /**< App log lock. */
    #define APP_LOG_UNLOCK()                                       /**< APP log unlock. */
#endif

#ifndef APP_LOG_TAG
    #define APP_LOG_TAG                 "NO_TAG"                   /**< Default app log tag. */
#endif

#define APP_LOG_UART_PORT               1                          /**< App log print via uart. */
#define APP_LOG_RTT_PORT                2                          /**< App log print via RTT. */
#define APP_LOG_ITM_PORT                3                          /**< App log print via ITM. */

#define APP_LOG_PROT_MODE               APP_LOG_UART_PORT          /**< Default app log print mode. */
#define APP_LOG_LINE_BUF_SIZE           256                        /**< Buffer size for every line's log. */
#define APP_LOG_CACHE_BUF_SIZE          8192                       /**< Buffer size for app log cache. */
#define APP_LOG_SEVERITY_LEVEL          APP_LOG_LVL_DEBUG          /**< Default log severity level. */
#define APP_LOG_TAG_LEN_MAX             20                         /**< Maximum length of output filter's tag. */
#define APP_LOG_LINE_NB_LEN_MAX         5                          /**< Maximum length of output line number. */
#define APP_LOG_NEWLINE_SIGN            "\r\n"                     /**< Newline sign output. */

/** @} */

#endif


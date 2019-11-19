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


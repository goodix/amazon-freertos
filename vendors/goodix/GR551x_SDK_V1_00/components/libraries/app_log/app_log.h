/**
 ****************************************************************************************
 *
 * @file app_log.h
 *
 * @brief App Log API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __APP_LOG_H__
#define __APP_LOG_H__

#include "custom_config.h"
#include "gr55xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_LOG_MAROC Defines
 * @{
 */
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

#ifndef APP_LOG_TAG_ENABLE
#define APP_LOG_TAG_ENABLE              0                          /**< Enable app log tag. */
#endif

#ifndef APP_LOG_ASYNC_OUTPUT_ENABLE
#define APP_LOG_ASYNC_OUTPUT_ENABLE     1                          /**< Enable app log asynchronous output mode. */
#endif

#define APP_LOG_LOCK()                  LOCAL_INT_DISABLE(BLE_IRQn) /**< App log lock. */
#define APP_LOG_UNLOCK()                LOCAL_INT_RESTORE()         /**< APP log unlock. */

#ifndef APP_LOG_TAG
    #define APP_LOG_TAG                 "NO_TAG"                   /**< Default app log tag. */
#endif

#define APP_LOG_LINE_BUF_SIZE           256                        /**< Buffer size for every line's log. */
#define APP_LOG_CACHE_BUF_SIZE          8192                       /**< Buffer size for app log cache. */
#define APP_LOG_SEVERITY_LEVEL          APP_LOG_LVL_DEBUG          /**< Default log severity level. */
#define APP_LOG_TAG_LEN_MAX             20                         /**< Maximum length of output filter's tag. */
#define APP_LOG_LINE_NB_LEN_MAX         5                          /**< Maximum length of output line number. */
#define APP_LOG_NEWLINE_SIGN            "\r\n"                     /**< Newline sign output. */

/**
 * @defgroup APP_LOG_FMT APP Log Formats
 * @{
 */
#define APP_LOG_FMT_NULL        (0x00)          /**< No add any app log format. */
#define APP_LOG_FMT_LVL         (1 << 0)        /**< Add app log level format. */
#define APP_LOG_FMT_TAG         (1 << 1)        /**< Add app log tag format. */
#define APP_LOG_FMT_DIR         (1 << 2)        /**< Add app log file directory and name format. */
#define APP_LOG_FMT_FUNC        (1 << 3)        /**< Add app log function name format. */
#define APP_LOG_FMT_LINE        (1 << 4)        /**< Add app log number format. */
#define APP_LOG_FMT_ALL         (0x1f)          /**< Add all app log formats. */
/** @} */

/**
 * @defgroup APP_LOG_SVT_LVL APP Log Severity Levels
 * @{
 */
#define APP_LOG_LVL_ERROR       (0)             /**< Error severity level. */
#define APP_LOG_LVL_WARNING     (1)             /**< Warning severity level. */
#define APP_LOG_LVL_INFO        (2)             /**< Info severity level. */
#define APP_LOG_LVL_DEBUG       (3)             /**< Debug severity level. */
#define APP_LOG_LVL_NB          (4)             /**< Number of all severity level.  */
/** @} */

#if APP_LOG_PRINTF_ENABLE
    #if APP_LOG_SEVERITY_LEVEL >= APP_LOG_LVL_ERROR
        #define APP_LOG_ERROR(...) app_log_output(APP_LOG_LVL_ERROR, APP_LOG_TAG, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
    #else
        #define APP_LOG_ERROR(...)
    #endif

    #if APP_LOG_SEVERITY_LEVEL >= APP_LOG_LVL_WARNING
        #define APP_LOG_WARNING(...) app_log_output(APP_LOG_LVL_WARNING, APP_LOG_TAG, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
    #else
        #define APP_LOG_WARNING(...)
    #endif

    #if APP_LOG_SEVERITY_LEVEL >= APP_LOG_LVL_INFO
        #define APP_LOG_INFO(...) app_log_output(APP_LOG_LVL_INFO, APP_LOG_TAG, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
    #else
        #define APP_LOG_INFO(...)
    #endif

    #if APP_LOG_SEVERITY_LEVEL >= APP_LOG_LVL_DEBUG
        #define APP_LOG_DEBUG(...) app_log_output(APP_LOG_LVL_DEBUG, APP_LOG_TAG, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__)
    #else
        #define APP_LOG_DEBUG(...)
    #endif

    #define APP_LOG_RAW_INFO(...)             app_log_raw_info( __VA_ARGS__)
    #define APP_LOG_HEX_DUMP(p_data, length)  app_log_hex_dump(p_data, length)
#else
    #define APP_LOG_ERROR(...)
    #define APP_LOG_WARNING(...)
    #define APP_LOG_INFO(...)
    #define APP_LOG_DEBUG(...)
    #define APP_LOG_RAW_INFO(...)
    #define APP_LOG_HEX_DUMP(p_data, length)
#endif
/** @} */

/**
 * @defgroup APP_LOG_TYPEDEF Typedefs
 * @{
 */
/**@brief  APP LOG transmit function type. */
typedef void (*app_log_trans_func_t)(uint8_t *p_data, uint16_t length);

/**@brief  APP LOG flush function type. */
typedef void (*app_log_flush_func_t)(void);
/** @} */

/**
 * @defgroup APP_LOG_STRUCT Structures
 * @{
 */
/**@brief App log filter. */
typedef struct
{
    uint8_t     level;                          /**< App log filter level. */
    char        tag[APP_LOG_TAG_LEN_MAX + 1];   /**< App log filter tag. */
} app_log_filter_t;

/**@brief App log init stucture. */
typedef struct
{
    app_log_filter_t  filter;                     /**< App log filter. */
    uint8_t           fmt_set[APP_LOG_LVL_NB];    /**< Format of app log. See @ref APP_LOG_FMT.*/
} app_log_init_t;
/** @} */

/**
 * @defgroup APP_LOG_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize app log module.
 *
 * @param[in] p_log_init: Pointer to app log initialization.
 * @param[in] trans_func: App log transmit function.
 * @param[in] flush_func: App log flush function.
 *
 * @return Result of initialization.
 *****************************************************************************************
 */
bool app_log_init(app_log_init_t *p_log_init, app_log_trans_func_t trans_func, app_log_flush_func_t flush_func);

/**
 *****************************************************************************************
 * @brief Output app log.
 *
 * @param[in] level: App log severity level.
 * @param[in] tag:   App log tag.
 * @param[in] file:  File name.
 * @param[in] func:  Funciton name.
 * @param[in] line： Line number.
 * @param[in] fomat: Output format.
 * @param[in] ...:   Arguments.
 *****************************************************************************************
 */
void app_log_output(uint8_t level, const char *tag, const char *file, const char *func, const long line, const char *format, ...);

/**
 *****************************************************************************************
 * @brief Output RAW format log
 *
 * @param[in] fomat: Output format.
 * @param[in] ...:   Arguments.
 *****************************************************************************************
 */
void app_log_raw_info(const char *format, ...);

/**
 *****************************************************************************************
 * @brief Dump the hex format data to log.
 *
 * @param[in] p_data: Pointer to data.
 * @param[in] length  Length of data.
 *****************************************************************************************
 */
void app_log_hex_dump(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Flush app log.
 *****************************************************************************************
 */
void app_log_flush(void);
/** @} */

#endif



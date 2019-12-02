/**
 ****************************************************************************************
 *
 * @file app_error_cfg.h
 *
 * @brief App Error Config API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __APP_ERROR_CFG_H__
#define __APP_ERROR_CFG_H__

#include "gr55xx.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/**
 * @defgroup APP_ERROR_CFG_MAROC Defines
 * @{
 */
#define APP_IS_USING_FREEROTS               false                                 /**< Is using FREEROTS or not. */
#define APP_ERROR_DUMP_STACK_INFO_ENABLE    1                                     /**< Enable dump stack information. */
#define APP_ERROR_INFO_PRINT_ENABLE         1                                     /**< Enable error information prinf. */
#define APP_ERROR_CALL_STACK_DEPTH_MAX      16                                    /**< Supported function call stack max depth, default is 16. */

#if APP_ERROR_INFO_PRINT_ENABLE
    #define APP_ERROR_INFO_PRINT(...)           printf(__VA_ARGS__);printf("\r\n");/**< Print line. */
#else
    #define APP_ERROR_INFO_PRINT(...)
#endif

#if APP_IS_USING_FREEROTS
    #include "FreeRTOS.h"
    extern uint32_t *vTaskStackAddr(void);
    extern uint32_t  vTaskStackSize(void);
    extern char     *vTaskName(void);
#endif

/** @} */

#endif


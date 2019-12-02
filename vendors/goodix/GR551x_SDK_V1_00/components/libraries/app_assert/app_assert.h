/**
 ****************************************************************************************
 *
 * @file app_assert.h
 *
 * @brief App Assert API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __APP_ASSERT_H__
#define __APP_ASSERT_H__

#include "gr55xx_sys.h"
#include <stdint.h>

/**
 * @defgroup APP_ASSERT_MAROC Defines
 * @{
 */
/**@brief Macro for calling error handler function if assert check failed. */
#define APP_ASSERT_CHECK(EXPR)                              \
    do                                                      \
    {                                                       \
        if (!(EXPR))                                        \
        {                                                   \
            app_assert_handler(#EXPR, __FILE__, __LINE__);  \
        }                                                   \
    } while(0)
/** @} */

/**
 * @defgroup APP_ASSERT_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Register assert default callbacks.
 *****************************************************************************************
 */
void app_assert_default_cb_register(void);

/**
 *****************************************************************************************
 * @brief Register user assert callbacks.
 *****************************************************************************************
 */
void app_assert_cb_register(sys_assert_cb_t *p_assert_cb);

/**
 *****************************************************************************************
 * @brief App assert handler.
 *
 * @param[in] expr:   Pxpression.
 * @param[in] file:   File name.
 * @param[in] line:  Line number.
 *****************************************************************************************
 */
void app_assert_handler(const char *expr, const char *file, int line);
/** @} */

#endif 



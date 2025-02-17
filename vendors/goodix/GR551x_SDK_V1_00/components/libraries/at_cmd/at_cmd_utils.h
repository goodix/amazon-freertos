/**
 *****************************************************************************************
 *
 * @file AT_CMD_UTILS.h
 *
 * @brief AT Command Utilities API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __AT_CMD_UTILS_H__
#define __AT_CMD_UTILS_H__

#include "at_cmd.h"
#include "gr55xx_hal_def.h"
#include "gr55xx_sys.h"

/**
 * @defgroup AT_CMD_UTILS_MACRO Defines
 * @{
 */
#define ERROR_CHECK(error_code)     do                          \
                                    {                           \
                                        if (0 != error_code)    \
                                        {                       \
                                            return error_code;  \
                                        }                       \
                                    } while(0)                                  /**< Error check. */

#define AT_CMD_RSP_DEF(at_cmd_rsp)  at_cmd_rsp_t at_cmd_rsp =              \
                                    {                                      \
                                        .error_code = AT_CMD_ERR_NO_ERROR, \
                                        .data       = {0},                 \
                                        .length     = 0,                   \
                                    }                                          /**< Define AT CMD response variable. */
/** @} */

/**
 * @defgroup CTS_C_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Prinf string and push to buffer.
 *
 * @param[in] p_buff: Pointer to buffer.
 *
 * @result Length of data push to buffer.
 *****************************************************************************************
 */
uint8_t at_cmd_printf_bush(uint8_t *p_buff, const char *format, ...);

/**
 *****************************************************************************************
 * @brief Check decimal number is valid and calculate.
 *
 * @param[in]  p_data: Pointer to data.
 * @param[in]  length: Length of dta.
 * @param[out] p_num:  Result of calculate.

 * @result Result of check.
 *****************************************************************************************
 */
bool at_cmd_decimal_num_check(uint8_t *p_data, uint16_t length, uint32_t *p_num);

/**
 *****************************************************************************************
 * @brief Check hexadecimal number is valid and calculate.
 *
 * @param[in]  p_data: Pointer to data.
 * @param[in]  length: Length of dta.
 * @param[out] p_num:  Result of calculate.
 *
 * @result Result of check.
 *****************************************************************************************
 */
bool at_cmd_hex_num_check(uint8_t *p_data, uint16_t length, uint32_t *p_num);

/**
 *****************************************************************************************
 * @brief Convert hal error code to AT CMD error code.
 *
 * @param[in]  error_code: HAL error code.
 *
 * @result Result of convert.
 *****************************************************************************************
 */
at_cmd_error_t at_cmd_hal_err_convert(hal_status_t error_code);

/**
 *****************************************************************************************
 * @brief Convert ble error code to AT CMD error code.
 *
 * @param[in] error_code: BLE error code.
 *
 * @result Result of convert.
 *****************************************************************************************
 */
at_cmd_error_t at_cmd_ble_err_convert(sdk_err_t   error_code);
/** @} */
#endif

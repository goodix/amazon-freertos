/**
 *****************************************************************************************
 *
 * @file app_error.c
 *
 * @brief App Error Implementation.
 *
 *****************************************************************************************
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

#define APP_LOG_TAG "app_error.c"

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_error.h"
#include "app_error_cfg.h"
#include "app_log.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * DEFINITIONS
 *****************************************************************************************
 */
#define APP_ERROR_INFO_LEN          1024
#define APP_ERROR_CODE_NB           30

/*
 * STRUCTURES
 *****************************************************************************************
 */
/**@brief SDK error code information. */
struct error_code_info_t
{
    sdk_err_t  error_code;
    char      *error_info;
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static char  s_error_print_info[APP_ERROR_INFO_LEN] = { 0 };

static struct error_code_info_t s_error_code_info[APP_ERROR_CODE_NB] =
{
    {SDK_SUCCESS,                              "Successful."},
    {SDK_ERR_INVALID_PARAM,                    "Invalid parameter supplied."},
    {SDK_ERR_POINTER_NULL,                     "Invalid pointer supplied."},
    {SDK_ERR_INVALID_CONN_IDX,                 "Invalid connection index supplied."},
    {SDK_ERR_INVALID_HANDLE,                   "Invalid handle supplied."},
    {SDK_ERR_PROFILE_COUNT,                    "Maximum SDK profile count exceeded."},
    {SDK_ERR_BUSY,                             "SDK is busy internally."},
    {SDK_ERR_TIMER_INSUFFICIENT,               "Timer is insufficient."},
    {SDK_ERR_NVDS_NOT_INIT,                    "NVDS is not initiated."},
    {SDK_ERR_LIST_ITEM_NOT_FOUND,              "Item not found in list."},
    {SDK_ERR_LIST_ITEM_ALREADY_EXISTED,        "Item already existed in list."},
    {SDK_ERR_LIST_FULL,                        "List is full."},
    {SDK_ERR_SDK_INTERNAL,                     "SDK internal error."},
    {SDK_ERR_INVALID_BUFF_LENGTH,              "The buffer's length is not enough."},
    {SDK_ERR_INVALID_DATA_LENGTH,              "Invalid data length supplied."},
    {SDK_ERR_DISALLOWED,                       "Operation is disallowed."},
    {SDK_ERR_NO_RESOURCES,                     "Not enough resources for operation."},
    {SDK_ERR_REQ_NOT_SUPPORTED,                "Request not supported."},
    {SDK_ERR_INVALID_OFFSET,                   "Offset exceeds current attribute value length."},
    {SDK_ERR_INVALID_ATT_VAL_LEN,              "Invalid length of the attribute value."},
    {SDK_ERR_INVALID_PERM,                     "Permission set in service/attribute are invalid."},
    {SDK_ERR_INVALID_ADV_IDX,                  "Invalid advertising index supplied."},
    {SDK_ERR_INVALID_ADV_DATA_TYPE,            "Invalid advertising data type supplied."},
    {SDK_ERR_INVALID_PSM_NUM,                  "Invalid psm number."},
    {SDK_ERR_INVALID_PSM_ALREADY_REGISTERED,   "The psm number has been registered."},
    {SDK_ERR_INVALID_PSM_EXCEEDED_MAX_PSM_NUM, "The maximum psm number limit is exceeded."},
    {SDK_ERR_NTF_DISABLED,                     "Notification Not Enabled."},
    {SDK_ERR_IND_DISABLED,                     "Indication Not Enabled."},
    {SDK_ERR_DISCONNECTED,                     "Disconnected occur."},
    {SDK_ERR_APP_ERROR,                        "Application error."},
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
__WEAK void app_error_fault_handler(app_error_info_t *p_error_info)
{
#if APP_ERROR_INFO_PRINT_ENABLE
    memset(s_error_print_info, 0, APP_ERROR_INFO_LEN);

    if (APP_ERROR_API_RET == p_error_info->error_type)
    {
        for (uint8_t i = 0; i < APP_ERROR_CODE_NB; i++)
        {
            if (p_error_info->value.error_code == s_error_code_info[i].error_code)
            {
                sprintf(s_error_print_info,
                        "Error code 0X%04X: %s",
                        p_error_info->value.error_code,
                        s_error_code_info[i].error_info);
            }
            else if (APP_ERROR_CODE_NB == i)
            {
                sprintf(s_error_print_info, "Error code 0X%04X: No found information.", p_error_info->value.error_code);
            }
        }
    }
    else if (APP_ERROR_BOOL_COMPARE == p_error_info->error_type)
    {
        sprintf(s_error_print_info,
                "(%s) is not established.",
                p_error_info->value.expr);
    }

    app_log_output(APP_LOG_LVL_ERROR,
                   APP_LOG_TAG,
                   p_error_info->file,
                   p_error_info->func,
                   p_error_info->line,
                   "%s",
                   s_error_print_info);

    app_log_flush();
    while(1);
#endif
}

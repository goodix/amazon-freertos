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
#define APP_ERROR_CODE_NB           43

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
    {SDK_ERR_INVALID_ADDRESS,                  "Invalid address supplied."},
    {SDK_ERR_INVALID_ADV_INTERVAL,             "Invalid advertising interval supplied."},
    {SDK_ERR_INVALID_DISVCOVERY_MODE,          "Invalid discovery mode supplied."},
    {SDK_ERR_INVALID_ADV_PARAM,                "Invalid advertising parameters supplied."},
    {SDK_ERR_INVALID_ADV_PEER_ADDR,            "Invalid peer address supplied."},
    {SDK_ERR_ADV_DATA_NOT_SET,                 "Legacy advertising data not set."},
    {SDK_ERR_PER_ADV_DATA_NOT_SET,             "Periodic advertising data not set."},
    {SDK_ERR_EXT_SCAN_RSP_DATA_NOT_SET,        "Extended scan response data not set."},
    {SDK_ERR_INVALID_DURATION_PARAM,           "Invalid duration parameter_supplied."},
    {SDK_ERR_INVALID_PER_SYNC_IDX,             "Invalid periodic synchronization index supplied."},
    {SDK_ERR_INVALID_CID,                      "Invalid CID supplied."},
    {SDK_ERR_INVALID_CHL_NUM,                  "Invalid channel number supplied."},
    {SDK_ERR_NOT_ENOUGH_CREDITS,               "Not enough credits."},
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
        for (uint8_t i = 0; ; i++)
        {
            if (p_error_info->value.error_code == s_error_code_info[i].error_code)
            {
                sprintf(s_error_print_info,
                        "Error code 0X%04X: %s",
                        p_error_info->value.error_code,
                        s_error_code_info[i].error_info);
                break;
            }
            else if (APP_ERROR_CODE_NB == i)
            {
                sprintf(s_error_print_info, "Error code 0X%04X: No found information.", p_error_info->value.error_code);
                break;
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
#endif
}

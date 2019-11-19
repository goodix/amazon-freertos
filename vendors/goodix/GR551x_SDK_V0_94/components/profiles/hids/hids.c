/**
 *****************************************************************************************
 *
 * @file hids.c
 *
 * @brief THe Implementation of Human Input Device Service.
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

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "hids.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"

/*
 * ENUMERATIONS
 *****************************************************************************************
 */
/**@brief HIDS Attributes database index list. */
enum hids_attr_idx_tag
{
    HIDS_IDX_SVC,                       // 00

    HIDS_IDX_HID_INFO_CHAR,             // 01
    HIDS_IDX_HID_INFO_VAL,              // 02

    HIDS_IDX_CTRL_POINT_CHAR,           // 03
    HIDS_IDX_CTRL_POINT_VAL,            // 04

    HIDS_IDX_REPORT_MAP_CHAR,           // 05
    HIDS_IDX_REPORT_MAP_VAL,            // 06

    HIDS_IDX_INPUT_REPORT_CHAR_1,       // 07
    HIDS_IDX_INPUT_REPORT_VAL_1,        // 08
    HIDS_IDX_INPUT_REPORT_CCCD_1,       // 09
    HIDS_IDX_INPUT_REPORT_REF_1,        // 10
    
#if INPUT_REPORT_COUNT > 1
    HIDS_IDX_INPUT_REPORT_CHAR_2,       // 07
    HIDS_IDX_INPUT_REPORT_VAL_2,        // 08
    HIDS_IDX_INPUT_REPORT_CCCD_2,       // 09
    HIDS_IDX_INPUT_REPORT_REF_2,        // 10
#endif

    HIDS_IDX_OUTPUT_REPORT_CHAR,        // 11
    HIDS_IDX_OUTPUT_REPORT_VAL,         // 12
    HIDS_IDX_OUTPUT_REPORT_REF,         // 13

    HIDS_IDX_FEATURE_REPORT_CHAR,       // 14
    HIDS_IDX_FEATURE_REPORT_VAL,        // 15
    HIDS_IDX_FEATURE_REPORT_REF,        // 16

    HIDS_IDX_BOOT_KB_IN_RPT_CHAR,       // 17
    HIDS_IDX_BOOT_KB_IN_RPT_VAL,        // 18
    HIDS_IDX_BOOT_KB_IN_RPT_CCCD,       // 19

    HIDS_IDX_BOOT_KB_OUT_RPT_CHAR,      // 20
    HIDS_IDX_BOOT_KB_OUT_RPT_VAL,       // 21

    HIDS_IDX_BOOT_MS_IN_RPT_CHAR,       // 22
    HIDS_IDX_BOOT_MS_IN_RPT_VAL,        // 23
    HIDS_IDX_BOOT_MS_IN_RPT_CCCD,       // 24

    HIDS_IDX_PROTOCOL_MODE_CHAR,        // 25
    HIDS_IDX_PROTOCOL_MODE_VAL,         // 26

    HIDS_IDX_NB,
};

/**@brief Full HID Service Database Description - Used to add attributes into the database. */
static const attm_desc_t hids_attr_tab[HIDS_IDX_NB] =
{
    //00 HID Service Declaration
    [HIDS_IDX_SVC] = {BLE_ATT_DECL_PRIMARY_SERVICE, READ_PERM_UNSEC, 0, 0},

    //01 HID Information Characteristic - Declaration
    [HIDS_IDX_HID_INFO_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //02 HID Information Characteristic - Value
    [HIDS_IDX_HID_INFO_VAL]  = {BLE_ATT_CHAR_HID_INFO,       READ_PERM(AUTH), ATT_VAL_LOC_STACK, sizeof(hids_hid_info_t)},

    //03 HID Control Point Characteristic - Declaration
    [HIDS_IDX_CTRL_POINT_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC,      0, 0},
    //04 HID Control Point Characteristic - Value
    [HIDS_IDX_CTRL_POINT_VAL]  = {BLE_ATT_CHAR_HID_CTNL_PT,    WRITE_CMD_PERM(AUTH), ATT_VAL_LOC_STACK, sizeof(uint8_t)},

    //05 Report Map Characteristic - Declaration
    [HIDS_IDX_REPORT_MAP_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //06 Report Map Characteristic - Value
    [HIDS_IDX_REPORT_MAP_VAL]  = {BLE_ATT_CHAR_REPORT_MAP,     READ_PERM(AUTH), ATT_VAL_LOC_STACK, REPORT_MAP_MAX_SIZE},

    //07 Input Report Characteristic - Declaration
    [HIDS_IDX_INPUT_REPORT_CHAR_1] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //08 Input Report Characteristic - Value
    [HIDS_IDX_INPUT_REPORT_VAL_1]  = {BLE_ATT_CHAR_REPORT, READ_PERM(AUTH) | NOTIFY_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_STACK, HIDS_INPUT_REPORT_MAX_LEN},
    //09 Input Report Characteristic - Descriptor: CCCD
    [HIDS_IDX_INPUT_REPORT_CCCD_1] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, READ_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_USER, 0},
    //10 Input Report Characteristic - Descriptor: Report Reference
    [HIDS_IDX_INPUT_REPORT_REF_1]  = {BLE_ATT_DESC_REPORT_REF, READ_PERM(AUTH), ATT_VAL_LOC_STACK, sizeof(hids_report_ref_t)},

#if INPUT_REPORT_COUNT > 1
    //07 Input Report Characteristic - Declaration
    [HIDS_IDX_INPUT_REPORT_CHAR_2] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //08 Input Report Characteristic - Value
    [HIDS_IDX_INPUT_REPORT_VAL_2]  = {BLE_ATT_CHAR_REPORT, READ_PERM(AUTH) | NOTIFY_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_STACK, HIDS_INPUT_REPORT_MAX_LEN},
    //09 Input Report Characteristic - Descriptor: CCCD
    [HIDS_IDX_INPUT_REPORT_CCCD_2] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, READ_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_USER, 0},
    //10 Input Report Characteristic - Descriptor: Report Reference
    [HIDS_IDX_INPUT_REPORT_REF_2]  = {BLE_ATT_DESC_REPORT_REF, READ_PERM(AUTH), ATT_VAL_LOC_STACK, sizeof(hids_report_ref_t)},
#endif

    //11 Output Report Characteristic - Declaration
    [HIDS_IDX_OUTPUT_REPORT_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //12 Output Report Characteristic - Value
    [HIDS_IDX_OUTPUT_REPORT_VAL]  = {BLE_ATT_CHAR_REPORT, READ_PERM(AUTH) | WRITE_REQ_PERM(AUTH) | WRITE_CMD_PERM(AUTH),
                                     ATT_VAL_LOC_STACK, HIDS_OUTPUT_REPORT_MAX_LEN},
    //13 Output Report Characteristic - Descriptor: Report Reference
    [HIDS_IDX_OUTPUT_REPORT_REF]  = {BLE_ATT_DESC_REPORT_REF, READ_PERM(AUTH), ATT_VAL_LOC_STACK, sizeof(hids_report_ref_t)},

    //14 Feature Report Characteristic - Declaration
    [HIDS_IDX_FEATURE_REPORT_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //15 Feature Report Characteristic - Value
    [HIDS_IDX_FEATURE_REPORT_VAL]  = {BLE_ATT_CHAR_REPORT, READ_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_STACK, HIDS_FEATURE_REPORT_MAX_LEN},
    //16 Feature Report Characteristic - Descriptor: Report Reference
    [HIDS_IDX_FEATURE_REPORT_REF]  = {BLE_ATT_DESC_REPORT_REF, READ_PERM(AUTH), ATT_VAL_LOC_STACK, sizeof(hids_report_ref_t)},

    //17 Boot Keyboard Input Report Characteristic - Declaration
    [HIDS_IDX_BOOT_KB_IN_RPT_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //18 Boot Keyboard Input Report Characteristic - Value
    [HIDS_IDX_BOOT_KB_IN_RPT_VAL]  = {BLE_ATT_CHAR_BOOT_KB_IN_REPORT,
                                      READ_PERM(AUTH) | NOTIFY_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_STACK, BOOT_KB_IN_REPORT_MAX_SIZE},
    //19 Boot Keyboard Input Report Characteristic - Descriptor: CCCD
    [HIDS_IDX_BOOT_KB_IN_RPT_CCCD] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, READ_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_USER, 0},

    //20 Boot Keyboard Output Report Characteristic - Declaration
    [HIDS_IDX_BOOT_KB_OUT_RPT_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //21 Boot Keyboard Output Report Characteristic - Value
    [HIDS_IDX_BOOT_KB_OUT_RPT_VAL]  = {BLE_ATT_CHAR_BOOT_KB_OUT_REPORT,
                                       READ_PERM(AUTH) | WRITE_REQ_PERM(AUTH) | WRITE_CMD_PERM(AUTH),
                                       ATT_VAL_LOC_STACK, BOOT_KB_OUT_REPORT_MAX_SIZE},

    //22 Boot Mouse Input Report Characteristic - Declaration
    [HIDS_IDX_BOOT_MS_IN_RPT_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //23 Boot Mouse Input Report Characteristic - Value
    [HIDS_IDX_BOOT_MS_IN_RPT_VAL]  = {BLE_ATT_CHAR_BOOT_MOUSE_IN_REPORT,
                                      READ_PERM(AUTH) | NOTIFY_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_STACK, BOOT_MOUSE_IN_REPORT_MAX_SIZE},
    //24 Boot Mouse Input Report Characteristic - Descriptor: CCCD
    [HIDS_IDX_BOOT_MS_IN_RPT_CCCD] = {BLE_ATT_DESC_CLIENT_CHAR_CFG, READ_PERM(AUTH) | WRITE_REQ_PERM(AUTH),
                                      ATT_VAL_LOC_USER, 0},

    //25 Protocol Mode Characteristic - Declaration
    [HIDS_IDX_PROTOCOL_MODE_CHAR] = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    //26 Protocol Mode Characteristic - Value
    [HIDS_IDX_PROTOCOL_MODE_VAL]  = {BLE_ATT_CHAR_PROTOCOL_MODE, READ_PERM(AUTH) | WRITE_CMD_PERM(AUTH),
                                     ATT_VAL_LOC_STACK, sizeof(uint8_t)},
};

/* 
 * STRUCT
 *****************************************************************************************
 */
/**@brief HID Service environment variable. */
typedef struct 
{
    hids_init_t hids_init;
    uint16_t    input_rep_cccds[INPUT_REPORT_COUNT];
    uint16_t    boot_kb_input_rep_cccd;
    uint16_t    boot_ms_input_rep_cccd;
    uint16_t    start_hdl;                 /**< HID Service start handle. */
    uint8_t     conn_idx;                  /**< Index of the current connection. */
} hids_env_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static hids_env_t s_hids_env;

/*
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************************
 */
static sdk_err_t hids_init(void);
static void      hids_write_att_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_param);
static void      hids_read_att_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_param);
static void      hids_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value);
static sdk_err_t hids_input_report_send(uint16_t length, uint8_t *p_data, uint8_t cccd_idx, uint8_t value_idx);
static void      hids_on_connect(uint8_t conn_idx);
static void      hids_on_disconnect(uint8_t conn_idx, uint8_t reason);

/**@brief HIDS interface required by profile manager. */
static ble_prf_manager_cbs_t hids_mgr_cbs =
{
    (prf_init_func_t)hids_init,
    hids_on_connect,
    hids_on_disconnect
};

/**@brief HIDS GATT server Callbacks. */
static gatts_prf_cbs_t hids_gatts_cbs =
{
    hids_read_att_cb,
    hids_write_att_cb,
    NULL,
    NULL,
    hids_cccd_set_cb
};

/**@brief HIDS Information. */
static const prf_server_info_t hids_prf_info =
{
    /* There shall be only one instance on a device */
    .max_connection_nb = 1,
    .manager_cbs       = &hids_mgr_cbs,
    .gatts_prf_cbs     = &hids_gatts_cbs
};

/**
 *****************************************************************************************
 * @brief Initialize HID service and create db in att.
 *
 * @return BLE_ATT_ERR_NO_ERROR on success, otherwise error code.
 *****************************************************************************************
 */
static sdk_err_t hids_init(void)
{
    sdk_err_t         error_code      = SDK_SUCCESS;
    const uint8_t     hids_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(BLE_ATT_SVC_HID);
    gatts_create_db_t gatts_db;
    uint16_t          start_hdl       = PRF_INVALID_HANDLE; /* The start hanlde is an in/out
                                                             * parameter of ble_gatts_srvc_db_create().
                                                             * It must be set with PRF_INVALID_HANDLE
                                                             * to be allocated automatically by BLE Stack.*/

    memset(&gatts_db, 0, sizeof(gatts_db));

    gatts_db.shdl                 = &start_hdl;
    gatts_db.uuid                 = (uint8_t*)hids_svc_uuid;
    gatts_db.attr_tab_cfg         = (uint8_t*)&s_hids_env.hids_init.char_mask;
    gatts_db.max_nb_attr          = HIDS_IDX_NB;
    gatts_db.srvc_perm            = 0;
    gatts_db.attr_tab_type        = SERVICE_TABLE_TYPE_16;
    gatts_db.attr_tab.attr_tab_16 = hids_attr_tab;
    gatts_db.inc_srvc_num         = s_hids_env.hids_init.included_srv.inc_srv_num < MAX_INC_SRVC_NUM ?
                                    s_hids_env.hids_init.included_srv.inc_srv_num : MAX_INC_SRVC_NUM;

    // Add included services
    for (uint8_t i = 0; i < gatts_db.inc_srvc_num; i++) {
        gatts_db.inc_srvc_handle[i] = s_hids_env.hids_init.included_srv.inc_srv_hdl_ptr[i];
    }

    error_code = ble_gatts_srvc_db_create(&gatts_db);
    if (SDK_SUCCESS == error_code)
    {
        uint16_t handle;
        uint8_t  char_value;

        s_hids_env.start_hdl = *gatts_db.shdl;

        // Mandatory characteristcs in HIDS
        // Initialize HID information
        handle = prf_find_handle_by_idx(HIDS_IDX_HID_INFO_VAL,
                                        s_hids_env.start_hdl,
                                        (uint8_t *)&s_hids_env.hids_init.char_mask);
        ble_gatts_value_set(handle, sizeof(s_hids_env.hids_init.hid_info), 0,
                            (const uint8_t *)&s_hids_env.hids_init.hid_info);

        // Initialize HID control point
        handle = prf_find_handle_by_idx(HIDS_IDX_CTRL_POINT_VAL,
                                        s_hids_env.start_hdl,
                                        (uint8_t *)&s_hids_env.hids_init.char_mask);
        char_value = HIDS_INIT_VALUE_CONTROL_POINT;
        ble_gatts_value_set(handle, sizeof(char_value), 0, &char_value);

        // Initialize Report Map
        if (s_hids_env.hids_init.report_map.map_len > REPORT_MAP_MAX_SIZE)
        {
            error_code = SDK_ERR_PROFILE_COUNT;
        }
        else
        {
            handle = prf_find_handle_by_idx(HIDS_IDX_REPORT_MAP_VAL,
                                            s_hids_env.start_hdl,
                                            (uint8_t *)&s_hids_env.hids_init.char_mask);
            ble_gatts_value_set(handle, s_hids_env.hids_init.report_map.map_len, 0,
s_hids_env.hids_init.report_map.p_map);
        }

        // Mandatory to support at least on Report Type if the Report Characteristic is supported.
        // Initialize Input Report
        handle = prf_find_handle_by_idx(HIDS_IDX_INPUT_REPORT_REF_1,
                                        s_hids_env.start_hdl,
                                        (uint8_t *)&s_hids_env.hids_init.char_mask);
        if (PRF_INVALID_HANDLE != handle)
        {
            ble_gatts_value_set(handle, sizeof(hids_report_ref_t), 0,
                                (uint8_t *)&s_hids_env.hids_init.input_report_refs[0]);
        }
#if INPUT_REPORT_COUNT > 1
        handle = prf_find_handle_by_idx(HIDS_IDX_INPUT_REPORT_REF_2,
                                        s_hids_env.start_hdl,
                                        (uint8_t *)&s_hids_env.hids_init.char_mask);
        if (PRF_INVALID_HANDLE != handle)
        {
            ble_gatts_value_set(handle, sizeof(hids_report_ref_t), 0,
                                (uint8_t *)&s_hids_env.hids_init.input_report_refs[1]);
        }
#endif

        // Initialize Output Report
        handle = prf_find_handle_by_idx(HIDS_IDX_OUTPUT_REPORT_REF,
                                        s_hids_env.start_hdl,
                                        (uint8_t *)&s_hids_env.hids_init.char_mask);
        if (PRF_INVALID_HANDLE != handle)
        {
            ble_gatts_value_set(handle, sizeof(s_hids_env.hids_init.output_report_ref), 0,
                                (uint8_t *)&s_hids_env.hids_init.output_report_ref);
        }

        // Initialize Feature Report
        handle = prf_find_handle_by_idx(HIDS_IDX_FEATURE_REPORT_REF,
                                        s_hids_env.start_hdl,
                                        (uint8_t *)&s_hids_env.hids_init.char_mask);
        if (PRF_INVALID_HANDLE != handle)
        {
            ble_gatts_value_set(handle, sizeof(s_hids_env.hids_init.feature_report_ref), 0,
                                (uint8_t *)&s_hids_env.hids_init.feature_report_ref);
        }

        // Mandatory for HID Devices supporting Boot Protocol Mode
        // Initialize protocol mode
        handle = prf_find_handle_by_idx(HIDS_IDX_PROTOCOL_MODE_VAL,
                                        s_hids_env.start_hdl,
                                        (uint8_t *)&s_hids_env.hids_init.char_mask);
        if (PRF_INVALID_HANDLE != handle)
        {
            char_value = HIDS_DEFAULT_PROTOCOL_MODE;
            ble_gatts_value_set(handle, sizeof(char_value), 0, &char_value);
        }

        s_hids_env.boot_kb_input_rep_cccd = PRF_CLI_STOP_NTFIND;
        s_hids_env.boot_ms_input_rep_cccd = PRF_CLI_STOP_NTFIND;
        s_hids_env.input_rep_cccds[0]     = PRF_CLI_STOP_NTFIND;
#if INPUT_REPORT_COUNT > 1
        s_hids_env.input_rep_cccds[1}     = PRF_CLI_STOP_NTFIND;
#endif
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle the connected event.
 *
 * @param[in] conn_idx: Connection index.
 *****************************************************************************************
 */
static void hids_on_connect(uint8_t conn_idx)
{
    uint16_t handle;

    s_hids_env.conn_idx = conn_idx;

    handle = prf_find_handle_by_idx(HIDS_IDX_PROTOCOL_MODE_VAL,
                                    s_hids_env.start_hdl,
                                    (uint8_t *)&s_hids_env.hids_init.char_mask);

    if (PRF_INVALID_HANDLE != handle)
    {
        uint8_t char_value     = HIDS_DEFAULT_PROTOCOL_MODE;
        sdk_err_t   error_code = ble_gatts_value_set(handle, sizeof(uint8_t), 0,
                                                     (uint8_t *)&char_value);

        if (error_code != BLE_SUCCESS && s_hids_env.hids_init.err_handler)
        {
            s_hids_env.hids_init.err_handler(error_code);
        }
    }
}

/**
 *****************************************************************************************
 * @brief Handle the disconnected event.
 *
 * @param[in] conn_idx: Connect index.
 * @param[in] reason:   The reason of disconnection.
 *****************************************************************************************
 */
static void hids_on_disconnect(uint8_t conn_idx, uint8_t reason)
{
    s_hids_env.conn_idx = GAP_INVALID_CONN_INDEX;
}

/**
 *****************************************************************************************
 * @brief Handle writing control point.
 *
 * @param[in] p_param: Pointer to writing parameters.
 *
 * @return BLE_ATT_ERR_NO_ERROR on success, otherwise error code.
 *****************************************************************************************
 */
static sdk_err_t hids_on_control_point_write(const gatts_write_req_cb_t *p_param)
{
    sdk_err_t error_code;

    if (p_param->length != sizeof(uint8_t))
    {
        error_code = SDK_ERR_INVALID_ATT_VAL_LEN;
    }
    else
    {
        hids_evt_t evt;
        uint8_t value = p_param->value[0];

        switch (value) {
        case HIDS_CONTROL_POINT_SUSPEND:
            evt.evt_type = HIDS_EVT_HOST_SUSP;
            break;

        case HIDS_CONTROL_POINT_EXIT_SUSPEND:
            evt.evt_type = HIDS_EVT_HOST_EXIT_SUSP;
            break;

        default:
            return SDK_ERR_APP_ERROR;
        }

        error_code = ble_gatts_value_set(p_param->handle, sizeof(uint8_t), 0, &value);

        if (BLE_SUCCESS == error_code && s_hids_env.hids_init.evt_handler)
        {
            s_hids_env.hids_init.evt_handler(&evt);
        }
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle writing protocol mode.
 *
 * @param[in] p_param: Pointer to writing parameters.
 *
 * @return BLE_ATT_ERR_NO_ERROR on success, otherwise error code.
 *****************************************************************************************
 */
static sdk_err_t hids_on_protocol_mode_write(const gatts_write_req_cb_t *p_param)
{
    sdk_err_t error_code;

    if (p_param->length != sizeof(uint8_t))
    {
        error_code = SDK_ERR_INVALID_ATT_VAL_LEN;
    }
    else
    {
        hids_evt_t evt;
        uint8_t    value = p_param->value[0];

        switch (value)
        {
            case HIDS_PROTOCOL_MODE_BOOT:
                evt.evt_type = HIDS_EVT_BOOT_MODE_ENTERED;
                break;

            case HIDS_PROTOCOL_MODE_REPORT:
                evt.evt_type = HIDS_EVT_REPORT_MODE_ENTERED;
                break;

            default:
                return 0x80;
        }

        error_code = ble_gatts_value_set(p_param->handle, sizeof(uint8_t), 0, &value);

        if (BLE_SUCCESS == error_code && s_hids_env.hids_init.evt_handler)
        {
            s_hids_env.hids_init.evt_handler(&evt);
        }
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle writing report CCCD.
 *
 * @param[in] p_param: Pointer to writing parameters.
 * @param[in] idx:     Index of report CCCD in HIDS attribute table.
 *
 * @return BLE_ATT_ERR_NO_ERROR on success, otherwise error code.
 *****************************************************************************************
 */
static sdk_err_t hids_on_report_cccd_write(const gatts_write_req_cb_t *p_param,
                                             uint8_t idx)
{
    sdk_err_t error_code = BLE_SUCCESS;

    if (p_param->length != sizeof(uint16_t))
    {
        error_code = SDK_ERR_INVALID_ATT_VAL_LEN;
    }
    else
    {
        hids_evt_t evt;
        uint16_t   value = le16toh(p_param->value);

        switch (idx)
        {
            case HIDS_IDX_INPUT_REPORT_CCCD_1:
                s_hids_env.input_rep_cccds[0] = value;
                break;

#if INPUT_REPORT_COUNT > 1
            case HIDS_IDX_INPUT_REPORT_CCCD_2:
                s_hids_env.input_rep_cccds[1] = value;
                break;
#endif

            case HIDS_IDX_BOOT_KB_IN_RPT_CCCD:
                s_hids_env.boot_kb_input_rep_cccd = value;
                break;

            case HIDS_IDX_BOOT_MS_IN_RPT_CCCD:
                s_hids_env.boot_ms_input_rep_cccd = value;
                break;

            default:
                error_code = BLE_ATT_ERR_INVALID_HANDLE;
                break;
        }

        if (s_hids_env.hids_init.evt_handler)
        {
            evt.evt_type = prf_is_notification_enabled(value) ?
                           HIDS_EVT_NOTIFY_ENABLED : HIDS_EVT_NOTIFY_DISABLED;
            evt.params.notification.char_id.uuid = hids_attr_tab[idx - 1].uuid;
            s_hids_env.hids_init.evt_handler(&evt);
        }
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handle writing report value.
 *
 * @param[in] p_param: Pointer to writing parameters.
 * @param[in] idx:     Index of report value in HIDS attribute table.
 *
 * @return BLE_ATT_ERR_NO_ERROR on success, otherwise error code.
 *****************************************************************************************
 */
static sdk_err_t hids_on_report_value_write(const gatts_write_req_cb_t *p_param,
                                            uint8_t idx)
{
    sdk_err_t error_code;

    error_code = ble_gatts_value_set(p_param->handle, p_param->length,
                                                  p_param->offset, p_param->value);
    if (BLE_SUCCESS == error_code && s_hids_env.hids_init.evt_handler)
    {
        hids_evt_t evt;
        uint8_t    report_type;

        switch (idx) 
        {
            case HIDS_IDX_INPUT_REPORT_VAL_1:
#if INPUT_REPORT_COUNT > 1
            case HIDS_IDX_INPUT_REPORT_VAL_2:
#endif
                report_type = HIDS_REPORT_TYPE_INPUT;
                break;

            case HIDS_IDX_OUTPUT_REPORT_VAL:
                report_type = HIDS_REPORT_TYPE_OUTPUT;
                break;

            case HIDS_IDX_FEATURE_REPORT_VAL:
                report_type = HIDS_REPORT_TYPE_FEATURE;
                break;

            default:
                report_type = HIDS_REPORT_TYPE_RESERVED;
                break;
        }

        evt.evt_type = HIDS_EVT_REP_CHAR_WRITE;

        evt.params.char_write.char_id.uuid        = hids_attr_tab[idx].uuid;
        evt.params.char_write.char_id.report_type = report_type;

        evt.params.char_write.offset = p_param->offset;
        evt.params.char_write.length = p_param->length;
        evt.params.char_write.data   = p_param->value;

        s_hids_env.hids_init.evt_handler(&evt);
    }

    return error_code;
}

/**
 *****************************************************************************************
 * @brief Handles reception of the read request.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the read request.
 *****************************************************************************************
 */
static void hids_read_att_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_param)
{
    gatts_read_cfm_t cfm;
    uint8_t          idx = prf_find_idx_by_handle(p_param->handle,
                                                  s_hids_env.start_hdl, HIDS_IDX_NB,
                                                  (uint8_t *)&s_hids_env.hids_init.char_mask);

    cfm.handle = p_param->handle;
    cfm.status = BLE_SUCCESS;

    switch(idx)
    {
        case HIDS_IDX_INPUT_REPORT_CCCD_1:
            cfm.value  = (uint8_t *)&s_hids_env.input_rep_cccds[0];
            cfm.length = sizeof(s_hids_env.input_rep_cccds[0]);
            break;

#if INPUT_REPORT_COUNT > 1
        case HIDS_IDX_INPUT_REPORT_CCCD_2:
            cfm.value  = (uint8_t *)&s_hids_env.input_rep_cccds[1];
            cfm.length = sizeof(s_hids_env.input_rep_cccds[1]);
            break;
#endif

        case HIDS_IDX_BOOT_KB_IN_RPT_CCCD:
            cfm.value  = (uint8_t *)&s_hids_env.boot_kb_input_rep_cccd;
            cfm.length = sizeof(s_hids_env.boot_kb_input_rep_cccd);
            break;

        case HIDS_IDX_BOOT_MS_IN_RPT_CCCD:
            cfm.value  = (uint8_t *)&s_hids_env.boot_ms_input_rep_cccd;
            cfm.length = sizeof(s_hids_env.boot_ms_input_rep_cccd);
            break;

        default:
            cfm.value  = NULL;
            cfm.length = 0 ;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_read_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_param:  Pointer to the parameters of the write request.
 *****************************************************************************************
 */
static void hids_write_att_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_param)
{
    gatts_write_cfm_t cfm;
    uint8_t           idx = prf_find_idx_by_handle(p_param->handle,
                                                   s_hids_env.start_hdl, HIDS_IDX_NB,
                                                   (uint8_t *)&s_hids_env.hids_init.char_mask);

    cfm.handle = p_param->handle;

    switch (idx)
    {
        case HIDS_IDX_CTRL_POINT_VAL:
            cfm.status = hids_on_control_point_write(p_param);
            break;

        case HIDS_IDX_INPUT_REPORT_CCCD_1:
#if INPUT_REPORT_COUNT > 1
        case HIDS_IDX_INPUT_REPORT_CCCD_2:
#endif
        case HIDS_IDX_BOOT_KB_IN_RPT_CCCD:
        case HIDS_IDX_BOOT_MS_IN_RPT_CCCD:
            cfm.status = hids_on_report_cccd_write(p_param, idx);
            break;

        case HIDS_IDX_INPUT_REPORT_VAL_1:
#if INPUT_REPORT_COUNT > 1
        case HIDS_IDX_INPUT_REPORT_VAL_2:
#endif
        case HIDS_IDX_OUTPUT_REPORT_VAL:
        case HIDS_IDX_FEATURE_REPORT_VAL:
        case HIDS_IDX_BOOT_KB_OUT_RPT_VAL:
        case HIDS_IDX_BOOT_MS_IN_RPT_VAL:
            cfm.status = hids_on_report_value_write(p_param, idx);
            break;

        case HIDS_IDX_PROTOCOL_MODE_VAL:
            cfm.status = hids_on_protocol_mode_write(p_param);
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conn_idx, &cfm);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the cccd recover request.
 *
 * @param[in]: conn_idx:   Connection index
 * @param[in]: handle:     The handle of cccd attribute.
 * @param[in]: cccd_value: The value of cccd attribute.
 *****************************************************************************************
 */
static void hids_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    hids_evt_t  evt;

    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    uint8_t idx = prf_find_idx_by_handle(handle, 
                                         s_hids_env.start_hdl,
                                         HIDS_IDX_NB,
                                         (uint8_t *)&s_hids_env.hids_init.char_mask);

    switch (idx) 
    {
        case HIDS_IDX_INPUT_REPORT_CCCD_1:
            s_hids_env.input_rep_cccds[0] = cccd_value;
            break;

#if INPUT_REPORT_COUNT > 1
        case HIDS_IDX_INPUT_REPORT_CCCD_2:
            s_hids_env.input_rep_cccds[1] = cccd_value;
            break;
#endif

        case HIDS_IDX_BOOT_KB_IN_RPT_CCCD:
            s_hids_env.boot_kb_input_rep_cccd = cccd_value;
            break;

        case HIDS_IDX_BOOT_MS_IN_RPT_CCCD:
            s_hids_env.boot_ms_input_rep_cccd = cccd_value;
            break;

        default:
            return;
    }

    if (s_hids_env.hids_init.evt_handler)
    {
        evt.evt_type = prf_is_notification_enabled(cccd_value) ?
                       HIDS_EVT_NOTIFY_ENABLED : HIDS_EVT_NOTIFY_DISABLED;
        evt.params.notification.char_id.uuid = hids_attr_tab[idx - 1].uuid;
        s_hids_env.hids_init.evt_handler(&evt);
    }
}

static sdk_err_t hids_input_report_send(uint16_t length, uint8_t *p_data, uint8_t cccd_idx, uint8_t value_idx)
{
    sdk_err_t   error_code = SDK_ERR_DISCONNECTED;

    if (s_hids_env.conn_idx != GAP_INVALID_CONN_INDEX)
    {
        bool is_noti_enabled;
        
        switch (cccd_idx)
        {
            case HIDS_IDX_INPUT_REPORT_CCCD_1:
                is_noti_enabled = prf_is_notification_enabled(s_hids_env.input_rep_cccds[0]);
                break;

#if INPUT_REPORT_COUNT > 1
            case HIDS_IDX_INPUT_REPORT_CCCD_2:
                is_noti_enabled = prf_is_notification_enabled(s_hids_env.input_rep_cccds[1]);
                break;
#endif

            case HIDS_IDX_BOOT_KB_IN_RPT_CCCD:
                is_noti_enabled = prf_is_notification_enabled(s_hids_env.boot_kb_input_rep_cccd);
                break;

            case HIDS_IDX_BOOT_MS_IN_RPT_CCCD:
                is_noti_enabled = prf_is_notification_enabled(s_hids_env.boot_ms_input_rep_cccd);
                break;

            default:
                is_noti_enabled = false;
                break;
        }

        if (is_noti_enabled)
        {
            gatts_noti_ind_t hids_evt;

            hids_evt.type   = BLE_GATT_NOTIFICATION;
            hids_evt.handle = prf_find_handle_by_idx(value_idx,
                                                     s_hids_env.start_hdl,
                                                     (uint8_t*)&s_hids_env.hids_init.char_mask);

            if (PRF_INVALID_HANDLE != hids_evt.handle)
            {
                hids_evt.length = length;
                hids_evt.value  = p_data;

                error_code = ble_gatts_noti_ind(s_hids_env.conn_idx, &hids_evt);
            }
            else
            {
                error_code = BLE_ATT_ERR_INVALID_HANDLE;
            }
        }
        else
        {
            error_code = SDK_ERR_NTF_DISABLED;
        }
    }

    return error_code;
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
sdk_err_t hids_service_init(hids_init_t *p_hids_init)
{
    if (NULL == p_hids_init)
    {
        return SDK_ERR_POINTER_NULL;
    }

    memcpy(&s_hids_env.hids_init, p_hids_init, sizeof(hids_init_t));

    return ble_server_prf_add(&hids_prf_info);
}

sdk_err_t   hids_input_report_1_send(uint16_t length, uint8_t *p_data)
{
    sdk_err_t error_code = hids_input_report_send(length, p_data,
                                                  HIDS_IDX_INPUT_REPORT_CCCD_1,
                                                  HIDS_IDX_INPUT_REPORT_VAL_1);

    return error_code;
}

#if INPUT_REPORT_COUNT > 1
sdk_err_t   hids_input_report_2_send(uint16_t length, uint8_t *p_data)
{
    sdk_err_t error_code = hids_input_report_send(length, p_data,
                                                  HIDS_IDX_INPUT_REPORT_CCCD_2,
                                                  HIDS_IDX_INPUT_REPORT_VAL_2);

    return error_code;
}
#endif

sdk_err_t   hids_boot_kb_input_report_send(uint16_t length, uint8_t *p_data)
{
    sdk_err_t error_code = hids_input_report_send(length, p_data,
                                                  HIDS_IDX_BOOT_KB_IN_RPT_CCCD,
                                                  HIDS_IDX_BOOT_KB_IN_RPT_VAL);

    return error_code;
}

sdk_err_t   hids_boot_mouse_input_report_send(uint8_t   btns,
                                              int8_t    x_delta,
                                              int8_t    y_delta,
                                              uint16_t  opt_data_len,
                                              uint8_t  *p_opt_data)
{
    sdk_err_t error_code = SDK_ERR_INVALID_PARAM;
    uint16_t  data_len   = BOOT_MOUSE_IN_REPORT_MIN_SIZE + opt_data_len;

    if (data_len <= BOOT_MOUSE_IN_REPORT_MAX_SIZE)
    {
        uint8_t buf[BOOT_MOUSE_IN_REPORT_MAX_SIZE];

        // Encode mouse data
        buf[0] = btns;
        buf[1] = (uint8_t)x_delta;
        buf[2] = (uint8_t)y_delta;

        if (opt_data_len)
        {
            memcpy(&buf[3], p_opt_data, opt_data_len);
        }

        error_code = hids_input_report_send(data_len, buf,
                                            HIDS_IDX_BOOT_MS_IN_RPT_CCCD,
                                            HIDS_IDX_BOOT_MS_IN_RPT_VAL);
    }

    return error_code;
}

sdk_err_t   hids_output_report_get(uint16_t length, uint8_t *p_output_report)
{
    sdk_err_t error_code = BLE_ATT_ERR_INVALID_HANDLE;
    uint16_t  buf_len    = length;
    uint16_t  handle     = prf_find_handle_by_idx(HIDS_IDX_OUTPUT_REPORT_VAL,
                                                    s_hids_env.start_hdl,
                                                    (uint8_t*)&s_hids_env.hids_init.char_mask);

    if (handle != PRF_INVALID_HANDLE)
    {
        error_code = ble_gatts_value_get(handle, &buf_len, p_output_report);
    }

    return error_code;
}


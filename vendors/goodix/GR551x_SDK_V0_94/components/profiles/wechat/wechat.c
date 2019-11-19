/**
 *******************************************************************************
 *
 * @file wechat.c
 *
 * @brief wechat Profile implementation.
 *
 *******************************************************************************
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
 *******************************************************************************
 */

/*
* INCLUDE FILES
********************************************************************************
*/
#include "wechat.h"
#include "ble_prf_types.h"
#include "ble_prf_utils.h"
#include "utility.h"
#include "epb_MmBp.h"
#include "ble_wechat_util.h"
#include "app_timer.h"
#include "airsync.h"

/*
 * DEFINES
 *******************************************************************************
 */
#define WECHAT_PRIMARY_SERVICE                                      0xFEE7
#define BLE_UUID_WECHAT_WRITE_CHARACTERISTICS                       0xFEC7
#define BLE_UUID_WECHAT_INDICATE_CHARACTERISTICS                    0xFEC8
#define BLE_UUID_WECHAT_READ_CHARACTERISTICS                        0xFEC9
#define WECHAT_PEDOMETER_MEASUREMENT                                0xFEA1
#define WECHAT_TARGET                                               0xFEA2
#define WECHAT_LOCAL_MAC                                            0xFEC9

/*
 * ENUMERATIONS
 *******************************************************************************
 */
/**@brief Heart Rate Service Attributes Indexes. */
enum hrs_attr_idx_t
{
    WECHAT_IDX_SVC,
    WECHAT_IDX_PEDO_CHAR,
    WECHAT_IDX_PEDO_VAL,
    WECHAT_IDX_PEDO_CFG,
    WECHAT_IDX_TARGET_CHAR,
    WECHAT_IDX_TARGET_VAL,
    WECHAT_IDX_TARGET_CFG,
    WECHAT_IDX_MAC_CHAR,
    WECHAT_IDX_MAC_VAL,

    WECHAR_AIRSYNC_WRITE_CHAR,
    WECHAR_AIRSYNC_WRITE_VAL,
    WECHAR_AIRSYNC_INDICATE_CHAR,
    WECHAR_AIRSYNC_INDICATE_VAL,
    WECHAR_AIRSYNC_INDICATE_CFG,
    WECHAR_AIRSYNC_READ_CHAR,
    WECHAR_AIRSYNC_READ_VAL,

    WECHAT_IDX_NB,
};

/*
* LOCAL FUNCTION DECLARATION
********************************************************************************
*/
static sdk_err_t wechat_db_init(void);
static void      wechat_read_att_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_param);
static void      wechat_write_att_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_param);
static void      wechat_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value);
static void      wechat_app_gatts_cmpl_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind);
/*
 * LOCAL VARIABLE DEFINITIONS
 *******************************************************************************
 */

static data_info      s_send_data;
static wechat_env_t  *s_p_wechat_env;
static CURR_PEDO_t    s_pedo_struct;
static TARGET_t       s_target_struct;

static const attm_desc_t wechat_attr_tab[WECHAT_IDX_NB] =
{
    // Heart Rate Service Declaration
    [WECHAT_IDX_SVC]          = {WECHAT_PRIMARY_SERVICE, READ_PERM_UNSEC, 0, 0},

    [WECHAT_IDX_PEDO_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    [WECHAT_IDX_PEDO_VAL]     = {WECHAT_PEDOMETER_MEASUREMENT, READ_PERM_UNSEC | NOTIFY_PERM_UNSEC, ATT_VAL_LOC_USER, 10},
    [WECHAT_IDX_PEDO_CFG]     = {BLE_ATT_DESC_CLIENT_CHAR_CFG, READ_PERM_UNSEC | WRITE_REQ_PERM_UNSEC, 0, 0},

    [WECHAT_IDX_TARGET_CHAR]  = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    [WECHAT_IDX_TARGET_VAL]   = {WECHAT_TARGET, READ_PERM_UNSEC | INDICATE_PERM_UNSEC | WRITE_REQ_PERM_UNSEC, ATT_VAL_LOC_USER, 10},
    [WECHAT_IDX_TARGET_CFG]   = {BLE_ATT_DESC_CLIENT_CHAR_CFG, READ_PERM_UNSEC | WRITE_REQ_PERM_UNSEC, 0, 0},

    [WECHAT_IDX_MAC_CHAR]     = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    [WECHAT_IDX_MAC_VAL]      = {WECHAT_LOCAL_MAC, READ_PERM_UNSEC, ATT_VAL_LOC_USER, 20},

    [WECHAR_AIRSYNC_WRITE_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    [WECHAR_AIRSYNC_WRITE_VAL]     = {BLE_UUID_WECHAT_WRITE_CHARACTERISTICS, WRITE_REQ_PERM_UNSEC, ATT_VAL_LOC_USER, 32},

    [WECHAR_AIRSYNC_INDICATE_CHAR]  = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    [WECHAR_AIRSYNC_INDICATE_VAL]   = {BLE_UUID_WECHAT_INDICATE_CHARACTERISTICS, INDICATE_PERM_UNSEC, ATT_VAL_LOC_USER, 128},
    [WECHAR_AIRSYNC_INDICATE_CFG]   = {BLE_ATT_DESC_CLIENT_CHAR_CFG, READ_PERM_UNSEC | WRITE_REQ_PERM_UNSEC, 0, 0},

    [WECHAR_AIRSYNC_READ_CHAR]    = {BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0},
    [WECHAR_AIRSYNC_READ_VAL]     = {BLE_UUID_WECHAT_READ_CHARACTERISTICS, READ_PERM_UNSEC, ATT_VAL_LOC_USER, 6},


};


/**@brief wechat interface required by profile manager. */
static ble_prf_manager_cbs_t wechat_mgr_cbs =
{
    (prf_init_func_t)wechat_db_init,
    NULL,
    NULL
};

/**@brief wechat GATT server Callbacks. */
static gatts_prf_cbs_t wechat_gatts_cbs =
{
    wechat_read_att_cb,
    wechat_write_att_cb,
    NULL,
    wechat_app_gatts_cmpl_cb,
    wechat_cccd_set_cb
};

/**@brief wechat Information. */
static const prf_server_info_t wechat_prf_info =
{
    .max_connection_nb = WECHAT_CONNECTION_MAX,
    .manager_cbs       = &wechat_mgr_cbs,
    .gatts_prf_cbs     = &wechat_gatts_cbs
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
/**@brief Initialize wechat service, and create DB in ATT.
 *
 * @return status code to know if service initialization succeed or not.
 */

static const uint16_t s_char_mask = 0xFFFF;
static sdk_err_t wechat_db_init(void)
{
    const uint8_t wechat_svc_uuid[] = BLE_ATT_16_TO_16_ARRAY(WECHAT_PRIMARY_SERVICE);
    gatts_create_db_t gatts_db;
    uint16_t start_hdl = PRF_INVALID_HANDLE; /* The start hanlde is an in/out
                                              * parameter of ble_gatts_srvc_db_create().
                                              * It must be set with PRF_INVALID_HANDLE
                                              * to be allocated automatically by BLE Stack.*/

    memset(&gatts_db, 0, sizeof(gatts_db));

    gatts_db.shdl = &start_hdl;
    gatts_db.uuid = wechat_svc_uuid;
    gatts_db.attr_tab_cfg  = (uint8_t *)&s_char_mask;
    gatts_db.max_nb_attr   = WECHAT_IDX_NB;
    gatts_db.srvc_perm     = 0;
    gatts_db.attr_tab_type = SERVICE_TABLE_TYPE_16;
    gatts_db.attr_tab.attr_tab_16 = wechat_attr_tab;

    sdk_err_t   status = ble_gatts_srvc_db_create(&gatts_db);

    if (SDK_SUCCESS == status)
    {
        s_p_wechat_env->start_hdl = *gatts_db.shdl;
    }

    return status;
}


/**@brief Handles reception of the attribute info request message.
 *
 * @param[in] conidx  Connection index.
 * @param[in] p_param Pointer to the parameters of the read request.
 */
static void   wechat_read_att_cb(uint8_t conidx, const gatts_read_req_cb_t *p_param)
{
    uint8_t handle = p_param->handle;
    uint8_t tab_index = prf_find_idx_by_handle(handle, s_p_wechat_env->start_hdl,
                        WECHAT_IDX_NB,
                        (uint8_t *)&s_char_mask);

    gatts_read_cfm_t cfm;
    cfm.handle = handle;
    cfm.status = BLE_SUCCESS;

    switch (tab_index)
    {
        case WECHAT_IDX_PEDO_VAL:
            cfm.length = sizeof (s_target_struct);
            cfm.value  = (uint8_t *)&s_pedo_struct;
            s_pedo_struct.flag = 0x1;
            s_pedo_struct.step_count[0] = 0x6A;
            s_pedo_struct.step_count[1] = 0x04;
            s_pedo_struct.step_count[2] = 0x01;
            break;

        case WECHAT_IDX_TARGET_VAL:
            cfm.length = sizeof (s_target_struct);
            cfm.value  = (uint8_t *)&s_target_struct;
            s_target_struct.flag = 0x1;
            s_target_struct.step_count[0] = 0x4a;
            s_target_struct.step_count[1] = 0x48;
            s_target_struct.step_count[2] = 0x00;
            break;

        case WECHAT_IDX_MAC_VAL:
            cfm.length = 6;
            cfm.value  = (uint8_t *)s_p_wechat_env->device_mac;
            break;

        case WECHAR_AIRSYNC_READ_VAL:
            break;

        default:
            cfm.length = 0;
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_read_cfm(conidx, &cfm);
}


/**@brief Return data up once a second
 *
 */
//static void pedo_timeout_handler(uint8_t timer_id)
//{
//    gatts_noti_ind_t hr_noti;
//    hr_noti.type   = BLE_GATT_NOTIFICATION;
//    hr_noti.handle = prf_find_handle_by_idx(WECHAT_IDX_PEDO_VAL,
//                                            s_p_wechat_env->start_hdl,
//                                            (uint8_t *)&s_char_mask);
//    hr_noti.length = 4;
//    hr_noti.value  = (uint8_t *)&s_pedo_struct;

//    s_pedo_struct.flag = 0x1;
//    s_pedo_struct.step_count[0] = 0x00;
//    s_pedo_struct.step_count[1] = 0x10;
//    s_pedo_struct.step_count[2] = 0x00;
//    ble_gatts_noti_ind(0, &hr_noti);
//}

/**@brief Synchronize current target values.
 *
 */
//static void target_timeout_handler(uint8_t timer_id)
//{
//    gatts_noti_ind_t hr_noti;
//    hr_noti.type   = BLE_GATT_INDICATION;
//    hr_noti.handle = prf_find_handle_by_idx(WECHAT_IDX_TARGET_VAL,
//                                            s_p_wechat_env->start_hdl,
//                                            (uint8_t *)&s_char_mask);
//    hr_noti.length = 4;
//    hr_noti.value  = (uint8_t *)&s_target_struct;

//    s_target_struct.flag = 0x1;
//    s_target_struct.step_count[0] = 0x4a;
//    s_target_struct.step_count[1] = 0x48;
//    s_target_struct.step_count[2] = 0x00;
//    ble_gatts_noti_ind(0, &hr_noti);
//}


/**
 *******************************************************************************
 * @brief Handles reception of the write request.
 *
 * @param[in] conidx  Connection index.
 * @param[in] p_param Pointer to the parameters of the write request.
 *******************************************************************************
 */
static void   wechat_write_att_cb(uint8_t conidx, const gatts_write_req_cb_t *p_param)
{
    uint16_t handle = p_param->handle;
    uint8_t tab_index = prf_find_idx_by_handle(handle, s_p_wechat_env->start_hdl,
                        WECHAT_IDX_NB,
                        (uint8_t *)&s_char_mask);
    uint16_t   cccd_value;
    gatts_write_cfm_t cfm;
    cfm.handle = handle;

    switch (tab_index)
    {
        /* for pedometer data notify configuration */
        case WECHAT_IDX_PEDO_CFG:
            cccd_value = le16toh(&p_param->value[0]);

            s_p_wechat_env->pedo_ntf_cfg = cccd_value;
            cfm.status = BLE_SUCCESS;
            break;

        /* for pedometer target notify configuration */
        case WECHAT_IDX_TARGET_CFG:
            cccd_value = le16toh(&p_param->value[0]);

            s_p_wechat_env->target_ntf_cfg = cccd_value;
            cfm.status = BLE_SUCCESS;
            break;

        /* for pedometer target value */
        case WECHAT_IDX_TARGET_VAL:
            cfm.status = BLE_SUCCESS;
            break;

        /* for AIRSYNC indicate configuration */
        case WECHAR_AIRSYNC_INDICATE_CFG:
            cfm.status = BLE_SUCCESS;
            set_next_step(WECHAT_AIRSYNC_REQ_AUTH);
            break;

        /* for AIRSYNC write value */
        case WECHAR_AIRSYNC_WRITE_VAL:
            cfm.status = BLE_SUCCESS;
            ble_wechat_process_received_data(p_param->value, p_param->length);
            break;

        default:
            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            break;
    }

    ble_gatts_write_cfm(conidx, &cfm);
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
static void wechat_cccd_set_cb(uint8_t conn_idx, uint16_t handle, uint16_t cccd_value)
{
    if (!prf_is_cccd_value_valid(cccd_value))
    {
        return;
    }

    uint8_t tab_index = prf_find_idx_by_handle(handle, s_p_wechat_env->start_hdl,
                        WECHAT_IDX_NB,
                        (uint8_t *)&s_char_mask);

    switch (tab_index)
    {
        /* for pedometer data notify configuration */
        case WECHAT_IDX_PEDO_CFG:
            s_p_wechat_env->pedo_ntf_cfg = cccd_value;
            break;

        /* for pedometer target notify configuration */
        case WECHAT_IDX_TARGET_CFG:
            s_p_wechat_env->target_ntf_cfg = cccd_value;
            break;

        default:
            break;
    }
}

static int ble_wechat_indicate_data_chunk(void)
{
    uint16_t chunk_len = 0;
    chunk_len = s_send_data.len - s_send_data.offset;
    chunk_len = chunk_len > BLE_WECHAT_MAX_DATA_LEN ? BLE_WECHAT_MAX_DATA_LEN : chunk_len;

    if (chunk_len == 0)
    {
        s_send_data.data = NULL;
        s_send_data.len = 0;
        s_send_data.offset = 0;
        return 0;
    }

    gatts_noti_ind_t wec_noti;
    wec_noti.type   = BLE_GATT_INDICATION;
    wec_noti.handle = prf_find_handle_by_idx(WECHAR_AIRSYNC_INDICATE_VAL,
                                             s_p_wechat_env->start_hdl,
                                             (uint8_t *)&s_char_mask);
    wec_noti.length = chunk_len;
    wec_noti.value  = s_send_data.data + s_send_data.offset;
    ble_gatts_noti_ind(0, &wec_noti);

    s_send_data.offset += chunk_len;
    return 1;
}

/**
 *******************************************************************************
 * @brief The interface sends several subpackages that are split up to the
 *        client in succession.
 *
 * @param[in] p_data : Pointer to the parameters of the write request.
 * @param[in] length : data length
 *
 * @return If the request was consumed or not.
 *******************************************************************************
 */
int ble_wechat_indicate_data(uint8_t *p_data, uint32_t length)
{
    if (p_data == NULL || length == 0)
    {
        return 0;
    }

    s_send_data.data = p_data;
    s_send_data.len = length;
    s_send_data.offset = 0;
    return (ble_wechat_indicate_data_chunk());
}

static void wechat_app_gatts_cmpl_cb(uint8_t conidx, uint8_t status,
                                    const ble_gatts_ntf_ind_t *p_ntf_ind)
{
    if (p_ntf_ind->type == BLE_GATT_INDICATION)
    {
        ble_wechat_indicate_data_chunk();
    }
}

sdk_err_t wechat_service_add(wechat_env_t *p_wecchat_env)
{
    if (NULL == p_wecchat_env)
    {
        return SDK_ERR_POINTER_NULL;
    }

    s_p_wechat_env = p_wecchat_env;

    return ble_server_prf_add(&wechat_prf_info);
}


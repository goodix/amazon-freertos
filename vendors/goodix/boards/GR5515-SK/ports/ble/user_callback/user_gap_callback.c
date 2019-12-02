/**
 *****************************************************************************************
 *
 * @file user_gap_callback.c
 *
 * @brief  BLE GAP Callback Function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr55xx_sys.h"
#include "user_app.h"
#include "app_log.h"
#include "FreeRTOS.h"
#include "gr_config.h"
#include "gr_porting.h"
#include "gr_message.h"

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_gap_param_set_cb(uint8_t status, const gap_param_set_op_id_t set_param_op);
static void app_gap_psm_manager_cb(uint8_t status, const gap_psm_manager_op_id_t psm_op);
static void app_gap_phy_update_cb(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy_ind);
static void app_gap_dev_info_get_cb(uint8_t status, const gap_dev_info_get_t *p_dev_info);
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t adv_stop_reason);
static void app_gap_scan_req_ind_cb(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr);
static void app_gap_adv_data_update_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_scan_start_cb(uint8_t status);
static void app_gap_scan_stop_cb(uint8_t status, gap_stopped_reason_t scan_stop_reason);
static void app_gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report);
static void app_gap_sync_establish_cb(uint8_t inst_idx, uint8_t status, const gap_sync_established_ind_t *p_sync_established_info);
static void app_gap_stop_sync_cb(uint8_t inst_idx, uint8_t status);
static void app_gap_sync_lost_cb(uint8_t inst_idx);
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param);
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason);
static void app_gap_connect_cancel_cb(uint8_t status);
static void app_gap_auto_connection_timeout_cb(void);
static void app_gap_peer_name_ind_cb(uint8_t conn_idx, const gap_peer_name_ind_t  *p_peer_name);
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info);
static void app_gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req);
static void app_gap_connection_info_get_cb(uint8_t conn_idx, uint8_t status, const gap_conn_info_param_t *p_conn_info);
static void app_gap_peer_info_get_cb(uint8_t conn_idx,  uint8_t status, const gap_peer_info_param_t *p_peer_dev_info);
static void app_gap_le_pkt_size_info_cb(uint8_t conn_idx,  uint8_t status, const gap_le_pkt_size_ind_t *p_supported_data_length_size);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const gap_cb_fun_t app_gap_callbacks =
{
    // -------------------------  Common Callbacks       ---------------------------------
    .app_gap_param_set_cb               = app_gap_param_set_cb,
    .app_gap_psm_manager_cb             = app_gap_psm_manager_cb,
    .app_gap_phy_update_cb              = app_gap_phy_update_cb,
    .app_gap_dev_info_get_cb            = app_gap_dev_info_get_cb,

    // -------------------------  Advertising Callbacks       ----------------------------
    .app_gap_adv_start_cb               = app_gap_adv_start_cb,
    .app_gap_adv_stop_cb                = app_gap_adv_stop_cb,
    .app_gap_scan_req_ind_cb            = app_gap_scan_req_ind_cb,
    .app_gap_adv_data_update_cb         = app_gap_adv_data_update_cb,

    // --------------------  Scanning/Periodic Synchronization Callbacks  ----------------
    .app_gap_scan_start_cb              = app_gap_scan_start_cb,
    .app_gap_scan_stop_cb               = app_gap_scan_stop_cb,
    .app_gap_adv_report_ind_cb          = app_gap_adv_report_ind_cb,
    .app_gap_sync_establish_cb          = app_gap_sync_establish_cb,
    .app_gap_stop_sync_cb               = app_gap_stop_sync_cb,
    .app_gap_sync_lost_cb               = app_gap_sync_lost_cb,   

    // -------------------------   Initiating Callbacks   --------------------------------
    .app_gap_connect_cb                 = app_gap_connect_cb,
    .app_gap_disconnect_cb              = app_gap_disconnect_cb,
    .app_gap_connect_cancel_cb          = app_gap_connect_cancel_cb,
    .app_gap_auto_connection_timeout_cb = app_gap_auto_connection_timeout_cb,
    .app_gap_peer_name_ind_cb           = app_gap_peer_name_ind_cb,

    // -------------------------   Connection Control Callbacks  -------------------------
    .app_gap_connection_update_cb       = app_gap_connection_update_cb,
    .app_gap_connection_update_req_cb   = app_gap_connection_update_req_cb,
    .app_gap_connection_info_get_cb     = app_gap_connection_info_get_cb,
    .app_gap_peer_info_get_cb           = app_gap_peer_info_get_cb,
    .app_gap_le_pkt_size_info_cb        = app_gap_le_pkt_size_info_cb,
};


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief This callback function will be called when the set param(s) operation has completed.
 *
 * @param[in] status:       The status of set param operation.
 * @param[in] set_param_op: The operation of setting. @see gap_param_set_op_id_t.
 ****************************************************************************************
 */
static void app_gap_param_set_cb(uint8_t status, const gap_param_set_op_id_t set_param_op)
{
    GRC_LOG(DEBUG, (">>> app_gap_param_set_cb called. status:%d ", status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the psm register/unregister operation has completed.
 *
 * @param[in] status:       The status of psm manager operations.
 * @param[in] set_param_op: The operation of register/unregister psm. @see gap_psm_op_id_t
 ****************************************************************************************
 */
static void app_gap_psm_manager_cb(uint8_t status, const gap_psm_manager_op_id_t psm_op)
{
    GRC_LOG(DEBUG, (">>> app_gap_psm_manager_cb called . status: %d ", status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when update phy completed.
 *
 * @param[in] conn_idx:  The index of connections.
 * @param[in] status:    The status of udpate phy operation.
 * @param[in] p_phy_ind: The phy info.
 ****************************************************************************************
 */
static void app_gap_phy_update_cb(uint8_t conn_idx, uint8_t status, const gap_le_phy_ind_t *p_phy_ind)
{
    GRC_LOG(DEBUG, (">>> app_gap_phy_update_cb called . status: %d ", status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the requested parameters has been got.
 *
 * @param[in] status:     GAP operation status.
 * @param[in] p_dev_info: The device info. See @ref gap_dev_info_get_t
 ****************************************************************************************
 */
static void app_gap_dev_info_get_cb(uint8_t status, const gap_dev_info_get_t *p_dev_info)
{
    GRC_LOG(DEBUG, (">>> app_gap_dev_info_get_cb called . status: %d ", status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the adv has started.
 *
 * @param[in] inst_idx:  The advertising index. valid range is: 0 - 4.
 * @param[in] status:    The status of starting a advertiser.
 ****************************************************************************************
 */
static void app_gap_adv_start_cb(uint8_t inst_idx, uint8_t status)
{
    static GR_CB_MSG_BASIC_T     s_adv_msg; 
    GR_CALLBACK_MSG_T            * msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    
    GRC_LOG(DEBUG, ("Adverting started idx: %d, status: (0X%02X). ", inst_idx, status));        
    
    if (BLE_SUCCESS == status){
        s_gr_ble_gap_params_ins.is_adv_started = true;
    }
    
    if(msg == NULL){
        return;
    }
    
    s_adv_msg.gr_index      = inst_idx;
    s_adv_msg.gr_status     = status;
    msg->msg_type           = GR_CB_MSG_ADV_START;
    msg->msg                = (void*) &s_adv_msg;
    
    gr_ble_cb_msg_send(msg, true);    
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the adv has stopped.
 *
 * @param[in] inst_idx: The advertising index. valid range is: 0 - 4.
 * @param[in] status:   The status of stopping a advertiser. If status is not success, adv_stop_reason is invalid.
 * @param[in] reason:   The stop reason. See @ref gap_stopped_reason_t.
 ****************************************************************************************
 */
static void app_gap_adv_stop_cb(uint8_t inst_idx, uint8_t status, gap_stopped_reason_t reason)
{
    if (BLE_SUCCESS == status)
    {
        s_gr_ble_gap_params_ins.is_adv_started = false;
        GRC_LOG(DEBUG, ("Advertising Stopped, reason: %d  ", reason));
    } else {
        GRC_LOG(DEBUG, ("Advertising Stop failed... "));
    }
    //no upper callback to notify
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when app has received the scan request.
 *
 * @param[in] inst_idx:       The advertising index. valid range is: 0 - 4.
 * @param[in] p_scanner_addr: The BD address. See @ref gap_bdaddr_t.
 ****************************************************************************************
 */
static void app_gap_scan_req_ind_cb(uint8_t inst_idx, const gap_bdaddr_t *p_scanner_addr)
{
    GRC_LOG(DEBUG, (">>> app_gap_scan_req_ind_cb called, Received the scan request from the peer %02X:%02X:%02X:%02X:%02X:%02X  ",
                   p_scanner_addr->gap_addr.addr[5],
                   p_scanner_addr->gap_addr.addr[4],
                   p_scanner_addr->gap_addr.addr[3],
                   p_scanner_addr->gap_addr.addr[2],
                   p_scanner_addr->gap_addr.addr[1],
                   p_scanner_addr->gap_addr.addr[0]));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when update adv data completed.
 *
 * @param[in] inst_idx:The advertising index. valid range is: 0 - 4.
 * @param[in] status:  The status of udpate phy operation.
 ****************************************************************************************
 */
static void app_gap_adv_data_update_cb(uint8_t inst_idx, uint8_t status)
{
    GRC_LOG(DEBUG, (">>> app_gap_adv_data_update_cb called . status: %d ", status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the scan has started.
 *
 * @param[in] status:  The status of starting a scanner.
 ****************************************************************************************
 */
static void app_gap_scan_start_cb(uint8_t status)
{
    GRC_LOG(DEBUG, (">>> app_gap_scan_start_cb called . status: %d ", status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the scanning activity has been stopped.
 *
 * @param[in] status: The status of stopping a scanner.
 * @param[in] reason: The stop reason. See @ref gap_stopped_reason_t.
 ****************************************************************************************
 */
static void app_gap_scan_stop_cb(uint8_t status, gap_stopped_reason_t reason)
{
    GRC_LOG(DEBUG, (">>> app_gap_scan_stop_cb called . status: %d, reason:%d  ", status, reason));
    
    if (GAP_STOPPED_REASON_TIMEOUT == reason)
    {
        GRC_LOG(DEBUG, ("Scan Timeout."));
    }
    else
    {
        //after stop the scanner, check the scan result and start the connect
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the advertising report has been received.
 *
 * @param[in] p_adv_report: The extended advertising report. See @ref gap_ext_adv_report_ind_t.
 ****************************************************************************************
 */
static void app_gap_adv_report_ind_cb(const gap_ext_adv_report_ind_t  *p_adv_report)
{
    GRC_LOG(DEBUG, (">>> app_gap_adv_report_ind_cb called  "));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the periodic advertising synchronization has been established.
 *
 * @param[in] status:                  The status of sync.
 * @param[in] p_sync_established_info: The established ind info.  See @ref gap_sync_established_ind_t.
 ****************************************************************************************
 */
static void app_gap_sync_establish_cb(uint8_t inst_idx, uint8_t status, const gap_sync_established_ind_t *p_sync_established_info)
{
    GRC_LOG(DEBUG, ("app_gap_sync_established_cb: ")); 
    GRC_LOG(DEBUG, ("-- phy: %d ", p_sync_established_info->phy));
    GRC_LOG(DEBUG, ("-- intv: %d ", p_sync_established_info->intv));
    GRC_LOG(DEBUG, ("-- adv_sid: %d ", p_sync_established_info->adv_sid));
    GRC_LOG(DEBUG, ("-- clk_acc: %d ", p_sync_established_info->clk_acc));
    GRC_LOG(DEBUG, ("-- addr_type: %d ", p_sync_established_info->bd_addr.addr_type));
    GRC_LOG(DEBUG, ("-- addr: %02x-%02x-%02x-%02x-%02x-%02x ", 
                    p_sync_established_info->bd_addr.gap_addr.addr[0],
                    p_sync_established_info->bd_addr.gap_addr.addr[1],
                    p_sync_established_info->bd_addr.gap_addr.addr[2],
                    p_sync_established_info->bd_addr.gap_addr.addr[3],
                    p_sync_established_info->bd_addr.gap_addr.addr[4],
                    p_sync_established_info->bd_addr.gap_addr.addr[5]));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when sync has stopped.
 *
 * @param[in] status: The status of stopping sync.
 ****************************************************************************************
 */
static void app_gap_stop_sync_cb(uint8_t inst_idx, uint8_t status)
{
    GRC_LOG(DEBUG, (">>> app_gap_stop_sync_cb called, status:%d  ", status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called once the periodic advertising synchronization has been lost.
 ****************************************************************************************
 */
static void app_gap_sync_lost_cb(uint8_t inst_idx)
{
    GRC_LOG(DEBUG, (">>> app_gap_sync_lost_cb called  "));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when connection completed.
 *
 * @param[in] conn_idx:     The connection index.
 * @param[in] status:       The status of operation. If status is not success, conn_idx and p_conn_param are invalid.
 * @param[in] p_conn_param: The connection param.  See @ref gap_conn_cmp_t.
 ****************************************************************************************
 */
static void app_gap_connect_cb(uint8_t conn_idx, uint8_t status, const gap_conn_cmp_t *p_conn_param)
{
    static GR_CB_MSG_BASIC_T     s_conn_msg; 
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    if (BLE_SUCCESS == status)
    {
        GRC_LOG(DEBUG, ("app_gap_connect_cb Connected with the peer %02X:%02X:%02X:%02X:%02X:%02X.  ",
                     p_conn_param->peer_addr.addr[5],
                     p_conn_param->peer_addr.addr[4],
                     p_conn_param->peer_addr.addr[3],
                     p_conn_param->peer_addr.addr[2],
                     p_conn_param->peer_addr.addr[1],
                     p_conn_param->peer_addr.addr[0]));
        
        s_gr_ble_gap_params_ins.is_connected = true;
        s_gr_ble_gap_params_ins.is_mtu_exchanged = false;
        s_gr_ble_gap_params_ins.cur_connect_id = conn_idx;
        
        memcpy(&s_gr_ble_gap_params_ins.gap_conn_cmp_param, p_conn_param, sizeof(gap_conn_cmp_t));
    
        msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
        if(msg == NULL){
            return;
        }
        
        s_conn_msg.gr_index      = conn_idx;
        s_conn_msg.gr_status     = status;
        msg->msg_type            = GR_CB_MSG_GAP_CONNECT;
        msg->msg                 = (void*) &s_conn_msg;
        
        gr_ble_cb_msg_send(msg, true);
    } else {
        GRC_LOG(DEBUG, ("app_gap_connect_cb connect fail: %d  ", status));
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when disconnect completed.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] status:   The status of operation. If status is not success, disconnect reason is invalid.
 * @param[in] reason:   The reason of disconnect. See @ref BLE_STACK_ERROR_CODES.
 ****************************************************************************************
 */
static void app_gap_disconnect_cb(uint8_t conn_idx, uint8_t status, uint8_t reason)
{
    static GR_CB_MSG_BASIC_T     s_disconn_msg; 
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    if (BLE_SUCCESS == status)
    {
        GRC_LOG(DEBUG, ("Disconnected, reason:%d  ", reason));
        
        s_gr_ble_gap_params_ins.is_connected = false;
        
        msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
        if(msg == NULL){
            return;
        }
        
        s_disconn_msg.gr_index   = conn_idx;
        s_disconn_msg.gr_status  = status;
        s_disconn_msg.gr_reason  = reason;         
        msg->msg_type            = GR_CB_MSG_GAP_DISCONNECT;
        msg->msg                 = (void*) &s_disconn_msg;
        
        gr_ble_cb_msg_send(msg, true);
    } else {
        GRC_LOG(DEBUG, ("app_gap_disconnect_cb connect fail: %d  ", status));
    }
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when connection canceled.
 *
 * @param[in] status: The status of cancel operation.
 ****************************************************************************************
*/
static void app_gap_connect_cancel_cb(uint8_t status)
{
    GRC_LOG(DEBUG, (">>> app_gap_connect_cancel_cb called, status:%d  ", status));
    s_gr_ble_gap_params_ins.is_connected = false;
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when an automatic connection timeout occurs.
 ****************************************************************************************
 */
static void app_gap_auto_connection_timeout_cb(void)
{
    GRC_LOG(DEBUG, (">>> app_gap_auto_connection_timeout_cb called "));
    s_gr_ble_gap_params_ins.is_connected = false;
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the peer name info has been got.
 *
 * @param[in] conn_idx:    The connection index.
 * @param[in] p_peer_name: The peer device name indication info. See @ref gap_peer_name_ind_t.
 ****************************************************************************************
 */
static void app_gap_peer_name_ind_cb(uint8_t conn_idx, const gap_peer_name_ind_t  *p_peer_name)
{
    GRC_LOG(DEBUG, (">>> app_gap_peer_name_ind_cb called "));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when connection update completed.
 *
 * @param[in] conn_idx:                 The connection index.
 * @param[in] status:                   The status of GAP operation.
 * @param[in] p_conn_param_update_info: The connection update complete param. See @ref gap_conn_update_cmp_t.
 ****************************************************************************************
 */
static void app_gap_connection_update_cb(uint8_t conn_idx, uint8_t status, const gap_conn_update_cmp_t *p_conn_param_update_info)
{
    static GR_CB_MSG_CONN_UPDATE_T conn_update;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, (">>> app_gap_connection_update_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
    
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }
    
    memcpy(&conn_update.update, p_conn_param_update_info, sizeof(gap_conn_update_cmp_t));
    conn_update.msg_basic.gr_index   = conn_idx;
    conn_update.msg_basic.gr_status  = status;    
    msg->msg_type            = GR_CB_MSG_GAP_CONNECT_UPDATE;
    msg->msg                 = (void*) &conn_update;
    
    gr_ble_cb_msg_send(msg, true);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when the peer device requests updating connection.
 *
 * @param[in] conn_idx:                The connection index.
 * @param[in] p_conn_param_update_req: The connection update request param. See @ref gap_conn_param_t.
 ****************************************************************************************
 */
static void app_gap_connection_update_req_cb(uint8_t conn_idx, const gap_conn_param_t *p_conn_param_update_req)
{
    GRC_LOG(DEBUG, (">>> app_gap_connection_update_req_cb called  "));
    ble_gap_conn_param_update_reply(conn_idx, true);
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when app has got the connection info.
 *
 * @param[in] conn_idx:                The connection index.
 * @param[in] status:                  The status of GAP operation.
 * @param[in] p_conn_param_update_req: The connection info. See @ref  gap_conn_info_param_t.
 ****************************************************************************************
 */
static void app_gap_connection_info_get_cb(uint8_t conn_idx, uint8_t status, const gap_conn_info_param_t *p_conn_info)
{
    GRC_LOG(DEBUG, (">>> app_gap_connection_info_get_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when app has got the peer info.
 *
 * @param[in]  conn_idx:        The connection index.
 * @param[in]  status:          The status of GAP operation.
 * @param[in]  p_peer_dev_info: The peer device info. See @ref gap_peer_info_param_t.
 ****************************************************************************************
 */
static void app_gap_peer_info_get_cb(uint8_t conn_idx,  uint8_t status, const gap_peer_info_param_t *p_peer_dev_info)
{
    GRC_LOG(DEBUG, (">>> app_gap_peer_info_get_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 ****************************************************************************************
 * @brief This callback function will be called when an app sets the length size of the supported data.
 *
 * @param[in]  conn_idx:                     The connection index.
 * @param[in]  status:                       The status of GAP operation.
 * @param[in]  p_supported_data_length_size: Supported data length size. See @ref gap_le_pkt_size_ind_t.
 ****************************************************************************************
 */
static void app_gap_le_pkt_size_info_cb(uint8_t conn_idx,  uint8_t status, const gap_le_pkt_size_ind_t *p_supported_data_length)
{
    GRC_LOG(DEBUG, (">>> app_gap_le_pkt_size_info_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

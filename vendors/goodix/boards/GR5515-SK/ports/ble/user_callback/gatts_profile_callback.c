/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr55xx_sys.h"
#include "user_app.h"
#include "iot_ble_gap_config.h"
#include "iot_ble_hal_internals.h"
#include "FreeRTOSConfig.h"
#include "bt_hal_manager_types.h"
#include "ble_prf_utils.h"
#include "gr_config.h"
#include "gr_utils.h"
#include "gr_message.h"
#include "gr_porting.h"




gr_srv_env_t s_gattsp_instance = {
    .cur_start_srv_index    = 0,
    .register_srv_num       = 0,
    .register_srv_handle    = {0xffff},
    .start_handle           = 0,
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static ble_err_t gatt_prf_db_init(void);
static void      srv_gatts_read_cb(uint8_t conidx, const gatts_read_req_cb_t *p_read_req);
static void      srv_gatts_write_cb(uint8_t conidx, const gatts_write_req_cb_t *p_write_req);
static void      srv_gatts_prep_write_cb(uint8_t conidx, const gatts_prep_write_req_cb_t *p_prep_req);
static void      srv_gatts_ntf_ind_cb(uint8_t conidx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind);


static ble_prf_manager_cbs_t srv_mgr_cbs =
{
    gatt_prf_db_init,
    NULL,
    NULL
};

static gatts_prf_cbs_t srv_gatts_cbs =
{
    srv_gatts_read_cb,
    srv_gatts_write_cb,
    srv_gatts_prep_write_cb,
    srv_gatts_ntf_ind_cb,
};

static const prf_server_info_t srv_prf_info =
{
    .max_connection_nb = GR_BLE_SRV_CONNECT_MAX,
    .manager_cbs       = &srv_mgr_cbs,
    .gatts_prf_cbs     = &srv_gatts_cbs
};

static bool     is_first_srv = true;

static uint8_t  s_attr_mask[GR_BLE_ATTR_MASK_LEN];

static ble_err_t gatt_prf_db_init(void)
{
    uint16_t           start_hdl  = PRF_INVALID_HANDLE;
    sdk_err_t          error_code = SDK_SUCCESS;
    gatts_create_db_t  gatts_db;
    BTGattServiceList_t * psrv;

    memset(&gatts_db, 0, sizeof(gatts_db));
    
    if(s_gattsp_instance.cur_start_srv_index >= s_gattsp_instance.register_srv_num){
        GRC_LOG(ERROR, ("gatt_prf_db_init, index exception... "));
        return BLE_ATT_ERR_INVALID_HANDLE;
    }
    
    uint16_t new_idx = s_gattsp_instance.cur_start_srv_index % GR_BLE_MAX_SERVICES;
    uint16_t this_handle = s_gattsp_instance.register_srv_handle[new_idx];    
    
    psrv = prvBTGattServiceListGet(this_handle);
    s_gattsp_instance.cur_start_srv_index ++;
    
    if(psrv == NULL){
        GRC_LOG(DEBUG, ("gatt_prf_db_init, invalid service handle... "));
        return SDK_ERR_INVALID_HANDLE;
    }
    for(int i = 0; i < GR_BLE_ATTR_MASK_LEN; i++){
        s_attr_mask[i] = 0xFF;
    }
    
    gatts_db.shdl                  = &start_hdl;
    gatts_db.attr_tab_cfg          = &s_attr_mask[0];
    gatts_db.max_nb_attr           = psrv->mGattNum;
    
    if (psrv->mUuidType == eBTuuidType128)
    {
        gatts_db.uuid                  = ((attm_desc_128_t *)psrv->pAttTable)->uuid;
        gatts_db.srvc_perm             = SRVC_UUID_TYPE_SET(UUID_TYPE_128);
        gatts_db.attr_tab_type         = SERVICE_TABLE_TYPE_128;
        gatts_db.attr_tab.attr_tab_128 = (attm_desc_128_t *)psrv->pAttTable;
    }
    else
    {
        uint8_t svc_uuid[2]            = BLE_ATT_16_TO_16_ARRAY(((attm_desc_t *)psrv->pAttTable)->uuid);
        
        gatts_db.uuid                  = &svc_uuid[0];
        gatts_db.srvc_perm             = 0;
        gatts_db.attr_tab_type         = SERVICE_TABLE_TYPE_16;
        gatts_db.attr_tab.attr_tab_16  = (attm_desc_t *)psrv->pAttTable;
    }

    error_code = ble_gatts_srvc_db_create(&gatts_db);

    if (SDK_SUCCESS == error_code && is_first_srv)
    {
        s_gattsp_instance.start_handle = *gatts_db.shdl;
        is_first_srv = false;
    }
    
    GRC_LOG(DEBUG, ("gatt_prf_db_init, code: %d, start handle: %d... ", error_code, s_gattsp_instance.start_handle));

    return error_code;
}

void gr_gatt_service_reset(void){
    is_first_srv = true;
    
    s_gattsp_instance.cur_start_srv_index    = 0;
    s_gattsp_instance.register_srv_num       = 0;    
    s_gattsp_instance.start_handle           = 0;
    
    for(int i=0; i< GR_BLE_MAX_SERVICES;i++) {
        s_gattsp_instance.register_srv_handle[i] = GR_BLE_GATT_INVALID_HANDLE;
    }
}

BTStatus_t gr_gatt_service_register_all(void) {
    sdk_err_t  error_code;
    BTGattServiceList_t * srvhead =  prvBTGattServiceListGetHead();
    BTGattServiceList_t srvlist;
        
    s_gattsp_instance.cur_start_srv_index = 0;
        
    for(int i = 0;i < GR_BLE_MAX_SERVICES; i++){
        srvlist = *((BTGattServiceList_t *)srvhead + i);
        if(srvlist.isUsed) {
            s_gattsp_instance.register_srv_handle[s_gattsp_instance.register_srv_num%GR_BLE_MAX_SERVICES] = srvlist.mServiceHandle;
            s_gattsp_instance.register_srv_num++;
            
            //every service calls gatt_prf_db_init each time
            error_code = ble_server_prf_add(&srv_prf_info);  
            
            if (error_code != SDK_SUCCESS){
                GRC_LOG(DEBUG, ("ble_server_prf_add fail: %d  ", error_code));
                return gr_util_to_afr_status_code(error_code);
            }
        }
    }
    gr_util_gatt_handle_map_print();
    
    return gr_util_to_afr_status_code(SDK_SUCCESS);
}

BTStatus_t gr_gatt_service_register(uint16_t serviceHandle) {
    BTStatus_t rStatus = eBTStatusSuccess;
    sdk_err_t  error_code;    
    BTGattServiceList_t * srvlist;
        
    srvlist = prvBTGattServiceListGet(serviceHandle);
    
    if(srvlist != NULL){
        s_gattsp_instance.register_srv_handle[s_gattsp_instance.register_srv_num%GR_BLE_MAX_SERVICES] = serviceHandle;
        s_gattsp_instance.register_srv_num++;
            
        //every service calls gatt_prf_db_init each time
        error_code = ble_server_prf_add(&srv_prf_info);  
        
        if (error_code != SDK_SUCCESS){
            GRC_LOG(DEBUG, ("ble_server_prf_add fail: %d  ", error_code));
            return gr_util_to_afr_status_code(error_code);
        } else {
            gr_util_gatt_handle_map_print();
        }
    } else {
        rStatus = eBTStatusUnHandled;
    }           
    
    return rStatus;
}

static void srv_gatts_read_cb(uint8_t conn_idx, const gatts_read_req_cb_t *p_read_req)
{
    static GR_CB_MSG_GATTS_READ_T  msg_read;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, ("srv_gatts_read_cb called...  "));
    
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }
    
    msg_read.gr_index            = conn_idx;
    memcpy(&msg_read.read_req, p_read_req, sizeof(*p_read_req));
    msg->msg_type                = GR_CB_MSG_GATTS_READ;
    msg->msg                     = (void*) &msg_read;
    
    gr_ble_cb_msg_send(msg, true);
}

static uint8_t s_safe_copy_buff[GR_BLE_GATTS_VAR_ATTR_LEN_MAX];

static void srv_gatts_write_cb(uint8_t conn_idx, const gatts_write_req_cb_t *p_write_req)
{
    static GR_CB_MSG_GATTS_WRITE_T msg_write;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, ("srv_gatts_write_cb called...  "));
    
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }
    
    memset(&s_safe_copy_buff[0], 0 , GR_BLE_GATTS_VAR_ATTR_LEN_MAX);

    msg_write.gr_index = conn_idx;
    msg_write.handle   = p_write_req->handle;
    msg_write.length   = p_write_req->length;
    msg_write.offset   = p_write_req->offset;
    msg_write.value    = &s_safe_copy_buff[0];    
    memcpy(msg_write.value, p_write_req->value, p_write_req->length);
    msg->msg_type      = GR_CB_MSG_GATTS_WRITE;
    msg->msg           = (void*) &msg_write;
    
    gr_ble_cb_msg_send(msg, true);
}

static void srv_gatts_prep_write_cb(uint8_t conn_idx, const gatts_prep_write_req_cb_t *p_prep_req)
{
    static GR_CB_MSG_GATTS_PREP_WRITE_T p_wr_msg;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, ("srv_gatts_prep_write_cb called...  "));
    
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }    
    p_wr_msg.gr_index           = conn_idx;    
    memcpy(&p_wr_msg.prep_write_req, p_prep_req, sizeof(gatts_prep_write_req_cb_t));
    
    msg->msg_type               = GR_CB_MSG_GATTS_PREP_WRITE;
    msg->msg                    = (void*) &p_wr_msg;
    
    gr_ble_cb_msg_send(msg, true);    
    
    
}

static void srv_gatts_ntf_ind_cb(uint8_t conn_idx, uint8_t status, const ble_gatts_ntf_ind_t *p_ntf_ind)
{
    static GR_CB_MSG_NTF_IND_T n_msg;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, ("srv_gatts_ntf_ind_cb called, idx:%d, status:%d...  ", conn_idx, status));
    
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }
    
    n_msg.msg_basic.gr_index    = conn_idx;
    n_msg.msg_basic.gr_status   = status;
    memcpy(&n_msg.ntf_ind, p_ntf_ind, sizeof(ble_gatts_ntf_ind_t));

    msg->msg_type               = GR_CB_MSG_GATTS_NTF_IND;
    msg->msg                    = (void*) &n_msg;
    
    gr_ble_cb_msg_send(msg, true);    
}


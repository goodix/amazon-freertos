#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "gr_message.h"
#include "gr_porting.h"
#include "gr_utils.h"

static QueueHandle_t                xGrQueue = NULL;
static TaskHandle_t                 xGrQueueTaskHandle = NULL;
static GR_CALLBACK_MSG_ARRAY_T      xGrCbMsgArray[GR_CB_MESSAGE_ARRAY_SIZE];
static volatile bool                xGrCbMsgTaskKilled = true;

#if GR_CALLBACK_TRACE_ENABLE > 0u
static char * gr_ble_get_cb_msg_name(GR_CALLBACK_MSG_E msg){
    char * name = NULL;
    
    switch (msg){
        case GR_CB_MSG_ADV_START:               name = "GR_CB_MSG_ADV_START";           break;
        case GR_CB_MSG_GAP_CONNECT:             name = "GR_CB_MSG_GAP_CONNECT";         break;
        case GR_CB_MSG_GAP_DISCONNECT:          name = "GR_CB_MSG_GAP_DISCONNECT";      break;
        case GR_CB_MSG_GAP_CONNECT_UPDATE:      name = "GR_CB_MSG_GAP_CONNECT_UPDATE";  break;
        case GR_CB_MSG_GATT_MTU_EXCHANGE:       name = "GR_CB_MSG_GATT_MTU_EXCHANGE";   break;
        case GR_CB_MSG_GATTS_WRITE:             name = "GR_CB_MSG_GATTS_WRITE";         break;
        case GR_CB_MSG_GATTS_READ:              name = "GR_CB_MSG_GATTS_READ";          break;
        case GR_CB_MSG_GATTS_PREP_WRITE:        name = "GR_CB_MSG_GATTS_PREP_WRITE";    break;
        case GR_CB_MSG_GATTS_NTF_IND:           name = "GR_CB_MSG_GATTS_NTF_IND";       break;
        case GR_CB_MSG_SM_ENC_REQ:              name = "GR_CB_MSG_SM_ENC_REQ";          break;
        case GR_CB_MSG_SM_PAIR_INC:             name = "GR_CB_MSG_SM_PAIR_INC";         break;
        case GR_CB_MSG_SM_BOND_INC:             name = "GR_CB_MSG_SM_BOND_INC";         break;
        case GR_CB_MSG_KILL_TASK:               name = "GR_CB_MSG_KILL_TASK";           break;
        default:
            name = "unknown";
            break;
    }
    
    return name;
}
#endif

/*-----------------------------------------------------------*/
/*
 * GR Ble callback always happens in isr, cannot call aws callback in isr, so handle them in single task!
 */
static void gr_ble_cb_msg_handle_task( void * p )
{
    GR_CALLBACK_MSG_T * gr_msg = NULL;
    bool is_exit = false;

    xGrCbMsgTaskKilled = false;
    is_exit = false;
    for(;;)
    {
        vTaskDelay(10);
        /* Block to wait for the next string to print. */
        if( xQueueReceive( xGrQueue, (void*)&gr_msg, portMAX_DELAY ) == pdPASS )
        {
            if(gr_msg == NULL) {
                continue;
            }

            GRC_LOG(INFO, (">>> handle cb msg: %s  ", gr_ble_get_cb_msg_name(gr_msg->msg_type)));
            switch(gr_msg->msg_type){
                case GR_CB_MSG_ADV_START:
                {
                    GR_CB_MSG_BASIC_T * adv_msg = (GR_CB_MSG_BASIC_T *) gr_msg->msg;                   
                    BTStatus_t xStatus = (adv_msg->gr_status == BLE_SUCCESS) ? eBTStatusSuccess : eBTStatusFail;        
                    if( xBTBleAdapterCallbacks.pxAdvStartCb )
                    {
                        xBTBleAdapterCallbacks.pxAdvStartCb( xStatus, ulGattServerIFhandle );
                    }                    
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_GAP_CONNECT:
                {
                    GR_CB_MSG_BASIC_T * conn_msg = (GR_CB_MSG_BASIC_T *) gr_msg->msg;
                    
                    if(xGattServerCb.pxConnectionCb != NULL){
                        xGattServerCb.pxConnectionCb(conn_msg->gr_index,
                                                     ulGattServerIFhandle,
                                                     true,
                                                     ( BTBdaddr_t * )(s_gr_ble_gap_params_ins.gap_conn_cmp_param.peer_addr.addr));
                    }
                    
                    gr_ble_cb_msg_free_mem(gr_msg);                    
                }
                break;
                
                case GR_CB_MSG_GAP_DISCONNECT:
                {
                    GR_CB_MSG_BASIC_T * disconn_msg = (GR_CB_MSG_BASIC_T *) gr_msg->msg;
                    if(xGattServerCb.pxConnectionCb != NULL){
                        xGattServerCb.pxConnectionCb(disconn_msg->gr_index,
                                                     ulGattServerIFhandle,
                                                     false,
                                                     ( BTBdaddr_t * )(s_gr_ble_gap_params_ins.gap_conn_cmp_param.peer_addr.addr));
                    }                    
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_GAP_CONNECT_UPDATE:
                {
                    GR_CB_MSG_CONN_UPDATE_T * conn_update = (GR_CB_MSG_CONN_UPDATE_T *) gr_msg->msg;
                    BTStatus_t xStatus = eBTStatusSuccess;
                    uint8_t status = conn_update->msg_basic.gr_status;
                                        
                    if(status == BLE_SUCCESS) {
                        
                        if(!s_gr_ble_gap_params_ins.is_mtu_exchanged){
                            ble_gattc_mtu_exchange(s_gr_ble_gap_params_ins.cur_connect_id);
                            s_gr_ble_gap_params_ins.is_mtu_exchanged = true;
                        }                        
                        
                        if( xBTBleAdapterCallbacks.pxConnParameterUpdateCb != NULL)
                        {
                            uint16_t usActualInterval = conn_update->update.interval;        

                            if( ( usActualInterval > s_gr_ble_gap_params_ins.gap_conn_param.ulMaxInterval ) || ( usActualInterval < s_gr_ble_gap_params_ins.gap_conn_param.ulMinInterval ) )
                            {
                                xStatus = eBTStatusFail;
                            }

                            xBTBleAdapterCallbacks.pxConnParameterUpdateCb( xStatus,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.pxBdAddr,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.ulMinInterval,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.ulMaxInterval,
                                                                            conn_update->update.latency,
                                                                            usActualInterval,
                                                                            conn_update->update.sup_timeout );
                        }
                    } else {
                        if( xBTBleAdapterCallbacks.pxConnParameterUpdateCb != NULL)
                        {
                            xBTBleAdapterCallbacks.pxConnParameterUpdateCb( eBTStatusFail,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.pxBdAddr,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.ulMinInterval,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.ulMaxInterval,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.ulLatency,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.ulMaxInterval,
                                                                            s_gr_ble_gap_params_ins.gap_conn_param.ulTimeout);
                        }
                    }
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_GATT_MTU_EXCHANGE:
                {
                    GR_CB_MSG_MTU_EXCHANGE_T * mtu_msg = (GR_CB_MSG_MTU_EXCHANGE_T *) gr_msg->msg;
                    
                    if( xGattServerCb.pxMtuChangedCb != NULL )
                    {
                        xGattServerCb.pxMtuChangedCb(mtu_msg->msg_basic.gr_index, mtu_msg->mtu);
                    }
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_GATTS_WRITE:
                {
                    GR_CB_MSG_GATTS_WRITE_T * wr_msg = (GR_CB_MSG_GATTS_WRITE_T *) gr_msg->msg;                    
                    uint16_t            port_handle  = gr_gatt_transto_porting_layer_handle(wr_msg->handle);

                    if(( port_handle != GR_BLE_GATT_INVALID_HANDLE ) && ( xGattServerCb.pxRequestWriteCb != NULL ))
                    {
                        BTBdaddr_t remote_addr;
                        BTGattEntity_t *  gent  = prvBTGattEntityGet(port_handle);
                        bool IsNeedRsp          = false;
                        bool IsPrep             = false;                        
                        
                        /*
                         * Characteristic - check prop in advance
                         * descriptor - chek perm in advance
                         */
                        if(gent != NULL){
                            
                            if( (gent->type == eBTDbCharacteristic) && 
                                (gent->raw_properties & (eBTPropWrite | eBTPropSignedWrite))
                            ){
                                IsNeedRsp       = true;
                            }
                            
                            if( (gent->type == eBTDbDescriptor) && 
                                (gent->raw_permissions & (eBTPermWrite | eBTPermWriteEncrypted | eBTPermWriteEncryptedMitm | eBTPermWriteSigned | eBTPermWriteSignedMitm))
                            ){
                                IsNeedRsp       = true; 
                            }
                        }
                        //gr_util_print_buffer(BLE_GATTS_TYPE_WRITE, port_handle, &wr_msg->value[wr_msg->offset], wr_msg->length);
                        memcpy(&remote_addr.ucAddress[0], &s_gr_ble_gap_params_ins.gap_conn_cmp_param.peer_addr.addr[0], btADDRESS_LEN);                        
                        xGattServerCb.pxRequestWriteCb( wr_msg->gr_index,
                                                        BLE_GATTS_TYPE_WRITE,
                                                        &remote_addr,
                                                        port_handle,
                                                        wr_msg->offset,
                                                        wr_msg->length,                                                        
                                                        IsNeedRsp,
                                                        IsPrep,
                                                        ( uint8_t * ) wr_msg->value );
                        //whatever, must give a cfm to stack
                        if(!IsNeedRsp){
                            gatts_write_cfm_t   cfm;                        
                            GRC_LOG(DEBUG, (">>> Special Attention: give a confirm to writeWithNoResponse... "));
                            cfm.handle = wr_msg->handle;
                            cfm.status = BLE_SUCCESS;
                            ble_gatts_write_cfm(wr_msg->gr_index, &cfm);
                        } else {
                            //ignore, will call response by upper layer api
                        }
                        
                    } else {
                        BTCacheGattValue_t *pgval        = prvBTGattValueHandleGet(port_handle);
                        gatts_write_cfm_t   cfm;
                        cfm.handle = wr_msg->handle;
                        cfm.status = BLE_SUCCESS;
                        
                        if(pgval == NULL){
                            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
                            ble_gatts_write_cfm(wr_msg->gr_index, &cfm);
                        } else {
                            uint16_t tlen = wr_msg->offset + wr_msg->length;
                            
                            if((pgval->mem_size == 0 ) || (tlen > pgval->mem_size)){
                                cfm.status = BLE_ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
                                ble_gatts_write_cfm(wr_msg->gr_index, &cfm);
                            } else {
                                memcpy(pgval->data + wr_msg->offset, wr_msg->value, wr_msg->length);
                                pgval->cur_size = tlen;
                                pgval->is_wrote = true;
                                
                                ble_gatts_write_cfm(wr_msg->gr_index, &cfm);        
                            }                        
                        }                    
                    }
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_GATTS_READ:
                {
                    GR_CB_MSG_GATTS_READ_T * rd_msg = (GR_CB_MSG_GATTS_READ_T *) gr_msg->msg;
                    gatts_read_cfm_t    cfm;
                    uint16_t            port_handle = gr_gatt_transto_porting_layer_handle(rd_msg->read_req.handle);  

                    if( xGattServerCb.pxRequestReadCb != NULL )
                    {
                        BTBdaddr_t remote_addr;
                        memcpy(&remote_addr.ucAddress[0], &s_gr_ble_gap_params_ins.gap_conn_cmp_param.peer_addr.addr[0], btADDRESS_LEN);
                        
                        xGattServerCb.pxRequestReadCb( rd_msg->gr_index,
                                                       BLE_GATTS_TYPE_READ,
                                                       &remote_addr,
                                                       port_handle,
                                                       0);
                    } else {
                        BTCacheGattValue_t *pgval       = prvBTGattValueHandleGet(port_handle);

                        cfm.handle = rd_msg->read_req.handle;
                        cfm.status = BLE_SUCCESS;
                        cfm.value  = NULL;
                        
                        if(pgval == NULL){
                            cfm.length = 0;
                            cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
                            ble_gatts_read_cfm(rd_msg->gr_index, &cfm);
                        } else {
                            if(!pgval->is_wrote || (pgval->cur_size == 0)){
                                cfm.length = strlen("unset");
                                cfm.value  = (uint8_t*)"unset";
                                cfm.status = BLE_SUCCESS;
                                //cfm.status = BLE_ATT_ERR_READ_NOT_PERMITTED;
                                ble_gatts_read_cfm(rd_msg->gr_index, &cfm);
                            } else {
                                cfm.length = pgval->cur_size;
                                cfm.value  = pgval->data;
                                cfm.status = BLE_SUCCESS;
                                ble_gatts_read_cfm(rd_msg->gr_index, &cfm);
                            }
                        }
                    }
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case  GR_CB_MSG_GATTS_PREP_WRITE:
                {
                    gatts_prep_write_cfm_t cfm;
                    GR_CB_MSG_GATTS_PREP_WRITE_T * p_wr_msg = (GR_CB_MSG_GATTS_PREP_WRITE_T *) gr_msg->msg;
                    uint16_t            port_handle         = gr_gatt_transto_porting_layer_handle(p_wr_msg->prep_write_req.handle);    
                    BTCacheGattValue_t *pgval               = prvBTGattValueHandleGet(port_handle);
                    
                    cfm.handle = p_wr_msg->prep_write_req.handle;
                    
                    if(pgval != NULL){
                        cfm.length = pgval->mem_size;
                        cfm.status = BLE_SUCCESS;
                    } else {
                        cfm.length = 0;
                        cfm.status = BLE_ATT_ERR_INSUFF_RESOURCE;
                    }
                    
                    ble_gatts_prepare_write_cfm(p_wr_msg->gr_index, &cfm);
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_GATTS_NTF_IND:
                {
                    GR_CB_MSG_NTF_IND_T * n_msg = (GR_CB_MSG_NTF_IND_T *) gr_msg->msg;
                    
                    if((n_msg->ntf_ind.type == BLE_GATT_INDICATION) && (xGattServerCb.pxIndicationSentCb != NULL)){
                    //if((xGattServerCb.pxIndicationSentCb != NULL)){
                        uint16_t conn_id = s_gr_ble_gap_params_ins.is_connected ? s_gr_ble_gap_params_ins.cur_connect_id : 0;                        
                        xGattServerCb.pxIndicationSentCb(conn_id, (n_msg->msg_basic.gr_status == BLE_SUCCESS) ? eBTStatusSuccess : eBTStatusFail);
                    }
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_SM_ENC_REQ:
                {
                    GR_CB_MSG_ENC_REQ_T * enc_msg = (GR_CB_MSG_ENC_REQ_T *) gr_msg->msg;                    
                    sec_cfm_enc_t       cfm_enc;
                    uint32_t            tk;
                    uint32_t            passkey = 0;
                    BTSspVariant_t      ssp = eBTsspVariantConsent;

                    memset((uint8_t *)&cfm_enc, 0, sizeof(cfm_enc));
                    
                    GRC_LOG(DEBUG, (">>> SEC REQ cmd: %d  ", enc_msg->enc_req.req_type));
                    
                    switch (enc_msg->enc_req.req_type)
                    {
                        // User needs to decide whether to accept the pair request.
                        case PAIR_REQ:
                        {
                            cfm_enc.req_type = PAIR_REQ;
                            cfm_enc.accept   = true;
                            ble_sec_enc_cfm(enc_msg->gr_index, &cfm_enc);
                            
                            ssp = eBTsspVariantConsent;
                            s_gr_ble_gap_params_ins.is_need_sec_cfm = false;
                        }
                        break;

                        // User needs to input the password.
                        case TK_REQ:
                        {
                            cfm_enc.req_type = TK_REQ;
                            cfm_enc.accept   = true;

                            tk = 123456;

                            memset(cfm_enc.data.tk.key, 0, sizeof(cfm_enc.data.tk.key));
                            cfm_enc.data.tk.key[0] = (uint8_t)((tk & 0x000000FF) >> 0);
                            cfm_enc.data.tk.key[1] = (uint8_t)((tk & 0x0000FF00) >> 8);
                            cfm_enc.data.tk.key[2] = (uint8_t)((tk & 0x00FF0000) >> 16);
                            cfm_enc.data.tk.key[3] = (uint8_t)((tk & 0xFF000000) >> 24);
                            ssp = eBTsspVariantPasskeyConfirmation;//eBTsspVariantPasskeyEntry;
                            //ble_sec_enc_cfm(enc_msg->gr_index, &cfm_enc);
                            s_gr_ble_gap_params_ins.is_need_sec_cfm = true;
                        }
                        break;

                        case NC_REQ:
                        {
                            cfm_enc.req_type = NC_REQ;
                            cfm_enc.accept   = true;
                            passkey          = *(uint32_t *)(enc_msg->enc_req.data.nc_data.value);
                            GRC_LOG(DEBUG, (">>> NC_REQ, passkey = %d\n", passkey));
                            memset(cfm_enc.data.tk.key, 0, sizeof(cfm_enc.data.tk.key));
                            cfm_enc.data.tk.key[0] = (uint8_t)((passkey & 0x000000FF) >> 0);
                            cfm_enc.data.tk.key[1] = (uint8_t)((passkey & 0x0000FF00) >> 8);
                            cfm_enc.data.tk.key[2] = (uint8_t)((passkey & 0x00FF0000) >> 16);
                            cfm_enc.data.tk.key[3] = (uint8_t)((passkey & 0xFF000000) >> 24);
                            ssp = eBTsspVariantPasskeyConfirmation;
                            //ble_sec_enc_cfm(enc_msg->gr_index, &cfm_enc);
                            s_gr_ble_gap_params_ins.is_need_sec_cfm = true;
                        }
                        break;
                        
                        case OOB_REQ:
                        default:
                            cfm_enc.req_type = OOB_REQ;
                            cfm_enc.accept   = true;
                            ble_sec_enc_cfm(enc_msg->gr_index, &cfm_enc);
                            ssp = eBTsspVariantPasskeyNotification;
                            s_gr_ble_gap_params_ins.is_need_sec_cfm = false;
                            break;
                    }

                    
                    memcpy(&s_gr_ble_gap_params_ins.sec_cfm, &cfm_enc, sizeof(sec_cfm_enc_t));
                    //call in SspReply : 
                    
                    if(xBTCallbacks.pxSspRequestCb != NULL) {
                        xBTCallbacks.pxSspRequestCb((BTBdaddr_t*)&s_gr_ble_gap_params_ins.gap_conn_cmp_param.peer_addr.addr[0],
                                                 NULL,
                                                 0,
                                                 ssp,
                                                 passkey);
                    }
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_SM_PAIR_INC:
                {
                    BTStatus_t xStatus;
                    bool bonded;
                    GR_CB_MSG_PAIR_INC_T * pair_msg = (GR_CB_MSG_PAIR_INC_T *) gr_msg->msg;
                    
                    xStatus = pair_msg->enc_pair_ind == ENC_SUCCESS ? eBTStatusSuccess : eBTStatusFail;

                    if( xBTCallbacks.pxPairingStateChangedCb )
                    {
                        xBTCallbacks.pxPairingStateChangedCb( xStatus,
                                                              (BTBdaddr_t *)&s_gr_ble_gap_params_ins.gap_conn_cmp_param.peer_addr.addr[0],
                                                              xStatus == eBTStatusSuccess ? eBTbondStateBonded : eBTbondStateNone,
                                                              gr_util_adjust_to_aws_security_level(xGRSecurityParameters.level),
                                                              (BTAuthFailureReason_t)0 );
                    }

                    if(xBTCallbacks.pxBondedCb){

                        bonded = (pair_msg->auth & AUTH_BOND) ? true : false;
                        
                        GRC_LOG(DEBUG, (">>> Bond Ret, bond = %d, auth = %d ", bonded, pair_msg->auth));
                        
                        //bonded = xStatus == eBTStatusSuccess ? true : false;
                        
                        xBTCallbacks.pxBondedCb( eBTStatusSuccess /*bonded ? eBTStatusSuccess : eBTStatusFail*/,
                                                (BTBdaddr_t *)&s_gr_ble_gap_params_ins.gap_conn_cmp_param.peer_addr.addr[0],
                                                bonded );
                    }
                    gr_ble_cb_msg_free_mem(gr_msg);
                }
                break;
                
                case GR_CB_MSG_SM_BOND_INC:
                {
                    //handle in GR_CB_MSG_SM_PAIR_INC
                }
                break;
                
                case GR_CB_MSG_KILL_TASK:
                {
                    gr_ble_cb_msg_free_mem(gr_msg);
                    is_exit = true;
                }
                break;
                
                default:
                    break;
            }
        }
        
        if(is_exit) {
            break;
        }
    }
    
    xGrQueueTaskHandle = NULL;
    
    xGrCbMsgTaskKilled = true;
    vTaskDelete(NULL);    
}

/*-----------------------------------------------------------*/

static BaseType_t gr_ble_cb_msg_task_init( uint16_t usStackSize, UBaseType_t uxPriority)
{
    BaseType_t xReturn = pdFAIL;
    
    if((xGrQueue != NULL) && (xGrQueueTaskHandle != NULL)){
        GRC_LOG(INFO, (">>> gr ble msg handler task is alive... "));
        return pdTRUE;
    }
    
    GRC_LOG(INFO, (">>> create gr ble msg handler task... "));
    xGrCbMsgTaskKilled = true;
    memset(&xGrCbMsgArray[0], 0 , sizeof(GR_CALLBACK_MSG_ARRAY_T)*GR_CB_MESSAGE_ARRAY_SIZE);
    /* Ensure the logging task has not been created already. */
    if( xGrQueue == NULL )
    {
        /* Create the queue used to pass pointers to strings to the logging task. */
        xGrQueue = xQueueCreate( GR_CB_MESSAGE_QUEUE_SIZE, sizeof( GR_CALLBACK_MSG_T) );

        if( xGrQueue != NULL )
        {
            if( xTaskCreate( gr_ble_cb_msg_handle_task, "GrMsg", usStackSize, NULL, uxPriority, &xGrQueueTaskHandle ) == pdPASS )
            {
                xReturn = pdPASS;
            }
            else
            {
                /* Could not create the task, so delete the queue again. */
                vQueueDelete( xGrQueue );
                xGrQueue = NULL;
            }
        }
    }

    return xReturn;
}

static void gr_ble_cb_msg_task_kill(void)
{
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, (">>> ready to kill the gr msg handler task! "));   
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }
    
    msg->msg_type            = GR_CB_MSG_KILL_TASK;
    msg->msg                 = (void*) NULL;
    
    gr_ble_cb_msg_send(msg, true);
}

static void gr_ble_cb_msg_task_wait_run(void){
    while(xGrCbMsgTaskKilled){
        vTaskDelay(50);
    }
}

void gr_ble_cb_msg_task_startup(void * ctxt){
    BaseType_t xReturn = pdFAIL;
    
    xReturn = gr_ble_cb_msg_task_init(GR_CB_MESSAGE_TASK_STACK_SIZE, GR_CB_MESSAGE_TASK_PRIO);    
    configASSERT(xReturn == pdPASS);
    gr_ble_cb_msg_task_wait_run();
    GRC_LOG(INFO, (">>> gr_ble_cb_msg_task is running... "));    
}

GR_CALLBACK_MSG_T * gr_ble_cb_msg_alloc_mem(void){
    for(int i =0; i< GR_CB_MESSAGE_ARRAY_SIZE; i++){
        if(!xGrCbMsgArray[i].is_used){
            xGrCbMsgArray[i].is_used = true;            
            return &xGrCbMsgArray[i].cb_msg;
        }
    }
    GRC_LOG(ERROR, (">>> gr_ble_cb_msg_alloc_mem: malloc mem failed. "));
    return NULL;
}

void gr_ble_cb_msg_free_mem(GR_CALLBACK_MSG_T * buf){
    for(int i =0; i< GR_CB_MESSAGE_ARRAY_SIZE; i++){
        if(((int)&xGrCbMsgArray[i].cb_msg) == ((int)buf)){            
            memset(&xGrCbMsgArray[i].cb_msg, 0, sizeof(GR_CALLBACK_MSG_T));
            xGrCbMsgArray[i].is_used = false;
            
            break;
        }
    }    
}

void gr_ble_cb_msg_task_deinit(void){
    //async action!    
    gr_ble_cb_msg_task_kill();
    
    while(!xGrCbMsgTaskKilled){
        vTaskDelay(500);
        GRC_LOG(INFO, (">>> wait killed... "));
    }
    if( xGrQueue != NULL ){
        vQueueDelete( xGrQueue );
        xGrQueue = NULL;
    }
    GRC_LOG(INFO, (">>> gr_ble_cb_msg_task is killed... "));    
}

bool gr_ble_cb_msg_send(GR_CALLBACK_MSG_T * msg, BaseType_t xHigherPriorityTaskWoken) {
    if(msg != NULL){
        if( xQueueSendFromISR(xGrQueue, (void *) &msg, &xHigherPriorityTaskWoken) == pdTRUE){
            return true;
        }
    }
    
    return false;
}

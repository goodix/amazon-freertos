/**
 *****************************************************************************************
 *
 * @file user_gatt_common_callback.c
 *
 * @brief  BLE GATT Callback Function Implementation.
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
#include "FreeRTOSConfig.h"
#include "gr_config.h"
#include "gr_porting.h"
#include "gr_message.h"

 /*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_gatt_mtu_exchange_cb(uint8_t conn_idx, uint8_t status, uint16_t mtu);
static void app_gatt_prf_register_cb(uint8_t status, uint8_t prf_index);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const gatt_common_cb_fun_t app_gatt_common_callback = 
{
    .app_gatt_mtu_exchange_cb = app_gatt_mtu_exchange_cb,
    .app_gatt_prf_register_cb = app_gatt_prf_register_cb,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This callback function will be called when the mtu has been exchanged.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] status:   The status of GATT operation.
 * @param[in] mtu:      The value of exchanged mtu.
 *****************************************************************************************
 */
static void app_gatt_mtu_exchange_cb(uint8_t conn_idx, uint8_t status, uint16_t mtu)
{
    static GR_CB_MSG_MTU_EXCHANGE_T mtu_msg;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
		//APP_LOG_DEBUG(">>> app_gatt_mtu_exchange_cb called, conn_idx:%d, status:%d, mtu:%d  ", conn_idx, status, mtu);
    GRC_LOG(DEBUG, (">>> app_gatt_mtu_exchange_cb called, conn_idx:%d, status:%d, mtu:%d  ", conn_idx, status, mtu));
    if(status == BLE_SUCCESS){
        
				//update local mtu again
				ble_gap_l2cap_params_set(mtu, mtu, GR_BLE_MAX_NB_LECB_DEFUALT);
				
        xProperties.xExchangedMtu       = mtu;
        
        msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
        if(msg == NULL){
            return;
        }
        
        mtu_msg.msg_basic.gr_index   = conn_idx;
        mtu_msg.msg_basic.gr_status  = status;
        mtu_msg.mtu                  = mtu;
        msg->msg_type                = GR_CB_MSG_GATT_MTU_EXCHANGE;
        msg->msg                     = (void*) &mtu_msg;
        
        gr_ble_cb_msg_send(msg, true);
    }
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when the mtu has been exchanged.
 *
 * @param[in] status:       The status of GATT operation.
 * @param[in] prf_index:    prf_index range is from 0 to profile count - 1. prf_index is current profile index
 *****************************************************************************************
 */
static void app_gatt_prf_register_cb(uint8_t status, uint8_t prf_index)
{
    GRC_LOG(DEBUG, (">>> app_gatt_prf_register_cb called, prf_index:%d, status:%d  ", prf_index, status));
}

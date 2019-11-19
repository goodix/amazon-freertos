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

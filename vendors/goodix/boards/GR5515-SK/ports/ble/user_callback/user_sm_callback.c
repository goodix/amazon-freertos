/**
 *****************************************************************************************
 *
 * @file user_sm_callback.c
 *
 * @brief  BLE SM Callback Function Implementation.
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
#include "gr_message.h"

 /*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static void app_sec_rcv_enc_req_cb(uint8_t conn_idx, sec_enc_req_t *p_enc_req);
static void app_sec_rcv_enc_ind_cb(uint8_t conn_idx, sec_enc_ind_t enc_ind, uint8_t auth);
static void app_sec_rcv_keypress_nofify_cb(uint8_t conn_idx, sec_keypress_notify_t notify_type);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const sec_cb_fun_t app_sec_callback =
{
    .app_sec_enc_req_cb         = app_sec_rcv_enc_req_cb,
    .app_sec_enc_ind_cb         = app_sec_rcv_enc_ind_cb,
    .app_sec_keypress_notify_cb = app_sec_rcv_keypress_nofify_cb
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving encryption request.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] p_enc_req: The information of SEC encryption request. See @ref sec_enc_req_t.
 *****************************************************************************************
 */
static void app_sec_rcv_enc_req_cb(uint8_t conn_idx, sec_enc_req_t *p_enc_req)
{
    static GR_CB_MSG_ENC_REQ_T     enc_msg;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, (">>> app_sec_rcv_enc_req_cb called, conn_idx:%d  ", conn_idx));    
    
    if (NULL == p_enc_req)
    {
        return;
    }
    
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }
    
    enc_msg.gr_index         = conn_idx;
    memcpy(&enc_msg.enc_req, p_enc_req, sizeof(sec_enc_req_t));
    msg->msg_type            = GR_CB_MSG_SM_ENC_REQ;
    msg->msg                 = (void*) &enc_msg;

    gr_ble_cb_msg_send(msg, true);
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving pair indication.
 *
 * @param[in] conn_idx: The connection index.
 * @param[in] enc_ind:  The result of SEC pair. See @ref sec_enc_ind_t.
 *****************************************************************************************
 */
static void app_sec_rcv_enc_ind_cb(uint8_t conn_idx, sec_enc_ind_t enc_ind, uint8_t auth)
{
    static GR_CB_MSG_PAIR_INC_T   pair_msg;
    GR_CALLBACK_MSG_T            * msg = NULL;
    
    GRC_LOG(DEBUG, (">>> app_sec_rcv_enc_ind_cb called, conn_idx:%d , status: %d , auth: %d", conn_idx, enc_ind, auth));
    
    msg = (GR_CALLBACK_MSG_T*) gr_ble_cb_msg_alloc_mem();
    if(msg == NULL){
        return;
    }
    
    pair_msg.gr_index           = conn_idx;
    pair_msg.enc_pair_ind       = enc_ind;
    pair_msg.auth               = auth;
    msg->msg_type               = GR_CB_MSG_SM_PAIR_INC;
    msg->msg                    = (void*) &pair_msg;

    gr_ble_cb_msg_send(msg, true);    
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving key press notify.
 *
 * @param[in] conn_idx:    The connection index.
 * @param[in] notify_type: The type of SEC key press. See @ref sec_keypress_notify_t.
 *****************************************************************************************
 */
static void app_sec_rcv_keypress_nofify_cb(uint8_t conn_idx, sec_keypress_notify_t notify_type)
{
    GRC_LOG(DEBUG, (">>> app_sec_rcv_keypress_nofify_cb called, conn_idx:%d  ", conn_idx));
}

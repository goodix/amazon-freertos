/**
 *****************************************************************************************
 *
 * @file user_gattc_callback.c
 *
 * @brief  BLE GATTC Callback Function Implementation.
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

/*
* LOCAL FUNCTION DECLARATION
******************************************************************************************
*/
static void app_gattc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t *p_prim_srvc_disc);
static void app_gattc_inc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t *p_inc_srvc_disc);
static void app_gattc_char_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t *p_char_disc);
static void app_gattc_char_desc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc); 
static void app_gattc_read_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp);
static void app_gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle);
static void app_gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);
static void app_gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc);

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
const gattc_cb_fun_t app_gattc_callback =
{
    .app_gattc_srvc_disc_cb      = app_gattc_srvc_disc_cb,
    .app_gattc_inc_srvc_disc_cb  = app_gattc_inc_srvc_disc_cb,
    .app_gattc_char_disc_cb      = app_gattc_char_disc_cb,
    .app_gattc_char_desc_disc_cb = app_gattc_char_desc_disc_cb,
    .app_gattc_write_cb          = app_gattc_write_cb,
    .app_gattc_read_cb           = app_gattc_read_cb,
    .app_gattc_ntf_ind_cb        = app_gattc_ntf_ind_cb,
    .app_gattc_srvc_browse_cb    = app_gattc_srvc_browse_cb,
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This callback function will be called when primary service has been discovered.
 *
 * @param[in] conn_idx:         The connection index.
 * @param[in] status:           The status of GATTC operation.
 * @param[in] p_prim_srvc_disc: The information of primary service. See @ref ble_gattc_srvc_disc_t.
 *****************************************************************************************
 */
static void app_gattc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_srvc_disc_t *p_prim_srvc_disc)
{
    GRC_LOG(DEBUG, (">>> app_gattc_srvc_disc_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when service relationship has been discovered.
 *
 * @param[in] conn_idx:        The connection index.
 * @param[in] status:          The status of GATTC operation.
 * @param[in] p_inc_srvc_disc: The information of service relationship. See @ref ble_gattc_incl_disc_t.
 *****************************************************************************************
 */
static void app_gattc_inc_srvc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_incl_disc_t *p_inc_srvc_disc)
{
    GRC_LOG(DEBUG, (">>> app_gattc_inc_srvc_disc_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when descriptor(s) has been discovered.
 *
 * @param[in] conn_idx:        The connection index.
 * @param[in] status:          The status of GATTC operation.
 * @param[in] p_char_disc: The information of descriptor(s). See @ref ble_gattc_char_disc_t.
 *****************************************************************************************
 */
static void app_gattc_char_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_disc_t *p_char_disc)
{
    GRC_LOG(DEBUG, (">>> app_gattc_char_disc_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when characteristic(s) has been discovered.
 *
 * @param[in] conn_idx:         The connection index.
 * @param[in] status:           The status of GATTC operation.
 * @param[in] p_char_desc_disc: The information of characteristic(s). See @ref ble_gattc_char_desc_disc_t.
 *****************************************************************************************
 */
static void app_gattc_char_desc_disc_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_char_desc_disc_t *p_char_desc_disc)
{
    GRC_LOG(DEBUG, (">>> app_gattc_char_desc_disc_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] handle:     The handle of attribute.
 *****************************************************************************************
 */
static void app_gattc_write_cb(uint8_t conn_idx, uint8_t status, uint16_t handle)
{
    GRC_LOG(DEBUG, (">>> app_gattc_write_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving read response.
 *
 * @param[in] conn_idx:   The connection index.
 * @param[in] status:     The status of GATTC operation.
 * @param[in] p_read_rsp: The information of read response. See @ref ble_gattc_read_rsp_t.
 *****************************************************************************************
 */
static void app_gattc_read_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_read_rsp_t *p_read_rsp)
{
    GRC_LOG(DEBUG, (">>> app_gattc_read_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving notification or indication.
 *
 * @param[in] conn_idx:  The connection index.
 * @param[in] status:    The status of GATTC operation.
 * @param[in] p_ntf_ind: The information of notification or indication. See @ref ble_gattc_ntf_ind_t.
 *****************************************************************************************
 */
static void app_gattc_ntf_ind_cb(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind)
{
    GRC_LOG(DEBUG, (">>> app_gattc_ntf_ind_cb called, conn_idx:%d  ", conn_idx));
}

/**
 *****************************************************************************************
 * @brief This callback function will be called when receiving browse service indication.
 *
 * @param[in] conn_idx:      The connection index.
 * @param[in] status:        The status of GATTC operation.
 * @param[in] p_browse_srvc: The information of service browse. See @ref ble_gattc_browse_srvc_t.
 *****************************************************************************************
 */
static void app_gattc_srvc_browse_cb(uint8_t conn_idx, uint8_t status, const ble_gattc_browse_srvc_t *p_browse_srvc)
{
    GRC_LOG(DEBUG, (">>> app_gattc_srvc_browse_cb called, conn_idx:%d, status:%d  ", conn_idx, status));
}

/**
 *****************************************************************************************
 *
 * @file ancs_c.h
 *
 * @brief ANCS Service API.
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_ANCS Apple Notification Center Service (ANCS)
 * @{
 * @brief Definitions and prototypes for the ANCS interface.
 
 * @details ANCS provides a way for BLE devices to receive IOS mobile phone notifications. 
 *          The service consists of three eigenvalues, including notification source, 
 *          data source, control point.
 *          
 *          The application needs to call \ref ancs_c_init() to initialize, and then 
 *          use \ref ancs_c_discovery_service() to discover ANCS-related services on IOS devices. 
 *          After discovery, ancs_c_on_browse_svc_evt function is called to parse and save 
 *          the handle corresponding to each service. These handles are used for data
 *          transmission.
 *      
 *          Secondly, use \ref ancs_c_notif_source_notif_enable and \ref ancs_c_data_source_notif_enable() 
 *          to enable notification source & data source's CCD. The IOS notification is then sent to the 
 *          BLE device side immediately.
 *      
 *          Finally, the application can use \ref ancs_c_gattc_notify_dispatch() to parse the 
 *          ANCS message and \ref ancs_c_write_control_point() to command the control point.
 *          For specific commands, please refer to the ANCS protocol specification.
 *
 */

#ifndef _ANCS_H_
#define _ANCS_H_

#include "gr55xx_sys.h"
#include <stdint.h>

/**
 * @defgroup ANCS_ENUM Enumerations
 * @{
 */
/**@brief ANCS discovery result. */
typedef enum
{
    ANCS_C_EVT_DISCOVERY_COMPLETE,
    ANCS_C_EVT_DISCOVERY_FAILED,
}ancs_c_evt_t;
/** @} */

/**
 * @defgroup ANCS_STRUCT Structures
 * @{
 */
/**@brief  ancs handle structure. */
typedef struct
{
    uint16_t ancs_service_handle;                   /**< Handle of ancs service as provided by a discovery. */
    uint16_t ancs_notification_source_handle;       /**< Handle of ancs  notification source characteristic as provided by a discovery. */
    uint16_t ancs_control_point_handle;             /**< Handle of ancs  control point characteristic as provided by a discovery. */
    uint16_t ancs_data_source_handle;               /**< Handle of ancs  data source characteristic as provided by a discovery. */
    uint16_t ancs_notify_source_cccd_handle;        /**< Handle of CCCD of ancs  control point characteristic as provided by a discovery. */
    uint16_t ancs_data_source_cccd_handle;          /**< Handle of CCCD of ancs data source characteristic as provided by a discovery. */
}ancs_c_att_handle_t;
/** @} */

/**
 * @defgroup ANCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize ANCS structure of handle
 *****************************************************************************************
 */
void ancs_c_init(void);

/**
 *****************************************************************************************
 * @brief To access phone's all service about ANCS
 *
 * @param[in] conn_idx: Connection index
 *****************************************************************************************
*/
void ancs_c_discovery_service(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief  enable ancs notification source CCCD
 *
 * @param[in] conn_idx: Connection index.
 * @return    success or not.
 *****************************************************************************************
 */
uint8_t ancs_c_notif_source_notif_enable(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief  disable ancs notification source CCCD
 *
 * @param[in] conn_idx: Connection index.
 * @return    success or not.
 *****************************************************************************************
 */
uint8_t ancs_c_notif_source_notif_disable(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief  enable ancs data source CCCD
 *
 * @param[in] conn_idx: Connection index.
 * @return    success or not.
 *****************************************************************************************
 */
uint8_t ancs_c_data_source_notif_enable(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief  disable ancs data source CCCD
 *
 * @param[in] conn_idx: Connection index.
 * @return    success or not.
 *****************************************************************************************
 */
uint8_t ancs_c_data_source_notif_disable(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief This function implements writing commands to control points
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to send out data
 * @param[in] length:   Length of data sent out
 *****************************************************************************************
 */
uint8_t ancs_c_write_control_point(uint8_t conn_idx, uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief dispatch all notify message from notification source channel
 *
 * @param[in] conn_idx:  Connection index.
 * @param[in] p_ntf_ind: Pointer to the parameters of the Notification and Indication value indication.
 *****************************************************************************************
 */
void ancs_c_gattc_notify_dispatch(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind);

/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx:      Connection index.
 * @param[in] p_browse_srvc: Pointer to the parameters of the browse services.
 *****************************************************************************************
 */
void ancs_c_on_browse_svc_evt(uint8_t conn_idx, const ble_gattc_browse_srvc_t *p_browse_srvc);
/** @} */

#endif
/** @} */
/** @} */

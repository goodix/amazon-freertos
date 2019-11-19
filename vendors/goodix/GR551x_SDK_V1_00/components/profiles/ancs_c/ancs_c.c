/**
 *****************************************************************************************
 *
 * @file ancs_c.c
 *
 * @brief ANCS Profile Server implementation.
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

#include "ancs_c.h"
#include "ancs_protocol.h"
#include "string.h"
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static const uint8_t ancs_service_uuid[] = 
{0xd0, 0x00, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 0xa4,0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x79};

static const uint8_t ancs_notification_source_uuid[] = 
{0xbd, 0x1d, 0xa2, 0x99, 0xe6, 0x25, 0x58, 0x8c,0xd9, 0x42, 0x01, 0x63, 0x0d, 0x12, 0xbf, 0x9f};

static const uint8_t ancs_control_point_uuid[] =
{0xd9, 0xd9, 0xaa, 0xfd, 0xbd, 0x9b, 0x21, 0x98,0xa8, 0x49, 0xe1, 0x45, 0xf3, 0xd8, 0xd1, 0x69};

static const uint8_t ancs_data_source_uuid[] =
{0xfb, 0x7b, 0x7c, 0xce, 0x6a, 0xb3, 0x44, 0xbe,0xb5, 0x4b, 0xd6, 0x24, 0xe9, 0xc6, 0xea, 0x22};


/**@brief  ANCS all service handle */
static ancs_c_att_handle_t ancs_c_att_handle;

 /*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief To access phone's all service about ANCS
 *
 * @param[in] conn_idx: Connection index
 *****************************************************************************************
*/
void ancs_c_discovery_service(uint8_t conn_idx)
{
    ble_uuid_t uuid = 
    {
        .uuid_len = BLE_ATT_UUID_128_LEN,
        .uuid = (uint8_t*)ancs_service_uuid,
    };
    ble_gattc_services_browse(conn_idx, &uuid);
}

/**
 *****************************************************************************************
 * @brief Handles reception of the attribute info request message.
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_browse_srvc: Pointer to the parameters of the browse services.
 *****************************************************************************************
 */
void ancs_c_on_browse_svc_evt(uint8_t conn_idx, const ble_gattc_browse_srvc_t *p_browse_srvc)
{
    uint8_t notify_source_index = 0;   //for find cccd, cccd is below of character value
    uint8_t data_source_index = 0; 
    if(p_browse_srvc->uuid_len == BLE_ATT_UUID_128_LEN)
    {
        if(memcmp(p_browse_srvc->uuid, ancs_service_uuid, BLE_ATT_UUID_128_LEN) == 0)//find ancs service
        {
            ancs_c_att_handle.ancs_service_handle = p_browse_srvc->start_hdl;
            uint16_t i;
            for(i=0; i<(p_browse_srvc->end_hdl - p_browse_srvc->start_hdl); i++)//find characteristic
            {
                if(p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_ATTR_VAL)
                {
                   if(memcmp(p_browse_srvc->info[i].attr.uuid, ancs_notification_source_uuid, BLE_ATT_UUID_128_LEN) == 0)//find ancs notification source characteristic
                   {
                        notify_source_index= i;
                        ancs_c_att_handle.ancs_notification_source_handle = ancs_c_att_handle.ancs_service_handle + i;
                   } 
                   else if(memcmp(p_browse_srvc->info[i].attr.uuid, ancs_control_point_uuid, BLE_ATT_UUID_128_LEN) == 0)//find ancs control point characteristic
                   {
                        ancs_c_att_handle.ancs_control_point_handle = ancs_c_att_handle.ancs_service_handle + i;
                   }
                   else if(memcmp(p_browse_srvc->info[i].attr.uuid, ancs_data_source_uuid, BLE_ATT_UUID_128_LEN) == 0)//find ancs data source characteristic
                   {
                        data_source_index = i;
                        ancs_c_att_handle.ancs_data_source_handle = ancs_c_att_handle.ancs_service_handle + i;
                   }
                }
                else if((p_browse_srvc->info[i].attr_type == BLE_GATTC_BROWSE_ATTR_DESC) &&   //find cccd
                        ((*(uint16_t *)(p_browse_srvc->info[i].attr.uuid)) == BLE_ATT_DESC_CLIENT_CHAR_CFG))
                {
                    if(i == (notify_source_index + 1))// find notification source cccd
                    {
                        ancs_c_att_handle.ancs_notify_source_cccd_handle = ancs_c_att_handle.ancs_service_handle + i;
                    }
                    else if(i == (data_source_index + 1))// find data source cccd
                    {
                        ancs_c_att_handle.ancs_data_source_cccd_handle = ancs_c_att_handle.ancs_service_handle + i;
                    }
                }   
            } 
        }
    }
}

/**
 *****************************************************************************************
 * @brief  enable ancs notification source CCCD
 *
 * @param[in] conn_idx: Connection index. 
 * @return    success or not..
 *****************************************************************************************
 */
uint8_t ancs_c_notif_source_notif_enable(uint8_t conn_idx)
{
   uint16_t cccd_value = 0x0001;
   if(ble_gattc_write(conn_idx, ancs_c_att_handle.ancs_notify_source_cccd_handle+1,0,sizeof(uint16_t),(uint8_t *)&cccd_value) == BLE_SUCCESS)
   {
        return SDK_SUCCESS;
   }
   return SDK_ERR_APP_ERROR;
}

/**
 *****************************************************************************************
 * @brief  enable ancs data source CCCD
 *
 * @param[in] conn_idx: Connection index.
 * @return    success or not.
 *****************************************************************************************
 */
uint8_t ancs_c_data_source_notif_enable(uint8_t conn_idx)
{
   uint16_t cccd_value = 0x0001;
   if(ble_gattc_write(conn_idx, ancs_c_att_handle.ancs_data_source_cccd_handle+1,0,sizeof(uint16_t),(uint8_t *)&cccd_value) == BLE_SUCCESS)
   {
       return SDK_SUCCESS;
   }
   return SDK_ERR_APP_ERROR;
}

/**
 *****************************************************************************************
 * @brief  disable ancs notification source CCCD
 *
 * @param[in] conn_idx: Connection index.
 * @return    success or not.
 *****************************************************************************************
 */
uint8_t ancs_c_notif_source_notif_disable(uint8_t conn_idx)
{
   uint16_t cccd_value = 0x0000;
   if(ble_gattc_write(conn_idx, ancs_c_att_handle.ancs_notify_source_cccd_handle+1,0,sizeof(uint16_t),(uint8_t *)&cccd_value) == BLE_SUCCESS)
   {
       return SDK_SUCCESS;
   }
   return SDK_ERR_APP_ERROR;
}

/**
 *****************************************************************************************
 * @brief  disable ancs data source CCCD
 *
 * @param[in] conn_idx: Connection index.
 * @return    success or not.
 *****************************************************************************************
 */
uint8_t ancs_c_data_source_notif_disable(uint8_t conn_idx)
{
   uint16_t cccd_value = 0x0000;
   if(ble_gattc_write(conn_idx, ancs_c_att_handle.ancs_data_source_cccd_handle+1,0,sizeof(uint16_t),(uint8_t *)&cccd_value) == BLE_SUCCESS)
   {
        return SDK_SUCCESS;
   }
   return SDK_ERR_APP_ERROR;
}

/**
 *****************************************************************************************
 * @brief dispatch all notify message from notification source channel
 *
 * @param[in] conn_idx:   Connection index.
 * @param[in] p_ntf_ind: Pointer to the parameters of the Notification and Indication 
 *                        value indication.
 *****************************************************************************************
 */
void ancs_c_gattc_notify_dispatch(uint8_t conn_idx, const ble_gattc_ntf_ind_t *p_ntf_ind) 
{
    if(p_ntf_ind->type == BLE_GATT_NOTIFICATION)
    {
        if(p_ntf_ind->handle == ancs_c_att_handle.ancs_notify_source_cccd_handle)
        {
            ancs_decode_notification_source(p_ntf_ind->p_value, p_ntf_ind->length);
        }
        else if(p_ntf_ind->handle == ancs_c_att_handle.ancs_data_source_cccd_handle)
        {
            ancs_decode_data_source(p_ntf_ind->p_value, p_ntf_ind->length); 
        }
    }
}

/**
 *****************************************************************************************
 * @brief This function implements writing commands to control points
 *
 * @param[in] conn_idx: Connection index.
 * @param[in] p_data:   Pointer to send out data
 * @param[in] length:   Length of data sent out
 *****************************************************************************************
 */
uint8_t ancs_c_write_control_point(uint8_t conn_idx, uint8_t *p_data, uint16_t length)
{
   if (ble_gattc_write(conn_idx, ancs_c_att_handle.ancs_control_point_handle+1, 0, length, p_data) == BLE_SUCCESS)
   {
       return SDK_SUCCESS;
   }
   return SDK_ERR_APP_ERROR;
}

/**
 *****************************************************************************************
 * @brief Initialization of ANCS global structure
 *****************************************************************************************
 */
void ancs_c_init(void)
{
    memset(&ancs_c_att_handle, 0, sizeof(ancs_c_att_handle_t));
}


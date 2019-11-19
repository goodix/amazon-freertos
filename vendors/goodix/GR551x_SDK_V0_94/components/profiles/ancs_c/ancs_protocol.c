/**
 *******************************************************************************
 *
 * @file ancs_protocol.c
 *
 * @brief ANCS protocal implementation.
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
 *****************************************************************************************
 */


#include "ancs_protocol.h"
#include "ancs_c.h"
#include <stdio.h>
#include <string.h>
#include "app_log.h"
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint32_t g_phone_call_uid;

/**@brief String literals for the iOS notification Category id types. Used then printing to UART. */
static char const * lit_catid[] =
{
    "Other",
    "Incoming Call",
    "Missed Call",
    "Voice Mail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "Health And Fitness",
    "Business And Finance",
    "Location",
    "Entertainment"
};

/**@brief String literals for the iOS notification event types. Used then printing to UART. */
static char const * lit_eventid[] =
{
    "Added",
    "Modified",
    "Removed"
};

/**@brief String literals for the iOS notification attribute types. Used when printing to UART. */
static char const * lit_attrid[] =
{
    "App Identifier",
    "Title",
    "Subtitle",
    "Message",
    "Message Size",
    "Date",
    "Positive Action Label",
    "Negative Action Label"
};

/**
 *****************************************************************************************
 * @brief Decode data source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: Length of read data
 *
 *****************************************************************************************
 */
void ancs_decode_data_source(uint8_t *p_data, uint16_t length)
{
    int idx ;
    char CommdId = p_data[0];
    int UID;
    char attrId = 0;
    short attrSize = 0;
    memcpy(&UID, &p_data[1], 4);
    memcpy(&attrSize, &p_data[6], 2);

    if (CommdId == CTRL_POINT_GET_NOTIFICATION_ATTRIBUTES)
    {
        attrId = p_data[5];
        APP_LOG_INFO("UID=%d, ATTR_ID:  %s", UID, lit_attrid[attrId]);
        for (idx=0; idx<attrSize; idx ++)
        {
            printf("%c", p_data[8+idx]);
        }
        APP_LOG_INFO("");
    }
    else if (CommdId == CTRL_POINT_GET_APP_ATTRIBUTES)
    {
        for (idx=0; idx<attrSize; idx ++)
        {
            printf("%c", p_data[idx]);
        }
        APP_LOG_INFO("");
    }
}

/**
 *****************************************************************************************
 * @brief Get notification attribute
 *
 * @param[in] uid: The uid of notify message
 * @param[in] noti_attr: The notification attribute 
 *
 *****************************************************************************************
 */
void ancs_notify_attr_get(int uid, char noti_attr)
{
    int len = 0;
    uint8_t buf[8];
    buf[0] = CTRL_POINT_GET_NOTIFICATION_ATTRIBUTES;
    memcpy(&buf[1], &uid, 4);
    buf[5] = noti_attr;
    len = 100;
    buf[6] = (len & 0xff);
    buf[7] = (len>>8) & 0xff;
    ancs_c_write_control_point(0, buf, 8);
}

/**
 *****************************************************************************************
 * @brief ancs perform action  
 *
 * @param[in] uid: The uid of notify message
 * @param[in] action: The action status defined by specification 
 *****************************************************************************************
 */
void ancs_action_perform(int uid, int action)
{
    uint8_t buf[6];
    buf[0] = CTRL_POINT_PERFORM_NOTIFICATION_ACTION;
    memcpy(&buf[1], &uid, 4);
    buf[5] = action;
    ancs_c_write_control_point(0, buf, 6);
} 

/**
 *****************************************************************************************
 * @brief print notification content with format
 *
 * @param[in] p_notif: Pointer to the parameters of the notification source data buffer
 *****************************************************************************************
 */
static void notification_content_print(notification_source_pdu_t *p_notif)
{
    APP_LOG_INFO("\r\nNotification");
    APP_LOG_INFO("Event:       %s", lit_eventid[p_notif->event_id]);
    APP_LOG_INFO("Category ID: %s", lit_catid[p_notif->category_id]);
    APP_LOG_INFO("Category Cnt:%u", (unsigned int) p_notif->category_count);
    APP_LOG_INFO("UID:         %u", (unsigned int) p_notif->notification_uid);
    APP_LOG_INFO("Flags: ");
    if (p_notif->event_flags.silent == 1)
    {
        APP_LOG_INFO(" Silent");
    }
    if (p_notif->event_flags.important == 1)
    {
        APP_LOG_INFO(" Important");
    }
    if (p_notif->event_flags.pre_existing == 1)
    {
        APP_LOG_INFO(" Pre-existing");
    }
    if (p_notif->event_flags.positive_action == 1)
    {
        APP_LOG_INFO(" Positive Action");
    }
    if (p_notif->event_flags.negative_action == 1)
    {
        APP_LOG_INFO(" Negative Action");
    }
}

/**
 *****************************************************************************************
 * @brief set phone call notify message uid
 *
 * @param[in] uid: the uid of notify message
 *****************************************************************************************
 */
static void ancs_set_phone_call_uid(int uid)
{
    g_phone_call_uid = uid;
}

/**
 *****************************************************************************************
 * @brief get ancs phone call uid
 *
 * @return phone call notify message uid
 *****************************************************************************************
 */
int ancs_get_phone_call_uid(void)
{
    return g_phone_call_uid;
}

/**
 *****************************************************************************************
 * @brief Decode notification source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: The Length of read value
 *****************************************************************************************
 */
void ancs_decode_notification_source(uint8_t *p_data, uint16_t length)
{
    notification_source_pdu_t *pdu = (notification_source_pdu_t*)p_data;
    notification_content_print(pdu);

    if (pdu->category_id == BLE_ANCS_CATEGORY_ID_INCOMING_CALL)
    {
        ancs_set_phone_call_uid((unsigned int) pdu->notification_uid);
    }
}

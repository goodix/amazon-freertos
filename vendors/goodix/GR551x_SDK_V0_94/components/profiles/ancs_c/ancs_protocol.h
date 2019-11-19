/**
 *******************************************************************************
 *
 * @file ancs_protocol.h
 *
 * @brief ANCS Protocol API.
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
#ifndef _ANCS_PROTOCOL_H_
#define _ANCS_PROTOCOL_H_

#include "gr55xx_sys.h"
#include <stdint.h>

/** Maximum allowed value for attribute length */
#ifndef CFG_ANCS_ATTRIBUTE_MAXLEN
#define CFG_ANCS_ATTRIBUTE_MAXLEN 128
#endif

/** Attribute ID element without maximum length */
#define ANCS_ATTR(ID) ((uint32_t) 0x80000000 | ((uint8_t) ID))

/** Attribute ID element with maximum length */
#define ANCS_ATTR_MAXLEN(ID, LEN) ((uint32_t) 0x80000000 | ((uint8_t) ID) | ((uint16_t) LEN << 8))


/**
 * @defgroup ANCS_ENUM Enumerations
 * @{
 */
/**@brief IDs for iOS notification attributes. */
typedef enum 
{
    ANCS_NOTIF_ATTR_ID_APP_IDENTIFIER = 0,     /**< Identifies that the attribute data is of an "App Identifier" type. */
    ANCS_NOTIF_ATTR_ID_TITLE,                  /**< Identifies that the attribute data is a "Title". */
    ANCS_NOTIF_ATTR_ID_SUBTITLE,               /**< Identifies that the attribute data is a "Subtitle". */
    ANCS_NOTIF_ATTR_ID_MESSAGE,                /**< Identifies that the attribute data is a "Message". */
    ANCS_NOTIF_ATTR_ID_MESSAGE_SIZE,           /**< Identifies that the attribute data is a "Message Size". */
    ANCS_NOTIF_ATTR_ID_DATE,                   /**< Identifies that the attribute data is a "Date". */
    ANCS_NOTIF_ATTR_ID_POSITIVE_ACTION_LABEL,  /**< The notification has a "Positive action" that can be executed associated with it. */
    ANCS_NOTIF_ATTR_ID_NEGATIVE_ACTION_LABEL,  /**< The notification has a "Negative action" that can be executed associated with it. */
} ancs_notification_attr_t;

/**@brief Event types that are passed from client to application on an event. */
typedef enum
{
    BLE_ANCS_C_EVT_DISCOVERY_COMPLETE,         /**< A successful connection has been established and the service was found on the connected peer. */
    BLE_ANCS_C_EVT_DISCOVERY_FAILED,           /**< It was not possible to discover the service or characteristics of the connected peer. */
    BLE_ANCS_C_EVT_NOTIF,                      /**< An iOS notification was received on the notification source control point. */
    BLE_ANCS_C_EVT_INVALID_NOTIF,              /**< An iOS notification was received on the notification source control point, but the format is invalid. */
    BLE_ANCS_C_EVT_NOTIF_ATTRIBUTE,            /**< A received iOS notification attribute has been parsed. */
    BLE_ANCS_C_EVT_APP_ATTRIBUTE,              /**< An iOS app attribute has been parsed. */
    BLE_ANCS_C_EVT_NP_ERROR,                   /**< An error has been sent on the ANCS Control Point from the iOS Notification Provider. */
} ble_ancs_c_evt_type_t;

/**@brief Category IDs for iOS notifications. */
typedef enum
{
    BLE_ANCS_CATEGORY_ID_OTHER,                /**< The iOS notification belongs to the "other" category.  */
    BLE_ANCS_CATEGORY_ID_INCOMING_CALL,        /**< The iOS notification belongs to the "Incoming Call" category. */
    BLE_ANCS_CATEGORY_ID_MISSED_CALL,          /**< The iOS notification belongs to the "Missed Call" category. */
    BLE_ANCS_CATEGORY_ID_VOICE_MAIL,           /**< The iOS notification belongs to the "Voice Mail" category. */
    BLE_ANCS_CATEGORY_ID_SOCIAL,               /**< The iOS notification belongs to the "Social" category. */
    BLE_ANCS_CATEGORY_ID_SCHEDULE,             /**< The iOS notification belongs to the "Schedule" category. */
    BLE_ANCS_CATEGORY_ID_EMAIL,                /**< The iOS notification belongs to the "E-mail" category. */
    BLE_ANCS_CATEGORY_ID_NEWS,                 /**< The iOS notification belongs to the "News" category. */
    BLE_ANCS_CATEGORY_ID_HEALTH_AND_FITNESS,   /**< The iOS notification belongs to the "Health and Fitness" category. */
    BLE_ANCS_CATEGORY_ID_BUSINESS_AND_FINANCE, /**< The iOS notification belongs to the "Buisness and Finance" category. */
    BLE_ANCS_CATEGORY_ID_LOCATION,             /**< The iOS notification belongs to the "Location" category. */
    BLE_ANCS_CATEGORY_ID_ENTERTAINMENT         /**< The iOS notification belongs to the "Entertainment" category. */
} ble_ancs_c_category_id_val_t;

/**@brief Event IDs for iOS notifications. */
typedef enum
{
    BLE_ANCS_EVENT_ID_NOTIFICATION_ADDED,      /**< The iOS notification was added. */
    BLE_ANCS_EVENT_ID_NOTIFICATION_MODIFIED,   /**< The iOS notification was modified. */
    BLE_ANCS_EVENT_ID_NOTIFICATION_REMOVED     /**< The iOS notification was removed. */
} ble_ancs_c_evt_id_values_t;

/**@brief Control point command IDs that the Notification Consumer can send to the Notification Provider. */
typedef enum
{
    BLE_ANCS_COMMAND_ID_GET_NOTIF_ATTRIBUTES,      /**< Requests attributes to be sent from the NP to the NC for a given notification. */
    BLE_ANCS_COMMAND_ID_GET_APP_ATTRIBUTES,        /**< Requests attributes to be sent from the NP to the NC for a given iOS app. */
    BLE_ANCS_COMMAND_ID_GET_PERFORM_NOTIF_ACTION,  /**< Requests an action to be performed on a given notification, for example, dismiss an alarm. */
} ble_ancs_c_cmd_id_val_t;

/**@brief ID for actions that can be performed for iOS notifications. */
typedef enum
{
    ACTION_ID_POSITIVE = 0,                    /**< Positive action. */
    ACTION_ID_NEGATIVE                         /**< Negative action. */
} ble_ancs_c_action_id_values_t;

/**@brief notification flags that can be performed for iOS notifications. */
typedef struct
{
    uint8_t silent          : 1;               //!< If this flag is set, the notification has a low priority.
    uint8_t important       : 1;               //!< If this flag is set, the notification has a high priority.
    uint8_t pre_existing    : 1;               //!< If this flag is set, the notification is pre-existing.
    uint8_t positive_action : 1;               //!< If this flag is set, the notification has a positive action that can be taken.
    uint8_t negative_action : 1;               //!< If this flag is set, the notification has a negative action that can be taken.
} ble_ancs_c_notif_flags_t;

/**@brief ctrl point command that can be performed for iOS notifications. */
enum ctrl_point
{
    CTRL_POINT_GET_NOTIFICATION_ATTRIBUTES = 0,           /**< Requests attributes to be sent from the NP to the NC for a given notification. */
    CTRL_POINT_GET_APP_ATTRIBUTES,                        /**< Requests attributes to be sent from the NP to the NC for a given iOS app. */
    CTRL_POINT_PERFORM_NOTIFICATION_ACTION,               /**< Requests an action to be performed on a given notification, for example, dismiss an alarm. */
};
/** @} */

/**
 * @defgroup ANCS_STRUCT Structures
 * @{
 */
/**@brief iOS notification structure. */
typedef struct 
{
    ble_ancs_c_evt_id_values_t event_id;          //!< Whether the notification was added, removed, or modified.
    ble_ancs_c_notif_flags_t event_flags;         //!< Whether the notification was added, removed, or modified.
    ble_ancs_c_category_id_val_t category_id;     //!< Classification of the notification type, for example, email or location.
    uint8_t category_count;                       //!< Current number of active notifications for this category ID.
    uint32_t notification_uid;                    //!< Notification UID.
} notification_source_pdu_t;
/** @} */

/**
 * @defgroup ANCS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Get notification attribute
 *
 * @param[in] uid: The uid of notify message
 * @param[in] noti_attr: The notification attribute 
 *
 *****************************************************************************************
 */
void ancs_notify_attr_get(int uid, char noti_attr);

/**
 *****************************************************************************************
 * @brief ancs perform action  
 *
 * @param[in] uid: The uid of notify message
 * @param[in] action: The action status defined by specification 
 *
 *****************************************************************************************
 */
void ancs_action_perform(int uid, int action);

/**
 *****************************************************************************************
 * @brief get ancs phone call uid
 *
 * @return phone call notify message uid
 *****************************************************************************************
 */
int ancs_get_phone_call_uid(void);

/**
 *****************************************************************************************
 * @brief Decode notification source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: The Length of read value
 *****************************************************************************************
 */
void ancs_decode_notification_source(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Decode data source message.
 *
 * @param[in] p_data: Pointer to the parameters of the read request.
 * @param[in] length: Length of read data
 *****************************************************************************************
 */
void ancs_decode_data_source(uint8_t *p_data, uint16_t length);
/** @} */
#endif



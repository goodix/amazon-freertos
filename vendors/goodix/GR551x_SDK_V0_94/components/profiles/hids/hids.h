/**
 *******************************************************************************
 *
 * @file hids.h
 *
 * @brief Human Interface Device Service API
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */

/**
 * @defgroup BLE_SDK_HIDS Human Input Device Service (HIDS)
 * @{
 * @brief Definitions and prototypes for the HIDS interface.
 *
 * @details The HID Service exposes data and associated formatting for HID Devices
 *          and HID Hosts. This module implements the HID Service with HID Information
 *          characteristic, HID Control Point characteristic, Report Map characteristic,
 *          Input/Output/Feature Report characteristics, Boot Keyboard Input characteristic,
 *          Boot Keyboard Output characteristic, Boot Mouse Input Report characteristic.
 *
 *          After \ref hids_init_t variable is initialed, the application must call \ref hids_service_init()
 *          to add the HID Service and the characteristics to the BLE Stack database. However
 *          the array of Report map locates in user space, application must make sure the
 *          array is available.
 *
 *          If Notify is enabled, the value of Input Report characteristic is sent to the
 *          peer when application calls \ref hids_input_report_1_send() function. The application is reponsible 
 *          for encoding Input Report data as “USB HID Spec”. If an event hanlder is provided by the application,
 *          HID Service will pass HIDS events to the application, e.g. Output Report characteristic is writtern.
 */

#ifndef __HIDS_H__
#define __HIDS_H__

#include "ble_prf_utils.h"
#include "gr55xx_sys.h"
#include "custom_config.h"
#include <stdint.h>

/**
 * @defgroup HIDS_MACRO Defines
 * @{
 */
#define INPUT_REPORT_COUNT          1                   /**< The count may be 1 or 2. */

/**
 * @defgroup HIDS_CHAR_MASK Characteristic Mask
 * @{
 * @brief Bit masks for the initialization of \ref hids_init_t.char_mask.
 */
#if INPUT_REPORT_COUNT == 1
    #define HIDS_CHAR_MOUSE         0x07C007FF          /**< Bit mask of the mouse with one input report. */
#else
    #define HIDS_CHAR_MOUSE         0x7C007FFF          /**< Bit mask of the mouse with two input reports. */
#endif

#define HIDS_CHAR_KEYBOARD          0x063E3FFF          /**< Bit mask of the keyboard which supports boot mode. */
#define HIDS_CHAR_KEYPAD            0x000007FF          /**< Bit mask of the keypad which doesn't support boot mode. */
#define HIDS_CHAR_FEATURE_REPORT    0x0001C000          /**< Bit mask of the feature report. */
/** @} */

// HID information Flags
#define HIDS_INFO_FLAG_REMOTE_WAKE_MSK          0x01    /**< Bit mask of Remote Wake flag in HIDS information. */
#define HIDS_INFO_FLAG_NORMALLY_CONNECTABLE_MSK 0x02    /**< Bit mask of Normally Connectable flag in HIDS information. */

// Maximum length of the various report
#define HIDS_INPUT_REPORT_MAX_LEN       8               /**< Maximum length of input report. */
#define HIDS_OUTPUT_REPORT_MAX_LEN      8               /**< Maximum length of output report. */
#define HIDS_FEATURE_REPORT_MAX_LEN     8               /**< Maximum length of feature report. */

// Protocol Mode values
#define HIDS_PROTOCOL_MODE_BOOT         0x00            /**< The value of Boot protocol mode. */
#define HIDS_PROTOCOL_MODE_REPORT       0x01            /**< The value of Report protocol mode. */

// HID Control Point values
#define HIDS_CONTROL_POINT_SUSPEND      0x00            /**< The value of SUSPEND for HID Control Point. */
#define HIDS_CONTROL_POINT_EXIT_SUSPEND 0x01            /**< The value of EXIT SUSPEND for HID Control Point. */

#define HIDS_DEFAULT_PROTOCOL_MODE      HIDS_PROTOCOL_MODE_REPORT       /**< The default protocol mode for HIDS. */
#define HIDS_INIT_VALUE_CONTROL_POINT   HIDS_CONTROL_POINT_SUSPEND      /**< The initial value for HID Control Point. */

#define BOOT_KB_IN_REPORT_MAX_SIZE      8   /**< Maximum size of Boot Keyboard Input Report, as per Appendix B in Device
                                             *   Class Definition for HID, version 1.11. */
#define BOOT_KB_OUT_REPORT_MAX_SIZE     1   /**< Maximum size of Boot Keyboard Output Report, as per Appendix B in Device
                                             *   Class Definition for HID, version 1.11. */
#define BOOT_MOUSE_IN_REPORT_MIN_SIZE   3   /**< Minimum size of Boot Mouse Input Report, as per Appendix B in Device
                                             *   Class Definition for HID, version 1.11. */
#define BOOT_MOUSE_IN_REPORT_MAX_SIZE   8   /**< Maximum size of Boot Mouse Input Report, as per Appendix B in Device
                                             *   Class Definition for HID, version 1.11. */

#define REPORT_MAP_MAX_SIZE             512 /**< Limitation of length, as per Section 2.6.1 in HIDS Spec, version 1.0 */
/** @} */

/**
 * @defgroup HIDS_ENUM Enumerations
 * @{
 */
/**@brief HID Service report type. */
typedef enum
{
    HIDS_REPORT_TYPE_RESERVED,              /**< The reserved report type. */
    HIDS_REPORT_TYPE_INPUT,                 /**< The input report type. */
    HIDS_REPORT_TYPE_OUTPUT,                /**< The output report type. */
    HIDS_REPORT_TYPE_FEATURE,               /**< The feature report type. */
} hids_report_type_t;

/**@brief HID Service event type. */
typedef enum
{
    HIDS_EVT_INVALID,                       /**< Invalid event. */
    HIDS_EVT_HOST_SUSP,                     /**< Suspend command received. */
    HIDS_EVT_HOST_EXIT_SUSP,                /**< Exit suspend command received. */
    HIDS_EVT_NOTIFY_ENABLED,                /**< Notification enabled event. */
    HIDS_EVT_NOTIFY_DISABLED,               /**< Notification disabled event. */
    HIDS_EVT_REP_CHAR_WRITE,                /**< New value has been writtern to a report characteristic */
    HIDS_EVT_BOOT_MODE_ENTERED,             /**< Boot mode entered */
    HIDS_EVT_REPORT_MODE_ENTERED,           /**< Report mode entered */
} hids_evt_type_t;

/**@brief HID Service CCCD types */
typedef enum
{
    HIDS_CCCD_INPUT_1,                      /**< The CCCD of Input Report 1. */
    HIDS_CCCD_INPUT_2,                      /**< The CCCD of Input Report 2. */
    HIDS_CCCD_BOOT_KEYBOARD_IN,             /**< The CCCD of Boot Keyboard Input Report. */
    HIDS_CCCD_BOOT_MOUSE_IN,                /**< The CCCD of Boot Mouase Input Report. */
} hids_cccd_type_t;
/** @} */

/**
 * @defgroup HIDS_TYPEDEF Typedefs
 * @{
 */
/**@brief HID service characteristic id. */
typedef struct
{
    uint16_t uuid;                  /**< UUID of characteristic. */
    uint8_t  report_type;           /**< Type of report, see @ref hids_report_type_t. */
} hids_char_id_t;

/**@brief HID Service event. */
typedef struct
{
    hids_evt_type_t evt_type;       /**< Type of event. */
    union
    {
        struct
        {
            hids_char_id_t char_id; /**< Id of characteristic for which notification has been started. */
        } notification;
        struct
        {
            hids_char_id_t char_id; /**< Id of characteristic having been written. */
            uint16_t       offset;  /**< Offset for the write operation. */
            uint16_t       length;  /**< Length of the incoming data. */
            const uint8_t *data;    /**< Incomming data, variable length. */
        } char_write;
    } params;
} hids_evt_t;

/**@brief HID Service event handler type. 
 *
 * @param[in] p_evt Pointer to a HID Service event variable.
 */
typedef void (*hids_evt_handler_t)(hids_evt_t *p_evt);

/**@brief HID information characteristic value. */
typedef struct
{
    uint16_t bcd_hid;               /**< Version of USB HID spec. */
    uint8_t  b_country_code;        /**< Identify which country the device is localized for. */
    uint8_t  flags;                 /**< bit0-RemoteWake
                                     *   bit1-NormallyConnectable
                                     *   others-reserved. */
} hids_hid_info_t;

/**@brief HID Service Included services. */
typedef struct
{
    uint16_t **inc_srv_hdl_ptr;     /**< Pointer to array of pointers to Included Service start handle. */
    uint8_t    inc_srv_num;         /**< Number of services to include in HID service. */
} hids_included_srv_t;

/**@brief HID Service Report Reference value. */
typedef struct
{
    uint8_t report_id;              /**< The id of report reference. */
    uint8_t report_type;            /**< The type of report reference. */
} hids_report_ref_t;

/**@brief HID Service Report Map characteristic value. */
typedef struct
{
    uint8_t *p_map;                 /**< Pointer to the report map. */
    uint16_t map_len;               /**< The length of report map. */
} hids_report_map_t;

/**@brief HID Service initialization variable. */
typedef struct 
{
    hids_evt_handler_t  evt_handler;                            /**< Handle events in HID Service. */
    prf_error_handler_t err_handler;                            /**< Handle error in HID Service. */

    hids_hid_info_t     hid_info;                               /**< Value of HID information characteristic. */
    hids_included_srv_t included_srv;                           /**< HID Service Included services. */
    hids_report_map_t   report_map;                             /**< HID Service Report Map characteristic value. */
    hids_report_ref_t   input_report_refs[INPUT_REPORT_COUNT];  /**< HID input Report Reference value. */
    hids_report_ref_t   output_report_ref;                      /**< HID output Report Reference value. */
    hids_report_ref_t   feature_report_ref;                     /**< HID feature Report Reference value. */
    uint32_t            char_mask;                              /**< Attribues in hids_attr_tab to add into ATT database, see @ref HIDS_CHAR_MASK */
} hids_init_t;
/** @} */

/**
 * @defgroup HIDS_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Initialize a HID Service instance in ATT DB.
 *
 * @param[in] p_hids_init: Pointer to a HID Service initialization variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t hids_service_init(hids_init_t *p_hids_init);

/**
 *****************************************************************************************
 * @brief Send an input report.
 *
 * @param[in] length: Length of data to be sent.
 * @param[in] p_data: Pointer to data to be sent.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_input_report_1_send(uint16_t length, uint8_t *p_data);

#if INPUT_REPORT_COUNT > 1
/**
 *****************************************************************************************
 * @brief Send an input report.
 *
 * @param[in] length: Length of data to be sent.
 * @param[in] p_data: Pointer to data to be sent.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_input_report_2_send(uint16_t length, uint8_t *p_data);
#endif

/**
 *****************************************************************************************
 * @brief Send boot keyboard input report.
 *
 * @param[in] length: Length of data to be sent.
 * @param[in] p_data: Pointer to data to be sent.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_boot_kb_input_report_send(uint16_t length, uint8_t *p_data);

/**
 *****************************************************************************************
 * @brief Send boot mouse input report.
 *
 * @param[in] btns:         State of mouse buttons.
 * @param[in] x_delta:      Horizontal movement.
 * @param[in] y_delta:      Vertical movement.
 * @param[in] opt_data_len: Length of optional part of boot mouse input report.
 * @param[in] p_opt_data:   Pointer to optional part of boot mouse input report.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_boot_mouse_input_report_send(uint8_t btns,
                                            int8_t x_delta,
                                            int8_t y_delta,
                                            uint16_t opt_data_len,
                                            uint8_t *p_opt_data);

/**
 *****************************************************************************************
 * @brief Get the current value of output report from BLE stack.
 *
 * @param[in]  length:          Length of buffer for output report.
 * @param[out] p_output_report: Pointer to the output report.
 *
 * @return BLE_SDK_SUCCESS on success, otherwise an error code.
 *****************************************************************************************
 */
sdk_err_t hids_output_report_get(uint16_t length, uint8_t *p_output_report);
/** @} */

#endif
/** @} */
/** @} */


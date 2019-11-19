/**
 *******************************************************************************
 *
 * @file ble_prf_types.h
 *
 * @brief Profile/Service Common Types
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
  @defgroup BLE_PRF_TYPE Profile/Service Common Types
  @{
  @brief  Definitions for BLE profile/service common types.
 */

#ifndef __BLE_PRF_TYPES_H__
#define __BLE_PRF_TYPES_H__

#include "gr55xx_sys.h"
#include <stdint.h>

/** @defgroup BLE_MACRO Defines
 * @{
 */
/** Attribute is mandatory. */
#define ATT_MANDATORY           (0xFF)
/** Attribute is optional. */
#define ATT_OPTIONAL            (0x00)
/** Characteristic Presentation Format Descriptor Size. */
#define PRF_CHAR_PRES_FMT_SIZE  (7)
/** The invalid profile handle. */
#define PRF_INVALID_HANDLE      (0x0000)
/** @} */

/** @defgroup BLE_PRF_TYPE_ENUM Enumerations
 * @{
 */
/**@brief The values for setting client configuration characteristics. */
typedef enum
{
    PRF_CLI_STOP_NTFIND = 0x0000,   /**< Stop notification/indication. */
    PRF_CLI_START_NTF,              /**< Start notification. */
    PRF_CLI_START_IND,              /**< Start indication. */
} prf_cli_conf_t;

/**@brief The values for setting server configuration characteristics. */
typedef enum
{
    PRF_SRV_STOP_BCST = 0x0000,     /**< Stop Broadcast. */
    PRF_SRV_START_BCST,             /**< Start Broadcast. */
} prf_srv_conf_t;

/**@brief Connection type. */
typedef enum
{
    PRF_CON_DISCOVERY = 0x00,       /**< Discovery type connection. */
    PRF_CON_NORMAL    = 0x01,       /**< Normal type connection. */
} prf_con_type_t;

/**@brief Service type. */
typedef enum
{
    PRF_PRIMARY_SERVICE   = 0x00,   /**< Profile primary service type. */
    PRF_SECONDARY_SERVICE = 0x01    /**< Profile secondary service type. */
} prf_svc_type_t;
/** @} */

/**
 * @defgroup BLE_PRF_TYPE_TYPEDEF Typedefs
 * @{
 */
/**@brief Function for handling profile/service error. */
typedef void (*prf_error_handler_t)(sdk_err_t err_code);
/** @} */

/** @defgroup BLE_PRF_TYPE_STRUCT Structures
 * @{
 */
/**@brief Characteristic Presentation Format Descriptor structure.
 *        The packed size is \ref PRF_CHAR_PRES_FMT_SIZE. */
typedef struct
{
    uint16_t unit;              /**< Unit (The Unit is a UUID). */
    uint16_t description;       /**< Description. */
    uint8_t  format;            /**< Format. */
    uint8_t  exponent;          /**< Exponent. */
    uint8_t  name_space;        /**< Name space. */
} prf_char_pres_fmt_t;

/**@brief The date and time structure. The packed size is 7 bytes. */
typedef struct
{
    uint16_t year;              /**< year time element. */
    uint8_t  month;             /**< month time element. */
    uint8_t  day;               /**< day time element. */
    uint8_t  hour;              /**< hour time element. */
    uint8_t  min;               /**< minute time element. */
    uint8_t  sec;               /**< second time element. */
} prf_date_time_t;

/**@brief Attribute information. */
typedef struct
{
    uint16_t  handle;                /**< Attribute Handle. */
    uint16_t  offset;                /**< Offset of the attribute value . */
    uint16_t  length;                /**< Attribute length. */
    uint8_t   status;                /**< Status of request. */
    uint8_t  *p_data;  /**< Attribute value. */
} prf_att_info_t;

/**@brief Service handles. */
typedef struct
{
    uint16_t shdl;      /**< Start handle of a service. */
    uint16_t ehdl;      /**< End handle of a service. */
} prf_svc_t;

/**@brief Included Service info. */
typedef struct
{
    uint16_t handle;                       /**< Attribute handle. */
    uint16_t start_hdl;                    /**< Included service start handle. */
    uint16_t end_hdl;                      /**< Included service end handle. */
    uint8_t  uuid_len;                     /**< UUID length. */
    uint8_t  uuid[BLE_ATT_UUID_128_LEN];   /**< UUID. */
} prf_incl_svc_t;

/**@brief Characteristic info. */
typedef struct
{
    uint16_t char_hdl;          /**< Characteristic handle. */
    uint16_t val_hdl;           /**< Value handle. */
    uint8_t  prop;              /**< Characteristic properties. */
    uint8_t  char_ehdl_off;     /**< End of characteristic offset. */
} prf_char_inf_t;

/**@brief Characteristic description. */
typedef struct
{
    uint16_t desc_hdl;          /**< Descriptor handle. */
} prf_char_desc_inf_t;

/**@brief Characteristic definition. */
typedef struct
{
    uint16_t uuid;              /**< Characteristic UUID. */
    uint8_t  req_flag;          /**< Requirement Attribute Flag. */
    uint8_t  prop_mand;         /**< Mandatory Properties. */
} prf_char_def_t;

/**@brief Characteristic Descriptor definition. */
typedef struct
{
    uint16_t uuid;              /**< Characteristic Descriptor uuid. */
    uint8_t  req_flag;          /**< Requirement attribute flag. */
    uint8_t  char_code;         /**< Corresponding characteristic code. */
} prf_char_desc_def_t;

/**@brief  The message structure is used to inform APP that a profile client role
 *         has been disabled. */
typedef struct
{
    uint8_t status;             /**< Status. */
} prf_client_disable_ind_t;

/**@brief  The message structure is used to inform APP that an error has occurred
 *         in the profile server role task. */
typedef struct
{
    uint8_t status;             /**< Status. */
} prf_server_error_ind_t;

/**@brief Slave preferred connection parameters. */
typedef struct
{
    uint16_t con_intv_min;      /**< Connection interval minimum. */
    uint16_t con_intv_max;      /**< Connection interval maximum. */
    uint16_t slave_latency;     /**< Slave latency. */
    uint16_t conn_timeout;      /**< Connection supervision timeout multiplier. */
} gap_slv_pref_t;
/** @} */

#endif /* _BLE_PRF_TYPES_H_ */
/** @} */
/** @} */


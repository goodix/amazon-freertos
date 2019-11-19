/**
 ****************************************************************************************
 *
 * @file ble_gattc_cache.h
 *
 * @brief BLE GATTC API
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
 * All Rights Reserved
 *
 ****************************************************************************************
 */
#if defined(GR551xx_D0)

 /**
 * @addtogroup BLE
 * @{
 */
 
  /**
 * @addtogroup BLE_GATT GATT Cache
 * @{
 * @brief Definitions and prototypes for the GATT Cache interface.
 */

/**
  @addtogroup BLE_SDK_GATTC Generic Attribute Profile (GATT) Client
  @{
  @brief Definitions and prototypes for the GATT client cache interfaces.
 */

#ifndef _BLE_GATTC_CACHE_H_
#define _BLE_GATTC_CACHE_H_

#include "ble_error.h"
#include "ble_gatt.h"
#include "ble_att.h"
#include "ble_gapc.h"
#include <stdint.h>
#include <stdbool.h>


/** @addtogroup BLE_GATTC_CACHE_ENUMERATIONS Enumerations
 * @{ */
/**
 * @brief GATT Client Cache Attribute type IDs.
 */
typedef enum
{
    BLE_GATTC_CACHE_SERV,           /**< Primary Service Declaration. */
    BLE_GATTC_CACHE_INC_SRVC,       /**< Included Service Declaration. */
    BLE_GATTC_CACHE_ATTR_CHAR,      /**< Characteristic Declaration. */
    BLE_GATTC_CACHE_ATTR_DESC,      /**< Characteristic Descriptor. */
} gattc_cache_attr_t;
/** @} */

/** @addtogroup BLE_GATTC_CACHE_STRUCTURES Structures
 * @{ */

/**
 * @brief GATTC service attribute structure.
 */
typedef struct
{   
    uint8_t     uuid_len;                       /**< Service UUID length. */
    uint8_t     uuid[BLE_ATT_UUID_128_LEN];     /**< Service UUID. */
    uint16_t    start_hdl;                      /**< Service start handle. */
    uint16_t    end_hdl;                        /**< Service end handle. */
} ble_gattc_service_attr_t;

/**
 * @brief GATTC include attribute structure.
 */
typedef struct
{   
    uint8_t     uuid_len;                       /**< Included Service UUID length. */
    uint8_t     uuid[BLE_ATT_UUID_128_LEN];     /**< Included Service UUID. */
    uint16_t    start_hdl;                      /**< Included Service start handle. */
    uint16_t    end_hdl;                        /**< Included Service end handle. */
} ble_gattc_include_attr_t;

/**
 * @brief GATTC characteristic attribute structure.
 */
typedef struct
{   
    uint8_t     uuid_len;                       /**< Characteristic UUID length. */
    uint8_t     uuid[BLE_ATT_UUID_128_LEN];     /**< Characteristic UUID. */
    uint8_t     prop;                           /**< Value property. */
    uint16_t    handle_value;                   /**< Handle of the Characteristic Value. */
} ble_gattc_char_attr_t;

/**
 * @brief GATTC characteristic descriptor attribute structure.
 */
typedef struct
{   
    uint8_t     uuid_len;                       /**< Descriptor UUID length. */
    uint8_t     uuid[BLE_ATT_UUID_128_LEN];     /**< Descriptor UUID. */
} ble_gattc_char_desc_attr_t;

/**
 * @brief GATTC cache attribute information uinon.
 */
union attr_cache_info 
{
    ble_gattc_service_attr_t     service_decl;           /**< Service Declaration. */
    ble_gattc_include_attr_t     include_decl;           /**< Include Declaration. */
    ble_gattc_char_attr_t        char_decl;              /**< Characteristic Declaration. */
    ble_gattc_char_desc_attr_t   char_desc;              /**< Characteristic Descriptor. */
};

/**
 * @brief GATTC cache attribute information structure.
 */
typedef struct
{
    uint16_t    handle;                             /**< Attribute Handle. */ 
    uint8_t     attr_type;                          /**< Attribute type. See @ref gattc_cache_attr_t. */
    union       attr_cache_info info;               /**< Attribute cache information. */
} attr_cache_info_t;
/** @} */

/** @addtogroup BLE_GATTC_FUNCTIONS Functions
 * @{ */

/**
 ****************************************************************************************
 * @brief GATTC cache feature enable.
 *
 * @note Callback @ref gattc_cb_fun_t::app_gattc_cache_update_cb will be called to indicate to APP once enabling
         peer cache feature and caching data finished.
 *
 * @param[in] conn_idx:     Current connection index.
 *
 * @retval ::SDK_SUCCESS: Successfully enable cache feature.
 * @retval ::BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 ****************************************************************************************
 */
uint16_t ble_gattc_cache_enable(uint8_t conn_idx);

/**
 ****************************************************************************************
 * @brief GATTC cache data get.
 *
 * @note User should get cache data after gattc_cb_fun_t::app_gattc_cache_update_cb called.
 *       Real cache data length will by returned whether user provide enough buf or not. 
 *
 * @param[in]     conn_idx:      Current connection index.
 * @param[in]     p_cache_data:  The attribute cache buf.
 * @param[in|out] p_cache_count: The count of attribute cache buf.
 *
 * @retval ::SDK_SUCCESS: Successfully get cache attributes info.
 * @retval ::BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 * @retval ::BLE_SDK_ERR_CACHE_NOT_ENABLE: Cache feature is not enabled.
 * @retval ::BLE_SDK_ERR_BUSY: Caching data operation is not finished.
 * @retval ::BLE_SDK_ERR_BUF_LEN_NOT_ENOUGH: The cache buf lenth is not enough.
  ****************************************************************************************
 */
uint16_t ble_gattc_cache_get(uint8_t conn_idx, attr_cache_info_t *p_cache_data, uint16_t *p_cache_count);

/**
 ****************************************************************************************
 * @brief GATTC cache nvds list get.
 *
 * @note Get cache list.Real cache list num will by returned whether user provide enough cache list buf or not. 
 *
 * @param[in]     p_bd_addr_list:    The cache list buf.
 * @param[in/out] p_list_num:    The list size of cache list.
 *
 * @retval ::SDK_SUCCESS: Successfully get cache list.
 * @retval ::BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 ****************************************************************************************
 */
uint16_t ble_gattc_cache_nvds_list_get(gap_bdaddr_t *p_bd_addr_list, uint16_t *p_list_num);

/**
 ****************************************************************************************
 * @brief GATTC cache date delete.
 * 
 * @param[in] p_peer_bd_addr:    Identity address of peer device.
 *
 * @retval ::SDK_SUCCESS: Successfully delete cache in nvds.
 * @retval ::BLE_SDK_ERR_BAD_PARAM: Invalid parameter(s) supplied.
 * @retval ::BLE_SDK_ERR_LIST_ITEM_NOT_FOUND: Item not found in list.
 ****************************************************************************************
 */
uint16_t ble_gattc_cache_delete(gap_bdaddr_t *p_peer_bd_addr);
/** @} */

#endif

/** @} */

/** @} */
/** @} */
#endif


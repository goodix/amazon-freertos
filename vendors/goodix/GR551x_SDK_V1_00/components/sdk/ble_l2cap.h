/**
 ****************************************************************************************
 *
 * @file ble_l2cap.h
 *
 * @brief BLE L2CAP API
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
 * All Rights Reserved
 *
 ****************************************************************************************
 */

  /**
 * @addtogroup BLE
 * @{
 */
 
 /**
  @addtogroup BLE_L2CAP Logical Link Control and Adaptation Protocol (L2CAP)
  @{
  @brief Definitions and prototypes for the L2CAP interface.
 */
 
#ifndef __BLE_L2CAP_H__
#define __BLE_L2CAP_H__

#include "ble_error.h"
#include "gr55xx_sys_cfg.h"
#include <stdint.h>  
#include <stdbool.h>

/**@addtogroup BLE_L2CAP_ENUMERATIONS Enumerations
 * @{ */

/**@brief LE credit based disconnection reasons. */
typedef enum
{
    REMOTE_USER_TERM_CON = 0x00,       /**< Remote user terminates the connection. */
    LOCAL_USER_TERM_CON  = 0x01,       /**< Local user terminates the connection. */
} lecb_disconnect_reason_t;

/** @} */

/** @addtogroup BLE_L2CAP_STRUCTURES Structures
 * @{ */
/** @brief The parameter of LE credit based connection request packet sending. 
  * @note  The le_psm should be registered by the peer device, otherwise the peer device will reject this request with result of LE_PSM not supported.
  * @note  The local_cid should be 0x0040-0x007F. If the local_cid is set to 0, the stack will assign it dynamically. 
  * @note  The local_credit is required to be sure that at least one SDU can be received, otherwise the stack will use the default value: (MTU  + MPS + 1) /MPS + 1.
  * @note  The MTU range is [23~max_mtu].
  * @note  The MPS range is [23~max_mps].
  * @note  About max_mtu and max_mps config, please see @ref ble_gap_l2cap_params_set. 
*/
typedef struct
{
    uint16_t le_psm;             /**< The le_psm number. */
    uint16_t local_cid;          /**< The local CID. */
    uint16_t local_credits;      /**< The local credits indicate the number of LE-frames that the peer device can send to the L2CAP layer entity sending the LE Credit Based Connection Request. */
    uint16_t mtu;                /**< The MTU field specifies the maximum SDU size (in octets) that the L2CAP layer entity sending the LE Credit Based Connection Request can receive on this channel. */
    uint16_t mps;                /**< The MPS field specifies the maximum payload size (in octets) that the L2CAP layer entity sending the LE Credit Based Connection Request is capable of receiving on this channel. */
} lecb_conn_req_t;

/** @brief LE credit based connection confirm parameter. 
  * @note  The accept flag indicates whether the App accepts the LE Credit Based connection request.
  * @note  The peer_cid represents the channel endpoint on the peer device.
  * @note  The local_cid should be 0x0040-0x007F. If the local_cid is set to 0, the stack will assign it dynamically.
  * @note  The local_credits required to be sure that at least one SDU can be received, otherwise the stack will use the default value: (MTU  + MPS + 1) /MPS + 1.
  * @note  The MTU range is [23~max_mtu].
  * @note  The MPS range is [23~max_mps].
  * @note  About the max_mtu and max_mps config, please see @ref ble_gap_l2cap_params_set.
*/
typedef struct
{
    bool     accept;             /**< Whether to accept the connection request. */
    uint16_t peer_cid;           /**< It represents the channel endpoint on the device sending the request and receiving the response. */
    uint16_t local_cid;          /**< Local CID. */
    uint16_t local_credits;      /**< It indicates the number of LE-frames that the peer device can send to the L2CAP layer entity  sending the LE Credit Based Connection Respone. */
    uint16_t mtu;                /**< The MTU field specifies the maximum SDU size (in octets) that the L2CAP layer entity sending 
                                      the LE Credit Based Connection Request can receive on this channel. */
    uint16_t mps;                /**< The MPS field specifies the maximum payload size (in octets) that the L2CAP layer entity sending 
                                      the LE Credit Based Connection Request is capable of receiving on this channel. */
} lecb_cfm_conn_t;

/** @brief LE flow control credit packet parameter.  */
typedef struct
{
    uint16_t local_cid;      /**< The local source channel ID. */
    uint16_t credits;        /**< Number of credits that the receiving device can increment. */
} lecb_add_credits_t;

/** @brief SDU packet parameter. 
  * @note  The length should be less than peer_mtu when sending sdu packet.
  * @note  The credits is 0 if this packet is being sent, or it represents the number of credits consumed by this sdu if this packet is received.
  * @note  When the application receives a sdu, it should firstly copy this sdu packet before handling it, because the stack will free it after invoking the callback function.
  * @note  Similarly, the application should free the packet if it is malloced after invoking the function to send sdu packet.
*/
typedef struct
{
    uint16_t cid;                 /**< The local source channel. */
    uint16_t credits;             /**< The credits is 0 if this packet is being sent, otherwise it represents the number of credits consumed by the sdu. */
    uint16_t length;              /**< The lenght of data. */
    uint8_t  data[__ARRAY_EMPTY]; /**< The data of this sdu packet. */
} lecb_sdu_t;

/** @brief Receive LE credit based connection request packet indication. */
typedef struct
{
    uint16_t le_psm;   /**< Le_psm number that should be registered by local device. */
    uint16_t peer_cid; /**< It represents the channel endpoint on the device sending the request and receiving the response. */
    uint16_t peer_mtu; /**< It indicates the maximum SDU size (in octets) that the L2CAP layer entity sending the LE Credit 
                            Based Connection Request can receive on this channel.  */
    uint16_t peer_mps; /**< It indicates the maximum payload size (in octets) that the L2CAP layer entity sending the LE Credit 
                            Based Connection Request is capable of receiving on this channe. */
} lecb_conn_req_ind_t;

/** @brief LE credit based connection created indication. */
typedef struct
{
    uint16_t le_psm;        /**< Le_psm number. */
    uint16_t local_cid;     /**< The local source channel ID. */
    uint16_t local_credits; /**< It indicates the number of LE-frames that the local device can receive. */
    uint16_t peer_credits;  /**< It indicates the number of LE-frames that the peer device can receive. */
    uint16_t peer_mtu;      /**< It indicates the maximum SDU size (in octets) that the L2CAP layer entity sending the LE Credit 
                                 Based Connection Request can receive on this channel.  */
    uint16_t peer_mps;      /**< It indicates the maximum payload size (in octets) that the L2CAP layer entity sending the LE Credit 
                                 Based Connection Request is capable of receiving on this channe. */
} lecb_conn_ind_t;

/** @brief LE credit based disconnect indication. */
typedef struct
{
    uint16_t local_cid;              /**< The local source channel ID. */
    uint8_t  reason;                 /**< The reason for disconnection, see @ref lecb_disconnect_reason_t . */
} lecb_disconn_ind_t;

/** @brief LE credit based connection addition indication. */
typedef struct
{
    uint16_t local_cid;                 /**< The local source channel ID. */
    uint16_t peer_added_credits;        /**< Represent number of credits the receiving device can increment. */
} lecb_add_credits_ind_t;

/** @brief LE credit based SDU sending complete event. */
typedef struct
{ 
    uint16_t     cid;            /**< Channel ID that is the local CID. */
    uint16_t     credits;        /**< Number of peer credit used. */
}lecb_sdu_send_evt_t;


#if defined(GR551xx_D0)

/** @brief The parameter of LE enhanced credit based connection request packet sending. 
  * @note  The le_psm should be registered by the peer device, otherwise the peer device will reject this request with result of LE_PSM not supported.
  * @note  The local_cid should be 0x0040-0x007F. If the local_cid is set to 0, the stack will assign it dynamically. 
  * @note  The local_credit is required to be sure that at least one SDU can be received, otherwise the stack will use the default value: (MTU  + MPS + 1) /MPS + 1.
  * @note  The MTU range is [64~max_mtu].
  * @note  The MPS range is [64~max_mps].
  * @note  About max_mtu and max_mps config, please see @ref ble_gap_l2cap_params_set. 
*/
typedef struct
{
   uint16_t le_psm;             /**< The le_psm number. */
   uint16_t mtu;                /**< The MTU field specifies the maximum SDU size (in octets) that the L2CAP layer entity sending the enhanced LE Credit Based Connection Request can receive on each of these channels. */
   uint16_t mps;                /**< The MPS field specifies the maximum payload size (in octets) that the L2CAP layer entity sending the enhanced LE Credit Based Connection Request is capable of receiving on these channels. */
   uint16_t local_credits;      /**< The local credits, it indicates the number of F-frames that the peer device can send to the L2CAP layer entity sending the LE Credit Based Connection Request. */
   uint16_t local_cid[5];       /**< The Source CID is an array of 5 two octet values and represents the channel endpoints on the device sending the request. */ 
} enh_lecb_conn_req_t;

/** @brief LE enhanced credit based connection created indication. */
typedef struct
{
    uint8_t  status;            /**< 0 success, otherwise failed. */
    uint16_t le_psm;            /**< Le_psm number. */
    uint16_t peer_credits;      /**< It indicates the number of F-frames that the peer device can. */
    uint16_t peer_mtu;          /**< It indicates the maximum SDU size (in octets) that the peer device can receive.  */
    uint16_t peer_mps;          /**< It indicates the maximum payload size (in octets)that the peer device can receive. */
    uint16_t chl_num;           /**< The chl_num field specifies the number of channel created. */
    uint16_t local_cid[5];      /**< The local source channel id. */
} enh_lecb_conn_ind_t;

/** @brief Receive LE enhanced credit based connection request packet indication. */
typedef struct {
   uint16_t le_psm;        /**< Le_psm number, it should be registered by local device. */
   uint16_t peer_mtu;      /**< It indicates the maximum SDU size (in octets) that the L2CAP layer entity sending the enhanced LE Credit Based Connection Request can receive on these channels. */
   uint16_t peer_mps;      /**< It indicates the maximum payload size (in octets) that the L2CAP layer entity sending the enhanced LE Credit Based Connection Request is capable of receiving on these channels.*/
   uint16_t peer_cid[5];   /**< It represents the channel endpoints on the device sending the request and receiving the response. */
} enh_lecb_conn_req_ind_t;

/** @brief LE enhanced credit based connection confirm parameter. 
  * @note  The accept flag indicates whether the App accepts the LE enhanced credit based connection request.
  * @note  The MTU range is [64~max_mtu].
  * @note  The MPS range is [64~max_mps].
  * @note  About the max_mtu and max_mps config, please see @ref ble_gap_l2cap_params_set.
  * @note  The local_credits required to be sure that at least one SDU can be received, otherwise the stack will use the default value: (MTU + MPS + 1) / MPS + 1.
  * @note  The chl_num indicates the number of channel.  
  * @note  The peer_cid represents the channel endpoint list on the peer device.
  * @note  The local_cid represents the channel endpoint list on the local device, the value should be 0x0040-0x007F.  
*/
typedef struct {
    bool accept;             /**< Whether to accept the enhanced lecb connection request. */     
    uint16_t mtu;            /**< The MTU field specifies the maximum SDU size (in octets) that the L2CAP layer entity sending the response can receive on these channels. */
    uint16_t mps;            /**< The MPS field specifies the maximum payload size (in octets) that the L2CAP layer entity sending the response is capable of receiving on these channels. */
    uint16_t local_credits;  /**< The local_credits field specifies the number of F-frames that the peer device can send to the L2CAP layer entity sending respone. */
    uint16_t peer_cid[5];    /**< The peer_cid is an array of 5 two octet values and represents the channel endpoints on the device sending the resquest. */
    uint16_t local_cid[5];   /**< The local_cid is an array of 5 two octet values and represents the channel endpoints on the device sending the response. */ 
} enh_lecb_cfm_conn_t;

/** @brief Reconfig LE enhanced credit based connection parameter. */
typedef struct
{
    uint16_t local_mtu;     /**< It indicates the maximum SDU size (in octets) that the L2CAP layer entity of local device can receive on these channels.
                                 The local_mtu shall be greater than or equal to the greatest current MTU size of the channels. */
    uint16_t local_mps;     /**< It indicates the maximum payload size (in octets) that the L2CAP layer entity of local device can receive on these channels.
                                 The local_mps can be greater than or less than or equal to the current MPS size of any channel.*/
    uint16_t chl_num;       /**< It specifies the number of channel need to reconfigure, it shall not greater than 5. */
    uint16_t local_cid[5];  /**< It represents the local channel endpoints need to reconfigure. */
} enh_lecb_reconfig_t;

/** @brief Receiving reconfig LE enhanced credit based connection indication parameter. */
typedef struct
{
    uint16_t peer_mtu;      /**< It indicates the maximum SDU size (in octets) that the L2CAP layer entity of peer device can receive on these channels. */
    uint16_t peer_mps;      /**< It indicates the maximum payload size (in octets) that the L2CAP layer entity of peer device can receive on these channels. */
    uint16_t chl_num;       /**< It specifies the number of channel need to reconfigure, it shall not greater than 5. */
    uint16_t local_cid[5];  /**< It represents the local channel endpoints need to reconfigure. */
} enh_lecb_reconfig_ind_t;

#endif

/**@brief Callback registered by APP. */
typedef struct 
{
    void (*app_l2cap_lecb_conn_req_cb)(uint8_t conn_idx, lecb_conn_req_ind_t *p_conn_req);                        /**< Callback for receiving LE credit based connection request. */
    void (*app_l2cap_lecb_conn_cb)(uint8_t conn_idx, uint8_t status, lecb_conn_ind_t *p_conn_ind);                /**< Callback for receiving LE credit based connection created indication. */
    void (*app_l2cap_lecb_add_credits_ind_cb)(uint8_t conn_idx, lecb_add_credits_ind_t *p_add_credits_ind);       /**< Callback for receiving LE credit based connection addition indication. */
    void (*app_l2cap_lecb_disconn_cb)(uint8_t conn_idx, uint8_t status, lecb_disconn_ind_t *p_disconn_ind);       /**< Callback for receiving LE credit based disconnection indication. */
    void (*app_l2cap_lecb_sdu_recv_cb)(uint8_t conn_idx, lecb_sdu_t *p_sdu);                                      /**< Callback for receiving SDU packet. */
    void (*app_l2cap_lecb_sdu_send_cb)(uint8_t conn_idx, uint8_t status, lecb_sdu_send_evt_t *p_sdu_send_evt);    /**< Callback for sending SDU operation complete event. */
    void (*app_l2cap_lecb_credit_add_cmp_cb)(uint8_t conn_idx, uint8_t status, uint16_t local_cid);               /**< Callback for LE credit add complete event. */
#if defined(GR551xx_D0)
    void (*app_l2cap_enh_lecb_conn_cb)(uint8_t conn_idx, uint8_t status, enh_lecb_conn_ind_t *p_enh_conn_ind);    /**< Callback for receiving LE enhanced credit based connection created indication. */
    void (*app_l2cap_enh_lecb_conn_req_cb)(uint8_t conn_idx, enh_lecb_conn_req_ind_t *p_enh_conn_req);            /**< Callback for receiving LE enhanced credit based connection request. */
    void (*app_l2cap_enh_lecb_reconfig_cmp_cb)(uint8_t conn_idx, uint8_t staus);                                  /**< Callback for reconfig LE enhanced credit based connection complete event. */
    void (*app_l2cap_enh_lecb_reconfig_ind_cb)(uint8_t conn_idx, enh_lecb_reconfig_ind_t *p_enh_reconfig_ind);    /**< Callback for receiving reconfig LE enhanced credit based connection indication. */
#endif
} l2cap_lecb_cb_fun_t;


/** @} */

/** @addtogroup BLE_L2CAP_FUNCTIONS Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief Create the LE credit based connection.
 * @note After the COC created, the callback @ref l2cap_lecb_cb_fun_t::app_l2cap_lecb_conn_cb will be called.
 *
 * @param[in] conn_idx:   ACL connection index. The first ACL connection index is 0, and the index will be increased one by one.
 * @param[in] p_conn_req: Pointer to the LE Credit Based Connection Request structure.
 *
 * @retval ::SDK_SUCCESS: The LE Credit Based connection request is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_lecb_conn_create(uint8_t conn_idx, const lecb_conn_req_t *p_conn_req);

/**
 ****************************************************************************************
 * @brief Confirm the LE credit based connection after receiving the connection request packet from the peer device.
 * @note This function should be invoked in the handle of callback @ref l2cap_lecb_cb_fun_t::app_l2cap_lecb_conn_req_cb. And after the COC created,
 *       the callback @ref l2cap_lecb_cb_fun_t::app_l2cap_lecb_conn_cb should be called.
 * @param[in] conn_idx:   ACL connection index. The first ACL connection index is 0 and the index will be increased one by one.
 * @param[in] p_cfm_conn: Pointer to the LE Credit Based Connection Confirm structure.
 *
 * @retval ::SDK_SUCCESS: The LE Credit Based connection confirmation is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_lecb_conn_cfm(uint8_t conn_idx, const lecb_cfm_conn_t *p_cfm_conn);

/**
 ****************************************************************************************
 * @brief Disconnect the LE credit based connection.
 * @note After COC disconnected, the callback @ref l2cap_lecb_cb_fun_t::app_l2cap_lecb_disconn_cb should be called.
 *
 * @param[in] conn_idx:  ACL connection index. The first ACL connection index is 0 and the index will be increased one by one.
 * @param[in] local_cid: The local source channel ID.
 *
 * @retval ::SDK_SUCCESS: LE Credit Based disconnection request is successfully set to the BLE stack.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_lecb_disconnect(uint8_t conn_idx, uint16_t local_cid);

/**
 ****************************************************************************************
 * @brief Send a LE Flow Control Credit packet when the device is capable of receiving additional LE-frames (for example after the device has processed the sdu).
 *
 * @param[in] conn_idx:      ACL connection index, the first ACL connection index is 0, and increased one by one.
 * @param[in] p_add_credits: Pointer to the LE Flow Control Credit structure.
 *
 * @retval ::SDK_SUCCESS: LE Flow Control Credit packet is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_lecb_credits_add(uint8_t conn_idx, const lecb_add_credits_t *p_add_credits);

/**
 ****************************************************************************************
 * @brief Send an SDU packet to the peer device.
 *
 * @param[in] conn_idx: ACL connection index. The first ACL connection index is 0 and the index will be increased one by one.
 * @param[in] p_sdu:    Pointer to the sdu packet structure.
 *
 * @retval ::SDK_SUCCESS: The sdu packet is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_lecb_sdu_send(uint8_t conn_idx, const lecb_sdu_t *p_sdu);

/**
 ****************************************************************************************
 * @brief Register the callback for the PSM.
 *
 * @param[in] le_psm: The le_psm number.
 * @param[in] p_cb:   Pointer to the callback structure.
 *
 * @retval ::SDK_SUCCESS: The callback is successfully registered to the BLE stack.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_INVALID_PSM_EXCEEDED_MAX_PSM_NUM: The maximum PSM number limit is exceeded.
 ****************************************************************************************
 */
uint16_t ble_l2cap_lecb_cb_register(uint16_t le_psm, const l2cap_lecb_cb_fun_t *p_cb);

#if defined(GR551xx_D0)
/**
 ****************************************************************************************
 * @brief Create the LE enhanced credit based connection.
 * @note After the COC created, the callback @ref l2cap_lecb_cb_fun_t:: app_l2cap_enh_lecb_conn_cb will be called.
 *
 * @param[in] conn_idx:       ACL connection index. The first ACL connection index is 0, and the index will be increased one by one.
 * @param[in] p_enh_conn_req: Pointer to the LE enhanced Credit Based Connection Request structure.
 *
 * @retval ::SDK_SUCCESS: The LE Credit Based connection request is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_enh_lecb_conn_create(uint8_t conn_idx, const enh_lecb_conn_req_t *p_enh_conn_req);

/**
 ****************************************************************************************
 * @brief Confirm the LE enhanced credit based connection after receiving the connection request packet from the peer device.
 * @note This function should be invoked in the handle of callback @ref l2cap_lecb_cb_fun_t::app_l2cap_enh_lecb_conn_req_cb.
 *       And after the COC created, the callback @ref l2cap_lecb_cb_fun_t::app_l2cap_enh_lecb_conn_cb should be called.
 * @param[in] conn_idx:       ACL connection index. The first ACL connection index is 0 and the index will be increased one by one.
 * @param[in] p_enh_cfm_conn: Pointer to the LE enhanced credit based connection confirm structure.
 *
 * @retval ::SDK_SUCCESS: The LE credit based connection confirmation is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_enh_lecb_conn_cfm(uint8_t conn_idx, const enh_lecb_cfm_conn_t *p_enh_cfm_conn);

/**
 ****************************************************************************************
 * @brief Reconfig the mtu and mps for the indicated channels.
 * @note After the reconfig complete, the callback @ref l2cap_lecb_cb_fun_t::app_l2cap_enh_lecb_reconfig_cmp_cb should be called.
 * @param[in] conn_idx:       ACL connection index. The first ACL connection index is 0 and the index will be increased one by one.
 * @param[in] p_enh_reconfig: Pointer to the LE enhanced credit based connection reconfig structure.
 *
 * @retval ::SDK_SUCCESS: The LE credit based connection confirmation is successfully set to the BLE stack.
 * @retval ::SDK_ERR_POINTER_NULL: Invalid pointer supplied.
 * @retval ::SDK_ERR_INVALID_CONN_IDX: Invalid connection index supplied.
 * @retval ::SDK_ERR_INVALID_PARAM: Invalid parameter supplied.
 * @retval ::SDK_ERR_NO_RESOURCES: Not enough resources.
 ****************************************************************************************
 */
uint16_t ble_l2cap_enh_lecb_reconfig(uint8_t conn_idx, const enh_lecb_reconfig_t *p_enh_reconfig);

#endif
/** @} */

#endif

/**
  @}
*/
/** @} */

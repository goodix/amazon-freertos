/**
 ****************************************************************************************
 *
 * @file    app_aes.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of AES app library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 ****************************************************************************************
 */

/**
 @addtogroup PERIPHERAL APP DRIVER
 @{
*/

/**
  @addtogroup PERIPHERAL_API_HAL_APP_AES_DRIVER HAL APP AES Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_AES_H_
#define _APP_AES_H_

#include "gr55xx_hal.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_AES_MODULE_ENABLED

/** @addtogroup HAL_APP_AES_STRUCTURES Structures
  * @{
  */

/**
  * @brief AES operating mode Enumerations definition
  */
typedef enum
{
    APP_AES_TYPE_INTERRUPT,          /**< Interrupt operation mode. */
    APP_AES_TYPE_POLLING,            /**< Polling operation mode.   */
    APP_AES_TYPE_DMA,                /**< DMA operation mode.       */
    APP_AES_TYPE_MAX                 /**< Only for check parameter, not used as input parameters. */
} app_aes_type_t;

/**
  * @brief AES encryption and decryption mode Enumerations definition
  */
typedef enum
{
    APP_AES_MODE_ECB,                /**< ECB encryption mode. */
    APP_AES_MODE_CBC,                /**< CBC encryption mode. */
    APP_AES_MODE_MAX                 /**< Only for check parameter, not used as input parameters. */
} app_aes_mode_t;

/**
  * @brief AES parameters structure definition
  */
typedef struct 
{
    app_aes_type_t      use_type;    /**< Specifies the operation mode of AES. */
    app_aes_mode_t      use_mode;    /**< AES encryption mode. */
    aes_init_t          init;        /**< AES operation parameters         */
} app_aes_params_t;

/** @} */

/** @addtogroup HAL_APP_AES_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief AES event Enumerations definition
  */
typedef enum
{
    APP_AES_EVT_ERROR,               /**< Error reported by AES peripheral. */
    APP_AES_EVT_DONE                 /**< Encryption and decryption completed. */
} app_aes_evt_type_t;

/**
  * @brief AES event structure definition
  */
typedef struct
{
    app_aes_evt_type_t type;         /**< Type of event. */
    uint32_t error_code;
} app_aes_evt_t;

/**
  * @brief AES event callback definition
  */
typedef void (*app_aes_evt_handler_t)(app_aes_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_AES_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the APP AES DRIVER according to the specified parameters
 *         in the app_aes_params_t and app_aes_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_aes_params_t parameter which contains the
 *                       configuration information for the specified AES module.
 * @param[in]  evt_handler: AES user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_aes_init(app_aes_params_t *p_params, app_aes_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP AES DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_aes_deinit(void);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in blocking mode.
 *
 * @param[in]  p_plain_data: Pointer to plain data buffer.
 * @param[in]  number: Amount of data to be Encrypted in bytes
 * @param[out] p_cypher_data: Pointer to cypher data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_aes_encrypt_sync(uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in blocking mode.
 *
 * @param[in]  p_cypher_data: Pointer to cypher data buffer.
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_aes_decrypt_sync(uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Encrypted an amount of data in non-blocking mode.
 *
 * @param[in]  p_cypher_data: Pointer to cypher data buffer.
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_aes_encrypt_async(uint32_t *p_plain_data, uint32_t number, uint32_t *p_cypher_data);

/**
 ****************************************************************************************
 * @brief  Decrypted an amount of data in non-blocking mode.
 *
 * @param[in]  p_cypher_data: Pointer to cypher data buffer.
 * @param[in]  number: Amount of data to be decrypted in bytes
 * @param[out] p_plain_data: Pointer to plain data buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_aes_decrypt_async(uint32_t *p_cypher_data, uint32_t number, uint32_t *p_plain_data);


/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

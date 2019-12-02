/**
 ****************************************************************************************
 *
 * @file    app_hmac.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of HMAC app library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_HMAC_DRIVER HAL APP HMAC Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_HMAC_H_
#define _APP_HMAC_H_

#include "gr55xx_hal.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_HMAC_MODULE_ENABLED

/** @addtogroup HAL_APP_HMAC_STRUCTURES Structures
  * @{
  */

/**
  * @brief HMAC operating mode Enumerations definition
  */
typedef enum
{
    APP_HMAC_TYPE_INTERRUPT,          /**< Interrupt operation mode */
    APP_HMAC_TYPE_POLLING,            /**< Polling operation mode   */
    APP_HMAC_TYPE_DMA,                /**< DMA operation mode       */
    APP_HMAC_TYPE_MAX                 /**< Only for check parameter, not used as input parameters. */
} app_hmac_type_t;

/**
  * @brief HMAC parameters structure definition
  */
typedef struct 
{
    app_hmac_type_t      use_type;   /**< Specifies the operation mode of I2C. */
    hmac_init_t          init;       /**< HMAC operation parameters            */
} app_hmac_params_t;

/** @} */

/** @addtogroup HAL_APP_HMAC_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief HMAC event Enumerations definition
  */
typedef enum
{
    APP_HMAC_EVT_ERROR,              /**< Error reported by HMAC peripheral. */
    APP_HMAC_EVT_DONE                /**< HMAC operation completed.          */
} app_hmac_evt_type_t;

/**
  * @brief HMAC event structure definition
  */
typedef struct
{
    app_hmac_evt_type_t type;       /**< Type of event.    */
    uint32_t error_code;            /**< HMAC Error code . */    
} app_hmac_evt_t;

/**
  * @brief HMAC event callback definition
  */
typedef void (*app_hmac_evt_handler_t)(app_hmac_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_HMAC_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP HMAC DRIVER according to the specified parameters
 *         in the app_hmac_params_t and app_hmac_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_hmac_params_t parameter which contains the
 *                       configuration information for the specified HMAC module.
 * @param[in]  evt_handler: HMAC user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_init(app_hmac_params_t *p_params, app_hmac_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP HMAC DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_deinit(void);

/**
 ****************************************************************************************
 * @brief  Update p_user_hash parameters.
 *
 * @param[in]  p_params: Pointer to p_user_hash.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_user_hash(uint32_t *p_user_hash);

/**
 ****************************************************************************************
 * @brief  xxx in blocking mode in SHA256 mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[out] p_digest: Pointer to digest buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_sync(uint32_t *p_message, uint32_t number, uint32_t *p_digest, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  xxx in non-blocking mode with interrupt in SHA256 mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[out] p_digest: Pointer to digest buffer
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_async(uint32_t *p_message, uint32_t number, uint32_t *p_digest);

/**
 ****************************************************************************************
 * @brief  xxx in blocking mode in HMAC mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_hmac_start_sync(uint32_t *p_message, uint32_t number, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  xxx in blocking mode in HMAC mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_hmac_continue_sync(uint32_t *p_message, uint32_t number, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  xxx in blocking mode in HMAC mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[out] digest: Pointer to digest buffer
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_hmac_finish_sync(uint32_t *p_message, uint32_t number, uint32_t *digest, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  xxx in non-blocking mode with interrupt in SHA mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_hmac_start_async(uint32_t *p_message, uint32_t number);

/**
 ****************************************************************************************
 * @brief  xxx in non-blocking mode with interrupt in SHA mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_hmac_continue_async(uint32_t *p_message, uint32_t number);

/**
 ****************************************************************************************
 * @brief  xxx in non-blocking mode with interrupt in SHA mode.
 *
 * @param[in]  p_message: Pointer to message buffer
 * @param[in]  number: Amount of data
 * @param[out] digest: Pointer to digest buffer
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_hmac_sha256_hmac_finish_async(uint32_t *p_message, uint32_t number, uint32_t *digest);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

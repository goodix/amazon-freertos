/**
 ****************************************************************************************
 *
 * @file    app_rng.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of RNG app library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_RNG_DRIVER HAL APP RNG Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_RNG_H_
#define _APP_RNG_H_

#include "gr55xx_hal.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_RNG_MODULE_ENABLED

/** @addtogroup HAL_APP_RNG_STRUCTURES Structures
  * @{
  */

/**
  * @brief RNG operating mode Enumerations definition
  */
typedef enum
{
    APP_RNG_TYPE_INTERRUPT,        /**< Interrupt operation mode */
    APP_RNG_TYPE_POLLING,          /**< Polling operation mode   */
    APP_RNG_TYPE_MAX               /**< Only for check parameter, not used as input parameters. */
} app_rng_type_t;

/**
  * @brief RNG parameters structure definition
  */
typedef struct 
{
    app_rng_type_t      use_type;  /**< Specifies the operation mode of RNG. */
    rng_init_t          init;      /**< RNG required parameters.             */
} app_rng_params_t;

/** @} */

/** @addtogroup HAL_APP_RNG_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief RNG event Enumerations definition
  */
typedef enum
{
    APP_RNG_EVT_DONE,             /**< Generated random by UART peripheral. */
    APP_RNG_EVT_ERROR,            /**< Error reported by UART peripheral.   */
} app_rng_evt_type_t;

/**
  * @brief RNG event structure definition
  */
typedef struct
{
    app_rng_evt_type_t  type;           /**< Type of event. */
    uint32_t            random_data;    /**< Random number. */
} app_rng_evt_t;

/**
  * @brief RNG event callback definition
  */
typedef void (*app_rng_evt_handler_t)(app_rng_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_RNG_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP RNG DRIVER according to the specified parameters
 *         in the app_rng_params_t and app_rng_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_rng_params_t parameter which contains the
 *                       configuration information for the specified RNG module.
 * @param[in]  evt_handler: RNG user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_rng_init(app_rng_params_t *p_params, app_rng_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP RNG DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_rng_deinit(void);

/**
 ****************************************************************************************
 * @brief  Generate a 32-bit random number.
 *
 * @param[in]  p_seed: user configured seeds. the seed is valid when seed_mode member of 
 *               rng_init_t is configured as RNG_SEED_USER. If 59-bit random number is
 *               selected, the seed need to provide [0~58] bit spaces. If 128-bit random 
 *               number is selected, the seed need to provide [0~127] bit spaces.
 * @param[out] p_random32bit: Pointer to generated random number variable if successful.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rng_generate_number_sync(uint16_t *p_seed, uint32_t *p_random32bit);

/**
 ****************************************************************************************
 * @brief  Generate a 32-bit random number in interrupt mode.
 *
 * @param[in]  p_seed: user configured seeds. the seed is valid when seed_mode member of 
 *               rng_init_t is configured as RNG_SEED_USER. If 59-bit random number is
 *               selected, the seed need to provide [0~58] bit spaces. If 128-bit random 
 *               number is selected, the seed need to provide [0~127] bit spaces.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_rng_generate_number_async(uint16_t *p_seed);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

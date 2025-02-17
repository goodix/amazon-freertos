/**
 ****************************************************************************************
 *
 * @file    app_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of TIM app library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_TIM_DRIVER HAL APP TIM Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_TIM_H_
#define _APP_TIM_H_

#include "gr55xx_hal.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_TIMER_MODULE_ENABLED

/** @addtogroup HAL_APP_TIM_STRUCTURES Structures
  * @{
  */

/**
  * @brief TIM module Enumerations definition
  */
typedef enum
{
    APP_TIM_ID_0,                /**< TIMER module 0 */
    APP_TIM_ID_1,                /**< TIMER module 1 */
    APP_TIM_ID_MAX               /**< Only for check parameter, not used as input parameters. */
} app_tim_id_t;

/**
  * @brief TIM parameters structure definition
  */
typedef struct 
{
    app_tim_id_t        id;      /**< specified TIMER module ID.      */
    timer_init_t          init;    /**< TIMER Base required parameters. */
} app_tim_params_t;

/** @} */

/** @addtogroup HAL_APP_TIM_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief TIM event Enumerations definition
  */
typedef enum
{
    APP_TIM_EVT_ERROR,           /**< Error reported by TIMER peripheral. */
    APP_TIM_EVT_DONE             /**< Interrupt done by TIMER peripheral. */
} app_tim_evt_t;

/**
  * @brief TIM event callback definition
  */
typedef void (*app_tim_evt_handler_t)(app_tim_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_TIM_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP TIM DRIVER according to the specified parameters
 *         in the app_tim_params_t and app_tim_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_tim_params_t parameter which contains the
 *                       configuration information for the specified TIM module.
 * @param[in]  evt_handler: TIM user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_tim_init(app_tim_params_t *p_params, app_tim_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP TIM DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_tim_deinit(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Starts the TIM counter in interrupt mode.
 * @param[in]  id: which TIM module want to start.
 *
 * @return Result of initialization.
 *
 ****************************************************************************************
 */
uint16_t app_tim_start(app_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Stops the TIM counter in interrupt mode.
 * @param[in]  id: which TIM module want to stop.
 *
 * @return Result of initialization.
 *
 ****************************************************************************************
 */
uint16_t app_tim_stop(app_tim_id_t id);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

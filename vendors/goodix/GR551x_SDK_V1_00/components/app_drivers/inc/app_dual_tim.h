/**
 ****************************************************************************************
 *
 * @file    app_dual_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DAUL TIM app library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_DUAL_TIM_DRIVER HAL APP DUAL TIM Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_DUAL_TIM_H_
#define _APP_DUAL_TIM_H_

#include "gr55xx_hal.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_DUAL_TIMER_MODULE_ENABLED

/** @addtogroup HAL_APP_TIM_STRUCTURES Structures
  * @{
  */

/**
  * @brief DUAL_TIM module Enumerations definition
  */
typedef enum
{
    APP_DUAL_TIM_ID_0,     /**< DUAL TIMER module 0 */
    APP_DUAL_TIM_ID_1,     /**< DUAL TIMER module 1 */
    APP_DUAL_TIM_ID_MAX    /**< Only for check parameter, not used as input parameters. */
} app_dual_tim_id_t;

/**
  * @brief DUAL_TIM parameters structure definition
  */
typedef struct 
{
    app_dual_tim_id_t   id;    /**< specified DUAL TIMER module ID.    */
    dual_timer_init_t   init;  /**< DUAL_TIM Base required parameters. */
} app_dual_tim_params_t;

/** @} */

/** @addtogroup HAL_APP_DUAL_TIM_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief DUAL_TIM event Enumerations definition
  */
typedef enum
{
    APP_DUAL_TIM_EVT_ERROR,    /**< Error reported by DUAL TIMER peripheral. */
    APP_DUAL_TIM_EVT_DONE      /**< Interrupt done by DUAL TIMER peripheral. */
} app_dual_tim_evt_t;

/**
  * @brief DUAL_TIM event callback definition
  */
typedef void (*app_dual_tim_evt_handler_t)(app_dual_tim_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_DUAL_TIM_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP DUAL TIM DRIVER according to the specified parameters
 *         in the app_dual_tim_params_t and app_dual_tim_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_dual_tim_params_t parameter which contains the
 *                       configuration information for the specified DUAL TIM module.
 * @param[in]  evt_handler: DUAL TIM user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_dual_tim_init(app_dual_tim_params_t *p_params, app_dual_tim_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP DUAL TIM DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_dual_tim_deinit(app_dual_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Starts the DUAL TIM counter in interrupt mode.
 * @param[in]  id: which DUAL TIM module want to start.
 *
 * @return Result of initialization.
 *
 ****************************************************************************************
 */
uint16_t app_dual_tim_start(app_dual_tim_id_t id);

/**
 ****************************************************************************************
 * @brief  Stops the DUAL TIM counter in interrupt mode.
 * @param[in]  id: which DUAL TIM module want to stop.
 *
 * @return Result of initialization.
 *
 ****************************************************************************************
 */
uint16_t app_dual_tim_stop(app_dual_tim_id_t id);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

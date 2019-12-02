/**
 ****************************************************************************************
 *
 * @file    app_comp.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of COMP app library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_COMP_DRIVER HAL APP COMP Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_COMP_H_
#define _APP_COMP_H_

#include "gr55xx_hal.h"
#include "app_drv_error.h"
#include "app_io.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_COMP_MODULE_ENABLED

/** @addtogroup HAL_APP_COMP_STRUCTURES Structures
  * @{
  */

/**
  * @brief COMP pins config Structures
  */
typedef struct
{
   app_io_type_t        type;         /**< Specifies the type of COMP IO. */
   app_io_mux_t         mux;          /**< Specifies the Peripheral to be connected to the selected pins. */
   uint32_t             pin;          /**< Specifies the IO pins to be configured.
                                           This parameter can be any value of @ref GR551x_pins. */
} app_comp_pin_t;

typedef struct
{
    app_comp_pin_t      input;        /**< Set the configuration of input pin. */
    app_comp_pin_t      vref;         /**< Set the configuration of reference pin. */
} app_comp_pin_cfg_t;

/**
  * @brief COMP parameters structure definition
  */
typedef struct 
{
    app_comp_pin_cfg_t  pin_cfg;      /**< the pin configuration information for the specified COMP module. */
    comp_init_t         init;         /**< COMP configuration parameters.      */
} app_comp_params_t;

/** @} */

/** @addtogroup HAL_APP_COMP_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief COMP event Enumerations definition
  */
typedef enum
{
    APP_COMP_EVT_DONE,
    APP_COMP_EVT_ERROR
} app_comp_evt_t;

/**
  * @brief COMP event callback definition
  */
typedef void (*app_comp_evt_handler_t)(app_comp_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_COMP_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP COMP DRIVER according to the specified parameters
 *         in the app_comp_params_t and app_comp_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_comp_params_t parameter which contains the
 *                       configuration information for the specified COMP module.
 * @param[in]  evt_handler: COMP user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_comp_init(app_comp_params_t *p_params, app_comp_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP COMP DRIVER peripheral.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_comp_deinit(void);

 /**
 ****************************************************************************************
 * @brief  Start the comparator.
 *
 @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_comp_start(void);

 /**
 ****************************************************************************************
 * @brief  Stop the comparator.
 *
 @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_comp_stop(void);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

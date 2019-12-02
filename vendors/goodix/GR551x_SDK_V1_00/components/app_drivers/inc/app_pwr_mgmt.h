/**
 ****************************************************************************************
 *
 * @file    app_pwr_mgmt.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of UART PWR library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_PWR_DRIVER HAL PWR Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_PWR_MGMT_H_
#define _APP_PWR_MGMT_H_

#include "gr55xx_pwr.h"
#include <stdint.h>
#include <stdbool.h>
/** @addtogroup HAL_APP_PWR_STRUCTURES Structures
  * @{
  */

/**
  * @brief PWR MAX value for sleep check
  */
#define APP_SLEEP_CB_MAX     16

/**
  * @brief PWR id
  */
typedef int16_t pwr_id_t;

/**
  * @brief PWR sleep check function Structure
  */
typedef struct 
{
    bool (*app_prepare_for_sleep)(void);
    void (*app_sleep_canceled)(void);
    void (*app_wake_up_ind)(void);
} app_sleep_callbacks_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_PWR_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief    set PWR sleep callback function
 * @param    p_cb : Device check callback function
 *
 * @return   ID
 ****************************************************************************************
 */
pwr_id_t pwr_register_sleep_cb(const app_sleep_callbacks_t *p_cb);

/**
 ****************************************************************************************
 * @brief    Unregister PWR sleep callback function
 * @param    id : which id want to unregister
 ****************************************************************************************
 */
void pwr_unregister_sleep_cb(pwr_id_t id);

/**
 ****************************************************************************************
 * @brief    Things to do after waking up.
 ****************************************************************************************
 */
void pwr_wake_up_ind(void);

/**
 ****************************************************************************************
 * @brief    Check peripheral status before going to sleep.
 ****************************************************************************************
 */
bool pwr_enter_sleep_check(void);
/** @} */

#endif

/** @} */
/** @} */

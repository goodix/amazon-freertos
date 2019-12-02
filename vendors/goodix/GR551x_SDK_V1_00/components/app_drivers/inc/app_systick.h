/**
 ****************************************************************************************
 *
 * @file    app_systick.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of systick app library.
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

#ifndef _APP_SYSTICK_H_
#define _APP_SYSTICK_H_

#include <stdint.h>

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_SYSTICK_DRIVER_FUNCTIONS Functions
  * @{
  */

/**
 ****************************************************************************************
 * @brief  This function configures time base source, NVIC and Low level hardware.
 *
 * @note   This function is called at the beginning of program after reset and before
 *         the clock configuration.
 *         The Systick configuration is based on AHB clock and the NVIC configuration
 *         is set to Priority group 4.
 *         The time base configuration is done, time base tick starts incrementing.
 *         In the default implementation, Systick is used as source of time base.
 *         The tick variable is incremented each 1ms in its ISR.
 *
 ****************************************************************************************
 */
void app_systick_init(void);

/**
 ****************************************************************************************
 * @brief  This function de-Initializes common part of the HAL and stops the source
 *         of time base.
 *
 * @note   This function is optional.
 *
 ****************************************************************************************
 */
void app_systick_deinit(void);

/**
 ****************************************************************************************
 * @brief  This function is called to increment  a global variable "g_tick"
 *         used as application time base.
 *
 * @note   In the default implementation, this variable is incremented by 1 each 1ms
 *         in Systick ISR.
 ****************************************************************************************
 */
uint32_t app_get_systick(void);

/** @} */

#endif

/** @} */


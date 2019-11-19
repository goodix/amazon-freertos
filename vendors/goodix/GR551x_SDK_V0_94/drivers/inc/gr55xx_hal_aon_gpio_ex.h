/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_aon_gpio_ex.h
 * @author  BLE Driver Team
 * @brief   Header file containing extended macro of AON GPIO HAL library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************************
 */

/** @addtogroup PERIPHERAL Peripheral Driver
  * @{
  */

/** @addtogroup HAL_DRIVER HAL Driver
  * @{
  */

/** @defgroup HAL_AON_GPIOEx AON_GPIOEx
  * @brief AON_GPIOEx HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_AON_GPIO_EX_H__
#define __GR55xx_HAL_AON_GPIO_EX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_gpio.h"

/* Exported types ------------------------------------------------------------*/

/**
  * @defgroup  HAL_AON_GPIOEX_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup AON_GPIOEx_Exported_Constants AON_GPIOEx Exported Constants
  * @{
  */

/** @defgroup AON_GPIOEx_Mux_Mode AON_GPIOEx Mux Mode definition
  * @{
  */
#define AON_GPIO_MUX_0                      LL_AON_GPIO_MUX_0   /**< AON GPIO mux mode 0 */
#define AON_GPIO_MUX_1                      LL_AON_GPIO_MUX_1   /**< AON GPIO mux mode 1 */
#define AON_GPIO_MUX_2                      LL_AON_GPIO_MUX_2   /**< AON GPIO mux mode 2 */
#define AON_GPIO_MUX_3                      LL_AON_GPIO_MUX_3   /**< AON GPIO mux mode 3 */
#define AON_GPIO_MUX_4                      LL_AON_GPIO_MUX_4   /**< AON GPIO mux mode 4 */
#define AON_GPIO_MUX_5                      LL_AON_GPIO_MUX_5   /**< AON GPIO mux mode 5 */
#define AON_GPIO_MUX_6                      LL_AON_GPIO_MUX_6   /**< AON GPIO mux mode 6 */
#define AON_GPIO_MUX_7                      LL_AON_GPIO_MUX_7   /**< AON GPIO mux mode 7 */
/** @} */

/** @defgroup AON_GPIOEx_Mux_Function_Selection AON_GPIOEx Mux function selection
  * @{
  */

#if defined (GR551xx)
/*---------------------------------- GR551xx ------------------------------*/

/** @defgroup AON_GPIOEx_Common_Selection AON_GPIO PIN common MUX selection(Available for all AON GPIO pins)
  * @{
  */

#define AON_GPIO_PIN_MUX_GPIO               AON_GPIO_MUX_7  /**< AON GPIO PIN x Mux Select GPIO */

/** @} */

/** @defgroup AON_GPIOEx_PIN0_Mux_Selection AON_GPIO_PIN1 MUX selection
  * @{
  */
#define AON_GPIO_PIN1_MUX_QSPIM0_CS_N       AON_GPIO_MUX_5  /**< AON_GPIO_PIN1 Mux Select QSPIM0_CS_N */
#define AON_GPIO_PIN1_MUX_COEX_BLE_TX       AON_GPIO_MUX_6  /**< AON_GPIO_PIN1 Mux Select COEX_BLE_TX */
/** @} */

/** @defgroup AON_GPIOEx_PIN2_Mux_Selection AON_GPIO_PIN2 MUX selection
  * @{
  */
#define AON_GPIO_PIN2_MUX_SIM_PRESENCE      AON_GPIO_MUX_0  /**< AON_GPIO_PIN2 Mux Select SIM_PRESENCE */
#define AON_GPIO_PIN2_MUX_QSPIM1_CS_N       AON_GPIO_MUX_1  /**< AON_GPIO_PIN2 Mux Select QSPIM1_CS_N */
#define AON_GPIO_PIN2_MUX_I2S_WS            AON_GPIO_MUX_2  /**< AON_GPIO_PIN2 Mux Select I2S_WS */
#define AON_GPIO_PIN2_MUX_I2S_S_WS          AON_GPIO_MUX_3  /**< AON_GPIO_PIN2 Mux Select I2S_S_WS */
#define AON_GPIO_PIN2_MUX_PWM0_C            AON_GPIO_MUX_5  /**< AON_GPIO_PIN2 Mux Select PWM0_C */
#define AON_GPIO_PIN2_MUX_COEX_BLE_PROC     AON_GPIO_MUX_6  /**< AON_GPIO_PIN2 Mux Select COEX_BLE_PROC */
/** @} */

/** @defgroup AON_GPIOEx_PIN3_Mux_Selection AON_GPIO_PIN3 MUX selection
  * @{
  */
#define AON_GPIO_PIN3_MUX_SIM_RST_N        AON_GPIO_MUX_0  /**< AON_GPIO_PIN3 Mux Select SIM_RST_N */
#define AON_GPIO_PIN3_MUX_QSPIM1_IO_0      AON_GPIO_MUX_1  /**< AON_GPIO_PIN3 Mux Select QSPIM1_IO_0 */
#define AON_GPIO_PIN3_MUX_I2S_TX_SDO       AON_GPIO_MUX_2  /**< AON_GPIO_PIN3 Mux Select I2S_TX_SDO */
#define AON_GPIO_PIN3_MUX_I2S_S_TX_SDO     AON_GPIO_MUX_3  /**< AON_GPIO_PIN3 Mux Select I2S_S_TX_SDO */
#define AON_GPIO_PIN3_MUX_PWM1_A           AON_GPIO_MUX_5  /**< AON_GPIO_PIN3 Mux Select PWM1_A */
#define AON_GPIO_PIN3_MUX_COEX_WLAN_RX     AON_GPIO_MUX_6  /**< AON_GPIO_PIN3 Mux Select COEX_WLAN_RX */
/** @} */

/** @defgroup AON_GPIOEx_PIN4_Mux_Selection AON_GPIO_PIN4 MUX selection
  * @{
  */
#define AON_GPIO_PIN4_MUX_SIM_IO           AON_GPIO_MUX_0  /**< AON_GPIO_PIN4 Mux Select SIM_IO */
#define AON_GPIO_PIN4_MUX_QSPIM1_IO_1      AON_GPIO_MUX_1  /**< AON_GPIO_PIN4 Mux Select QSPIM1_IO_1 */
#define AON_GPIO_PIN4_MUX_I2S_RX_SDI       AON_GPIO_MUX_2  /**< AON_GPIO_PIN4 Mux Select I2S_RX_SDI */
#define AON_GPIO_PIN4_MUX_I2S_S_RX_SDI     AON_GPIO_MUX_3  /**< AON_GPIO_PIN4 Mux Select I2S_S_RX_SDI */
#define AON_GPIO_PIN4_MUX_PWM1_B           AON_GPIO_MUX_5  /**< AON_GPIO_PIN4 Mux Select PWM1_B */
#define AON_GPIO_PIN4_MUX_COEX_BLE_RX      AON_GPIO_MUX_6  /**< AON_GPIO_PIN4 Mux Select COEX_BLE_RX */
/** @} */

/** @defgroup AON_GPIOEx_PIN5_Mux_Selection AON_GPIO_PIN5 MUX selection
  * @{
  */
#define AON_GPIO_PIN5_MUX_SIM_CLK          AON_GPIO_MUX_0  /**< AON_GPIO_PIN5 Mux Select SIM_CLK */
#define AON_GPIO_PIN5_MUX_QSPIM1_CLK       AON_GPIO_MUX_1  /**< AON_GPIO_PIN5 Mux Select QSPIM1_CLK */
#define AON_GPIO_PIN5_MUX_I2S_SCLK         AON_GPIO_MUX_2  /**< AON_GPIO_PIN5 Mux Select I2S_SCLK */
#define AON_GPIO_PIN5_MUX_I2S_S_SCLK       AON_GPIO_MUX_3  /**< AON_GPIO_PIN5 Mux Select I2S_S_SCLK */
#define AON_GPIO_PIN5_MUX_PWM1_C           AON_GPIO_MUX_5  /**< AON_GPIO_PIN5 Mux Select PWM1_C */
#define AON_GPIO_PIN5_MUX_COEX_WLAN_TX     AON_GPIO_MUX_6  /**< AON_GPIO_PIN5 Mux Select COEX_WLAN_TX */
/** @} */

/**
  * @brief Check if AON GPIO mux mode is valid.
  * @param __MUX__ AON GPIO mux mode.
  * @retval SET (__ACTION__ is valid) or RESET (__ACTION__ is invalid)
  */
#define IS_AON_GPIO_MUX(__MUX__)        (((__MUX__) <= AON_GPIO_MUX_7))

/*------------------------------------------------------------------------------------------*/
#endif /* GR551xx */

/** @} */

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_AON_GPIO_EX_H__ */

/** @} */

/** @} */

/** @} */


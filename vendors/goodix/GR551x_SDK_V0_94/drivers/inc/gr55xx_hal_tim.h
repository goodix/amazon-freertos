/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of TIM HAL library.
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

/** @defgroup HAL_TIM TIM
  * @brief TIM HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_TIM_H__
#define __GR55xx_HAL_TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_tim.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_TIM_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_TIM_state HAL TIM state
  * @{
  */

/**
  * @brief  HAL TIM State Enumerations definition
  */
typedef enum
{
    HAL_TIM_STATE_RESET             = 0x00,    /**< Peripheral not yet initialized or disabled  */
    HAL_TIM_STATE_READY             = 0x01,    /**< Peripheral Initialized and ready for use    */
    HAL_TIM_STATE_BUSY              = 0x02,    /**< An internal process is ongoing              */
    HAL_TIM_STATE_ERROR             = 0x04     /**< Reception process is ongoing                */
} hal_tim_state_t;
/** @} */

/** @} */

/** @addtogroup HAL_TIM_STRUCTURES Structures
  * @{
  */

/** @defgroup TIM_Configuration TIM Configuration
  * @{
  */

/**
  * @brief TIM init Structure definition
  */
typedef struct _tim_init
{
    uint32_t auto_reload;                   /**< Specifies the auto-reload value. */

} tim_init_t;

/** @} */

/** @defgroup TIM_handle TIM handle
  * @{
  */

/**
  * @brief TIM handle Structure definition
  */
typedef struct _tim_handle
{
    tim_regs_t               *p_instance;     /**< Register base address        */

    tim_init_t               init;          /**< TIM Base required parameters */

    __IO hal_lock_t          lock;          /**< Locking object               */

    __IO hal_tim_state_t     state;         /**< TIM operation state          */

} tim_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_TIM_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_TIM_Callback Callback
  * @{
  */

/**
  * @brief HAL_TIM Callback function definition
  */

typedef struct _hal_tim_callback
{
    void (*tim_msp_init)(tim_handle_t *p_timer);
    void (*tim_msp_deinit)(tim_handle_t *p_timer);
    void (*tim_period_elapsed_callback)(tim_handle_t *p_timer);
} hal_tim_callback_t;

/** @} */

/** @} */

/**
  * @defgroup  HAL_TIM_MACRO Defines
  * @{
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TIM_Exported_Macros TIM Exported Macros
  * @{
  */

/** @brief  Reset TIM handle states.
  * @param  __HANDLE__ TIM handle.
  * @retval None
  */
#define __HAL_TIM_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_TIM_STATE_RESET)

/** @brief  Enable the specified TIM peripheral.
  * @param  __HANDLE__ Specifies the TIM Handle.
  * @retval None
  */
#define __HAL_TIM_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL, TIM_CTRL_EN)

/** @brief  Disable the specified TIM peripheral.
  * @param  __HANDLE__ Specifies the TIM Handle.
  * @retval None
  */
#define __HAL_TIM_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL, TIM_CTRL_EN)

/** @brief  Enable the TIM interrupt.
  * @param  __HANDLE__ Specifies the TIM Handle.
  * @retval None
  */
#define __HAL_TIM_ENABLE_IT(__HANDLE__)                        SET_BITS((__HANDLE__)->p_instance->CTRL, TIM_CTRL_INTEN)

/** @brief  Disable the TIM interrupt.
  * @param  __HANDLE__ Specifies the TIM Handle.
  * @retval None
  */
#define __HAL_TIM_DISABLE_IT(__HANDLE__)                       CLEAR_BITS((__HANDLE__)->p_instance->CTRL, TIM_CTRL_INTEN)

/** @brief  Check whether the TIM interrupt has occurred or not.
  * @param  __HANDLE__ Specifies the TIM Handle.
  * @retval The new state of TIM interrupt (SET or RESET).
  */
#define __HAL_TIM_GET_FLAG_IT(__HANDLE__)                      ll_tim_is_active_flag_it(__HANDLE__->p_instance)

/** @brief  Clear the TIM interrupt flag.
  * @param  __HANDLE__ Specifies the TIM Handle.
  * @retval None
  */
#define __HAL_TIM_CLEAR_FLAG_IT(__HANDLE__)                    ll_tim_clear_flag_it(__HANDLE__->p_instance)

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_TIM_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup TIM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and de-initialization functions
  *
  * @verbatim
===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
        This section provides functions allowing to:
        (+) Initialize and configure the TIM.
        (+) De-initialize the TIM.
        (+) Start the Timer.
        (+) Stop the Timer.
        (+) Start the Timer and enable interrupt.
        (+) Stop the Timer and disable interrupt.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the TIM according to the specified parameters
 *         in the tim_init_t and initialize the associated handle.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_tim_init(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the TIM peripheral.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_tim_deinit(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Initialize the TIM MSP.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_tim_msp_init could be implemented in the user file.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 ****************************************************************************************
 */
void hal_tim_msp_init(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the TIM MSP.
 * @note   This function should not be modified. When the callback is needed,
 *         the hal_tim_msp_deinit could be implemented in the user file.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 ****************************************************************************************
 */
void hal_tim_msp_deinit(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Starts the TIM counter.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_tim_start(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Stops the TIM counter.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_tim_stop(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Starts the TIM counter in interrupt mode.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_tim_start_it(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Stops the TIM counter in interrupt mode.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_tim_stop_it(tim_handle_t *p_timer);

/** @} */

/** @addtogroup TIM_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief Handle TIM interrupt request.
 * @param[in] p_timer: TIM handle.
 ****************************************************************************************
 */
void hal_tim_irq_handler(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  Period elapsed callback in non-blocking mode.
 * @note   This function should not be modified. When the callback is needed,
            the hal_tim_period_elapsed_callback can be implemented in the user file.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 ****************************************************************************************
 */
void hal_tim_period_elapsed_callback(tim_handle_t *p_timer);

/** @} */

/** @addtogroup TIM_Exported_Functions_Group2 Peripheral Control and State functions
 *  @brief   TIM Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral Control and State functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the TIM handle state.
      (+) Configure the TIM.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the TIM handle state.
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                 information for the specified TIM module.
 * @retval ::HAL_TIM_STATE_RESET: Peripheral not yet initialized or disabled.
 * @retval ::HAL_TIM_STATE_READY: Peripheral Initialized and ready for use.
 * @retval ::HAL_TIM_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_TIM_STATE_ERROR: Reception process is ongoing.
 ****************************************************************************************
 */
hal_tim_state_t hal_tim_get_state(tim_handle_t *p_timer);

/**
 ****************************************************************************************
 * @brief  TIM configuration
 * @param[in]  p_timer: Pointer to a TIM handle which contains the configuration
 *                      information for the specified TIM module.
 * @param[in]  p_structure: The TIM configuration structure
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_tim_set_config(tim_handle_t *p_timer, tim_init_t *p_structure);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_TIM_H__ */

/** @} */

/** @} */

/** @} */

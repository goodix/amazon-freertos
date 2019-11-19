/**
 ****************************************************************************************
 *
 * @file    gr55xx_hal_dual_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DUAL TIM HAL library. 
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

/** @defgroup HAL_DUAL_TIM DUAL TIM
  * @brief DUAL TIM HAL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55xx_HAL_DUAL_TIM_H__
#define __GR55xx_HAL_DUAL_TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal_def.h"
#include "gr55xx_ll_dual_tim.h"

/* Exported types ------------------------------------------------------------*/
/** @addtogroup HAL_DUAL_TIM_ENUMERATIONS Enumerations
  * @{
  */

/** @defgroup HAL_DUAL_TIM_state HAL DUAL TIM state
  * @{
  */

/**
  * @brief  HAL DUAL TIM State Enumerations definition
  */
typedef enum
{
    HAL_DUAL_TIM_STATE_RESET             = 0x00,    /**< Peripheral not yet initialized or disabled  */
    HAL_DUAL_TIM_STATE_READY             = 0x01,    /**< Peripheral Initialized and ready for use    */
    HAL_DUAL_TIM_STATE_BUSY              = 0x02,    /**< An internal process is ongoing              */
    HAL_DUAL_TIM_STATE_ERROR             = 0x04     /**< Reception process is ongoing                */
} hal_dual_tim_state_t;
/** @} */

/** @} */

/** @addtogroup HAL_DUAL_TIM_STRUCTURES Structures
  * @{
  */

/** @defgroup DUAL_TIM_Configuration DUAL TIM Configuration
  * @{
  */

/**
  * @brief DUAL TIM init Structure definition
  */
typedef struct _dual_tim_init
{
    uint32_t prescaler;     /**< Specifies the prescaler value used to divide the DUAL_TIM clock.
                                 This parameter can be a value of @ref DUAL_TIM_Prescaler_Div */

    uint32_t counter_mode;  /**< Specifies the counter mode.
                                 This parameter can be a value of @ref DUAL_TIM_Counter_Mode */

    uint32_t auto_reload;   /**< Specifies the auto-reload value. */

} dual_tim_init_t;

/** @} */

/** @defgroup DUAL_TIM_handle DUAL TIM handle
  * @{
  */

/**
  * @brief DUAL_TIM handle Structure definition
  */
typedef struct _dual_tim_handle
{
    dual_tim_regs_t                *p_instance;     /**< Register base address               */

    dual_tim_init_t                init;          /**< DUAL_TIM Base required parameters   */

    __IO hal_lock_t                lock;          /**< Locking object                      */

    __IO hal_dual_tim_state_t      state;         /**< DUAL_TIM operation state            */

} dual_tim_handle_t;
/** @} */

/** @} */

/** @addtogroup HAL_DUAL_TIM_STRUCTURES Callback Structures
  * @{
  */

/** @defgroup HAL_DUAL_TIM_Callback Callback
  * @{
  */

/**
  * @brief HAL_DUAL_TIM Callback function definition
  */

typedef struct _hal_dual_tim_callback
{
    void (*dual_tim_msp_init)(dual_tim_handle_t *p_dual_timer);
    void (*dual_tim_msp_deinit)(dual_tim_handle_t *p_dual_timer);
    void (*dual_tim_period_elapsed_callback)(dual_tim_handle_t *p_dual_timer);
} hal_dual_tim_callback_t;

/**
  * @defgroup  HAL_DUAL_TIM_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup DUAL_TIM_Exported_Constants DUAL TIM Exported Constants
  * @{
  */

/** @defgroup DUAL_TIM_Prescaler_Div DUAL TIM Prescaler Division
  * @{
  */
#define DUAL_TIM_PRESCALER_DIV0         LL_DUAL_TIM_PRESCALER_DIV0      /**< 0 stages of prescale, clock is divided by 1.   */
#define DUAL_TIM_PRESCALER_DIV16        LL_DUAL_TIM_PRESCALER_DIV16     /**< 4 stages of prescale, clock is divided by 16.  */
#define DUAL_TIM_PRESCALER_DIV256       LL_DUAL_TIM_PRESCALER_DIV256    /**< 8 stages of prescale, clock is divided by 256. */
/** @} */

/** @defgroup DUAL_TIM_Counter_Mode DUAL TIM Counter Mode
  * @{
  */
#define DUAL_TIM_COUNTERMODE_LOOP       0x00000000U                     /**< DUAL TIM Loop mode.*/
#define DUAL_TIM_COUNTERMODE_ONESHOT    DUAL_TIM_CTRL_ONESHOT           /**< DUAL TIM One-shot mode. */
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup DUAL_TIM_Exported_Macros DUAL TIM Exported Macros
  * @{
  */

/** @brief  Reset DUAL TIM handle states.
  * @param  __HANDLE__ DUAL TIM handle.
  * @retval None
  */
#define __HAL_DUAL_TIM_RESET_HANDLE_STATE(__HANDLE__)               ((__HANDLE__)->state = HAL_DUAL_TIM_STATE_RESET)

/** @brief  Enable the specified DUAL TIM peripheral.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval None
  */
#define __HAL_DUAL_TIM_ENABLE(__HANDLE__)                           SET_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIM_CTRL_EN)

/** @brief  Disable the specified DUAL TIM peripheral.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval None
  */
#define __HAL_DUAL_TIM_DISABLE(__HANDLE__)                          CLEAR_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIM_CTRL_EN)

/** @brief  Enable the DUAL TIM interrupt.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval None
  */
#define __HAL_DUAL_TIM_ENABLE_IT(__HANDLE__)                        SET_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIM_CTRL_INTEN)

/** @brief  Disable the DUAL TIM interrupt.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval None
  */
#define __HAL_DUAL_TIM_DISABLE_IT(__HANDLE__)                       CLEAR_BITS((__HANDLE__)->p_instance->CTRL, DUAL_TIM_CTRL_INTEN)

/** @brief  Check whether the DUAL TIM interrupt has occurred or not.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval The new state of DUAL TIM interrupt (SET or RESET).
  */
#define __HAL_DUAL_TIM_GET_FLAG_IT(__HANDLE__)                      ll_dual_tim_is_active_flag_it(__HANDLE__->p_instance)

/** @brief  Clear the DUAL TIM interrupt flag.
  * @param  __HANDLE__ Specifies the DUAL TIM Handle.
  * @retval None.
  */
#define __HAL_DUAL_TIM_CLEAR_FLAG_IT(__HANDLE__)                    ll_dual_tim_clear_flag_it(__HANDLE__->p_instance)

/** @} */

/* Private macros ------------------------------------------------------------*/
/** @defgroup DUAL_TIM_Private_Macros DUAL TIM Private Macros
  * @{
  */

/** @brief  Check if DUAL TIM prescaler is valid.
  * @param  __PRESCALER__ DUAL TIM prescaler.
  * @retval SET (__PRESCALER__ is valid) or RESET (__PRESCALER__ is invalid)
  */
#define IS_DUAL_TIM_PRESCALER(__PRESCALER__)                        (((__PRESCALER__) == DUAL_TIM_PRESCALER_DIV0)  || \
                                                                     ((__PRESCALER__) == DUAL_TIM_PRESCALER_DIV16) || \
                                                                     ((__PRESCALER__) == DUAL_TIM_PRESCALER_DIV256))

/** @brief  Check if DUAL TIM counter mode is valid.
  * @param  __MODE__ DUAL TIM counter mode.
  * @retval SET (__MODE__ is valid) or RESET (__MODE__ is invalid)
  */
#define IS_DUAL_TIM_COUNTERMODE(__MODE__)                           (((__MODE__) == DUAL_TIM_COUNTERMODE_LOOP)  || \
                                                                     ((__MODE__) == DUAL_TIM_COUNTERMODE_ONESHOT))
/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_DUAL_TIM_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @addtogroup DUAL_TIM_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief    Initialization and de-initialization functions
  *
  * @verbatim
===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]
        This section provides functions allowing to:
        (+) Initialize and configure the DUAL TIM.
        (+) De-initialize the DUAL TIM.
        (+) Start the Timer.
        (+) Stop the Timer.
        (+) Start the Timer and enable interrupt.
        (+) Stop the Timer and disable interrupt.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Initialize the DUAL TIM according to the specified parameters
 *         in the dual_tim_init_t and initialize the associated handle.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_tim_init(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  De-initialize the DUAL TIM peripheral.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_tim_deinit(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief Initialize the DUAL TIM MSP.
 *
 * @note  This function should not be modified. When the callback is needed,
 *         the hal_dual_tim_msp_init could be implemented in the user file
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 ****************************************************************************************
 */
void hal_dual_tim_msp_init(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief De-initialize the DUAL TIM MSP.
 *
 * @note  This function should not be modified. When the callback is needed,
 *         the hal_dual_tim_msp_deinit could be implemented in the user file
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 ****************************************************************************************
 */
void hal_dual_tim_msp_deinit(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Starts the DUAL TIM counter.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_tim_start(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Stops the DUAL TIM counter.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_tim_stop(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Starts the DUAL TIM counter in interrupt mode.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_tim_start_it(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Stops the DUAL TIM counter in interrupt mode.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_tim_stop_it(dual_tim_handle_t *p_dual_timer);

/** @} */

/** @addtogroup DUAL_TIM_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */

/**
 ****************************************************************************************
 * @brief Handle DUAL TIM interrupt request.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 ****************************************************************************************
 */
void hal_dual_tim_irq_handler(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  Period elapsed callback in non-blocking mode.
 *
 * @note   This function should not be modified. When the callback is needed,
 *          the hal_dual_tim_period_elapsed_callback can be implemented in the user file.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 ****************************************************************************************
 */
void hal_dual_tim_period_elapsed_callback(dual_tim_handle_t *p_dual_timer);

/** @} */

/** @addtogroup DUAL_TIM_Exported_Functions_Group2 Peripheral Control and State functions
 *  @brief   DUAL TIM Peripheral State functions
 *
@verbatim
  ==============================================================================
            ##### Peripheral Control and State functions #####
  ==============================================================================
    [..]
    This subsection provides functions allowing to :
      (+) Return the DUAL TIM handle state.
      (+) Configure the DUAL TIM.

@endverbatim
  * @{
  */

/**
 ****************************************************************************************
 * @brief  Return the DUAL TIM handle state.
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 *
 * @retval ::HAL_DUAL_TIM_STATE_RESET: Peripheral not yet initialized or disabled.
 * @retval ::HAL_DUAL_TIM_STATE_READY: Peripheral Initialized and ready for use.
 * @retval ::HAL_DUAL_TIM_STATE_BUSY: An internal process is ongoing.
 * @retval ::HAL_DUAL_TIM_STATE_ERROR: Reception process is ongoing.
 ****************************************************************************************
 */
hal_dual_tim_state_t hal_dual_tim_get_state(dual_tim_handle_t *p_dual_timer);

/**
 ****************************************************************************************
 * @brief  DUAL TIM configuration
 *
 * @param[in]  p_dual_timer: Pointer to a DUAL_TIM handle which contains the configuration information for the specified DUAL TIM.
 * @param[in]  p_structure: The DUAL TIM configuration structure
 *
 * @retval ::HAL_OK: Operation is OK.
 * @retval ::HAL_ERROR: Parameter error or operation not supported.
 * @retval ::HAL_BUSY: Driver is busy.
 * @retval ::HAL_TIMEOUT: Timeout occurred.
 ****************************************************************************************
 */
hal_status_t hal_dual_tim_set_config(dual_tim_handle_t *p_dual_timer, dual_tim_init_t *p_structure);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __GR55xx_HAL_DUAL_TIM_H__ */

/** @} */

/** @} */

/** @} */

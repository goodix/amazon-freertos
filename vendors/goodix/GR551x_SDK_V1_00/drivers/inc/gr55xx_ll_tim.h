/**
 ****************************************************************************************
 *
 * @file    gr55xx_ll_tim.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of TIM LL library.
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

/** @addtogroup LL_DRIVER LL Driver
  * @{
  */

/** @defgroup LL_TIM TIM
  * @brief TIM LL module driver.
  * @{
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GR55XX_LL_TIM_H__
#define __GR55XX_LL_TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gr55xx.h"

#if defined (TIM0) || defined (TIM1)

/** @defgroup TIM_LL_STRUCTURES Structures
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup TIM_LL_ES_INIT TIM Exported init structures
  * @{
  */

/**
  * @brief LL TIM init Structure definition
  */
typedef struct _ll_tim_init_t
{
    uint32_t auto_reload;        /**< Specifies the auto reload value to be loaded into the active
                                     Auto-Reload Register at the next update event.
                                     This parameter must be a number between Min_Data=0x00000000 and Max_Data=0xFFFFFFFF.
                                     Some timer instances may support 32 bits counters. In that case this parameter must be a number between 0x0000 and 0xFFFFFFFF.

                                     This feature can be modified afterwards using unitary function @ref ll_tim_set_auto_reload().*/
} ll_tim_init_t;
/** @} */

/** @} */

/**
  * @defgroup  TIM_LL_TIM_MACRO Defines
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Constants TIM Exported Constants
  * @{
  */

/** @defgroup TIM_LL_EC_DEFAULT_CONFIG InitStrcut default configuartion
  * @{
  */
/**
  * @brief LL TIM InitStrcut default configuartion
  */
#define TIM_DEFAULT_CONFIG                \
{                                         \
    .auto_reload = SystemCoreClock - 1,    \
}
/** @} */

/** @} */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup TIM_LL_Exported_Macros TIM Exported Macros
  * @{
  */

/** @defgroup TIM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */

/**
  * @brief  Write a value in TIMER register
  * @param  __instance__ TIMER instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
#define LL_TIM_WriteReg(__instance__, __REG__, __VALUE__)   WRITE_REG(__instance__->__REG__, (__VALUE__))

/**
  * @brief  Read a value in TIMER register
  * @param  __instance__ TIMER instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
#define LL_TIM_ReadReg(__instance__, __REG__)               READ_REG(__instance__->__REG__)

/** @} */

/** @} */

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @defgroup TIM_LL_DRIVER_FUNCTIONS Functions
  * @{
  */

/** @defgroup TIM_LL_EF_Configuration Configuration functions
  * @{
  */

/**
  * @brief  Enable timer counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_tim_enable_counter(tim_regs_t *TIMx)
{
    SET_BITS(TIMx->CTRL, TIM_CTRL_EN);
}

/**
  * @brief  Disable timer counter.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_tim_disable_counter(tim_regs_t *TIMx)
{
    CLEAR_BITS(TIMx->CTRL, TIM_CTRL_EN);
}

/**
  * @brief  Indicates whether the timer counter is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | EN
  *
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_tim_is_enabled_counter(tim_regs_t *TIMx)
{
    return (READ_BITS(TIMx->CTRL, TIM_CTRL_EN) == (TIM_CTRL_EN));
}

/**
  * @brief  Set the counter value.
  *
  *  Register|BitsName
  *  --------|--------
  *  VALUE | VALUE
  *
  * @param  TIMx Timer instance
  * @param  counter Counter value (between Min_Data=0 and Max_Data=0xFFFFFFFF)
  * @retval None
  */
__STATIC_INLINE void ll_tim_set_counter(tim_regs_t *TIMx, uint32_t counter)
{
    WRITE_REG(TIMx->VALUE, counter);
}

/**
  * @brief  Get the counter value.
  *
  *  Register|BitsName
  *  --------|--------
  *  VALUE | VALUE
  *
  * @param  TIMx Timer instance
  * @retval Counter value (between Min_Data=0 and Max_Data=0xFFFFFFFF)
  */
__STATIC_INLINE uint32_t ll_tim_get_counter(tim_regs_t *TIMx)
{
    return (uint32_t)(READ_REG(TIMx->VALUE));
}

/**
  * @brief  Set the auto-reload value.
  * @note   The counter is blocked while the auto-reload value is null.
  *
  *  Register|BitsName
  *  --------|--------
  *  RELOAD | RELOAD
  *
  * @param  TIMx Timer instance
  * @param  auto_reload between Min_Data=0 and Max_Data=0xFFFFFFFF
  * @retval None
  */
__STATIC_INLINE void ll_tim_set_auto_reload(tim_regs_t *TIMx, uint32_t auto_reload)
{
    WRITE_REG(TIMx->RELOAD, auto_reload);
}

/**
  * @brief  Get the auto-reload value.
  *
  *  Register|BitsName
  *  --------|--------
  *  RELOAD | RELOAD
  *
  * @param  TIMx Timer instance
  * @retval Auto-reload value
  */
__STATIC_INLINE uint32_t ll_tim_get_auto_reload(tim_regs_t *TIMx)
{
    return (uint32_t)(READ_REG(TIMx->RELOAD));
}

/** @} */

/** @defgroup TIM_LL_EF_IT_Management IT_Management
  * @{
  */

/**
  * @brief  Enable timer interrupt.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_tim_enable_it(tim_regs_t *TIMx)
{
    SET_BITS(TIMx->CTRL, TIM_CTRL_INTEN);
}

/**
  * @brief  Disable timer interrput.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_tim_disable_it(tim_regs_t *TIMx)
{
    CLEAR_BITS(TIMx->CTRL, TIM_CTRL_INTEN);
}

/**
  * @brief  Indicates whether the timer interrput is enabled.
  *
  *  Register|BitsName
  *  --------|--------
  *  CTRL | INTEN
  *
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_tim_is_enabled_it(tim_regs_t *TIMx)
{
    return (READ_BITS(TIMx->CTRL, TIM_CTRL_INTEN) == (TIM_CTRL_INTEN));
}

/** @} */

/** @defgroup TIM_LL_EF_FLAG_Management FLAG_Management
  * @{
  */

/**
  * @brief  Clear the interrupt flag (INTSTAT).
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  TIMx Timer instance
  * @retval None
  */
__STATIC_INLINE void ll_tim_clear_flag_it(tim_regs_t *TIMx)
{
    WRITE_REG(TIMx->INTSTAT, TIM_INT_STAT);
}

/**
  * @brief  Indicate whether interrupt flag (INTSTAT) is set (interrupt is pending).
  *
  *  Register|BitsName
  *  --------|--------
  *  INTSTAT | INTSTAT
  *
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
__STATIC_INLINE uint32_t ll_tim_is_active_flag_it(tim_regs_t *TIMx)
{
    return (READ_BITS(TIMx->INTSTAT, TIM_INT_STAT) == (TIM_INT_STAT));
}

/** @} */

/** @defgroup TIM_LL_Init Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  De-initialize TIM registers (Registers restored to their default values).
  * @param  TIMx TIM instance
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: TIM registers are de-initialized
  *          - ERROR: TIM registers are not de-initialized
  */
error_status_t ll_tim_deinit(tim_regs_t *TIMx);

/**
  * @brief  Initialize TIM registers according to the specified
  *         parameters in TIM_InitStruct.
  * @param  TIMx TIM instance
  * @param  p_tim_init Pointer to a ll_tim_init_t structure that contains the configuration
  *                        information for the specified TIM peripheral.
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: TIM registers are initialized according to p_tim_init content
  *          - ERROR: Problem occurred during TIM Registers initialization
  */
error_status_t ll_tim_init(tim_regs_t *TIMx, ll_tim_init_t *p_tim_init);

/**
  * @brief Set each field of a @ref ll_tim_init_t type structure to default value.
  * @param p_tim_init  Pointer to a @ref ll_tim_init_t structure
  *                        whose fields will be set to default values.
  * @retval None
  */
void ll_tim_struct_init(ll_tim_init_t *p_tim_init);

/** @} */

/** @} */

#endif /* TIM0 || TIM1 */

#ifdef __cplusplus
}
#endif

#endif /* __GR55XX_LL_TIM_H__ */

/** @} */

/** @} */

/** @} */

/**
  ****************************************************************************************
  * @file    gr55xx_hal.c
  * @author  BLE Driver Team
  * @brief   HAL module driver.
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

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup HAL_Private Constants
  * @{
  */
/**
 * @brief GR55xx HAL Driver version number V1.0.0
   */
#define __GR55xx_HAL_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __GR55xx_HAL_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __GR55xx_HAL_VERSION_SUB2   (0x00U) /*!< [15:8]  sub2 version */
#define __GR55xx_HAL_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */
#define __GR55xx_HAL_VERSION         ((__GR55xx_HAL_VERSION_MAIN << 24U)\
                                     |(__GR55xx_HAL_VERSION_SUB1 << 16U)\
                                     |(__GR55xx_HAL_VERSION_SUB2 << 8U )\
                                     |(__GR55xx_HAL_VERSION_RC))

#define IDCODE_DEVID_MASK    (0x00000FFFU)
/** @} */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/** @defgroup HAL_Private_Variables HAL Private Variables
  * @{
  */
__IO uint32_t g_tick;
/** @} */

/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/** @defgroup HAL_Exported_Functions HAL Exported Functions
  * @{
  */

/** @defgroup HAL_Exported_Functions_Group1 Initialization and de-initialization Functions
 *  @brief    Initialization and de-initialization functions
  * @{
  */

__WEAK hal_status_t hal_init(void)
{
    /* Set Interrupt Group Priority */
    hal_nvic_set_priority_grouping(NVIC_PRIORITYGROUP_4);

    /* Enable systick and configure 1ms tick (default clock after Reset is HSI) */
    hal_init_tick(TICK_INT_PRIORITY);

    /* init the low level hardware */
    hal_msp_init();

    /* Return function status */
    return HAL_OK;
}

__WEAK hal_status_t hal_deinit(void)
{
    /* Reset of all peripherals */

    /* Disable Systick */
    SysTick->CTRL = 0;
    hal_nvic_disable_irq(SysTick_IRQn);

    /* De-init the low level hardware */
    hal_msp_deinit();

    /* Return function status */
    return HAL_OK;
}

__WEAK void hal_msp_init(void)
{
    /* NOTE : This function Should not be modified, when the callback is needed,
            the hal_msp_deinit could be implemented in the user file
    */
}

__WEAK void hal_msp_deinit(void)
{
    /* NOTE : This function Should not be modified, when the callback is needed,
            the hal_msp_deinit could be implemented in the user file
    */
}

__WEAK hal_status_t hal_init_tick(uint32_t tick_priority)
{
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    /*Configure the SysTick to have interrupt in 1ms time basis*/
    hal_systick_config(SystemCoreClock / 1000U);

    /*Configure the SysTick IRQ priority */
    hal_nvic_set_priority(SysTick_IRQn, tick_priority, 0U);

    /* Return function status */
    return HAL_OK;
}

/** @} */

/** @defgroup HAL_Exported_Functions_Group2 HAL Control functions
 *  @brief    HAL Control functions
  * @{
  */

__WEAK void hal_increment_tick(void)
{
    g_tick++;
}

__WEAK uint32_t hal_get_tick(void)
{
    return g_tick;
}

__WEAK void hal_delay(__IO uint32_t delay)
{
    uint32_t tickstart = hal_get_tick();
#if 0
    uint32_t wait = delay;

    /* Add a period to guarantee minimum wait */
    if (HAL_MAX_DELAY > wait)
    {
        wait++;
    }

    while ((hal_get_tick() - tickstart) < wait)
    {
    }
#else
    while ((uint32_t)(hal_get_tick() - tickstart) < delay)
    {
    }
#endif
}

__WEAK void hal_suspend_tick(void)
{
    /* Disable SysTick Interrupt */
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

__WEAK void hal_resume_tick(void)
{
    /* Enable SysTick Interrupt */
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

__WEAK uint32_t hal_get_hal_version(void)
{
    return __GR55xx_HAL_VERSION;
}

/** @} */

/** @} */

#endif /* HAL_MODULE_ENABLED */

/** @} */

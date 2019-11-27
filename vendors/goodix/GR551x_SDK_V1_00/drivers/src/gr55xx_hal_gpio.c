/**
  ****************************************************************************************
  * @file    gr55xx_hal_gpio.c
  * @author  BLE Driver Team
  * @brief   GPIO HAL module driver.
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

#ifdef HAL_GPIO_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup GPIO_Private_Defines GPIO Private Defines
  * @{
  */
#define GPIO_MODE             (0x00000003U)

#define GPIO_MODE_IT_Pos      (4U)
#define GPIO_MODE_IT          (0x7U << GPIO_MODE_IT_Pos)

#define GPIO_NUMBER           (16U)

#define __GPIO_DEFAULT_PIN_MUX    LL_GPIO_MUX_7

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup GPIO_Private_Macros GPIO Private Macros
  * @{
  */
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup GPIO_Exported_Functions GPIO Exported Functions
  * @{
  */

/** @defgroup GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions
  * @{
  */

__WEAK void hal_gpio_init(gpio_regs_t *GPIOx, gpio_init_t *p_gpio_init)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    gr_assert_param(IS_GPIO_PIN(p_gpio_init->pin));
    gr_assert_param(IS_GPIO_MODE(p_gpio_init->mode));

    ll_gpio_init_t init;

    init.pin = p_gpio_init->pin;
    init.mode = p_gpio_init->mode & GPIO_MODE;
    init.pull = p_gpio_init->pull;
    init.mux = p_gpio_init->mux;
    init.trigger = (p_gpio_init->mode >> GPIO_MODE_IT_Pos);

    ll_gpio_init(GPIOx, &init);
}

__WEAK void hal_gpio_deinit(gpio_regs_t *GPIOx, uint32_t gpio_pin)
{
    uint32_t current_pin = 0x00000000U;
    uint32_t pin_tmp     = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));

    /* Data output register set to default reset values */
    ll_gpio_reset_output_pin(GPIOx, gpio_pin);
    /* Output enable register set to default reset values */
    ll_gpio_set_pin_mode(GPIOx, gpio_pin, GPIO_MODE_INPUT);
    /* Disable Interrupt */
    ll_gpio_disable_it(GPIOx, gpio_pin);
    /* Interrupt status clear*/
    __HAL_GPIO_IT_CLEAR_IT(GPIOx, gpio_pin);

    ll_gpio_set_pin_pull(GPIOx, gpio_pin, GPIO_PULLDOWN);

    pin_tmp = gpio_pin;
    while (pin_tmp)
    {
        current_pin = (0x1U << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1);

        if (GPIO_PIN_8 > current_pin)
        {
            if ((GPIO0 == GPIOx) && ((GPIO_PIN_0 == current_pin) || (GPIO_PIN_1 == current_pin)))
            {
                ll_gpio_set_mux_pin_0_7(GPIOx, current_pin, LL_GPIO_MUX_0);
            }
            else
            {
                ll_gpio_set_mux_pin_0_7(GPIOx, current_pin, __GPIO_DEFAULT_PIN_MUX);
            }
        }
        else
        {
            ll_gpio_set_mux_pin_8_15(GPIOx, current_pin, __GPIO_DEFAULT_PIN_MUX);
        }
    }
}

/**
  * @}
  */

/** @defgroup GPIO_Exported_Functions_Group2 IO operation functions
 *  @brief GPIO Read, Write, Toggle, Lock and EXTI management functions.
  * @{
  */

__WEAK gpio_pin_state_t hal_gpio_read_pin(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_PIN(gpio_pin));

    return (gpio_pin_state_t)ll_gpio_is_input_pin_set(GPIOx, gpio_pin);
}

__WEAK void hal_gpio_write_pin(gpio_regs_t* GPIOx, uint16_t gpio_pin, gpio_pin_state_t pin_state)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_PIN(gpio_pin));
    gr_assert_param(IS_GPIO_PIN_ACTION(pin_state));

    if (GPIO_PIN_RESET != pin_state)
    {
        ll_gpio_set_output_pin(GPIOx, gpio_pin);
    }
    else
    {
        ll_gpio_reset_output_pin(GPIOx, gpio_pin);
    }

}

__WEAK void hal_gpio_toggle_pin(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_GPIO_PIN(gpio_pin));

    ll_gpio_toggle_pin(GPIOx, gpio_pin);
}

__WEAK void hal_gpio_exti_irq_handler(gpio_regs_t *GPIOx)
{
    uint16_t Triggered_Pin = __HAL_GPIO_IT_GET_IT(GPIOx, GPIO_PIN_ALL);

    /* GPIO pin interrupt detected */
    if (RESET != Triggered_Pin)
    {
        __HAL_GPIO_IT_CLEAR_IT(GPIOx, Triggered_Pin);
        hal_gpio_exti_callback(GPIOx, Triggered_Pin);
    }
}

__WEAK void hal_gpio_exti_callback(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(GPIOx);
    UNUSED(gpio_pin);
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_GPIO_MODULE_ENABLED */

/**
  * @}
  */
 

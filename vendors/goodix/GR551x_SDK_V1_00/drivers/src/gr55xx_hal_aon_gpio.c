/**
  ****************************************************************************************
  * @file    gr55xx_hal_aon_gpio.c
  * @author  BLE Driver Team
  * @brief   AON GPIO HAL module driver.
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

#ifdef HAL_AON_GPIO_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup AON_GPIO_Private_Defines AON_GPIO Private Defines
  * @{
  */
#define AON_GPIO_MODE             (0x00000003U)

#define AON_GPIO_MODE_IT_Pos      (4U)
#define AON_GPIO_MODE_IT          (0x7U << AON_GPIO_MODE_IT_Pos)

#define AON_GPIO_NUMBER           (6U)

#define __AON_GPIO_DEFAULT_PIN_MUX    LL_GPIO_MUX_0
/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup AON_GPIO_Private_Macros AON_GPIO Private Macros
  * @{
  */
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup AON_GPIO_Exported_Functions AON_GPIO Exported Functions
  * @{
  */

/** @defgroup AON_GPIO_Exported_Functions_Group1 Initialization/de-initialization functions
  * @{
  */

__WEAK void hal_aon_gpio_init(aon_gpio_init_t *p_aon_gpio_init)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(p_aon_gpio_init->pin));
    gr_assert_param(IS_AON_GPIO_MODE(p_aon_gpio_init->mode));
    gr_assert_param(IS_AON_GPIO_PULL(p_aon_gpio_init->pull));

    ll_aon_gpio_init_t aon_gpio_init;

    aon_gpio_init.pin  = p_aon_gpio_init->pin;
    aon_gpio_init.mode = p_aon_gpio_init->mode & AON_GPIO_MODE;
    aon_gpio_init.pull = p_aon_gpio_init->pull;
    aon_gpio_init.mux  = p_aon_gpio_init->mux;
    aon_gpio_init.trigger = p_aon_gpio_init->mode >> AON_GPIO_MODE_IT_Pos;

    ll_aon_gpio_init(&aon_gpio_init);
}

__WEAK void hal_aon_gpio_deinit(uint32_t aon_gpio_pin)
{
    uint32_t current_pin = 0x00000000U;
    uint32_t pin_tmp     = 0x00000000U;

    /* Data output register set to default reset values */
    ll_aon_gpio_reset_output_pin(aon_gpio_pin);
    /* Output enable register set to default reset values */
    ll_aon_gpio_set_pin_mode(aon_gpio_pin, GPIO_MODE_INPUT);
    /* Disable Interrupt */
    ll_aon_gpio_disable_it(aon_gpio_pin);
    /* Interrupt status clear*/
    __HAL_AON_GPIO_IT_CLEAR_IT(aon_gpio_pin);

    ll_aon_gpio_set_pin_pull(aon_gpio_pin, GPIO_PULLDOWN);

    pin_tmp = aon_gpio_pin;
    while (pin_tmp)
    {
        current_pin = (0x1U << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1);

        ll_aon_gpio_set_mux_pin_0_7(current_pin, LL_AON_GPIO_MUX_7);
    }
}

/**
  * @}
  */

/** @defgroup AON_GPIO_Exported_Functions_Group2 IO operation functions
  * @{
  */

__WEAK aon_gpio_pin_state_t hal_aon_gpio_read_pin(uint16_t aon_gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(aon_gpio_pin));

    return (aon_gpio_pin_state_t)ll_aon_gpio_is_input_pin_set(aon_gpio_pin);
}

__WEAK void hal_aon_gpio_write_pin(uint16_t aon_gpio_pin, aon_gpio_pin_state_t pin_state)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(aon_gpio_pin));
    gr_assert_param(IS_AON_GPIO_PIN_ACTION(pin_state));

    if (AON_GPIO_PIN_RESET != pin_state)
    {
        ll_aon_gpio_set_output_pin(aon_gpio_pin);
    }
    else
    {
        ll_aon_gpio_reset_output_pin(aon_gpio_pin);
    }

}

__WEAK void hal_aon_gpio_toggle_pin(uint16_t aon_gpio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_AON_GPIO_PIN(aon_gpio_pin));

    ll_aon_gpio_toggle_pin(aon_gpio_pin);
}


__WEAK void hal_aon_gpio_irq_handler(void)
{
    uint16_t Triggered_Pin = __HAL_AON_GPIO_IT_GET_IT(AON_GPIO_PIN_ALL);

    /* AON_GPIO pin interrupt detected */
    if (RESET != Triggered_Pin)
    {
        __HAL_AON_GPIO_IT_CLEAR_IT(Triggered_Pin);

        /* Clear external wakeup status */
        ll_pwr_clear_ext_wakeup_status(Triggered_Pin);

        /* Clear external wakeup event */
        ll_pwr_clear_wakeup_event(LL_PWR_WKUP_EVENT_EXT);
		
        hal_aon_gpio_callback(Triggered_Pin);
    }
}

__WEAK void hal_aon_gpio_callback(uint16_t aon_gpio_pin)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(aon_gpio_pin);
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_AON_GPIO_MODULE_ENABLED */

/**
  * @}
  */

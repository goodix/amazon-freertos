/**
  ****************************************************************************************
  * @file    gr55xx_hal_msio.c
  * @author  BLE Driver Team
  * @brief   MSIO HAL module driver.
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

#ifdef HAL_MSIO_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @defgroup MSIO_Private_Macros MSIO Private Macros
  * @{
  */
/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup MSIO_Exported_Functions MSIO Exported Functions
  * @{
  */

/** @defgroup MSIO_Exported_Functions_Group1 Initialization/de-initialization functions
  * @{
  */

__WEAK void hal_msio_init(msio_init_t *p_msio_init)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(p_msio_init->pin));
    gr_assert_param(IS_MSIO_DIRECTION(p_msio_init->direction));
    gr_assert_param(IS_MSIO_MODE(p_msio_init->mode));

    /* ------------------------- Configure the port pins ---------------- */
    ll_msio_init((ll_msio_init_t *)p_msio_init);
}

__WEAK void hal_msio_deinit(uint32_t msio_pin)
{
    uint32_t current_pin = 0x00000000U;
    uint32_t pin_tmp     = 0x00000000U;

    /* Output enable register set to default reset values */
    ll_msio_set_pin_direction(msio_pin, MSIO_DIRECTION_INPUT);
    ll_msio_set_pin_mode(msio_pin, MSIO_MODE_DIGITAL);
    ll_msio_set_pin_pull(msio_pin, MSIO_PULLDOWN);

    pin_tmp = msio_pin;
    while (pin_tmp)
    {
        current_pin = (0x1U << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1);

        ll_msio_set_pin_mux(current_pin, LL_MSIO_MUX_7);
    }
}

/**
  * @}
  */

/** @defgroup MSIO_Exported_Functions_Group2 IO operation functions
  * @{
  */

__WEAK msio_pin_state_t hal_msio_read_pin(uint16_t msio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(msio_pin));

    return (msio_pin_state_t)ll_msio_is_input_pin_set(msio_pin);
}

__WEAK void hal_msio_write_pin(uint16_t msio_pin, msio_pin_state_t pin_state)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(msio_pin));
    gr_assert_param(IS_MSIO_PIN_ACTION(pin_state));

    if (MSIO_PIN_RESET != pin_state)
    {
        ll_msio_set_output_pin(msio_pin);
    }
    else
    {
        ll_msio_reset_output_pin(msio_pin);
    }

}

__WEAK void hal_msio_toggle_pin(uint16_t msio_pin)
{
    /* Check the parameters */
    gr_assert_param(IS_MSIO_PIN(msio_pin));

    ll_msio_toggle_pin(msio_pin);
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_MSIO_MODULE_ENABLED */

/**
  * @}
  */

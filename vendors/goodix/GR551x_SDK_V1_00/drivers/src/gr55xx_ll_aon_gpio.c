/**
  ****************************************************************************************
  * @file    gr55xx_ll_aon_gpio.c
  * @author  BLE Driver Team
  * @brief   AON GPIO LL module driver.
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
#include "gr55xx_ll_aon_gpio.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */


/** @addtogroup GPIO_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup GPIO_LL_Private_Macros
  * @{
  */
#define IS_LL_AON_GPIO_PIN(__PIN__)         (((0x00000000U) < (__PIN__)) && ((__PIN__) <= LL_AON_GPIO_PIN_ALL))

#define IS_LL_AON_GPIO_MODE(__MODE__)       (((__MODE__) == LL_AON_GPIO_MODE_INPUT)  ||\
                                             ((__MODE__) == LL_AON_GPIO_MODE_OUTPUT) ||\
                                             ((__MODE__) == LL_AON_GPIO_MODE_MUX))

#define IS_LL_AON_GPIO_PULL(__PULL__)       (((__PULL__) == LL_AON_GPIO_PULL_NO)   ||\
                                             ((__PULL__) == LL_AON_GPIO_PULL_UP)   ||\
                                             ((__PULL__) == LL_AON_GPIO_PULL_DOWN))

#define IS_LL_AON_GPIO_MUX(__MUX__)         ((__PULL__) <= LL_AON_GPIO_MUX_8)

#define IS_LL_AON_GPIO_TRIGGER(__TRIGGER__) (((__TRIGGER__) == LL_AON_GPIO_TRIGGER_NONE)   ||\
                                             ((__TRIGGER__) == LL_AON_GPIO_TRIGGER_RISING) ||\
                                             ((__TRIGGER__) == LL_AON_GPIO_TRIGGER_FALLING)||\
                                             ((__TRIGGER__) == LL_AON_GPIO_TRIGGER_HIGH)   ||\
                                             ((__TRIGGER__) == LL_AON_GPIO_TRIGGER_LOW))

/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup GPIO_LL_Exported_Functions
  * @{
  */

/** @addtogroup GPIO_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize GPIO registers (Registers restored to their default values).
  * @param  gpio GPIO Port
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are de-initialized
  *          - ERROR:   Wrong GPIO Port
  */
__WEAK error_status_t ll_aon_gpio_deinit(void)
{
    error_status_t status = SUCCESS;

    /* Data AON_PAD_CTL1 register set to default reset values */
    LL_AON_GPIO_WriteReg(AON, AON_PAD_CTL1, 0x0000FFU);
    /* Output enable register set to default reset values */
    LL_AON_GPIO_WriteReg(AON, AON_PAD_CTL0, 0x000000U);
    /* Interrupt type register set to default reset values */
    LL_AON_GPIO_WriteReg(GPIO2, INTPOLCLR, LL_AON_GPIO_PIN_ALL);
    /* Interrupt type register set to default reset values */
    LL_AON_GPIO_WriteReg(GPIO2, INTTYPECLR, LL_AON_GPIO_PIN_ALL);
    /* Interrupt enable register set to default reset values */
    LL_AON_GPIO_WriteReg(GPIO2, INTENCLR, LL_AON_GPIO_PIN_ALL);
    /* Interrupt status clear*/
    LL_AON_GPIO_WriteReg(GPIO2, INTSTAT, LL_AON_GPIO_PIN_ALL);

    return (status);
}

/**
  * @brief  Initialize GPIO registers according to the specified parameters in aon_gpio_init_t.
  * @param  gpio GPIO Port
  * @param  p_aon_gpio_init pointer to a @ref aon_gpio_init_t structure
  *         that contains the AON_GPIO_InitStructuration information for the specified GPIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to aon_gpio_init_t content
  *          - ERROR:   Not applicable
  */
__WEAK error_status_t ll_aon_gpio_init(ll_aon_gpio_init_t *p_aon_gpio_init)
{
    uint32_t current_pin = 0x00000000U;
    uint32_t pin_tmp     = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_LL_AON_GPIO_PIN(p_aon_gpio_init->pin));
    gr_assert_param(IS_LL_AON_GPIO_MODE(p_aon_gpio_init->mode));
    gr_assert_param(IS_LL_AON_GPIO_PULL(p_aon_gpio_init->pull));
    gr_assert_param(IS_LL_AON_GPIO_MUX(p_aon_gpio_init->mux));
    gr_assert_param(IS_LL_AON_GPIO_TRIGGER(p_aon_gpio_init->trigger));

    /* ------------------------- Configure the port pins ---------------- */
    pin_tmp = p_aon_gpio_init->pin;
    while (pin_tmp)
    {
        current_pin = (0x1U << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1);
        ll_aon_gpio_set_mux_pin_0_7(current_pin, p_aon_gpio_init->mux);
    }

    ll_aon_gpio_set_pin_pull(p_aon_gpio_init->pin, p_aon_gpio_init->pull);
    ll_aon_gpio_set_pin_mode(p_aon_gpio_init->pin, p_aon_gpio_init->mode);

    ll_aon_gpio_disable_it(p_aon_gpio_init->pin);
    if (LL_AON_GPIO_MODE_INPUT == p_aon_gpio_init->mode)
    {
        ll_aon_gpio_clear_flag_it(p_aon_gpio_init->pin);
        switch (p_aon_gpio_init->trigger)
        {
        case LL_AON_GPIO_TRIGGER_FALLING:
            ll_aon_gpio_enable_falling_trigger(p_aon_gpio_init->pin);
            ll_aon_gpio_enable_it(p_aon_gpio_init->pin);
            break;

        case LL_AON_GPIO_TRIGGER_RISING:
            ll_aon_gpio_enable_rising_trigger(p_aon_gpio_init->pin);
            ll_aon_gpio_enable_it(p_aon_gpio_init->pin);
            break;

        case LL_AON_GPIO_TRIGGER_HIGH:
            ll_aon_gpio_enable_high_trigger(p_aon_gpio_init->pin);
            ll_aon_gpio_enable_it(p_aon_gpio_init->pin);
            break;

        case LL_AON_GPIO_TRIGGER_LOW:
            ll_aon_gpio_enable_low_trigger(p_aon_gpio_init->pin);
            ll_aon_gpio_enable_it(p_aon_gpio_init->pin);
            break;

        default:
            break;
        }
    }

    return (SUCCESS);
}

/**
  * @brief Set each @ref ll_aon_gpio_init_t field to default value.
  * @param p_aon_gpio_init pointer to a @ref ll_aon_gpio_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_aon_gpio_struct_init(ll_aon_gpio_init_t *p_aon_gpio_init)
{
    /* Reset GPIO init structure parameters values */
    p_aon_gpio_init->pin        = LL_AON_GPIO_PIN_ALL;
    p_aon_gpio_init->mode       = LL_AON_GPIO_MODE_INPUT;
    p_aon_gpio_init->pull       = LL_AON_GPIO_PULL_DOWN;
    p_aon_gpio_init->mux        = LL_AON_GPIO_MUX_7;
    p_aon_gpio_init->trigger    = LL_AON_GPIO_TRIGGER_NONE;
}

/** @} */

/** @} */

/** @} */

/** @} */

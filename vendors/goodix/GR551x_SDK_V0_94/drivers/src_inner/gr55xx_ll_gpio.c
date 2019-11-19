/**
  ****************************************************************************************
  * @file    gr55xx_ll_gpio.c
  * @author  BLE Driver Team
  * @brief   GPIO LL module driver.
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
#include "gr55xx_ll_gpio.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */

#if defined (GPIO0) || defined (GPIO1)

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
#define IS_LL_GPIO_PIN(__PIN__)             (((0x00000000U) < (__PIN__)) && ((__PIN__) <= LL_GPIO_PIN_ALL))

#define IS_LL_GPIO_MODE(__MODE__)           (((__MODE__) == LL_GPIO_MODE_INPUT)  ||\
                                             ((__MODE__) == LL_GPIO_MODE_OUTPUT) ||\
                                             ((__MODE__) == LL_GPIO_MODE_MUX))

#define IS_LL_GPIO_PULL(__PULL__)           (((__PULL__) == LL_GPIO_PULL_NO)   ||\
                                             ((__PULL__) == LL_GPIO_PULL_UP)   ||\
                                             ((__PULL__) == LL_GPIO_PULL_DOWN))

#define IS_LL_GPIO_MUX(__MUX__)             ((__MUX__) <= LL_GPIO_MUX_8)

#define IS_LL_GPIO_TRIGGER(__TRIGGER__)     (((__TRIGGER__) == LL_GPIO_TRIGGER_NONE)   ||\
                                             ((__TRIGGER__) == LL_GPIO_TRIGGER_RISING) ||\
                                             ((__TRIGGER__) == LL_GPIO_TRIGGER_FALLING)||\
                                             ((__TRIGGER__) == LL_GPIO_TRIGGER_HIGH)   ||\
                                             ((__TRIGGER__) == LL_GPIO_TRIGGER_LOW))

#define __LL_GPIO_DEFAULT_MUX_PIN0_7        0x77777700U
#define __LL_GPIO_DEFAULT_MUX_PIN8_15       0x77777777U
#define __LL_GPIO_DEFAULT_MUX_PIN16_23      0x77777777U
#define __LL_GPIO_DEFAULT_MUX_PIN24_31      0x77777777U

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
__WEAK error_status_t ll_gpio_deinit(gpio_regs_t *GPIOx)
{
    error_status_t status = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));

    /* Data output register set to default reset values */
    LL_GPIO_WriteReg(GPIOx, DATAOUT, 0x0000);
    /* Output enable register set to default reset values */
    LL_GPIO_WriteReg(GPIOx, OUTENCLR, LL_GPIO_PIN_ALL);
    /* Alternate function register set to default reset values */
    LL_GPIO_WriteReg(GPIOx, ALTFUNCCLR, LL_GPIO_PIN_ALL);
    /* Interrupt type register set to default reset values */
    LL_GPIO_WriteReg(GPIOx, INTPOLCLR, LL_GPIO_PIN_ALL);
    /* Interrupt type register set to default reset values */
    LL_GPIO_WriteReg(GPIOx, INTTYPECLR, LL_GPIO_PIN_ALL);
    /* Interrupt enable register set to default reset values */
    LL_GPIO_WriteReg(GPIOx, INTENCLR, LL_GPIO_PIN_ALL);
    /* Interrupt status clear*/
    LL_GPIO_WriteReg(GPIOx, INTSTAT, LL_GPIO_PIN_ALL);

    if (GPIO0 == GPIOx)
    {
        /* Dpad mux control register set to default values */
        LL_GPIO_WriteReg(MCU_SUB, DPAD_MUX_CTL0_7, __LL_GPIO_DEFAULT_MUX_PIN0_7);
        LL_GPIO_WriteReg(MCU_SUB, DPAD_MUX_CTL8_15, __LL_GPIO_DEFAULT_MUX_PIN8_15);
        /* Resistor enable register set to default values */
        MODIFY_REG(MCU_SUB->DPAD_RE_N_BUS, LL_GPIO_PIN_ALL, 0x0U);
        /* Resistor type register set to default values */
        MODIFY_REG(MCU_SUB->DPAD_RTYP_BUS, LL_GPIO_PIN_ALL, 0x0U);
    }
    else
    {
        /* Dpad mux control register set to default values */
        LL_GPIO_WriteReg(MCU_SUB, DPAD_MUX_CTL16_23, __LL_GPIO_DEFAULT_MUX_PIN16_23);
        LL_GPIO_WriteReg(MCU_SUB, DPAD_MUX_CTL24_31, __LL_GPIO_DEFAULT_MUX_PIN24_31);

        /* Resistor enable register set to default values */
        MODIFY_REG(MCU_SUB->DPAD_RE_N_BUS, (uint32_t)(LL_GPIO_PIN_ALL << 16), 0x0U);
        /* Resistor type register set to default values */
        MODIFY_REG(MCU_SUB->DPAD_RTYP_BUS, (uint32_t)(LL_GPIO_PIN_ALL << 16), 0x0U);
    }
    return (status);
}

/**
  * @brief  Initialize GPIO registers according to the specified parameters in p_gpio_init.
  * @param  gpio GPIO Port
  * @param  p_gpio_init   pointer to a @ref ll_gpio_init_t structure
  *         that contains the GPIO_InitStructuration information for the specified GPIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: GPIO registers are initialized according to p_gpio_init content
  *          - ERROR:   Not applicable
  */
__WEAK error_status_t ll_gpio_init(gpio_regs_t *GPIOx, ll_gpio_init_t *p_gpio_init)
{
    uint32_t current_pin = 0x00000000U;
    uint32_t pin_tmp     = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
    gr_assert_param(IS_LL_GPIO_PIN(p_gpio_init->pin));
    gr_assert_param(IS_LL_GPIO_MODE(p_gpio_init->mode));
    gr_assert_param(IS_LL_GPIO_PULL(p_gpio_init->pull));
    gr_assert_param(IS_LL_GPIO_MUX(p_gpio_init->mux));
    gr_assert_param(IS_LL_GPIO_TRIGGER(p_gpio_init->trigger));

    /* ------------------------- Configure the port pins ---------------- */
    pin_tmp = p_gpio_init->pin;
    while (pin_tmp)
    {
        current_pin = (0x1U << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1);

        if (LL_GPIO_PIN_8 > current_pin)
        {
            ll_gpio_set_mux_pin_0_7(GPIOx, current_pin, p_gpio_init->mux);
        }
        else
        {
            ll_gpio_set_mux_pin_8_15(GPIOx, current_pin, p_gpio_init->mux);
        }
    }

    ll_gpio_set_pin_pull(GPIOx, p_gpio_init->pin, p_gpio_init->pull);
    ll_gpio_set_pin_mode(GPIOx, p_gpio_init->pin, p_gpio_init->mode);

    ll_gpio_disable_it(GPIOx, p_gpio_init->pin);
    if (LL_GPIO_MODE_INPUT == p_gpio_init->mode)
    {
        ll_gpio_clear_flag_it(GPIOx, p_gpio_init->pin);
        switch (p_gpio_init->trigger)
        {
        case LL_GPIO_TRIGGER_FALLING:
            ll_gpio_enable_falling_trigger(GPIOx, p_gpio_init->pin);
            ll_gpio_enable_it(GPIOx, p_gpio_init->pin);
            break;

        case LL_GPIO_TRIGGER_RISING:
            ll_gpio_enable_rising_trigger(GPIOx, p_gpio_init->pin);
            ll_gpio_enable_it(GPIOx, p_gpio_init->pin);
            break;

        case LL_GPIO_TRIGGER_HIGH:
            ll_gpio_enable_high_trigger(GPIOx, p_gpio_init->pin);
            ll_gpio_enable_it(GPIOx, p_gpio_init->pin);
            break;

        case LL_GPIO_TRIGGER_LOW:
            ll_gpio_enable_low_trigger(GPIOx, p_gpio_init->pin);
            ll_gpio_enable_it(GPIOx, p_gpio_init->pin);
            break;

        default:
            break;
        }
    }

    return (SUCCESS);
}

/**
  * @brief Set each @ref ll_gpio_init_t field to default value.
  * @param p_gpio_init pointer to a @ref ll_gpio_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK __WEAK void ll_gpio_struct_init(ll_gpio_init_t *p_gpio_init)
{
    /* Reset GPIO init structure parameters values */
    p_gpio_init->pin        = LL_GPIO_PIN_ALL;
    p_gpio_init->mode       = LL_GPIO_MODE_INPUT;
    p_gpio_init->pull       = LL_GPIO_PULL_DOWN;
    p_gpio_init->mux        = LL_GPIO_MUX_7;
    p_gpio_init->trigger    = LL_GPIO_TRIGGER_NONE;
}

/** @} */

/** @} */

/** @} */

#endif /* defined (GPIO0) || defined (GPIO1) */

/** @} */


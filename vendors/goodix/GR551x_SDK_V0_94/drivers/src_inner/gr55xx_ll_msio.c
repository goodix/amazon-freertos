/**
  ****************************************************************************************
  * @file    gr55xx_ll_msio.c
  * @author  BLE Driver Team
  * @brief   MSIO LL module driver.
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
#include "gr55xx_ll_msio.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */


/** @addtogroup MSIO_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup MSIO_LL_Private_Macros
  * @{
  */
#define IS_LL_MSIO_PIN(__PIN__)             (((0x00000000U) < (__PIN__)) && ((__PIN__) <= (LL_MSIO_PIN_ALL)))

#define IS_LL_MSIO_DIRECTION(__DRCT__)      (((__DRCT__) == LL_MSIO_DIRECTION_NONE)  ||\
                                             ((__DRCT__) == LL_MSIO_DIRECTION_INPUT) ||\
                                             ((__DRCT__) == LL_MSIO_DIRECTION_OUTPUT)||\
                                             ((__DRCT__) == LL_MSIO_DIRECTION_INOUT))

#define IS_LL_MSIO_MODE(__MODE__)           (((__MODE__) == LL_MSIO_MODE_ANALOG) ||\
                                             ((__MODE__) == LL_MSIO_MODE_DIGITAL))

#define IS_LL_MSIO_PULL(__PULL__)           (((__PULL__) == LL_MSIO_PULL_NO) ||\
                                             ((__PULL__) == LL_MSIO_PULL_UP) ||\
                                             ((__PULL__) == LL_MSIO_PULL_DOWN))

#define IS_LL_MSIO_MUX(__MUX__)             ((__MUX__) <= LL_MSIO_MUX_7)

/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup MSIO_LL_Exported_Functions
  * @{
  */

/** @addtogroup MSIO_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize MSIO registers (Registers restored to their default values).
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: MSIO registers are de-initialized
  *          - ERROR:   Wrong MSIO Port
  */
__WEAK error_status_t ll_msio_deinit(void)
{
    error_status_t status = SUCCESS;

    /* MISO_PAD_CFG_0 register set to default reset values */
    LL_MSIO_WriteReg(AON, MISO_PAD_CFG_0, 0xFFFFFFFFU);

    /* MISO_PAD_CFG_1 register set to analog mode */
    ll_msio_set_pin_mode(LL_MSIO_PIN_ALL, LL_MSIO_MODE_ANALOG);

    /* MISO_PAD_CFG_1 register set to GPIO */
    CLEAR_BITS(AON->MISO_PAD_CFG_1, AON_MISO_PAD_CFG_1_MCU_OVR);
    return (status);
}

/**
  * @brief  Initialize MSIO registers according to the specified parameters in msio_init_t.
  * @param  p_msio_init pointer to a @ref msio_init_t structure
  *         that contains the MSIO_InitStructuration information for the specified MSIO peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: MSIO registers are initialized according to msio_init_t content
  *          - ERROR:   Not applicable
  */
__WEAK error_status_t ll_msio_init(ll_msio_init_t *p_msio_init)
{
    uint32_t current_pin = 0x00000000U;
    uint32_t pin_tmp     = 0x00000000U;

    /* Check the parameters */
    gr_assert_param(IS_LL_MSIO_PIN(p_msio_init->pin));
    gr_assert_param(IS_LL_MSIO_DIRECTION(p_msio_init->direction));
    gr_assert_param(IS_LL_MSIO_MODE(p_msio_init->mode));
    gr_assert_param(IS_LL_MSIO_PULL(p_msio_init->pull));
    gr_assert_param(IS_LL_MSIO_MUX(p_msio_init->mux));

    /* ------------------------- Configure the port pins ---------------- */
    pin_tmp = p_msio_init->pin;
    while (pin_tmp)
    {
        current_pin = (0x1U << POSITION_VAL(pin_tmp));
        /* Clear the lowest bit 1 */
        pin_tmp &= (pin_tmp - 1);
        ll_msio_set_pin_mux(current_pin, p_msio_init->mux);
    }

    ll_msio_set_pin_pull(p_msio_init->pin, p_msio_init->pull);
    ll_msio_set_pin_mode(p_msio_init->pin, p_msio_init->mode);
    ll_msio_set_pin_direction(p_msio_init->pin, p_msio_init->direction);

    return (SUCCESS);
}

/**
  * @brief Set each @ref ll_msio_init_t field to default value.
  * @param p_msio_init pointer to a @ref ll_msio_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_msio_struct_init(ll_msio_init_t *p_msio_init)
{
    /* Reset GPIO init structure parameters values */
    p_msio_init->pin        = LL_MSIO_PIN_ALL;
    p_msio_init->direction  = LL_MSIO_DIRECTION_INPUT;
    p_msio_init->mode       = LL_MSIO_MODE_DIGITAL;
    p_msio_init->pull       = LL_MSIO_PULL_DOWN;
    p_msio_init->mux        = LL_MSIO_MUX_7;
}

/** @} */

/** @} */

/** @} */

/** @} */

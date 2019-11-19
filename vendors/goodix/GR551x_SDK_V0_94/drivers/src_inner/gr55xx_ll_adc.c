/**
  ****************************************************************************************
  * @file    gr55xx_ll_adc.c
  * @author  BLE Driver Team
  * @brief   ADC LL module driver.
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
#include "gr55xx_ll_adc.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */


/** @addtogroup ADC_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup ADC_LL_Private_Macros
  * @{
  */
#define IS_LL_ADC_INPUT(__INPUT__)          (((__INPUT__) == LL_ADC_INPUT_SRC_IO0) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO1) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO2) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO3) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_IO4) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_TMP) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_BAT) || \
                                             ((__INPUT__) == LL_ADC_INPUT_SRC_REF))

#define IS_LL_ADC_INPUT_MODE(__MODE__)      (((__MODE__) == LL_ADC_INPUT_SINGLE) || \
                                             ((__MODE__) == LL_ADC_INPUT_DIFFERENTIAL)

#define IS_LL_ADC_REF(__INPUT__)            (((__INPUT__) == LL_ADC_REF_SRC_BUF_INT) || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_INT)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_BAT)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO0)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO1)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO2)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO3)     || \
                                             ((__INPUT__) == LL_ADC_REF_SRC_IO4))

#define IS_LL_ADC_REF_VALUE(__VALUE__)      (((__VALUE__) >= LL_ADC_REF_VALUE_0P5) && \
                                             ((__VALUE__) <= LL_ADC_REF_VALUE_2P0))

#define IS_LL_ADC_CLOCK(__CLOCK__)          (((__CLOCK__) == LL_ADC_CLK_16) || \
                                             ((__CLOCK__) == LL_ADC_CLK_8)  || \
                                             ((__CLOCK__) == LL_ADC_CLK_4)  || \
                                             ((__CLOCK__) == LL_ADC_CLK_2)  || \
                                             ((__CLOCK__) == LL_ADC_CLK_1)  || \
                                             ((__CLOCK__) == LL_ADC_CLK_1P6))

/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup ADC_LL_Exported_Functions
  * @{
  */

/** @addtogroup ADC_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize ADC registers (Registers restored to their default values).
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are de-initialized
  *          - ERROR:   Wrong ADC Port
  */
__WEAK error_status_t ll_adc_deinit(void)
{
    /* Reset configuration */
    LL_ADC_WriteReg(AON, SNSADC_CFG, 0x0708070AU);

    /* Disable clock */
    ll_adc_disable_clock();
    ll_adc_set_clock(LL_ADC_CLK_1P6);

    return SUCCESS;
}

/**
  * @brief  Initialize ADC registers according to the specified parameters in adc_init_t.
  * @param  p_adc_init pointer to a @ref adc_init_t structure
  *         that contains the ADC_InitStructuration information for the specified ADC peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: ADC registers are initialized according to adc_init_t content
  *          - ERROR:   Not applicable
  */
__WEAK error_status_t ll_adc_init(ll_adc_init_t *p_adc_init)
{
    /* Check the parameters */
    gr_assert_param(IS_LL_ADC_INPUT(p_adc_init->channel_p));
    gr_assert_param(IS_LL_ADC_INPUT(p_adc_init->channel_n));
    gr_assert_param(IS_LL_ADC_INPUT_MODE(p_adc_init->input_mode));
    gr_assert_param(IS_LL_ADC_REF(p_adc_init->ref_source));
    gr_assert_param(IS_LL_ADC_REF_VALUE(p_adc_init->ref_value));
    gr_assert_param(IS_LL_ADC_CLOCK(p_adc_init->clock));

    /* ------------------------- Configure ADC ---------------- */
    ll_adc_set_channelp(p_adc_init->channel_p);
    ll_adc_set_channeln(p_adc_init->channel_n);
    
    if ((LL_ADC_INPUT_SRC_TMP == p_adc_init->channel_p) || (LL_ADC_INPUT_SRC_TMP == p_adc_init->channel_n))
        ll_adc_enable_temp();
    if ((LL_ADC_INPUT_SRC_BAT == p_adc_init->channel_p) || (LL_ADC_INPUT_SRC_BAT == p_adc_init->channel_n))
        ll_adc_enable_vbat();
    ll_adc_set_input_mode(p_adc_init->input_mode);
    ll_adc_set_ref(p_adc_init->ref_source);
    if (LL_ADC_REF_SRC_BUF_INT == p_adc_init->ref_source)
        ll_adc_set_ref_value(p_adc_init->ref_value);
    ll_adc_set_clock(p_adc_init->clock);

    return SUCCESS;
}

/**
  * @brief Set each @ref ll_adc_init_t field to default value.
  * @param p_adc_init pointer to a @ref ll_adc_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_adc_struct_init(ll_adc_init_t *p_adc_init)
{
    /* Reset ADC init structure parameters values */
    p_adc_init->channel_p  = LL_ADC_INPUT_SRC_IO1;
    p_adc_init->channel_n  = LL_ADC_INPUT_SRC_IO1;
    p_adc_init->input_mode = LL_ADC_INPUT_DIFFERENTIAL;
    p_adc_init->ref_source = LL_ADC_REF_SRC_BUF_INT;
    p_adc_init->ref_value  = LL_ADC_REF_VALUE_0P5;
    p_adc_init->clock      = LL_ADC_CLK_1P6;
}

/** @} */

/** @} */

/** @} */

/** @} */

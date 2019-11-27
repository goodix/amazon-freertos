/**
  ****************************************************************************************
  * @file    gr55xx_ll_comp.c
  * @author  BLE Driver Team
  * @brief   COMP LL module driver.
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
#include "gr55xx_ll_comp.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */


/** @addtogroup COMP_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup COMP_LL_Private_Macros
  * @{
  */
#define IS_LL_COMP_INPUT(__INPUT__)         (((__INPUT__) == LL_COMP_INPUT_SRC_IO0) || \
                                             ((__INPUT__) == LL_COMP_INPUT_SRC_IO1) || \
                                             ((__INPUT__) == LL_COMP_INPUT_SRC_IO2) || \
                                             ((__INPUT__) == LL_COMP_INPUT_SRC_IO3) || \
                                             ((__INPUT__) == LL_COMP_INPUT_SRC_IO4))

#define IS_LL_COMP_REF(__INPUT__)           (((__INPUT__) == LL_COMP_REF_SRC_IO0)  || \
                                             ((__INPUT__) == LL_COMP_REF_SRC_IO1)  || \
                                             ((__INPUT__) == LL_COMP_REF_SRC_IO2)  || \
                                             ((__INPUT__) == LL_COMP_REF_SRC_IO3)  || \
                                             ((__INPUT__) == LL_COMP_REF_SRC_IO4)  || \
                                             ((__INPUT__) == LL_COMP_REF_SRC_VBAT) || \
                                             ((__INPUT__) == LL_COMP_REF_SRC_VREF))

/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup COMP_LL_Exported_Functions
  * @{
  */

/** @addtogroup COMP_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize COMP registers (Registers restored to their default values).
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: COMP registers are de-initialized
  *          - ERROR:   Wrong COMP Port
  */
__WEAK error_status_t ll_comp_deinit(void)
{
    /* Reset configuration */
    LL_COMP_WriteReg(AON, RF_REG_10, 0x00000000U);

    return SUCCESS;
}

/**
  * @brief  Initialize COMP registers according to the specified parameters in comp_init_t.
  * @param  p_comp_init pointer to a @ref comp_init_t structure
  *         that contains the COMP_InitStructuration information for the specified COMP peripheral.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: COMP registers are initialized according to comp_init_t content
  *          - ERROR:   Not applicable
  */
__WEAK error_status_t ll_comp_init(ll_comp_init_t *p_comp_init)
{
    /* Check the parameters */
    gr_assert_param(IS_LL_COMP_INPUT(p_comp_init->input_source));
    gr_assert_param(IS_LL_COMP_REF(p_comp_init->ref_source));

    /* ------------------------- Configure COMP ---------------- */
    ll_comp_set_input_src(p_comp_init->input_source);
    ll_comp_set_ref_src(p_comp_init->ref_source);
    if (LL_COMP_REF_SRC_VBAT == p_comp_init->ref_source)
        ll_comp_set_vbatt_lvl(p_comp_init->ref_value);
    if (LL_COMP_REF_SRC_VREF == p_comp_init->ref_source)
        ll_comp_set_vref_lvl(p_comp_init->ref_value);

    return SUCCESS;
}

/**
  * @brief Set each @ref ll_comp_init_t field to default value.
  * @param p_comp_init pointer to a @ref ll_com[_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_comp_struct_init(ll_comp_init_t *p_comp_init)
{
    /* Reset COMP init structure parameters values */
    p_comp_init->input_source = LL_COMP_INPUT_SRC_IO0;
    p_comp_init->ref_source   = LL_COMP_REF_SRC_IO1;
}

/** @} */

/** @} */

/** @} */

/** @} */


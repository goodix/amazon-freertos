/**
  ****************************************************************************************
  * @file    gr55xx_ll_cgc.c
  * @author  BLE Driver Team
  * @brief   CGC LL module driver.
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
#include "gr55xx_ll_cgc.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */


/** @addtogroup CGC_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup CGC_LL_Exported_Functions
  * @{
  */

/** @addtogroup CGC_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize CGC registers (Registers restored to their default values).
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: CGC registers are de-initialized
  *          - ERROR:   Wrong CGC Port
  */
__WEAK error_status_t ll_cgc_deinit(void)
{
    error_status_t status = SUCCESS;

    /* CG_CTRL_0 register set to default reset values */
    LL_CGC_WriteReg(MCU_SUB, MCU_SUBSYS_CG_CTRL[0], 0x00000000U);

    /* CG_CTRL_1 register set to default reset values */
    LL_CGC_WriteReg(MCU_SUB, MCU_SUBSYS_CG_CTRL[1], 0x00000000U);

    /* CG_CTRL_2 register set to default reset values */
    LL_CGC_WriteReg(MCU_SUB, MCU_SUBSYS_CG_CTRL[2], 0x00000000U);

    /* CG_CTRL_2 register set to default reset values */
    LL_CGC_WriteReg(MCU_SUB, MCU_PERIPH_CG, 0x00000000U);
    return (status);
}

/**
  * @brief  Initialize CGC registers according to the specified parameters in cgc_init_t.
  * @param  p_cgc_init pointer to a @ref cgc_init_t structure
  *         that contains the CGC_InitStructuration information for the specified CGC register.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: CGC registers are initialized according to cgc_init_t content
  *          - ERROR:   Not applicable
  */
__WEAK error_status_t ll_cgc_init(ll_cgc_init_t *p_cgc_init)
{
    /* ------------------------- Configure the clock gate ---------------- */
    ll_cgc_set_wfi_off_hclk_0(p_cgc_init->wfi_clk0);
    ll_cgc_set_wfi_off_hclk_1(p_cgc_init->wfi_clk1);
    ll_cgc_set_wfi_off_hclk_2(p_cgc_init->wfi_clk2);

    ll_cgc_set_force_off_hclk_0(p_cgc_init->force_clk0);
    ll_cgc_set_force_off_hclk_1(p_cgc_init->force_clk1);
    ll_cgc_set_force_off_hclk_2(p_cgc_init->force_clk2);
    return (SUCCESS);
}

/**
  * @brief Set each @ref ll_cgc_init_t field to default value.
  * @param p_cgc_init pointer to a @ref ll_cgc_init_t structure
  *                          whose fields will be set to default values.
  * @retval None
  */

__WEAK void ll_cgc_struct_init(ll_cgc_init_t *p_cgc_init)
{
    /* Reset CGC init structure parameters values */
    p_cgc_init->wfi_clk0   = 0;
    p_cgc_init->wfi_clk1   = 0;
    p_cgc_init->wfi_clk2   = 0;
    p_cgc_init->force_clk0 = 0;
    p_cgc_init->force_clk1 = 0;
    p_cgc_init->force_clk2 = 0;
}

/** @} */

/** @} */

/** @} */

/** @} */


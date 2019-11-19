/**
  ****************************************************************************************
  * @file    gr55xx_hal_cgc.c
  * @author  BLE Driver Team
  * @brief   CGC HAL module driver.
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

#ifdef HAL_CGC_MODULE_ENABLED
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/** @defgroup GPIO_Private_Defines GPIO Private Defines
  * @{
  */

#define CGC_WFI_HCLK0_Pos               (0U)
#define CGC_WFI_HCLK1_pos               (12U)
#define CGC_WFI_HCLK2_pos               (15U)
#define CGC_WFI_HCLK0_Msk               (0x00000FFFU)
#define CGC_WFI_HCLK1_Msk               (0x00007000U)
#define CGC_WFI_HCLK2_Msk               (0x00028000U)


#define CGC_FRC_HCLK0_Pos               (0U)
#define CGC_FRC_HCLK1_Pos               (12U)
#define CGC_FRC_HCLK2_Pos0              (15U)
#define CGC_FRC_HCLK2_Pos1              (24U)
#define CGC_FRC_HCLK0_Msk               (0x00000FFFU)
#define CGC_FRC_HCLK1_Msk               (0x00007000U)
#define CGC_FRC_HCLK2_Msk0              (0x00FF8000U)
#define CGC_FRC_HCLK2_Msk1              (0x05000000U)

/**
  * @}
  */

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/** @defgroup CGC_Exported_Functions CGC Exported Functions
  * @{
  */

/** @defgroup CGC_Exported_Functions_Group Initialization/de-initialization functions
 *  @brief    Initialization and Configuration functions
  * @{
  */
__WEAK void hal_cgc_init(cgc_init_t *p_cgc_init)
{
    ll_cgc_init_t cgc_init;

    cgc_init.wfi_clk0 = (p_cgc_init->wfi_clk & CGC_WFI_HCLK0_Msk);
    cgc_init.wfi_clk1 = ((p_cgc_init->wfi_clk & CGC_WFI_HCLK1_Msk) >> CGC_WFI_HCLK1_pos);
    cgc_init.wfi_clk2 = ((p_cgc_init->wfi_clk & CGC_WFI_HCLK2_Msk) << (MCU_SUB_WFI_SECU_DIV4_PCLK_Pos - CGC_WFI_HCLK2_pos));
    cgc_init.force_clk0 = (p_cgc_init->force_clk & CGC_FRC_HCLK0_Msk);
    cgc_init.force_clk1 = ((p_cgc_init->force_clk & CGC_FRC_HCLK1_Msk) << (MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos - CGC_FRC_HCLK1_Pos));
    cgc_init.force_clk2 = ((p_cgc_init->force_clk & CGC_FRC_HCLK2_Msk0) >> (CGC_FRC_HCLK2_Pos0 - MCU_SUB_FORCE_UART0_HCLK_Pos));
    cgc_init.force_clk2 += ((p_cgc_init->force_clk & CGC_FRC_HCLK2_Msk1) << (MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos - CGC_FRC_HCLK2_Pos1));

    ll_cgc_init(&cgc_init);
    return;
}

__WEAK void hal_cgc_deinit(void)
{
    ll_cgc_deinit();
}

/**
  * @}
  */

/** @addtogroup CGC_Exported_Functions_Group IO operation functions
  *  @brief Clock Gate Open and Closemanagement functions.
  * @{
  */

__WEAK void hal_cgc_config_wfi_clk(uint32_t blocks, cgc_clk_state_t clk_state)
{
    uint32_t reg_value   = 0x00000000U;
    uint32_t reg_msk     = 0x00000000U;

    if(blocks & CGC_WFI_HCLK0_Msk)
    {
        reg_msk = (blocks & CGC_WFI_HCLK0_Msk);
        reg_value = ll_cgc_get_wfi_off_hclk_0();
        reg_value = (CGC_CLK_ON == clk_state) ? (reg_value | reg_msk) : (reg_value & (~reg_msk));
        ll_cgc_set_wfi_off_hclk_0(reg_value);
    }
    if(blocks & CGC_WFI_HCLK1_Msk)
    {
        reg_msk = (blocks & CGC_WFI_HCLK1_Msk) >> CGC_WFI_HCLK1_pos;
        reg_value = ll_cgc_get_wfi_off_hclk_1();
        reg_value = (CGC_CLK_ON == clk_state) ? (reg_value | reg_msk) : (reg_value & (~reg_msk));
        ll_cgc_set_wfi_off_hclk_1(reg_value);
    }
    if(blocks & CGC_WFI_HCLK2_Msk)
    {
        reg_msk = (blocks & CGC_WFI_HCLK2_Msk) << (MCU_SUB_WFI_SECU_DIV4_PCLK_Pos - CGC_WFI_HCLK2_pos);
        reg_value = ll_cgc_get_wfi_off_hclk_2();
        reg_value = (CGC_CLK_ON == clk_state) ? (reg_value | reg_msk) : (reg_value & (~reg_msk));
        ll_cgc_set_wfi_off_hclk_2(reg_value);
    }
    return;
}

__WEAK cgc_clk_state_t hal_cgc_get_wfi_clk(uint32_t block)
{
    uint32_t reg_value   = 0x00000000U;
    uint32_t reg_msk     = 0x00000000U;

    if(block & CGC_WFI_HCLK0_Msk)
    {
        reg_msk = (block & CGC_WFI_HCLK0_Msk);
        reg_value = ll_cgc_get_wfi_off_hclk_0() & reg_msk;
    }
    if(block & CGC_WFI_HCLK1_Msk)
    {
        reg_msk = (block & CGC_WFI_HCLK1_Msk) >> CGC_WFI_HCLK1_pos;
        reg_value = ll_cgc_get_wfi_off_hclk_1() & reg_msk;
    }
    if(block & CGC_WFI_HCLK2_Msk)
    {
        reg_msk = (block & CGC_WFI_HCLK2_Msk) << (MCU_SUB_WFI_SECU_DIV4_PCLK_Pos - CGC_WFI_HCLK2_pos);
        reg_value = ll_cgc_get_wfi_off_hclk_2() & reg_msk;
    }
    return reg_value ? CGC_CLK_ON : CGC_CLK_OFF;
}

__WEAK void hal_cgc_config_force_clk(uint32_t blocks, cgc_clk_state_t clk_state)
{
    uint32_t reg_value   = 0x00000000U;
    uint32_t reg_msk     = 0x00000000U;

    if(blocks & CGC_FRC_HCLK0_Msk)
    {
        reg_msk = (blocks & CGC_FRC_HCLK0_Msk);
        reg_value = ll_cgc_get_force_off_hclk_0();
        reg_value = (CGC_CLK_ON == clk_state) ? (reg_value & (~reg_msk)) : (reg_value | reg_msk);
        ll_cgc_set_force_off_hclk_0(reg_value);
    }
    if(blocks & CGC_FRC_HCLK1_Msk)
    {
        reg_msk = (blocks & CGC_FRC_HCLK1_Msk) << (MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos - CGC_FRC_HCLK1_Pos);
        reg_value = ll_cgc_get_force_off_hclk_1();
        reg_value = (CGC_CLK_ON == clk_state) ? (reg_value & (~reg_msk)) : (reg_value | reg_msk);
        ll_cgc_set_force_off_hclk_1(reg_value);
    }
    if(blocks & CGC_FRC_HCLK2_Msk0)
    {
        reg_msk = (blocks & CGC_FRC_HCLK2_Msk0) >> (CGC_FRC_HCLK2_Pos0 - MCU_SUB_FORCE_UART0_HCLK_Pos);
        reg_value = ll_cgc_get_force_off_hclk_2();
        reg_value = (CGC_CLK_ON == clk_state) ? (reg_value & (~reg_msk)) : (reg_value | reg_msk);
        ll_cgc_set_force_off_hclk_2(reg_value);
    }
    if(blocks & CGC_FRC_HCLK2_Msk1)
    {
        reg_msk = (blocks & CGC_FRC_HCLK2_Msk1) << (MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos - CGC_FRC_HCLK2_Pos1);
        reg_value = ll_cgc_get_force_off_hclk_2();
        reg_value = (CGC_CLK_ON == clk_state) ? (reg_value & (~reg_msk)) : (reg_value | reg_msk);
        ll_cgc_set_force_off_hclk_2(reg_value);
    }
    return;
}

__WEAK cgc_clk_state_t hal_cgc_get_force_clk(uint32_t block)
{
    uint32_t reg_value   = 0x00000000U;
    uint32_t reg_msk     = 0x00000000U;

    if(block & CGC_FRC_HCLK0_Msk)
    {
        reg_msk = (block & CGC_FRC_HCLK0_Msk);
        reg_value = ll_cgc_get_force_off_hclk_0() | reg_msk;
    }
    if(block & CGC_FRC_HCLK1_Msk)
    {
        reg_msk = (block & CGC_FRC_HCLK1_Msk) << (MCU_SUB_FORCE_AON_MCUSUB_HCLK_Pos - CGC_FRC_HCLK1_Pos);
        reg_value = ll_cgc_get_force_off_hclk_1() | reg_msk;
    }
    if(block & CGC_FRC_HCLK2_Msk0)
    {
        reg_msk = (block & CGC_FRC_HCLK2_Msk0) >> (CGC_FRC_HCLK2_Pos0 - MCU_SUB_FORCE_UART0_HCLK_Pos);
        reg_value = ll_cgc_get_force_off_hclk_2() | reg_msk;
    }
    if(block & CGC_FRC_HCLK2_Msk1)
    {
        reg_msk = (block & CGC_FRC_HCLK2_Msk1) << (MCU_SUB_FORCE_SECU_DIV4_PCLK_Pos - CGC_FRC_HCLK2_Pos1);
        reg_value = ll_cgc_get_force_off_hclk_2() | reg_msk;
    }
    return reg_value ? CGC_CLK_ON : CGC_CLK_OFF;
}

/**
  * @}
  */


/**
  * @}
  */

#endif /* HAL_CGC_MODULE_ENABLED */

/**
  * @}
  */
 



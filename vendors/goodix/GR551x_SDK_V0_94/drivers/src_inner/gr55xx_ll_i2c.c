/**
  ****************************************************************************************
  * @file    gr55xx_ll_i2c.c
  * @author  BLE Driver Team
  * @brief   I2C LL module driver.
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
#include "gr55xx_ll_i2c.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */

#if defined (I2C0) || defined (I2C1)

/** @defgroup I2C_LL I2C
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup I2C_LL_Private_Macros
  * @{
  */

#define IS_LL_I2C_OWN_ADDRESS(__VALUE__)       (((__VALUE__) <= 0x000003FFU) && \
                                                ((__VALUE__) > 0x07U) && \
                                                (((__VALUE__) < 0x78U) || ((__VALUE__) > 0x7FU)))

#define IS_LL_I2C_OWN_ADDRSIZE(__VALUE__)      (((__VALUE__) == LL_I2C_OWNADDRESS_7BIT) || \
                                                 ((__VALUE__) == LL_I2C_OWNADDRESS_10BIT))

#define IS_LL_I2C_SPEED(__VALUE__)             (((__VALUE__) == LL_I2C_SPEED_100K) || \
                                                ((__VALUE__) == LL_I2C_SPEED_400K) || \
                                                ((__VALUE__) == LL_I2C_SPEED_1000K) || \
                                                ((__VALUE__) == LL_I2C_SPEED_3500K))
/** @} */

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup I2C_LL_Exported_Functions
  * @{
  */

/** @addtogroup I2C_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize the I2C registers to their default reset values.
  * @param  I2Cx I2C instance.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: I2C registers are de-initialized
  *          - ERROR: I2C registers are not de-initialized
  */
__WEAK error_status_t ll_i2c_deinit(i2c_regs_t *I2Cx)
{
    error_status_t status = SUCCESS;

    /* Check the I2C instance I2Cx */
    gr_assert_param(IS_I2C_ALL_INSTANCE(I2Cx));

    ll_i2c_disable(I2Cx);

    /* IC_INTR_MASK register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, INTR_MASK, 0x000008FFU);
    /* Clear interrupt. */
    ll_i2c_clear_flag_intr(I2Cx);

    /* IC_CON register set to default reset values. */
    LL_I2C_WriteReg(I2Cx, CON, 0x0000007FU);
    /* IC_TAR register set to default reset values. */
    LL_I2C_WriteReg(I2Cx, TAR, 0x00000055U);
    /* IC_SAR register set to default reset values. */
    LL_I2C_WriteReg(I2Cx, SAR, 0x00000055U);

    /* IC_SS_SCL_HCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SS_SCL_HCNT, 0x00000190U);
    /* IC_SS_SCL_LCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SS_SCL_LCNT, 0x000001d6U);
    /* IC_FS_SCL_HCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, FS_SCL_HCNT, 0x0000003cU);
    /* IC_FS_SCL_LCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, FS_SCL_LCNT, 0x00000082U);
    /* IC_HS_SCL_HCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, HS_SCL_HCNT, 0x00000006U);
    /* IC_HS_SCL_LCNT register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, HS_SCL_LCNT, 0x00000010U);

    /* IC_RX_TL register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, RX_TL, 0x00000000U);
    /* IC_TX_TL register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, TX_TL, 0x00000000U);

    /* IC_DMA_CR register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, DMA_CR, 0x00000000U);
    /* IC_DMA_TDLR register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, DMA_TDLR, 0x00000000U);
    /* IC_DMA_RDLR register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, DMA_RDLR, 0x00000000U);

    /* IC_SDA_HOLD register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SDA_HOLD, 0x00000001U);
    /* IC_SDA_SETUP register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, SDA_SETUP, 0x00000064U);

    /* IC_ACK_GENERAL_CALL register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, ACK_GENERAL_CALL, 0x00000001U);

    /* IC_FS_SPKLEN register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, FS_SPKLEN, 0x00000005U);
    /* IC_HS_SPKLEN register set to default reset values.*/
    LL_I2C_WriteReg(I2Cx, HS_SPKLEN, 0x00000001U);

    return status;
}

/**
  * @brief  Initialize the I2C registers according to the specified parameters in p_i2c_init.
  * @param  I2Cx I2C instance.
  * @param  p_i2c_init pointer to a @ref ll_i2c_init_t structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: I2C registers are initialized
  *          - ERROR: Not applicable
  */
__WEAK error_status_t ll_i2c_init(i2c_regs_t *I2Cx, ll_i2c_init_t *p_i2c_init)
{
    /* Check the I2C instance I2Cx */
    gr_assert_param(IS_I2C_ALL_INSTANCE(I2Cx));
    /* Check the I2C parameters from I2C_InitStruct */
    gr_assert_param(IS_LL_I2C_OWN_ADDRESS(p_i2c_init->own_address));
    gr_assert_param(IS_LL_I2C_OWN_ADDRSIZE(p_i2c_init->own_addr_size));
    gr_assert_param(IS_LL_I2C_SPEED(p_i2c_init->speed));

    /* Disable the selected I2Cx Peripheral */
    ll_i2c_disable(I2Cx);

    /*---------------------------- I2Cx Speed Configuration --------------------*/
    uint32_t hcnt, lcnt;
    ll_i2c_set_speed_mode(I2Cx, __LL_I2C_CONVERT_SPEED_MODE(p_i2c_init->speed));
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    lcnt = SystemCoreClock / 2 / p_i2c_init->speed - 1;
    if (p_i2c_init->speed < LL_I2C_SPEED_2000K)
    {
        hcnt = SystemCoreClock / 2 / p_i2c_init->speed - 7 - ll_i2c_get_spike_len_fs(I2Cx);
        if (p_i2c_init->speed < LL_I2C_SPEED_400K)
        {
            ll_i2c_set_clock_high_period_ss(I2Cx, hcnt);
            ll_i2c_set_clock_low_period_ss(I2Cx, lcnt);
        }
        else
        {
            ll_i2c_set_clock_high_period_fs(I2Cx, hcnt);
            ll_i2c_set_clock_low_period_fs(I2Cx, lcnt);
        }
    }
    else
    {
        hcnt = SystemCoreClock / 2 / p_i2c_init->speed - 7 - ll_i2c_get_spike_len_hs(I2Cx);
        ll_i2c_set_clock_high_period_hs(I2Cx, hcnt);
        ll_i2c_set_clock_low_period_hs(I2Cx, lcnt);
    }

    /*---------------------------- I2Cx Own Address Configuration -------------*/
    ll_i2c_set_own_address(I2Cx, p_i2c_init->own_address, p_i2c_init->own_addr_size);

    /* Enable the selected I2Cx Peripheral */
    ll_i2c_enable(I2Cx);

    return SUCCESS;
}

/**
  * @brief  Set each @ref ll_i2c_init_t field to default value.
  * @param  p_i2c_init Pointer to a @ref ll_i2c_init_t structure.
  * @retval None
  */
__WEAK void ll_i2c_struct_init(ll_i2c_init_t *p_i2c_init)
{
    /* Set I2C_InitStruct fields to default values */
    p_i2c_init->speed           = LL_I2C_SPEED_400K;
    p_i2c_init->own_address     = 0x55U;
    p_i2c_init->own_addr_size   = LL_I2C_OWNADDRESS_7BIT;
}
/** @} */

/** @} */

/** @} */

#endif /* I2C1 || I2C2 || I2C3 */

/** @} */


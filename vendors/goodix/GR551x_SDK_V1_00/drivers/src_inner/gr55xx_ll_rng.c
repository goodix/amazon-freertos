/**
  ****************************************************************************************
  * @file    gr55xx_ll_rng.c
  * @author  BLE Driver Team
  * @brief   RNG LL module driver.
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
#include "gr55xx_ll_rng.h"
#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

/** @addtogroup GR55xx_LL_Driver
  * @{
  */
#if defined (RNG)

/** @addtogroup RNG_LL
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
#define LL_RNG_CONFIG_LFSR_XOR_FRO        (0x4UL << RNG_CONFIG_LFSR_XOR_SEL_Pos)
/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/** @addtogroup RNG_LL_Exported_Functions
  * @{
  */

/** @addtogroup RNG_LL_EF_Init
  * @{
  */

/**
  * @brief  De-initialize RNG registers (Registers restored to their default values).
  * @retval An error_status_t enumeration value:
  *          - SUCCESS: RNG registers are de-initialized
  *          - ERROR: RNG registers are not de-initialized
  */
__WEAK error_status_t ll_rng_deinit(rng_regs_t *RNGx)
{
    error_status_t status = SUCCESS;

    /* Disable the selected RNGx Peripheral */
    ll_rng_disable(RNGx);

    /* Clear status flag. */
    ll_rng_clear_flag_sts(RNGx);

    /* CONFIG register set to default reset values. */
    LL_RNG_WriteReg(RNGx, CONFIG, 0x9004);

    /* TSCON register set to default reset values. */
    LL_RNG_WriteReg(RNGx, TSCON, 0x00007864U);

    /* TSCON register set to default reset values. */
    LL_RNG_WriteReg(RNGx, FROCFG, 0x0000FFFFU);

    /* USER_SEED register set to default reset values. */
    LL_RNG_WriteReg(RNGx, USER_SEED, 0x00000000U);

    return status;
}

/**
  * @brief  Initialize the RNG registers according to the specified parameters in p_rng_init.
  * @param  RNGx RNG instance.
  * @param  p_rng_init pointer to a @ref ll_rng_init_t structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RNG registers are initialized
  *          - ERROR: Not applicable
  */
__WEAK error_status_t ll_rng_init(rng_regs_t *RNGx, ll_rng_init_t *p_rng_init)
{
    uint32_t config_value = 0;
    
    if((LL_RNG_SEED_USER ==  p_rng_init->seed) && (LL_RNG_OUTPUT_FR0_S0 == p_rng_init->out_mode))
    {
        return ERROR;
    }
    
    /* Note: rng registers are WRITE ONLY before enable rng module */
    /* Therefore we CAN NOT use expressions like "rng->config |= 8" to set 3rd bit */
    /* Disable the selected RNGx Peripheral */
    ll_rng_disable(RNGx);

    config_value = p_rng_init->post_mode | p_rng_init->out_mode | p_rng_init->lfsr_mode | \
                   p_rng_init->seed | p_rng_init->interrupt | LL_RNG_CONFIG_LFSR_XOR_FRO;
    if(LL_RNG_SEED_FR0_S0 == p_rng_init->seed)
    {
        config_value |= RNG_CONFIG_FRO_EN;
    }
    else
    {
        config_value &= ~(RNG_CONFIG_FRO_EN);
    }


    /* set the value of p_rng_init to CONFIG register. */
    LL_RNG_WriteReg(RNGx, CONFIG, config_value);

    return SUCCESS;
}

/**
  * @brief  Set each @ref ll_rng_init_t field to default value.
  * @param  p_rng_init Pointer to a @ref ll_rng_init_t structure.
  * @retval None
  */
__WEAK void ll_rng_struct_init(ll_rng_init_t *p_rng_init)
{
    /* Set RNG_InitStruct fields to default values */
    p_rng_init->seed       = LL_RNG_SEED_FR0_S0;
    p_rng_init->lfsr_mode  = LL_RNG_LFSR_MODE_59BIT;
    p_rng_init->out_mode   = LL_RNG_POST_PRO_NOT;
    p_rng_init->post_mode  = LL_RNG_OUTPUT_FR0_S0;
}

/** @} */

/** @} */

/** @} */
#endif /* RNG */
/** @} */

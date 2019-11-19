/**
  ****************************************************************************************
  * @file    gr55xx_ll_pkc.c
  * @author  BLE Driver Team
  * @brief   PKC LL module driver.
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
#include "gr55xx_ll_pkc.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (PKC)

/**
  * @brief  Set PKCx registers to their reset values.
  * @param  PKCx PKC instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PKCx registers are de-initialized
  *          - ERROR: invalid PKCx instance
  */
__WEAK error_status_t ll_pkc_deinit(pkc_regs_t *PKCx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_PKC_ALL_INSTANCE(PKCx));

    ll_pkc_enable(PKCx);
    ll_pkc_disable_reset(PKCx);
    ll_pkc_enable_reset(PKCx);
    ll_pkc_disable(PKCx);

    return result;
}

/**
  * @brief  Set the fields of the PKC base unit configuration data structure
  *         to their default values.
  * @param  p_pkc_init pointer to a @ref ll_pkc_init_t structure (PKC base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_pkc_struct_init(ll_pkc_init_t *p_pkc_init)
{
    static ll_ecc_curve_init_t LL_ECC_CurveInit = LL_ECC_CURVE_DEFAULT_CONFIG;

    /* Set the default configuration */
    p_pkc_init->p_ecc_curve = &LL_ECC_CurveInit;
}

/**
  * @brief  Configure the PKCx base unit.
  * @param  PKCx PKC Instance
  * @param  p_pkc_init pointer to a @ref ll_pkc_init_t structure (PKCx base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: PKCx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_pkc_init(pkc_regs_t *PKCx, ll_pkc_init_t *p_pkc_init)
{
    /* Check the parameters */
    gr_assert_param(IS_PKC_ALL_INSTANCE(PKCx));

    ll_pkc_enable(PKCx);
    ll_pkc_set_operation_word_length(PKCx, p_pkc_init->data_bits);

    return SUCCESS;
}

#endif /* PKC */


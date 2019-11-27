/**
  ****************************************************************************************
  * @file    gr55xx_ll_hmac.c
  * @author  BLE Driver Team
  * @brief   HMAC LL module driver.
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
#include "gr55xx_ll_hmac.h"
#include <stdio.h>

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (HMAC)

/**
  * @brief  Set HMACx registers to their reset values.
  * @param  HMACx HMAC instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: HMACx registers are de-initialized
  *          - ERROR: invalid HMACx instance
  */
__WEAK error_status_t ll_hmac_deinit(hmac_regs_t *HMACx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_HMAC_INSTANCE(HMACx));

    HMACx->CTRL      = 0;
    HMACx->CONFIG    = 0;
    HMACx->INTERRUPT = 1;

    return result;
}

/**
  * @brief  Set the fields of the HMAC base unit configuration data structure
  *         to their default values.
  * @param  p_hmac_init pointer to a @ref ll_hmac_init_t structure (HMAC base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_hmac_struct_init(ll_hmac_init_t *p_hmac_init)
{
    p_hmac_init->p_key  = NULL;
    p_hmac_init->p_hash = NULL;
}

/**
  * @brief  Configure the HMACx base unit.
  * @param  HMACx HMAC Instance
  * @param  p_hmac_init pointer to a @ref ll_hmac_init_t structure (HMACx base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: HMACx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_hmac_init(hmac_regs_t *HMACx, ll_hmac_init_t *p_hmac_init)
{
    /* Check the parameters */
    gr_assert_param(IS_HMAC_INSTANCE(HMACx));

    ll_hmac_enable(HMACx);
    ll_hmac_enable_little_endian(HMACx);

    return SUCCESS;
}

#endif /* HMAC */


/**
  ****************************************************************************************
  * @file    gr55xx_ll_aes.c
  * @author  BLE Driver Team
  * @brief   AES LL module driver.
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
#include "gr55xx_ll_aes.h"
#include <stdio.h>

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (AES)

/**
  * @brief  Set AESx registers to their reset values.
  * @param  AESx AES instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: AESx registers are de-initialized
  *          - ERROR: invalid AESx instance
  */
__WEAK error_status_t ll_aes_deinit(aes_regs_t *AESx)
{
    error_status_t result = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_AES_INSTANCE(AESx));

    AESx->CTRL      = 0;
    AESx->CONFIG    = 0;
    AESx->INTERRUPT = 1;

    return result;
}

/**
  * @brief  Set the fields of the AES base unit configuration data structure
  *         to their default values.
  * @param  p_aes_init pointer to a @ref ll_aes_init_t structure (AES base unit configuration data structure)
  * @retval None
  */
__WEAK void ll_aes_struct_init(ll_aes_init_t *p_aes_init)
{
    p_aes_init->key_size      = LL_AES_KEY_SIZE_128;
    p_aes_init->p_key         = NULL;
    p_aes_init->p_init_vector = NULL;
    p_aes_init->p_seed        = NULL;
}

/**
  * @brief  Configure the AESx base unit.
  * @param  AESx AES Instance
  * @param  p_aes_init pointer to a @ref ll_aes_init_t structure (AESx base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: AESx registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_aes_init(aes_regs_t *AESx, ll_aes_init_t *p_aes_init)
{
    /* Check the parameters */
    gr_assert_param(IS_AES_INSTANCE(AESx));

    ll_aes_enable(AESx);
    ll_aes_enable_little_endian(AESx);

    return SUCCESS;
}

#endif /* AES */


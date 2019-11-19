/**
  ****************************************************************************************
  * @file    gr55xx_hal_rng.c
  * @author  BLE Driver Team
  * @brief   RNG HAL module driver.
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

#ifdef HAL_RNG_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup RNG_Private_Constants RNG_Private_Constants
  * @{
  */
#define RNG_TIMEOUT_VALUE     200
/** @} */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/** @defgroup RNG_Exported_Functions RNG Exported Functions
  * @{
  */

/** @defgroup RNG_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions.
  * @{
  */

__WEAK hal_status_t hal_rng_init(rng_handle_t *p_rng)
{
    /* Check the RNG handle allocation */
    if (NULL == p_rng)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_RNG_ALL_INSTANCE(p_rng->p_instance));

    /* Process locked */
    __HAL_LOCK(p_rng);

    if (HAL_RNG_STATE_RESET == p_rng->state)
    {
        /* Allocate lock resource and initialize it */
        p_rng->lock = HAL_UNLOCKED;

        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
        ll_cgc_disable_wfi_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* init the low level hardware : CLOCK */
        hal_rng_msp_init(p_rng);
    }

    p_rng->state = HAL_RNG_STATE_BUSY;

    ll_rng_disable(p_rng->p_instance);

    /* Initialize the TIM state */
    p_rng->state = HAL_RNG_STATE_READY;

    /* Release Lock */
    __HAL_UNLOCK(p_rng);

    /* Return function status */
    return HAL_OK;
}

__WEAK hal_status_t hal_rng_deinit(rng_handle_t *p_rng)
{
    /* Check the RNG handle allocation */
    if (NULL == p_rng)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_rng);

    /* Disable the RNG Peripheral */
    ll_rng_deinit(p_rng->p_instance);

    /* DeInit the low level hardware: CLOCK... */
    hal_rng_msp_deinit(p_rng);

    /* Initialize the TIM state */
    p_rng->state = HAL_RNG_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_rng);

    return HAL_OK;
}

__WEAK void hal_rng_msp_init(rng_handle_t *p_rng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_rng);

    /* NOTE: This function should not be modified, when the callback is needed,
        the hal_rng_msp_init could be implemented in the user file
    */
}

__WEAK void hal_rng_msp_deinit(rng_handle_t *p_rng)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_rng);

    /* NOTE: This function should not be modified, when the callback is needed,
        the hal_rng_msp_init could be implemented in the user file
    */
}

/** @} */

/** @defgroup RNG_Exported_Functions_Group2 Peripheral Control functions
 *  @brief    Peripheral Control functions
  * @{
  */

__WEAK hal_status_t hal_rng_generate_random_number(rng_handle_t *p_rng, uint16_t *p_seed, uint32_t *p_random32bit)
{
  uint32_t tickstart = 0;    
  ll_rng_init_t rng_init = {0};
  hal_status_t status = HAL_OK;

  /* Process Locked */
  __HAL_LOCK(p_rng); 
  
  /* Check RNS peripheral state */
  if(p_rng->state == HAL_RNG_STATE_READY)
  {
    /* Change RNG peripheral state */  
    p_rng->state = HAL_RNG_STATE_BUSY;

    rng_init.seed = p_rng->init.seed_mode;
    rng_init.lfsr_mode = p_rng->init.lfsr_mode;
    rng_init.out_mode = p_rng->init.out_mode;
    rng_init.post_mode = p_rng->init.post_mode;
    rng_init.interrupt = LL_RNG_IT_DISABLE;

    ll_rng_init(p_rng->p_instance, &rng_init);
    
    if((RNG_SEED_USER == p_rng->init.seed_mode) && (NULL != p_seed))
    {
        if(RNG_LFSR_MODE_59BIT == p_rng->init.lfsr_mode)
        {
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 1));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 2));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 3));
        }
        else
        {
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 1));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 2));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 3));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 4));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 5));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 6));
            ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 7));
        }
    }

    ll_rng_enable(p_rng->p_instance);

    /* Get tick */
    tickstart = hal_get_tick();
  
    /* Check if data register contains valid random data */
    while(!ll_rng_is_active_flag_sts(RNG))
    {
      if((hal_get_tick() - tickstart ) > RNG_TIMEOUT_VALUE)
      {    
        p_rng->state = HAL_RNG_STATE_ERROR;

        /* Process Unlocked */
        __HAL_UNLOCK(p_rng);
      
        return HAL_TIMEOUT;
      } 
    }
  
    /* Get a 32bit Random number */
    p_rng->random_number =  ll_rng_read_random_data32(RNG);
    *p_random32bit = p_rng->random_number;
  
    p_rng->state = HAL_RNG_STATE_READY;
  }
  else
  {
    status = HAL_ERROR;
  }
  
  /* Process Unlocked */
  __HAL_UNLOCK(p_rng);

  return status;
}

__WEAK hal_status_t hal_rng_generate_random_number_it(rng_handle_t *p_rng, uint16_t *p_seed)
{
    ll_rng_init_t rng_init = {0};
    hal_status_t status = HAL_OK;
  
    /* Process Locked */
    __HAL_LOCK(p_rng);

    /* Check RNG peripheral state */
    if(p_rng->state == HAL_RNG_STATE_READY)
    {
        /* Change RNG peripheral state */  
        p_rng->state = HAL_RNG_STATE_BUSY;  

        rng_init.seed = p_rng->init.seed_mode;
        rng_init.lfsr_mode = p_rng->init.lfsr_mode;
        rng_init.out_mode = p_rng->init.out_mode;
        rng_init.post_mode = p_rng->init.post_mode;
        rng_init.interrupt = LL_RNG_IT_ENABLE;

        ll_rng_init(p_rng->p_instance, &rng_init);

        if((RNG_SEED_USER == p_rng->init.seed_mode) && (NULL != p_seed))
        {
            if(RNG_LFSR_MODE_59BIT == p_rng->init.lfsr_mode)
            {
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 1));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 2));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 3));
            }
            else
            {
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 1));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 2));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 3));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 4));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 5));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 6));
                ll_rng_set_user_seed(p_rng->p_instance, *(p_seed + 7));
            }
        }

        /* Process Unlocked */
        __HAL_UNLOCK(p_rng);

        ll_rng_enable(p_rng->p_instance);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(p_rng);

        status = HAL_ERROR;
    }
  
  return status;
}

__WEAK uint32_t hal_rng_read_last_random_number(rng_handle_t *p_rng)
{
      return(p_rng->random_number);
}

/** @} */

/** @addtogroup RNG_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
  * @brief    IRQ Handler and Callbacks functions
 * @{
 */
__WEAK void hal_rng_irq_handler(rng_handle_t *p_rng)
{
    if(ll_rng_is_active_flag_sts(p_rng->p_instance))
    {
        p_rng->random_number =  ll_rng_read_random_data32(RNG);
        ll_rng_disable(p_rng->p_instance);

        if(p_rng->state != HAL_RNG_STATE_ERROR)
        {
            /* Change RNG peripheral state */
            p_rng->state = HAL_RNG_STATE_READY; 
            /* Call legacy weak Data Ready callback */ 
            hal_rng_ready_data_callback(p_rng, p_rng->random_number);
        }
    }
}

__weak void hal_rng_ready_data_callback(rng_handle_t *p_rng, uint32_t random32bit)
{
    UNUSED(p_rng);
    UNUSED(random32bit);
    /* NOTE : This function should not be modified. When the callback is needed,
            function hal_rng_ready_data_callback must be implemented in the user file.
     */
}

/** @} */

/** @defgroup RNG_Exported_Functions_Group3 Peripheral State functions
  * @brief   RNG control functions
 * @{
 */
__WEAK hal_rng_state_t hal_rng_get_state(rng_handle_t *p_rng)
{
    return p_rng->state;
}

/** @} */

#endif /* HAL_WDT_MODULE_ENABLED */

/** @} */

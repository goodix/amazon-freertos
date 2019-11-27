/**
  ****************************************************************************************
  * @file    gr55xx_hal_comp.c
  * @author  BLE Driver Team
  * @brief   COMP HAL module driver.
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

#ifdef HAL_COMP_MODULE_ENABLED

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
__WEAK hal_status_t hal_comp_init(comp_handle_t *p_comp)
{
    hal_status_t   status = HAL_OK;
    error_status_t err    = SUCCESS;

    /* Check the ADC handle allocation */
    if (NULL == p_comp)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_comp);

    if (HAL_COMP_STATE_RESET == p_comp->state)
    {
        /* Allocate lock resource and initialize it */
        p_comp->lock = HAL_UNLOCKED;

        /* init the low level hardware : MSIO, NVIC */
        hal_comp_msp_init(p_comp);
    }

    /* Configure COMP peripheral */
    err = ll_comp_init(&p_comp->init);

    if (SUCCESS == err)
    {
        /* Set COMP error code to none */
        p_comp->error_code = HAL_COMP_ERROR_NONE;

        /* Initialize the ADC state */
        p_comp->state = HAL_COMP_STATE_READY;
    }
    else
    {
        status = HAL_ERROR;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_comp);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_comp_deinit(comp_handle_t *p_comp)
{
    /* Check the COMP handle allocation */
    if (NULL == p_comp)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_comp);

    /* Reset COMP Peripheral */
    ll_comp_deinit();

    /* DeInit the low level hardware: GPIO, NVIC... */
    hal_comp_msp_deinit(p_comp);

    /* Set COMP error code to none */
    p_comp->error_code = HAL_COMP_ERROR_NONE;

    /* Initialize the COMP state */
    p_comp->state = HAL_COMP_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_comp);

    return HAL_OK;
}

__WEAK void hal_comp_msp_init(comp_handle_t *p_comp)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_comp_msp_init can be implemented in the user file
     */
}

__WEAK void hal_comp_msp_deinit(comp_handle_t *p_comp)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_comp_msp_deinit can be implemented in the user file
     */
}

__WEAK hal_status_t hal_comp_start(comp_handle_t *p_comp)
{
    hal_status_t status = HAL_OK;
    
    /* Process locked */
    __HAL_LOCK(p_comp);

    if (HAL_COMP_STATE_READY == p_comp->state)
    {
        p_comp->error_code = HAL_COMP_ERROR_NONE;

        /* Update COMP state */
        p_comp->state = HAL_COMP_STATE_BUSY;

        /* Enable the comparator. */
        ll_comp_enable();
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_comp);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_comp_stop(comp_handle_t *p_comp)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_comp);
    
    if ((HAL_ADC_STATE_READY == p_comp->state) ||
        (HAL_ADC_STATE_BUSY == p_comp->state))
    {
        p_comp->error_code = HAL_ADC_ERROR_NONE;
        
        /* Disable the comparator. */
        ll_comp_disable();

        p_comp->state = HAL_COMP_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_comp);

    /* Return function status */
    return status;
}

__WEAK void hal_comp_irq_handler(comp_handle_t *p_comp)
{
    if(ll_comp_is_active_flag_it())
    {
        /* Clear COMP pending bit */
        ll_comp_clear_flag_it();

        if (HAL_ADC_STATE_BUSY == p_comp->state)
        {
            /* Change state of COMP */
            //p_comp->state = HAL_COMP_STATE_READY;

            hal_comp_trigger_callback(p_comp);
        }
    }
    return;
}


__WEAK void hal_comp_trigger_callback(comp_handle_t *p_comp)
{
    return;
}

__WEAK hal_comp_state_t hal_comp_get_state(comp_handle_t *p_comp)
{
    /* Return COMP handle state */
    return p_comp->state;
}

__WEAK uint32_t hal_comp_get_error(comp_handle_t *p_comp)
{
    return p_comp->error_code;
}




#endif /* HAL_ADC_MODULE_ENABLED */


/**
  ****************************************************************************************
  * @file    gr55xx_hal_pkc.c
  * @author  BLE Driver Team
  * @brief   PKC HAL module driver.
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
#include <string.h>

#ifdef HAL_PKC_MODULE_ENABLED

typedef struct _pkc_sw_operation_init
{
    uint32_t op_mode;
    uint32_t enable_it;
    uint32_t ptrA;
    uint32_t ptrB;
    uint32_t ptrC;
    uint32_t ptrP;
    uint32_t timeout;
} pkc_sw_operation_init_t;

typedef struct _ecc_point_multi_init
{
    uint32_t enable_it;
    pkc_ecc_point_multi_t *p_input;
    ecc_point_t *p_result;
    uint32_t timeout;
} ecc_point_multi_init_t;

static hal_status_t pkc_wait_flag_state_until_timeout(pkc_handle_t *p_pkc, uint32_t flag, \
        flag_status_t state, uint32_t tick_start, uint32_t timeout);
static void pkc_read_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr);
static void pkc_write_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr);
static uint32_t pkc_find_msb(uint32_t *p_data, uint32_t data_bits);
static uint32_t pkc_get_bit(uint32_t *p_data, uint32_t ofs, uint32_t data_bits);
static int32_t pkc_compare_const(uint32_t *p_data, uint32_t Const, uint32_t data_bits);
//static int32_t pkc_compare_number(uint32_t *pDataA, uint32_t *pDataB, uint32_t data_bits);
static uint32_t ecc_is_infinite_point(ecc_point_t *pPiont);
static hal_status_t pkc_software_operation(pkc_handle_t *p_pkc, pkc_sw_operation_init_t *p_input);
static hal_status_t ecc_point_multiply(pkc_handle_t *p_pkc, ecc_point_multi_init_t *p_input);

__WEAK hal_status_t hal_pkc_init(pkc_handle_t *p_pkc)
{
    hal_status_t   status    = HAL_ERROR;
    error_status_t err       = SUCCESS;
    uint32_t       tickstart = hal_get_tick();
    ll_pkc_init_t  pkc_init;

    /* Check the PKC handle allocation */
    if (NULL == p_pkc)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_PKC_ALL_INSTANCE(p_pkc->p_instance));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_RESET == p_pkc->state)
    {
        /* Allocate lock resource and initialize it */
        p_pkc->lock = HAL_UNLOCKED;

        /* Enable security blocks clock and Automatic turn off security blocks clock during WFI. */
        ll_cgc_disable_force_off_secu_hclk();
        ll_cgc_disable_force_off_secu_div4_pclk();
        ll_cgc_disable_wfi_off_secu_hclk();
        ll_cgc_disable_wfi_off_secu_div4_hclk();

        /* Init the low level hardware : CLOCK, NVIC */
        hal_pkc_msp_init(p_pkc);

        /* Configure the default timeout for the PKC calculate */
        hal_pkc_set_timeout(p_pkc, HAL_PKC_TIMEOUT_DEFAULT_VALUE);
    }

    /* Enable PKC */
    __HAL_PKC_ENABLE(p_pkc);

    /* Wait till BUSY flag reset */
    status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, p_pkc->timeout);

    if (HAL_OK == status)
    {
        pkc_init.data_bits  = p_pkc->init.data_bits;
        pkc_init.p_ecc_curve = p_pkc->init.p_ecc_curve;
        err = ll_pkc_init(p_pkc->p_instance, &pkc_init);

        if (SUCCESS == err)
        {
            /* Set PKC error code to none */
            p_pkc->error_code = HAL_PKC_ERROR_NONE;

            /* Initialize the PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
        else
        {
            status = HAL_ERROR;
        }
    }

    /* Release Lock */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_deinit(pkc_handle_t *p_pkc)
{
    /* Check the PKC handle allocation */
    if (NULL == p_pkc)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_pkc);

    /* Disable the PKC Peripheral Clock */
    ll_pkc_deinit(p_pkc->p_instance);

    /* DeInit the low level hardware: CLOCK, NVIC... */
    hal_pkc_msp_deinit(p_pkc);

    /* Set PKC error code to none */
    p_pkc->error_code = HAL_PKC_ERROR_NONE;

    /* Initialize the PKC state */
    p_pkc->state = HAL_PKC_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_pkc);

    return HAL_OK;
}

__WEAK void hal_pkc_msp_init(pkc_handle_t *p_pkc)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pkc_msp_init can be implemented in the user file
     */
}

__WEAK void hal_pkc_msp_deinit(pkc_handle_t *p_pkc)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
            the hal_pkc_msp_deinit can be implemented in the user file
     */
}

__WEAK void hal_pkc_irq_handler(pkc_handle_t *p_pkc)
{
    uint32_t itsource = READ_REG(p_pkc->p_instance->INTSTAT);
    uint32_t op_mode, out_ptr;

    if (itsource & PKC_IT_ERR)
    {
        __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_ERR);

        if (PKC_OPERATION_MODE_CMP == ll_pkc_get_operation_mode(p_pkc->p_instance))
        {
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

            out_ptr = ll_pkc_get_mas_c_pointer(p_pkc->p_instance);
            pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);

            if (HAL_PKC_STATE_BUSY == p_pkc->state)
            {
                /* Change state of PKC */
                p_pkc->state = HAL_PKC_STATE_READY;

                /* Error callback */
                hal_pkc_done_callback(p_pkc);
            }
        }
        else
        {
            /* Disable all the PKC Interrupts */
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_ERR | PKC_IT_OVF | PKC_IT_DONE);

            /* Set error code */
            p_pkc->error_code |= HAL_PKC_ERROR_TRANSFER;

            /* Change state of PKC */
            p_pkc->state = HAL_PKC_STATE_READY;

            /* Error callback */
            hal_pkc_error_callback(p_pkc);
        }
    }

    if (itsource & PKC_IT_OVF)
    {
        __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_OVF);

        __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_OVF);

        /* Set error code */
        p_pkc->error_code |= HAL_PKC_ERROR_OVERFLOW;

//        /* Change state of PKC */
//        p_pkc->state = HAL_PKC_STATE_READY;

        /* Error callback */
        hal_pkc_overflow_callback(p_pkc);
    }

    if (itsource & PKC_IT_DONE)
    {
        __HAL_PKC_CLEAR_FLAG_IT(p_pkc, PKC_IT_DONE);

        if (ll_pkc_is_enabled_software(p_pkc->p_instance))
        {
            if (PKC_OPERATION_MODE_LSHIFT != ll_pkc_get_operation_mode(p_pkc->p_instance))
            {
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

                op_mode = ll_pkc_get_operation_mode(p_pkc->p_instance);
                switch(op_mode)
                {
                case PKC_OPERATION_MODE_MULTI:
                    out_ptr = ll_pkc_get_mm_c_pointer(p_pkc->p_instance);
                    break;
                case PKC_OPERATION_MODE_INVER:
                    out_ptr        = ll_pkc_get_mi_x1_pointer(p_pkc->p_instance);
                    *p_pkc->p_kout = ll_pkc_get_mik_output(p_pkc->p_instance);
                    break;
                case PKC_OPERATION_MODE_ADD:
                case PKC_OPERATION_MODE_SUB:
                case PKC_OPERATION_MODE_CMP:
                    out_ptr = ll_pkc_get_mas_c_pointer(p_pkc->p_instance);
                    break;
                case PKC_OPERATION_MODE_BIGMULTI:
                    out_ptr = ll_pkc_get_bm_c_pointer(p_pkc->p_instance);
                    break;
                case PKC_OPERATION_MODE_BIGADD:
                    out_ptr = ll_pkc_get_ba_c_pointer(p_pkc->p_instance);
                    break;
                default:
                    break;
                }
                pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);

                if (HAL_PKC_STATE_BUSY == p_pkc->state)
                {
                    /* Change state of PKC */
                    p_pkc->state = HAL_PKC_STATE_READY;

                    /* Error callback */
                    hal_pkc_done_callback(p_pkc);
                }
            }
            else
            {
                p_pkc->shift_count--;
                if (0 < p_pkc->shift_count)
                {
                    ll_pkc_disable_software_start(p_pkc->p_instance);
                    ll_pkc_enable_software_start(p_pkc->p_instance);
                }
                else
                {
                    __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

                    out_ptr = ll_pkc_get_mas_c_pointer(p_pkc->p_instance);
                    pkc_read_spram(p_pkc, p_pkc->p_result, out_ptr);

                    if (HAL_PKC_STATE_BUSY == p_pkc->state)
                    {
                        /* Change state of PKC */
                        p_pkc->state = HAL_PKC_STATE_READY;

                        /* Error callback */
                        hal_pkc_done_callback(p_pkc);
                    }
                }
            }
        }
        else
        {
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE);

            pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->X, 0x48);
            pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->Y, 0x50);

            if (HAL_PKC_STATE_BUSY == p_pkc->state)
            {
                /* Change state of PKC */
                p_pkc->state = HAL_PKC_STATE_READY;

                /* Error callback */
                hal_pkc_done_callback(p_pkc);
            }
        }
    }
}

__WEAK hal_status_t hal_pkc_rsa_modular_exponent(pkc_handle_t *p_pkc, pkc_rsa_modular_exponent_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            uint32_t i = 0;
            uint32_t op_out_ptr = 0;
            uint32_t op_ina_ptr = 1 * (p_pkc->init.data_bits >> 5);
            uint32_t op_inb_ptr = 2 * (p_pkc->init.data_bits >> 5);
            uint32_t op_inp_ptr = 3 * (p_pkc->init.data_bits >> 5);

            do {
                /* Step 1. Find most significant 1 in input b[] */
                i = pkc_find_msb(p_input->p_B, p_pkc->init.data_bits);

                if (1 >= i)
                {
                    if (0 == p_input->p_B[(p_pkc->init.data_bits >> 5) - 1])
                    {
                        memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3);
                        ((uint32_t *)p_pkc->p_result)[(p_pkc->init.data_bits >> 5) - 1] = 1;
                    }
                    else
                    {
                        memcpy(p_pkc->p_result, p_input->p_A, p_pkc->init.data_bits >> 3);
                    }
                    break;
                }
                /* Begins hardware computation */
                /* Step 2. Enable PKC and load data */
                __HAL_PKC_RESET(p_pkc);
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

                pkc_write_spram(p_pkc, p_input->p_A,  op_ina_ptr);   // input A
                pkc_write_spram(p_pkc, p_input->p_P_R2, op_inb_ptr); // R^2 mod P
                pkc_write_spram(p_pkc, p_input->p_P,  op_inp_ptr);   // input P
                ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                /* Step 3. Calculate MM(A, R^2), store in op_out_ptr */
                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
                pkc_sw_init.enable_it = DISABLE;
                pkc_sw_init.ptrA      = op_ina_ptr;
                pkc_sw_init.ptrB      = op_inb_ptr;
                pkc_sw_init.ptrC      = op_out_ptr;
                pkc_sw_init.ptrP      = op_inp_ptr;
                pkc_sw_init.timeout   = timeout;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_pkc->p_result,  op_out_ptr);
                pkc_write_spram(p_pkc, p_pkc->p_result, op_ina_ptr);

                /* Step 4. if(getbit == 0){x = x^2 } else {x = x^2 * a;} */
                pkc_sw_init.ptrA = op_out_ptr;
                for (i--; i > 0; i--)
                {
                    pkc_sw_init.ptrB = op_out_ptr;
                    status = pkc_software_operation(p_pkc, &pkc_sw_init);
                    if (1 == pkc_get_bit(p_input->p_B, i - 1, p_pkc->init.data_bits))
                    {
                        pkc_sw_init.ptrB = op_ina_ptr;
                        status = pkc_software_operation(p_pkc, &pkc_sw_init);
                        if (HAL_OK != status)
                            break;
                    }
                }
                if (HAL_OK != status)
                    break;

                /* Step 5. Calculate out = MM(x,1) */
                memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3);
                ((uint32_t *)p_pkc->p_result)[(p_pkc->init.data_bits >> 5) - 1] = 1;
                pkc_write_spram(p_pkc, p_pkc->p_result, op_inb_ptr);
                pkc_sw_init.ptrB = op_inb_ptr;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_pkc->p_result,  op_out_ptr);
            } while(0);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_ecc_point_multi(pkc_handle_t *p_pkc, pkc_ecc_point_multi_t *p_input, uint32_t timeout)
{
    hal_status_t           status    = HAL_OK;
    uint32_t               tickstart = hal_get_tick();
    ecc_point_multi_init_t ecc_point_multi_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            do {
                if (NULL == p_input->p_ecc_point)
                {
                    p_input->p_ecc_point = &p_pkc->init.p_ecc_curve->G;
                }
                else if (ecc_is_infinite_point(p_input->p_ecc_point))
                {
                    memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                    break;
                }

                if (0 == pkc_compare_const(p_input->p_K, 0, p_pkc->init.data_bits))
                {
                    memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                    break;
                }

                if (0 == pkc_compare_const(p_input->p_K, 1, p_pkc->init.data_bits))
                {
                    memcpy(p_pkc->p_result, p_input->p_ecc_point, sizeof(ecc_point_t));
                    break;
                }

                ecc_point_multi_init.enable_it = DISABLE;
                ecc_point_multi_init.p_input   = p_input;
                ecc_point_multi_init.p_result  = p_pkc->p_result;
                ecc_point_multi_init.timeout   = timeout;
                status = ecc_point_multiply(p_pkc, &ecc_point_multi_init);

                pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->X, 0x48);
                pkc_read_spram(p_pkc, ((ecc_point_t *)p_pkc->p_result)->Y, 0x50);
            } while(0);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_ecc_point_multi_it(pkc_handle_t *p_pkc, pkc_ecc_point_multi_t *p_input)
{
    hal_status_t           status    = HAL_OK;
    uint32_t               tickstart = hal_get_tick();
    ecc_point_multi_init_t ecc_point_multi_init;
    uint32_t               callback_flag = 0;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            do {
                if (NULL == p_input->p_ecc_point)
                {
                    p_input->p_ecc_point = &p_pkc->init.p_ecc_curve->G;
                }
                else if (ecc_is_infinite_point(p_input->p_ecc_point))
                {
                    memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                    callback_flag = 1;
                    break;
                }

                if (0 == pkc_compare_const(p_input->p_K, 0, p_pkc->init.data_bits))
                {
                    memset(p_pkc->p_result, 0, sizeof(ecc_point_t));
                    callback_flag = 1;
                    break;
                }

                if (0 == pkc_compare_const(p_input->p_K, 1, p_pkc->init.data_bits))
                {
                    memcpy(p_pkc->p_result, p_input->p_ecc_point, sizeof(ecc_point_t));
                    callback_flag = 1;
                    break;
                }

                ecc_point_multi_init.enable_it   = ENABLE;
                ecc_point_multi_init.p_input     = p_input;
                ecc_point_multi_init.p_result    = p_pkc->p_result;
                status = ecc_point_multiply(p_pkc, &ecc_point_multi_init);
            } while(0);

            if (callback_flag)
            {
                /* Update PKC state */
                p_pkc->state = HAL_PKC_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_pkc);

                hal_pkc_done_callback(p_pkc);
            }
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_add(pkc_handle_t *p_pkc, pkc_modular_add_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 2 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrP = 3 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_ADD;
            pkc_sw_init.enable_it = DISABLE;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);

            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_add_it(pkc_handle_t *p_pkc, pkc_modular_add_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 2 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrP = 3 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_ADD;
            pkc_sw_init.enable_it = ENABLE;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_sub(pkc_handle_t *p_pkc, pkc_modular_sub_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 2 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrP = 3 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_SUB;
            pkc_sw_init.enable_it = DISABLE;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);

            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_sub_it(pkc_handle_t *p_pkc, pkc_modular_sub_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 2 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrP = 3 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_SUB;
            pkc_sw_init.enable_it = ENABLE;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_left_shift(pkc_handle_t *p_pkc, pkc_modular_shift_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 0xFFFFFFFF;

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_LSHIFT;
            pkc_sw_init.enable_it = DISABLE;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
            for (uint32_t i = 1; i < p_input->shift_bits; i++)
            {
                ll_pkc_disable_software_start(p_pkc->p_instance);
                ll_pkc_enable_software_start(p_pkc->p_instance);
                status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);
            }

            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_left_shift_it(pkc_handle_t *p_pkc, pkc_modular_shift_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 0xFFFFFFFF;

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            p_pkc->shift_count = p_input->shift_bits;
            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_LSHIFT;
            pkc_sw_init.enable_it = ENABLE;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_compare(pkc_handle_t *p_pkc, pkc_modular_compare_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 0xFFFFFFFF;

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
            pkc_sw_init.enable_it = DISABLE;
            pkc_sw_init.timeout  = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);

            pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_modular_compare_it(pkc_handle_t *p_pkc, pkc_modular_compare_t *p_input)
{
    hal_status_t            status        = HAL_OK;
    uint32_t                tickstart     = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;
    uint32_t                callback_flag = 0;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrC = 0;
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrB = 0xFFFFFFFF;

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
            pkc_sw_init.enable_it = ENABLE;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);

            if (callback_flag)
            {
                /* Update PKC state */
                p_pkc->state = HAL_PKC_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_pkc);

                hal_pkc_done_callback(p_pkc);
            }
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_multi(pkc_handle_t *p_pkc, pkc_montgomery_multi_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            do {
                pkc_sw_init.ptrC = 0;
                pkc_sw_init.ptrA = 0;
                pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrB = 0xFFFFFFFF;

                __HAL_PKC_RESET(p_pkc);
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

                pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
                pkc_sw_init.enable_it = DISABLE;
                pkc_sw_init.timeout   = timeout;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_input->p_A,  pkc_sw_init.ptrC);

                __HAL_PKC_RESET(p_pkc);

                pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrA);   // input B
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_input->p_B,  pkc_sw_init.ptrC);

                if (pkc_compare_const(p_input->p_A, 0, p_pkc->init.data_bits) == 0 || \
                    pkc_compare_const(p_input->p_B, 0, p_pkc->init.data_bits) == 0)
                {
                    memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3);
                    break;
                }

                pkc_sw_init.ptrC = 0;
                pkc_sw_init.ptrA = 1 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrB = 2 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrP = 3 * (p_pkc->init.data_bits >> 5);

                __HAL_PKC_RESET(p_pkc);

                pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
                pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
                pkc_sw_init.enable_it = DISABLE;
                pkc_sw_init.timeout   = timeout;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);

                pkc_read_spram(p_pkc, p_pkc->p_result,  pkc_sw_init.ptrC);
            } while(0);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_multi_it(pkc_handle_t *p_pkc, pkc_montgomery_multi_t *p_input)
{
    hal_status_t            status        = HAL_OK;
    uint32_t                tickstart     = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;
    uint32_t                callback_flag = 0;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            do {
                pkc_sw_init.ptrC = 0;
                pkc_sw_init.ptrA = 0;
                pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrB = 0xFFFFFFFF;

                __HAL_PKC_RESET(p_pkc);
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

                pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
                pkc_sw_init.enable_it = DISABLE;
                pkc_sw_init.timeout   = HAL_PKC_TIMEOUT_DEFAULT_VALUE;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_input->p_A,  pkc_sw_init.ptrC);

                __HAL_PKC_RESET(p_pkc);

                pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrA);   // input B
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_input->p_B,  pkc_sw_init.ptrC);

                if (pkc_compare_const(p_input->p_A, 0, p_pkc->init.data_bits) == 0 || \
                    pkc_compare_const(p_input->p_B, 0, p_pkc->init.data_bits) == 0)
                {
                    memset(p_pkc->p_result, 0, p_pkc->init.data_bits >> 3);
                    callback_flag = 1;
                    break;
                }

                pkc_sw_init.ptrC = 0;
                pkc_sw_init.ptrA = 1 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrB = 2 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrP = 3 * (p_pkc->init.data_bits >> 5);

                __HAL_PKC_RESET(p_pkc);

                pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
                pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_constp(p_pkc->p_instance, p_input->ConstP);
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_MULTI;
                pkc_sw_init.enable_it = ENABLE;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
            } while(0);

            if (callback_flag)
            {
                /* Update PKC state */
                p_pkc->state = HAL_PKC_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_pkc);

                hal_pkc_done_callback(p_pkc);
            }
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_inversion(pkc_handle_t *p_pkc, pkc_montgomery_inversion_t *p_input, uint32_t *p_K, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            do {
                if (1 != (p_input->p_P[(p_pkc->init.data_bits >> 5) - 1] & 0x1))
                {
                    status = HAL_ERROR;
                    p_pkc->error_code = HAL_PKC_ERROR_INVALID_PARAM;
                    break;
                }

                pkc_sw_init.ptrC = 0;
                pkc_sw_init.ptrA = 0;
                pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrB = 0xFFFFFFFF;

                __HAL_PKC_RESET(p_pkc);
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

                pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
                pkc_sw_init.enable_it = DISABLE;
                pkc_sw_init.timeout   = timeout;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_input->p_A,  pkc_sw_init.ptrC);

                if (0 == pkc_compare_const(p_input->p_A, 0, p_pkc->init.data_bits))
                {
                    status = HAL_ERROR;
                    p_pkc->error_code = HAL_PKC_ERROR_INVALID_PARAM;
                    break;
                }

                uint32_t inX1[64]   = {0};
                uint32_t op_ina_ptr = 0;
                uint32_t op_inp_ptr = 1 * (p_pkc->init.data_bits >> 5);
                uint32_t op_x1_ptr  = 2 * (p_pkc->init.data_bits >> 5);
                uint32_t op_x2_ptr  = 3 * (p_pkc->init.data_bits >> 5);
                uint32_t op_tmp_ptr = 4 * (p_pkc->init.data_bits >> 5);

                __HAL_PKC_RESET(p_pkc);

                pkc_write_spram(p_pkc, p_input->p_A, op_ina_ptr);   // input A
                pkc_write_spram(p_pkc, p_input->p_P, op_inp_ptr);   // input P
                pkc_write_spram(p_pkc, inX1, op_x2_ptr);
                inX1[(p_pkc->init.data_bits >> 5) - 1] = 1;
                pkc_write_spram(p_pkc, inX1, op_x1_ptr);

                ll_pkc_set_mi_x1_pointer(p_pkc->p_instance, op_x1_ptr);
                ll_pkc_set_mi_x2_pointer(p_pkc->p_instance, op_x2_ptr);
                ll_pkc_set_swmi_tmp_pointer(p_pkc->p_instance, op_tmp_ptr);
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_INVER;
                pkc_sw_init.enable_it = DISABLE;
                pkc_sw_init.timeout   = timeout;
                pkc_sw_init.ptrA      = op_ina_ptr;
                pkc_sw_init.ptrP      = op_inp_ptr;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_pkc->p_result, op_x1_ptr);
                pkc_read_spram(p_pkc, p_input->p_A, op_ina_ptr);
                *p_K = ll_pkc_get_mik_output(p_pkc->p_instance);

                if ((*p_K < p_pkc->init.data_bits) || (*p_K > (p_pkc->init.data_bits << 1)))
                {
                    memset(p_K, 0, p_pkc->init.data_bits >> 3);
                    status = HAL_ERROR;
                    p_pkc->error_code = HAL_PKC_ERROR_INVERSE_K;
                }

                if ((HAL_OK == status) && (0 != pkc_compare_const(p_input->p_A, 1, p_pkc->init.data_bits)))
                {
                    memset(p_K, 0, p_pkc->init.data_bits >> 3);
                    status = HAL_ERROR;
                    p_pkc->error_code = HAL_PKC_ERROR_IRREVERSIBLE;
                }
            } while(0);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_montgomery_inversion_it(pkc_handle_t *p_pkc, pkc_montgomery_inversion_t *p_input, uint32_t *p_K)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            do {
                if (1 != (p_input->p_P[(p_pkc->init.data_bits >> 5) - 1] & 0x1))
                {
                    status = HAL_ERROR;
                    break;
                }

                pkc_sw_init.ptrC = 0;
                pkc_sw_init.ptrA = 0;
                pkc_sw_init.ptrP = 1 * (p_pkc->init.data_bits >> 5);
                pkc_sw_init.ptrB = 0xFFFFFFFF;

                __HAL_PKC_RESET(p_pkc);
                __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

                pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
                pkc_write_spram(p_pkc, p_input->p_P, pkc_sw_init.ptrP);   // input P
                ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

                pkc_sw_init.op_mode   = PKC_OPERATION_MODE_CMP;
                pkc_sw_init.enable_it = DISABLE;
                pkc_sw_init.timeout   = HAL_PKC_TIMEOUT_DEFAULT_VALUE;
                status = pkc_software_operation(p_pkc, &pkc_sw_init);
                if (HAL_OK != status)
                    break;

                pkc_read_spram(p_pkc, p_input->p_A,  pkc_sw_init.ptrC);

                if (0 == pkc_compare_const(p_input->p_A, 0, p_pkc->init.data_bits))
                    status = HAL_ERROR;
            } while(0);

            if (HAL_OK != status)
            {
                p_pkc->error_code = HAL_PKC_ERROR_INVALID_PARAM;
                /* Update PKC state */
                p_pkc->state = HAL_PKC_STATE_READY;
                /* Process unlocked */
                __HAL_UNLOCK(p_pkc);
                return status;
            }

            uint32_t inX1[64]   = {0};
            uint32_t op_ina_ptr = 0;
            uint32_t op_inp_ptr = 1 * (p_pkc->init.data_bits >> 5);
            uint32_t op_x1_ptr  = 2 * (p_pkc->init.data_bits >> 5);
            uint32_t op_x2_ptr  = 3 * (p_pkc->init.data_bits >> 5);
            uint32_t op_tmp_ptr = 4 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);

            pkc_write_spram(p_pkc, p_input->p_A, op_ina_ptr);   // input A
            pkc_write_spram(p_pkc, p_input->p_P, op_inp_ptr);   // input P
            pkc_write_spram(p_pkc, inX1, op_x2_ptr);
            inX1[(p_pkc->init.data_bits >> 5) - 1] = 1;
            pkc_write_spram(p_pkc, inX1, op_x1_ptr);
            p_pkc->p_kout = p_K;

            ll_pkc_set_mi_x1_pointer(p_pkc->p_instance, op_x1_ptr);
            ll_pkc_set_mi_x2_pointer(p_pkc->p_instance, op_x2_ptr);
            ll_pkc_set_swmi_tmp_pointer(p_pkc->p_instance, op_tmp_ptr);
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_INVER;
            pkc_sw_init.enable_it = ENABLE;
            pkc_sw_init.ptrA      = op_ina_ptr;
            pkc_sw_init.ptrP      = op_inp_ptr;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_multi(pkc_handle_t *p_pkc, pkc_big_number_multi_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BIGMULTI_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrB = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrC = 2 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGMULTI;
            pkc_sw_init.enable_it = DISABLE;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);

            pkc_read_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrC);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_multi_it(pkc_handle_t *p_pkc, pkc_big_number_multi_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BIGMULTI_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrB = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrC = 2 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGMULTI;
            pkc_sw_init.enable_it = ENABLE;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_add(pkc_handle_t *p_pkc, pkc_big_number_add_t *p_input, uint32_t timeout)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, timeout);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrB = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrC = 2 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGADD;
            pkc_sw_init.enable_it = DISABLE;
            pkc_sw_init.timeout   = timeout;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);

            pkc_read_spram(p_pkc, p_pkc->p_result, pkc_sw_init.ptrC);
        }
        p_pkc->state = HAL_PKC_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_pkc_big_number_add_it(pkc_handle_t *p_pkc, pkc_big_number_add_t *p_input)
{
    hal_status_t            status    = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;

    gr_assert_param(IS_PKC_SECURE_MODE(p_pkc->init.secure_mode));
    gr_assert_param(IS_PKC_BITS_LENGTH(p_pkc->init.data_bits));

    /* Process locked */
    __HAL_LOCK(p_pkc);

    if (HAL_PKC_STATE_READY == p_pkc->state)
    {
        p_pkc->error_code = HAL_PKC_ERROR_NONE;

        /* Update PKC state */
        p_pkc->state = HAL_PKC_STATE_BUSY;

        /* Wait till BUSY flag reset */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, HAL_PKC_TIMEOUT_DEFAULT_VALUE);

        if (HAL_OK == status)
        {
            pkc_sw_init.ptrA = 0;
            pkc_sw_init.ptrB = 1 * (p_pkc->init.data_bits >> 5);
            pkc_sw_init.ptrC = 2 * (p_pkc->init.data_bits >> 5);

            __HAL_PKC_RESET(p_pkc);
            __HAL_PKC_DISABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR | PKC_IT_OVF);

            pkc_write_spram(p_pkc, p_input->p_A, pkc_sw_init.ptrA);   // input A
            pkc_write_spram(p_pkc, p_input->p_B, pkc_sw_init.ptrB);   // input B
            ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

            pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGADD;
            pkc_sw_init.enable_it = ENABLE;
            status = pkc_software_operation(p_pkc, &pkc_sw_init);
        }
        else
        {
            /* Update PKC state */
            p_pkc->state = HAL_PKC_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_pkc);

    /* Return function status */
    return status;
}

__WEAK void hal_pkc_done_callback(pkc_handle_t *p_pkc)
{

}

__WEAK void hal_pkc_error_callback(pkc_handle_t *p_pkc)
{

}

__WEAK void hal_pkc_overflow_callback(pkc_handle_t *p_pkc)
{

}

__WEAK void hal_pkc_abort_cplt_callback(pkc_handle_t *p_pkc)
{

}

__WEAK hal_pkc_state_t hal_pkc_get_state(pkc_handle_t *p_pkc)
{
    return p_pkc->state;
}

__WEAK uint32_t hal_pkc_get_error(pkc_handle_t *p_pkc)
{
    return p_pkc->error_code;
}

__WEAK void hal_pkc_set_timeout(pkc_handle_t *p_pkc, uint32_t timeout)
{
    p_pkc->timeout = timeout;
}

static hal_status_t pkc_wait_flag_state_until_timeout(pkc_handle_t *p_pkc,
                                                      uint32_t      flag,
                                                      flag_status_t state,
                                                      uint32_t      tick_start,
                                                      uint32_t      timeout)
{
    /* Wait until flag is in expected state */
    while ((__HAL_PKC_GET_FLAG(p_pkc, flag)) != state)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if ((0 == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                p_pkc->state     = HAL_PKC_STATE_ERROR;
                p_pkc->error_code |= HAL_PKC_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    }
    return HAL_OK;
}

static void pkc_read_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr)
{
    uint32_t *preg         = (uint32_t *)(PKC_SPRAM_BASE + (ptr << 2));
    uint32_t data_word_len = p_pkc->init.data_bits >> 5;

    if (PKC_OPERATION_MODE_BIGMULTI == ll_pkc_get_operation_mode(p_pkc->p_instance))
        data_word_len <<= 1;

    for (uint32_t i = 0; i < data_word_len; i++)
    {
        p_data[data_word_len - i - 1] = preg[i];
    }
}

static void pkc_write_spram(pkc_handle_t *p_pkc, uint32_t *p_data, uint32_t ptr)
{
    uint32_t *preg = (uint32_t *)(PKC_SPRAM_BASE + (ptr << 2));
    uint32_t data_word_len = p_pkc->init.data_bits >> 5;

    for (uint32_t i = 0; i < data_word_len; i++)
    {
        preg[i] = p_data[data_word_len - i - 1];
    }
}

/*
    return value: 0, input data = 0
    return value: 1~DataBits, input data > 0
*/
static uint32_t pkc_find_msb(uint32_t *p_data, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5;
    uint32_t ret = 0, i, j;

    for (i = 0; i < data_word_len; i++)
    {
        if (0 != p_data[i])
        {
            for (j = 32; j > 0; j--)
            {
                if ((p_data[i] >> (j - 1)) & 0x1)
                {
                    ret = j + ((data_word_len - i - 1) << 5);
                    break;
                }
            }
        }
    }

    return ret;
}

/* ofs: 0 ~ data_bits-1 */
static uint32_t pkc_get_bit(uint32_t *p_data, uint32_t ofs, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5;

    if (p_data[data_word_len - 1 - (ofs >> 5)] & (1 << (ofs & 0x1F)))
        return 1;
    else
        return 0;
}

static int32_t pkc_compare_const(uint32_t *p_data, uint32_t Const, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5;
    uint32_t i;

    if (p_data[data_word_len - 1] > Const)
        return 1;

    for (i = 0; i < data_word_len - 1; i++)
    {
        if (0 < p_data[i])
            return 1;
    }

    if (p_data[data_word_len - 1] == Const)
        return 0;

    return -1;
}

#if 0
static int32_t pkc_compare_number(uint32_t *pDataA, uint32_t *p_dataB, uint32_t data_bits)
{
    uint32_t data_word_len = data_bits >> 5;
    uint32_t i;

    for (i = 0; i < data_word_len; i++)
    {
        if (pDataA[i] > p_dataB[i])
            return 1;
        if (pDataA[i] < p_dataB[i])
            return -1;
    }

    return 0;
}
#endif

//ret : 0 --> not infinite point , 1--> is infinite point
static uint32_t ecc_is_infinite_point(ecc_point_t *p_Piont)
{
    for (uint32_t i = 0; i < ECC_U32_LENGTH; i++)
    {
        if ((0 != p_Piont->X[i]) || (0 != p_Piont->Y[i]))
            return 0;
    }

    return 1;
}

static hal_status_t pkc_software_operation(pkc_handle_t *p_pkc, pkc_sw_operation_init_t *p_input)
{
    hal_status_t status    = HAL_OK;
    uint32_t     tickstart = hal_get_tick();

    switch(p_input->op_mode)
    {
    case PKC_OPERATION_MODE_MULTI:
        ll_pkc_set_mm_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_mm_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_mm_p_pointer(p_pkc->p_instance, p_input->ptrP);
        ll_pkc_set_mm_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    case PKC_OPERATION_MODE_INVER:
        ll_pkc_set_mi_u_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_mi_v_pointer(p_pkc->p_instance, p_input->ptrP);
        break;
    case PKC_OPERATION_MODE_CMP:
    case PKC_OPERATION_MODE_LSHIFT:
    case PKC_OPERATION_MODE_ADD:
    case PKC_OPERATION_MODE_SUB:
        ll_pkc_set_mas_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_mas_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_mas_p_pointer(p_pkc->p_instance, p_input->ptrP);
        ll_pkc_set_mas_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    case PKC_OPERATION_MODE_BIGMULTI:
        ll_pkc_set_bm_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_bm_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_bm_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    case PKC_OPERATION_MODE_BIGADD:
        ll_pkc_set_ba_a_pointer(p_pkc->p_instance, p_input->ptrA);
        ll_pkc_set_ba_b_pointer(p_pkc->p_instance, p_input->ptrB);
        ll_pkc_set_ba_c_pointer(p_pkc->p_instance, p_input->ptrC);
        break;
    default:
        status = HAL_ERROR;
        break;
    }

    if (HAL_OK == status)
    {
        /* Set operation mode */
        ll_pkc_set_operation_mode(p_pkc->p_instance, p_input->op_mode);
        /* Enable SW mode */
        ll_pkc_enable_software(p_pkc->p_instance);
        ll_pkc_disable_software_start(p_pkc->p_instance);
        if (PKC_SECURE_MODE_ENABLE == p_pkc->init.secure_mode)
        {
            ll_pkc_set_dummy_multiply_seed(p_pkc->p_instance, p_pkc->init.random_func());
            ll_pkc_set_random_clock_gating_seed(p_pkc->p_instance, p_pkc->init.random_func());

            ll_pkc_enable_random_clock_gating(p_pkc->p_instance);
        }

        if (PKC_OPERATION_MODE_BIGMULTI != p_input->op_mode)
        {
            ll_pkc_enable_dummy_multi(p_pkc->p_instance);
        }

        if (DISABLE == p_input->enable_it)
        {
            ll_pkc_enable_software_start(p_pkc->p_instance);
            /* Wait for finish */
            status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, p_input->timeout);

            if ((HAL_OK == status) && (PKC_OPERATION_MODE_CMP != p_input->op_mode))
            {
                if (p_pkc->p_instance->INTSTAT & PKC_INTSTAT_ERR)
                    status = HAL_ERROR;
            }
            ll_pkc_disable_software(p_pkc->p_instance);
        }
        else
        {
            __HAL_PKC_ENABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR);
            if (PKC_OPERATION_MODE_BIGADD == p_input->op_mode)
                __HAL_PKC_ENABLE_IT(p_pkc, PKC_IT_OVF);

            ll_pkc_enable_software_start(p_pkc->p_instance);
        }
    }

    return status;
}

static hal_status_t ecc_point_multiply(pkc_handle_t *p_pkc, ecc_point_multi_init_t *p_input)
{
    hal_status_t            status = HAL_OK;
    uint32_t                tickstart = hal_get_tick();
    pkc_sw_operation_init_t pkc_sw_init;
    uint32_t                i;
    uint32_t                r_random[ECC_U32_LENGTH << 1] = {0};
    uint32_t                gz_random[ECC_U32_LENGTH] = {0};
    uint32_t                tmpk[ECC_U32_LENGTH] = {0};

    //clear SPRAM
    memset((uint8_t *)PKC_SPRAM_BASE, 0x00, 2048);

    pkc_sw_init.ptrC = 0;
    pkc_sw_init.ptrA = 0;
    pkc_sw_init.ptrB = 2 * (p_pkc->init.data_bits >> 5);

    if (PKC_SECURE_MODE_ENABLE == p_pkc->init.secure_mode)
    {
        uint32_t rg[ECC_U32_LENGTH] = {0};

        rg[ECC_U32_LENGTH - 1] = p_pkc->init.random_func();
        rg[ECC_U32_LENGTH - 2] = p_pkc->init.random_func();
        rg[ECC_U32_LENGTH - 2] |= 0x80000000;

        for (i = 0; i < ECC_U32_LENGTH; i++)
        {
            r_random[i] = p_pkc->init.random_func();
            r_random[i + ECC_U32_LENGTH] = p_pkc->init.random_func();
            gz_random[i] = p_pkc->init.random_func();
        }

        pkc_write_spram(p_pkc, rg, pkc_sw_init.ptrA);   // input A
        pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->N, pkc_sw_init.ptrB);   // input B
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGMULTI;
        pkc_sw_init.enable_it = DISABLE;
        pkc_sw_init.timeout   = HAL_PKC_TIMEOUT_DEFAULT_VALUE;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        pkc_write_spram(p_pkc, p_input->p_input->p_K, pkc_sw_init.ptrB);
        ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits << 1);

        pkc_sw_init.op_mode   = PKC_OPERATION_MODE_BIGADD;
        status = pkc_software_operation(p_pkc, &pkc_sw_init);

        ll_pkc_set_dummy_multiply_seed(p_pkc->p_instance, p_pkc->init.random_func());
        ll_pkc_set_random_clock_gating_seed(p_pkc->p_instance, p_pkc->init.random_func());

        ll_pkc_enable_random_clock_gating(p_pkc->p_instance);
    }
    else
    {
        pkc_write_spram(p_pkc, p_input->p_input->p_K, pkc_sw_init.ptrA);
        r_random[0] = p_pkc->init.random_func();

        for (i = 0; i < ECC_U32_LENGTH; i++)
        {
            r_random[i] = r_random[0] * (i + 1);
            r_random[i + ECC_U32_LENGTH] = r_random[0] * (0xFABCD971 + 3 * i);
            gz_random[i] = r_random[0] * (0xDFE11111 + 17 * i);
        }
    }
    ll_pkc_enable_dummy_multi(p_pkc->p_instance);

    p_pkc->init.data_bits <<= 1;
    pkc_write_spram(p_pkc, r_random, 0x10);                          //write r_random
    p_pkc->init.data_bits >>= 1;
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->P, 0x20);           //write p
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->PRSquare, 0x28);    //write R^2 mod p
    pkc_write_spram(p_pkc, p_input->p_input->p_ecc_point->X, 0x30);        //write Point's x axis
    pkc_write_spram(p_pkc, p_input->p_input->p_ecc_point->Y, 0x38);        //write Point's y axis
    pkc_write_spram(p_pkc, gz_random, 0x40);                         //write random
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->A, 0xD0);           //write a*R
    pkc_write_spram(p_pkc, p_pkc->init.p_ecc_curve->B, 0xD8);           //write b*R

    /* Compute tmpk = R mod p = 0xFFFF...FFF - p + 1 = 0xFFFFFF...FF xor p + 1 */
    for (i = 0; i < ECC_U32_LENGTH; i++)
    {
        tmpk[i] = 0xFFFFFFFF ^ p_pkc->init.p_ecc_curve->P[i];
    }
    tmpk[ECC_U32_LENGTH - 1]++;

    pkc_write_spram(p_pkc, tmpk, 0xC8);                              //write R
    ll_pkc_set_constp(p_pkc->p_instance, p_pkc->init.p_ecc_curve->ConstP);
    ll_pkc_set_operation_word_length(p_pkc->p_instance, p_pkc->init.data_bits);

    if (DISABLE == p_input->enable_it)
    {
        ll_pkc_enable_hardware_start(p_pkc->p_instance);
        /* Wait for finish */
        status = pkc_wait_flag_state_until_timeout(p_pkc, PKC_FLAG_BUSY, RESET, tickstart, p_input->timeout);

        if ((HAL_OK == status) && (p_pkc->p_instance->INTSTAT & PKC_INTSTAT_ERR))
            status = HAL_ERROR;

        ll_pkc_disable_hardware_start(p_pkc->p_instance);
    }
    else
    {
        __HAL_PKC_ENABLE_IT(p_pkc, PKC_IT_DONE | PKC_IT_ERR);

        ll_pkc_enable_hardware_start(p_pkc->p_instance);
    }

    return status;
}

#endif /* HAL_PKC_MODULE_ENABLED */


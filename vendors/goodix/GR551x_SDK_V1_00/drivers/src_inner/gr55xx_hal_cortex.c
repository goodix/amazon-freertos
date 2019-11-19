/**
  ******************************************************************************
  * @file    gr55xx_hal_cortex.c
  * @author  BLE Driver Team
  * @brief   CORTEX HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the CORTEX:
  *           + Initialization and de-initialization functions
  *           + Peripheral Control functions
  *
  *  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================

    [..]
    *** How to configure Interrupts using CORTEX HAL driver ***
    ===========================================================
    [..]
    This section provides functions allowing to configure the NVIC interrupts (IRQ).
    The Cortex-M4 exceptions are managed by CMSIS functions.

    (#) Configure the NVIC Priority Grouping using hal_nvic_set_priority_grouping() function

     (#)  Configure the priority of the selected IRQ Channels using hal_nvic_set_priority()

     (#)  Enable the selected IRQ Channels using hal_nvic_enable_irq()


     -@- When the NVIC_PRIORITYGROUP_0 is selected, IRQ pre-emption is no more possible.
         The pending IRQ priority will be managed only by the sub priority.

     -@- IRQ priority order (sorted by highest to lowest priority):
        (+@) Lowest pre-emption priority
        (+@) Lowest sub priority
        (+@) Lowest hardware priority (IRQ number)

    [..]
    *** How to configure Systick using CORTEX HAL driver ***
    ========================================================
    [..]
    Setup SysTick Timer for time base

   (+) The hal_systick_config() function calls the systick_config() function which
       is a CMSIS function that:
        (++) Configures the SysTick Reload register with value passed as function parameter.
        (++) Configures the SysTick IRQ priority to the lowest value (0x0FU).
        (++) Resets the SysTick Counter register.
        (++) Configures the SysTick Counter clock source to be Core Clock Source (HCLK).
        (++) Enables the SysTick Interrupt.
        (++) Starts the SysTick Counter.

   (+) You can change the SysTick Clock source to be HCLK_Div8 by calling the macro
       __HAL_CORTEX_SYSTICKCLK_CONFIG(SYSTICK_CLKSOURCE_REFCLK) just after the
       hal_systick_config() function call. The __HAL_CORTEX_SYSTICKCLK_CONFIG() macro is defined
       inside the gr55xx_hal_cortex.h file.

   (+) You can change the SysTick IRQ priority by calling the
       hal_nvic_set_priority(SysTick_IRQn,...) function just after the hal_systick_config() function
       call. The hal_nvic_set_priority() call the nvic_set_priority() function which is a CMSIS function.

   (+) To adjust the SysTick time base, use the following formula:

       Reload Value = SysTick Counter Clock (Hz) x  Desired Time base (s)
       (++) Reload Value is the parameter to be passed for hal_systick_config() function
       (++) Reload Value should not exceed 0xFFFFFF

  @endverbatim
  */

/*
  Additional Tables: CORTEX_NVIC_Priority_Table
     The table below gives the allowed values of the pre-emption priority and subpriority according
     to the Priority Grouping configuration performed by hal_nvic_set_priority_grouping() function
       ==========================================================================================================================
         NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  |       Description
       ==========================================================================================================================
        NVIC_PRIORITYGROUP_0  |                0                  |            0U-255           | 0 bits for pre-emption priority
                              |                                   |                             | 8 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_1  |                0U-1               |            0U-127           | 1 bits for pre-emption priority
                              |                                   |                             | 7 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_2  |                0U-3               |            0U-63            | 2 bits for pre-emption priority
                              |                                   |                             | 6 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_3  |                0U-7               |            0U-31            | 3 bits for pre-emption priority
                              |                                   |                             | 5 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_4  |                0U-15              |            0U-15            | 4 bits for pre-emption priority
                              |                                   |                             | 4 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_5  |                0U-31              |            0U-7             | 5 bits for pre-emption priority
                              |                                   |                             | 3 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_6  |                0U-63              |            0U-3             | 6 bits for pre-emption priority
                              |                                   |                             | 2 bits for subpriority
       --------------------------------------------------------------------------------------------------------------------------
        NVIC_PRIORITYGROUP_7  |                0U-127             |            0U-1             | 7 bits for pre-emption priority
                              |                                   |                             | 1 bits for subpriority
       ==========================================================================================================================

*/

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_CORTEX_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/

/** @defgroup CORTEX_Exported_Functions CORTEX Exported Functions
  * @{
  */


/** @defgroup CORTEX_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

__WEAK void hal_nvic_set_priority_grouping(uint32_t priority_group)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_PRIORITY_GROUP(priority_group));

    /* Set the PRIGROUP[10:8] bits according to the PriorityGroup parameter value */
    NVIC_SetPriorityGrouping(priority_group);
}

__WEAK void hal_nvic_set_priority(IRQn_Type IRQn, uint32_t preempt_priority, uint32_t sub_priority)
{
    uint32_t prioritygroup = 0x00U;

    /* Check the parameters */
    gr_assert_param(IS_NVIC_SUB_PRIORITY(sub_priority));
    gr_assert_param(IS_NVIC_PREEMPTION_PRIORITY(preempt_priority));

    prioritygroup = NVIC_GetPriorityGrouping();

    NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, preempt_priority, sub_priority));
}

__WEAK void hal_nvic_enable_irq(IRQn_Type IRQn)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_DEVICE_IRQ(IRQn));

    /* Enable interrupt */
    NVIC_EnableIRQ(IRQn);
}

__WEAK void hal_nvic_disable_irq(IRQn_Type IRQn)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_DEVICE_IRQ(IRQn));

    /* Disable interrupt */
    NVIC_DisableIRQ(IRQn);
}

__WEAK void hal_nvic_system_reset(void)
{
    /* System Reset */
    NVIC_SystemReset();
}

__WEAK uint32_t hal_systick_config(uint32_t ticks_number)
{
    return SysTick_Config(ticks_number);
}
/** @} */

/** @defgroup CORTEX_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */

#if (__MPU_PRESENT == 1U)

__WEAK void hal_mpu_disable(void)
{
    /* Disable fault exceptions */
    SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;

    /* Disable the MPU */
    MPU->CTRL = 0U;
}

__WEAK void hal_mpu_enable(uint32_t mpu_control)
{
    /* Enable the MPU */
    MPU->CTRL   = mpu_control | MPU_CTRL_ENABLE_Msk;

    /* Enable fault exceptions */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
}

__WEAK void hal_mpu_config_region(mpu_region_init_t *p_mpu_init)
{
    /* Check the parameters */
    gr_assert_param(IS_MPU_REGION_NUMBER(p_mpu_init->number));
    gr_assert_param(IS_MPU_REGION_ENABLE(p_mpu_init->enable));

    /* Set the Region number */
    MPU->RNR = p_mpu_init->number;

    if (RESET != (p_mpu_init->enable))
    {
        /* Check the parameters */
        gr_assert_param(IS_MPU_INSTRUCTION_ACCESS(p_mpu_init->disable_exec));
        gr_assert_param(IS_MPU_REGION_PERMISSION_ATTRIBUTE(p_mpu_init->access_permission));
        gr_assert_param(IS_MPU_TEX_LEVEL(p_mpu_init->type_tex_field));
        gr_assert_param(IS_MPU_ACCESS_SHAREABLE(p_mpu_init->is_shareable));
        gr_assert_param(IS_MPU_ACCESS_CACHEABLE(p_mpu_init->is_cacheable));
        gr_assert_param(IS_MPU_ACCESS_BUFFERABLE(p_mpu_init->is_bufferable));
        gr_assert_param(IS_MPU_SUB_REGION_DISABLE(p_mpu_init->subregion_disable));
        gr_assert_param(IS_MPU_REGION_SIZE(p_mpu_init->size));

        MPU->RBAR = p_mpu_init->base_address;
        MPU->RASR = ((uint32_t)p_mpu_init->disable_exec             << MPU_RASR_XN_Pos)   |
                    ((uint32_t)p_mpu_init->access_permission        << MPU_RASR_AP_Pos)   |
                    ((uint32_t)p_mpu_init->type_tex_field            << MPU_RASR_TEX_Pos)  |
                    ((uint32_t)p_mpu_init->is_shareable             << MPU_RASR_S_Pos)    |
                    ((uint32_t)p_mpu_init->is_cacheable             << MPU_RASR_C_Pos)    |
                    ((uint32_t)p_mpu_init->is_bufferable            << MPU_RASR_B_Pos)    |
                    ((uint32_t)p_mpu_init->subregion_disable        << MPU_RASR_SRD_Pos)  |
                    ((uint32_t)p_mpu_init->size                    << MPU_RASR_SIZE_Pos) |
                    ((uint32_t)p_mpu_init->enable                  << MPU_RASR_ENABLE_Pos);
    }
    else
    {
        MPU->RBAR = 0x00U;
        MPU->RASR = 0x00U;
    }
}
#endif /* __MPU_PRESENT */

__WEAK uint32_t hal_nvic_get_priority_grouping(void)
{
    /* Get the PRIGROUP[10:8] field value */
    return NVIC_GetPriorityGrouping();
}

__WEAK void hal_nvic_get_priority(IRQn_Type IRQn, uint32_t priority_group, uint32_t *p_preempt_priority, uint32_t *p_sub_priority)
{
    /* Check the parameters */
    gr_assert_param(IS_NVIC_PRIORITY_GROUP(priority_group));
    /* Get priority for Cortex-M system or device specific interrupts */
    NVIC_DecodePriority(NVIC_GetPriority(IRQn), priority_group, p_preempt_priority, p_sub_priority);
}

__WEAK void hal_nvic_set_pending_irq(IRQn_Type IRQn)
{
    /* Set interrupt pending */
    NVIC_SetPendingIRQ(IRQn);
}

__WEAK uint32_t hal_nvic_get_pending_irq(IRQn_Type IRQn)
{
    /* Return 1 if pending else 0U */
    return NVIC_GetPendingIRQ(IRQn);
}

__WEAK void hal_nvic_clear_pending_irq(IRQn_Type IRQn)
{
    /* Clear pending interrupt */
    NVIC_ClearPendingIRQ(IRQn);
}

__WEAK uint32_t hal_nvic_get_active(IRQn_Type IRQn)
{
    /* Return 1 if active else 0U */
    return NVIC_GetActive(IRQn);
}

__WEAK void hal_systick_clk_source_config(uint32_t clk_source)
{
    /* Check the parameters */
    gr_assert_param(IS_SYSTICK_CLK_SOURCE(clk_source));
    if (SYSTICK_CLKSOURCE_HCLK == clk_source)
    {
        SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
    }
    else
    {
        SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK;
    }
}

__WEAK void hal_systick_irq_handler(void)
{
    hal_systick_callback();
}

__WEAK void hal_systick_callback(void)
{
    /* NOTE : This function Should not be modified, when the callback is needed,
            the hal_systick_callback could be implemented in the user file
    */
}


/** @} */

/** @} */

#endif /* HAL_CORTEX_MODULE_ENABLED */
/** @} */

/** @} */


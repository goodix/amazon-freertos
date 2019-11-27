/**************************************************************************//**
 * @file     system_gr55xx.c
 * @brief    CMSIS Device System Source File for
 *           Device GR55xx
 * @version  V1.00
 * @date     12. June 2018
 ******************************************************************************/
/*
 * Copyright (c) 2009-2016 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include "gr55xx.h"
#include "gr55xx_sys.h"
#include "platform_sdk.h"
#include "custom_config.h"
#include "patch.h"

/*----------------------------------------------------------------------------
  WEAK Functions
 *----------------------------------------------------------------------------*/
__WEAK void sdk_init(void){}

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define SOFTWARE_REG_WAKEUP_FLAG_POS   (8)
#define REG_PL_WR(addr, value)       (*(volatile uint32_t *)(addr)) = (value)
#define REG_PL_RD(addr)              (*(volatile uint32_t *)(addr))

#ifdef ROM_RUN_IN_FLASH
#define READ_VERSION_ADDR()          REG_PL_RD(0x01085004)
#else
#define READ_VERSION_ADDR()          REG_PL_RD(0x45004)
#endif

#define CALIB_LP_CYCLE_COUNT 20

static const uint32_t systemClock[CLK_TYPE_NUM] = {
                                        CLK_64M, /*CPLL_S64M_CLK*/
                                        CLK_48M, /*CPLL_F48M_CLK*/
                                        CLK_16M, /*XO_S16M_CLK*/
                                        CLK_24M, /*CPLL_T24M_CLK*/
                                        CLK_16M, /*CPLL_S16M_CLK*/
                                        CLK_32M, /*CPLL_T32M_CLK*/
                                        };

// xqspi clock table by sys_clk_type
const uint32_t mcu_clk_2_qspi_clk[CLK_TYPE_NUM] = {
                                        [CPLL_S64M_CLK] = QSPI_64M_CLK, 
                                        [CPLL_F48M_CLK] = QSPI_48M_CLK,  
                                        [CPLL_T32M_CLK] = QSPI_32M_CLK,  
                                        [CPLL_T24M_CLK] = QSPI_24M_CLK,  
                                        [CPLL_S16M_CLK] = QSPI_16M_CLK,  
                                        [XO_S16M_CLK] = QSPI_16M_CLK, 
                                        }; 
/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = CLK_64M;  /* System Core Clock Frequency as 64Mhz     */
                                        
                                        

void SystemCoreSetClock(mcu_clock_type_t clock_type)
{
    if (clock_type >= CLK_TYPE_NUM)
        return;        // input parameter is out of range

    system_pmu_deinit();
    if ((AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL) != clock_type)
    {
        uint32_t temp = AON->PWR_RET01 & (~AON_PWR_REG01_SYS_CLK_SEL);
        //When a 16M or 64M clock is switched to another clock, it needs to be switched to 32M first.
        AON->PWR_RET01 = (temp | (CPLL_T32M_CLK<<AON_PWR_REG01_SYS_CLK_SEL_Pos));
        __asm ("nop;nop;nop;nop;");
        AON->PWR_RET01 = (temp | (clock_type<<AON_PWR_REG01_SYS_CLK_SEL_Pos));
        
        //Sync qspi clock with system frequency
        temp = AON->PWR_RET01 & (~AON_PWR_REG01_XF_SCK_CLK_SEL_Msk);
        AON->PWR_RET01 = (temp | (mcu_clk_2_qspi_clk[clock_type]<<AON_PWR_REG01_XF_SCK_CLK_SEL_Pos));
    }

	SystemCoreClock = systemClock[clock_type];   
	system_pmu_init(clock_type);

    return;
}

void SystemCoreGetClock(mcu_clock_type_t *clock_type)
{
    *clock_type = (mcu_clock_type_t)(AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL);
}

void SystemCoreUpdateClock(void)
{
    SystemCoreClock  = systemClock[AON->PWR_RET01 & AON_PWR_REG01_SYS_CLK_SEL];
}

static inline uint32_t get_wakeup_flag(void)
{
    return (AON->SOFTWARE_2 & (1 << SOFTWARE_REG_WAKEUP_FLAG_POS));
}

#ifdef CFG_PATCH_BY_FPB
#define PRIORITY_GROUP_NUMBER      (3)

#endif
void set_msp()
{
    #ifndef DRIVER_TEST
    #ifdef APP_CODE_RUN_ADDR
    __DMB();
     __set_MSP(REG_PL_RD(APP_CODE_RUN_ADDR));
    __DSB();
    #endif
    #endif
}

void system_priority_init(void)
{
    uint32_t i, group;

    if (7848 == READ_VERSION_ADDR())
    {
        group = NVIC_GetPriorityGrouping();
        for(i = 0; i < MAX_NUMS_IRQn; i++)
        {
            NVIC_SetPriority((IRQn_Type)i, NVIC_EncodePriority(group, 8, 0));
        }

        NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(group, 0, 0));
        NVIC_SetPriority(BLE_IRQn, NVIC_EncodePriority(group, 2, 0));
        NVIC_SetPriority(BLESLP_IRQn, NVIC_EncodePriority(group, 2, 0));
        NVIC_SetPriority(DMA_IRQn, NVIC_EncodePriority(group, 6, 0));
        NVIC_SetPriority(UART0_IRQn, NVIC_EncodePriority(group, 10, 0));
        NVIC_SetPriority(UART1_IRQn, NVIC_EncodePriority(group, 10, 0));
        NVIC_SetPriority(TIMER0_IRQn, NVIC_EncodePriority(group, 12, 0));
        NVIC_SetPriority(TIMER1_IRQn, NVIC_EncodePriority(group, 12, 0));
        NVIC_SetPriority(DUAL_TIMER_IRQn, NVIC_EncodePriority(group, 12, 0));
        NVIC_SetPriority(SLPTIMER_IRQn, NVIC_EncodePriority(group, 12, 0));
        NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(group, 15, 15));
        NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(group, 15, 15));
        NVIC_SetPriority(BLE_SDK_IRQn, NVIC_EncodePriority(group, 15, 15));
    }
}

void SystemInit(void)
{
    system_priority_init();

    #if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                    (3UL << 11*2)  );               /* set CP11 Full Access */
    #endif

    #ifdef UNALIGNED_SUPPORT_DISABLE
    SCB->CCR |= SCB_CCR_UNALIGN_TRP_Msk;
    #endif

    if(WARM_BOOT == get_wakeup_flag())
    {
        platform_rc_calibration();
    }
    else
    {
        sdk_init();
    }
    return;
}

void system_platform_init(void)
{
#ifdef ROM_RUN_IN_FLASH
    extern void rom_init(void);
    rom_init();
#endif
	
		set_patch_flag(MANDATORY_PATCH);

    platform_init();
 
    return;
}

void main_init(void)
{
    uint32_t boot_flag = get_wakeup_flag();
    if(COLD_BOOT == boot_flag)
    {
        extern void __main(void);
        __main();
    }
    else
    {
        warm_boot();
        while (1);
    }
    // Never execute here
}


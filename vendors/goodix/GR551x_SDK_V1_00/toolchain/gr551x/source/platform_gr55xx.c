/**
 *******************************************************************************
 *
 * @file   platform_gr55xx.c
 *
 * @brief  Platform Initialization Routines.
 *
 *******************************************************************************
 
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
 *******************************************************************************
 */

/*
 * INCLUDE FILES
 *******************************************************************************
 */
#include "gr55xx.h"
#include "gr55xx_sys.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "platform_sdk.h"

extern uint32_t nvds_get_start_addr(void);

static uint32_t SVC_Table[4] __attribute__((section("SVC_TABLE")));

/*****************************************************************
 Default resistor configurations for SK board and EV Board.
 Developers will have to configure this table based on the hardware
 configuration.
 ******************************************************************/
#ifdef GR5515_SK
io_table_t io_table =
{
   .gpio = GPIO_PIN(2)  | GPIO_PIN(12) | GPIO_PIN(13) |\
           GPIO_PIN(14) | GPIO_PIN(15) |\
           GPIO_PIN(28) | GPIO_PIN(29),
   .aon_gpio = 0,
   .msio = 0,
};
#else
io_table_t io_table =
{
   .gpio = GPIO_PIN(5)  | GPIO_PIN(8)  | GPIO_PIN(9)  | GPIO_PIN(10) |\
           GPIO_PIN(11) | GPIO_PIN(12) | GPIO_PIN(13) | GPIO_PIN(14) |\
           GPIO_PIN(15) | GPIO_PIN(17) | GPIO_PIN(27) | GPIO_PIN(28) |\
           GPIO_PIN(29),
   .aon_gpio = 0,
   .msio = 0,
};
#endif

static void nvds_setup(void)
{
#ifdef NVDS_START_ADDR
    uint8_t err_code = nvds_init(NVDS_START_ADDR, NVDS_NUM_SECTOR);
#else
    uint8_t err_code = nvds_init(0, NVDS_NUM_SECTOR);
#endif

    switch(err_code)
    {
        case NVDS_FAIL:
        case NVDS_STORAGE_ACCESS_FAILED:
            {
                uint32_t start_addr  = nvds_get_start_addr();
                uint32_t sector_size = hal_flash_sector_size();
                if (hal_flash_erase(start_addr, NVDS_NUM_SECTOR * sector_size))
                {
                    err_code = nvds_init(start_addr, NVDS_NUM_SECTOR);
                    if (NVDS_SUCCESS == err_code)
                    {
                        break;
                    }
                }
                /* Flash fault, cannot startup.
                 * TODO: Output log via UART or Dump an error code to flash. */
                while(1);
            }
        case NVDS_SUCCESS:
            break;
        default:
            /* Illegal NVDS Parameters.
             * Please check the start address and number of sectors. */
            while(1);
    }
}

void platform_init(void)
{
    /* set sram power state. */
    system_lp_mem_mode_set(MEM_POWER_AUTO_MODE);
    
    if (!hal_flash_init())
    {
        /* Flash fault, cannot startup.
         * TODO: Output log via UART or Dump an error code to flash. */
        while(1);
    }

    /* nvds module init process. */
    nvds_setup();

    /* IO leakage protect configuration. */
    system_io_leakage_protect(&io_table);
    
    /* To choose the System clock source and set the accuracy of OSC. */
    platform_clock_init((mcu_clock_type_t)SYSTEM_CLOCK, RTC_OSC_CLK, CFG_LF_ACCURACY_PPM, 0);
  
    /* Register the SVC Table. */
    svc_table_register(SVC_Table);
  
    /* platform init process. */ 
    platform_sdk_init();
    
    #ifndef DRIVER_TEST
    /* Enable auto pmu calibration function. */
    system_pmu_calibration_start(10000);
    #endif
    
    SystemCoreSetClock((mcu_clock_type_t)SYSTEM_CLOCK);
    
    return ;
}

extern void system_platform_init(void);
#if defined ( __GNUC__ )
void __main(void)
{
    __asm("ldr    r1, =__etext\n");
    __asm("ldr    r2, =__data_start__\n");
    __asm("ldr    r3, =__data_end__\n");
    __asm(".L_loop1:\n");
    __asm("cmp    r2, r3\n");
    __asm("ittt   lt\n");
    __asm("ldrlt  r0, [r1], #4\n");
    __asm("strlt  r0, [r2], #4\n");
    __asm("blt    .L_loop1\n");
    __asm("ldr    r1, =__bss_start__\n");
    __asm("ldr    r2, =__bss_end__\n");
    __asm("movs   r0, 0\n");
    __asm(".L_loop3:\n");
    __asm("cmp    r1, r2\n");
    __asm("itt    lt\n");
    __asm("strlt  r0, [r1], #4\n");
    __asm("blt    .L_loop3\n");
    system_platform_init();
    main();
}
#endif

#if defined ( __CC_ARM )
extern void $Super$$main(void);
void $Sub$$main(void)
{
    system_platform_init();
    $Super$$main();
}
#endif


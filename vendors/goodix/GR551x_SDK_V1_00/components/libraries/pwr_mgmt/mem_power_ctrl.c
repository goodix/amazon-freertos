/**
 *****************************************************************************************
 *
 * @file mem_power_ctrl.c
 *
 * @brief  User Periph Init Function Implementation.
 *
 *****************************************************************************************
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
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "user_periph_setup.h"
#include "gr55xx_sys.h"
#include "dbg_printf.h"
#include "hal_flash.h"
#include "custom_config.h"
#include "boards.h"
#include "gr55xx_delay.h"
#include "mem_power_ctrl.h"
#include "pwr_mgmt.h"

uint32_t mem_region_tab[] =
{
    0x30000000,
    0x30002000,
    0x30004000,
    0x30006000,
    0x30008000,
    0x30010000,
    0x30018000,
    0x30020000,
    0x30028000,
    0x30030000,
    0x30038000,
    0x30040000,
};

uint32_t addr_2_block(uint32_t addr,uint32_t len)
{
    int start = 0;
    int end = 0;
    uint32_t ret_mem_ctrl = 0;
    
    for(start = sizeof(mem_region_tab)/sizeof(mem_region_tab[0])-1; start >= 0 ; start--)
    {
            if(addr >= mem_region_tab[start])
                    break;
    }
    for(end = sizeof(mem_region_tab)/sizeof(mem_region_tab[0])-1; end >= 0; end--)
    {
            if(addr +len > mem_region_tab[end])
                    break;
    }
    for(uint8_t i = start; i <= end; i++)
    {
            ret_mem_ctrl |= 0X3<<(i*2);
    }

    return ret_mem_ctrl ;
}


uint32_t scatter_info_mem_ctrl()
{
    sactter_copy_info_t sactter_copy_info = {0};
    uint32_t mem_region_ctrl_flag = LL_PWR_MEM_SRAM_8K_0|LL_PWR_MEM_SRAM_8K_1|LL_PWR_MEM_HOPPING_TABLE|LL_PWR_MEM_PACKET_MEM|AON_MEM_PWR_SLP_PD_MCU_KEYRAM;

    uint32_t sp_start = (uint32_t )&Image$$ARM_LIB_STACKHEAP$$ZI$$Base;
    uint32_t sp_end = (uint32_t )&Image$$ARM_LIB_STACKHEAP$$ZI$$Limit;
    uint32_t bin_addr_start = (uint32_t )&Image$$ER_FLASH$$Base;
    uint32_t bin_addr_end = (uint32_t )&Image$$ER_FLASH$$Limit;

    mem_region_ctrl_flag |= addr_2_block(sp_start,sp_end- sp_start);

    if(bin_addr_start>= 0x30000000)
    {
        mem_region_ctrl_flag |= addr_2_block(bin_addr_start,bin_addr_end - bin_addr_start);
    }
    for(int i=0;i<((uint32_t)&(Region$$Table$$Limit)-((uint32_t)&(Region$$Table$$Base)))/(sizeof(sactter_copy_info_t));i++)
    {
        memcpy((void *)&sactter_copy_info,(void *)((uint32_t)&Region$$Table$$Base+i*sizeof(sactter_copy_info_t)),sizeof(sactter_copy_info_t));

        if(sactter_copy_info.ram_addr >= 0x30000000)
        {
            mem_region_ctrl_flag |= addr_2_block(sactter_copy_info.ram_addr,sactter_copy_info.len);
        }
    }
    return mem_region_ctrl_flag;
}
extern sleep_env_t g_sleep_env;

void scatter_info_mem_ctrl_init()
{
      g_sleep_env.sleep_mem_region = scatter_info_mem_ctrl();
      g_sleep_env.wkup_mem_region = scatter_info_mem_ctrl();
}

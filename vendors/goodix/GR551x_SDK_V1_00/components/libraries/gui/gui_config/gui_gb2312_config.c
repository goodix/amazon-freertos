/**
 *****************************************************************************************
 *
 * @file gui_lcm_config.c
 *
 * @brief Users should implement the timer-related interface themselves
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gui_config.h"

#if FONT_GB2312_EN==1
#include "hal_flash.h"

/*
 * DEFINES
 *****************************************************************************************
 */
//You can convert bin files in the GUI config folder into hex files that specify flash addresses 
#define GB2312_FLASH_START_ADDR  (0x0103d000)           /**< Flash address of gb2312 font code. */
#define ASCII_FLASH_ADDR         (0x0103d000 + 0x3FE40) /**< Flash address of ascii font code. */

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void gui_read_gb2312(uint32_t offset, uint8_t* read_buf)
{
    hal_flash_read(GB2312_FLASH_START_ADDR + offset, read_buf, 32);
}


void gui_read_ascii(uint32_t offset, uint8_t* read_buf)
{
    hal_flash_read(ASCII_FLASH_ADDR + offset, read_buf, 16);
}

#endif



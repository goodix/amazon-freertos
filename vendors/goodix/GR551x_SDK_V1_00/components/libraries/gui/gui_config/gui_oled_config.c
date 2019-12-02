/**
 *****************************************************************************************
 *
 * @file gui_oled_config.c
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
#include "ssd1316.h"
#include "gui_animation.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void  gui_init(void)
{
    oled_init();
    gui_animation_init(s_oled_gram, s_oled_buffer);
}


void  gui_fill(T_COLOR dat)
{
    oled_fill(dat);
}


void  gui_point(uint16_t x, uint8_t y, T_COLOR color)
{
    oled_draw_point(x, y, color);
}


T_COLOR gui_read_point(uint16_t x, uint8_t y)
{
    return oled_read_point(x, y);
}


void gui_refresh(void)
{
    oled_set_memory(MEM_GRAM);
    oled_refresh();
}



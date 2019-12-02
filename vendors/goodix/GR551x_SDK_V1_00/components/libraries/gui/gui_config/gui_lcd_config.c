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
#include "st7735.h"
#include "gui_animation.h"

static bool gui_refresh_flag = false;
/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
 
void gui_init(void)
{
    lcd_init();
    gui_animation_init(g_lcd_gram, g_lcd_buffer);
}

void gui_fill_mem(T_COLOR color)
{
    lcd_fill_mem(color);
}

void gui_rectangle_fill_mem(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, T_COLOR color)
{
    lcd_rectangle_fill_mem(x0, y0, x1, y1, color);
}

void gui_point(uint16_t x, uint16_t y, T_COLOR color)
{
   lcd_draw_point(x, y, color);
}

T_COLOR gui_read_point(uint16_t x, uint16_t y)
{
    return lcd_read_point(x, y);
}

void gui_refresh(void)
{
    gui_refresh_flag = true;
}

void gui_rectangle_refresh(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    lcd_rectangle_refresh(x0, y0, x1, y1);
}

void gui_set_refresh_mem(bool gram_set)
{
    lcd_set_memory(gram_set);
}

void gui_refresh_schedule(void)
{
    if(gui_refresh_flag)
    {
        lcd_refresh();
        gui_refresh_flag = false;
    }
}


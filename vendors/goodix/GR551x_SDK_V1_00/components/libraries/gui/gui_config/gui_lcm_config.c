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
#include "uc1701.h"
#include "gui_animation.h"


/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void  gui_init(void)
{
    lcm_init();
    gui_animation_init(s_lcm_gram, s_lcm_buffer);
}


void  gui_fill(T_COLOR dat)
{
    lcm_fill(dat);
}


void  gui_point(uint16_t x, uint16_t y, T_COLOR color)
{
    lcm_draw_point(x, y, color);
}


T_COLOR gui_read_point(uint16_t x, uint16_t y)
{
    return lcm_read_point(x, y);
}


void gui_refresh(void)
{
    lcm_set_memory(MEM_GRAM);
    lcm_refresh();
}



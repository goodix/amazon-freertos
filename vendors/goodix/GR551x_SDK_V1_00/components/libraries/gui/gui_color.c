/**
 ****************************************************************************************
 *
 * @file gui_color.c
 *
 * @brief Function of set color
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include  "gui_color.h"
#include  "gui_config.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t const  DCB2HEX_TAB[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
static T_COLOR  s_disp_color;/**< Display color. */
static T_COLOR  s_back_color;/**< Back color. */


uint8_t gui_dcb_to_hex(uint8_t dcb)
{
    return DCB2HEX_TAB[dcb];
}

void gui_set_color(T_COLOR disp_color, T_COLOR back_color)
{  
    s_disp_color = disp_color;
    s_back_color = back_color; 
}

T_COLOR gui_get_back_color(void)
{  
    return s_back_color;
}

T_COLOR gui_get_disp_color(void)
{  
    return s_disp_color;
}


void  gui_exchange_color(void)
{  
    T_COLOR  bakc;
    bakc = s_disp_color;
    s_disp_color = s_back_color;
    s_back_color = bakc;
}


void gui_point_color(uint8_t x, uint8_t y, uint8_t font_dat, uint8_t num)
{
    uint8_t j;
    uint8_t x_coord = x;
    T_COLOR   bakc;
    for(j=0; j<num; j++)
    {  
        if( (font_dat&DCB2HEX_TAB[j])==0 )
            bakc = s_back_color;
        else  
            bakc = s_disp_color;
        gui_point(x_coord, y, bakc);       
        x_coord++;
    }
}

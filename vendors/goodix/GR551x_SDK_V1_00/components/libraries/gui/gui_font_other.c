/**
 *****************************************************************************************
 *
 * @file gui_font_other.c
 *
 * @brief Function of show other modulus font
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
#include "gui_font_other.h"

#if FONT_OTHER_EN==1

#include "gui_color.h"
#include "gui_basic.h"


/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
uint8_t gui_put_char16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t index)
{
    uint8_t i,j;
    
    if( x>(GUI_DISPLAY_X_MAX-16) ) return(0);
    if( y>(GUI_DISPLAY_Y_MAX-32) ) return(0);
    for(i=0; i<16; i++)
    {
        for(j=0; j<2; j++)
        {
            gui_point_color(x+j*8, y+i, code16_32[(index*4)+(j*2)][i], 8);
            gui_point_color(x+j*8, y+16+i, code16_32[(index*4)+(j*2+1)][i], 8);
        }
    }
    return (1);
}

void gui_put_string16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t num)
{
    uint16_t i;
    
    for(i=0; i<num; i++)
    {
        if(gui_put_char16_32(x, y, code16_32, i) == 0)
        {
            break;
        }
        x += 16;
    }
}

uint8_t gui_put_char32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t index)
{
    uint8_t i,j;
    
    if( x>(GUI_DISPLAY_X_MAX-32) ) return(0);
    if( y>(GUI_DISPLAY_Y_MAX-64) ) return(0);
    for(i=0; i<16; i++)
    {
        for(j=0; j<4; j++)
        {
            gui_point_color(x+j*8, y+i, code32_64[(index*16)+(j*4)][i], 8);
            gui_point_color(x+j*8, y+16+i, code32_64[(index*16)+(j*4+1)][i], 8);
            gui_point_color(x+j*8, y+32+i, code32_64[(index*16)+(j*4+2)][i], 8);
            gui_point_color(x+j*8, y+48+i, code32_64[(index*16)+(j*4+3)][i], 8);  
        }
    }
    return (1);
}

void gui_put_string32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t num)
{
    uint16_t i;
    
    for(i=0; i<num; i++)
    {
        if(gui_put_char32_64(x, y, code32_64, i) == 0)
        {
            break;
        }
        x += 32;
    }
}
#endif


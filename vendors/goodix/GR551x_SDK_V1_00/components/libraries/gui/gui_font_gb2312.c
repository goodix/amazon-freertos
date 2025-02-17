/**
 ****************************************************************************************
 *
 * @file gui_font_gb2312.c
 *
 * @brief Function of show gb2312 string
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
#include "gui_font_gb2312.h"
#include "gui_color.h"

#if FONT_GB2312_EN==1
/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Show one gb2312 char.
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] str1: GB2312 code high order.
 * @param[in] str2: GB2312 code low order.
 *
 * @return 0x01--success, 0x00--fail(the coordinate value is out of range)
 *****************************************************************************************
 */    
static uint8_t gui_put_one_gb2312(uint16_t x, uint16_t y, char str1, char str2)
{  
    uint8_t   i;
    uint32_t  offset;
    uint8_t gb2312_buf[32];
    uint16_t x_coord = x;
    uint16_t y_coord = y;
    
    if( x_coord > (GUI_DISPLAY_X_MAX-16) )
        return(0);
    if( y_coord > (GUI_DISPLAY_Y_MAX-16) ) 
        return(0);
    offset = (((str1-0xa1)*94) + (str2 - 0xa1)) * 32;
    gui_read_gb2312(offset, gb2312_buf);
   for(i=0; i<16; i++)
   {  
      gui_point_color(x_coord, y_coord, gb2312_buf[i], 8);
      gui_point_color(x_coord+8, y_coord, gb2312_buf[16+i], 8);
      y_coord++;                                                        
   }
   
   return(1);
}

/**
 *****************************************************************************************
* @brief Display one 8*16 ascii.
* @note  Display value between 0x20 to 0x7f, others show ' '.
*
* @return 0x01--success, 0x00--fail(the coordinate value is out of range)
*****************************************************************************************
 */ 
static uint8_t gui_put_ascii_char(uint16_t x, uint16_t y, char show_char)
{
    uint8_t   i;
    uint32_t  offset;
    uint8_t ascii_buf[16];
    uint16_t x_coord = x;
    uint16_t y_coord = y;

    if( x_coord > (GUI_DISPLAY_X_MAX-8) )
        return(0);
    if( y_coord > (GUI_DISPLAY_Y_MAX-16) ) 
        return(0);
    offset = (show_char-0x20)*16;
    gui_read_ascii(offset, ascii_buf);
    for(i=0; i<16; i++)
    {  
        gui_point_color(x_coord, y_coord, ascii_buf[i], 8);
        y_coord++;                                                            
    }
     return (1);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void gui_put_string(uint16_t x, uint16_t y, char *p_str)
{
    uint16_t i=0;
    uint16_t x_coord = x;
    uint16_t y_coord = y;
    while(p_str[i] > 0x00)
    {
        if((p_str[i]>=0x20) &&(p_str[i]<=0x7e))//assic 
        {                            
            if(gui_put_ascii_char(x_coord, y_coord, p_str[i]))
            {
                x_coord += 8; 
                if( x_coord > (GUI_DISPLAY_X_MAX-8) )
                {
                    x_coord = 0;
                    y_coord += 16;
                }
            }
               
            i++; 
        }
        else
        {
            if(gui_put_one_gb2312(x_coord, y_coord, p_str[i], p_str[i+1]))
            {
                i+=2;
                x_coord += 16;
                if( x_coord > (GUI_DISPLAY_X_MAX-16) )
                {
                    x_coord = 0;
                    y_coord += 16;
                }
            }
            else
            {
                i++;
            }
        }
    }
}

#endif

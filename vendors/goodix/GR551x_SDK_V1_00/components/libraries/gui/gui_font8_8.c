/**
 ****************************************************************************************
 *
 * @file gui_font8_8.c
 *
 * @brief Function of show 8*8 ascii
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
#include "gui_font8_8.h"
#include "gui_font_macro.h"
#include "gui_color.h"
#include "gui_basic.h"

#if  FONT8x8_EN==1

/**@brief 8*8 ascii define. */
const uint8_t  s_ascii_font8_8[][8] = {
/* space */
  {
   ________,
   ________,
   ________,
   ________,
   ________,
   ________,
   ________,
   ________} 
    
/*  !  */   
 ,{
   ___XX___,
   __XXXX__,
   __XXXX__,
   ___XX___,
   ___XX___,
   ________,
   ___XX___,
   ________}  

/*  "  */
 ,{
   _XX__XX_,
   _XX__XX_,
   __X__X__,
   ________,
   ________,
   ________,
   ________,
   ________}  
/*  #  */
 ,{
   _XX_XX__,
   _XX_XX__,
   XXXXXXX_,
   _XX_XX__,
   XXXXXXX_,
   _XX_XX__,
   _XX_XX__,
   ________}  

/*  $  */
 ,{
   ___XX___,
   __XXXXX_,
   _XX_____,
   __XXXX__,
   _____XX_,
   _XXXXX__,
   ___XX___,
   ________}  

/*  %  */
 ,{
   ________,
   XX___XX_,
   XX__XX__,
   ___XX___,
   __XX____,
   _XX__XX_,
   XX___XX_,
   ________}  

/*  &  */
 ,{
   __XXX___,
   _XX_XX__,
   __XXX___,
   _XXX_XX_,
   XX_XXX__,
   XX__XX__,
   _XXX_XX_,
   ________}  

/*  '  */
 ,{
   ___XX___,
   ___XX___,
   __XX____,
   ________,
   ________,
   ________,
   ________,
   ________}  

/*  (  */
 ,{
   ____XX__,
   ___XX___,
   __XX____,
   __XX____,
   __XX____,
   ___XX___,
   ____XX__,
   ________}  

/*  )  */
 ,{
   __XX____,
   ___XX___,
   ____XX__,
   ____XX__,
   ____XX__,
   ___XX___,
   __XX____,
   ________}  

/*  *  */
 ,{
   ________,
   _XX__XX_,
   __XXXX__,
   XXXXXXXX,
   __XXXX__,
   _XX__XX_,
   ________,
   ________}  

/*  +  */
 ,{
   ________,
   ___XX___,
   ___XX___,
   _XXXXXX_,
   ___XX___,
   ___XX___,
   ________,
   ________}  

/*  ,  */
 ,{
   ________,
   ________,
   ________,
   ________,
   ________,
   ___XX___,
   ___XX___,
   __XX____}  

/*  -  */
 ,{
   ________,
   ________,
   ________,
   _XXXXXX_,
   ________,
   ________,
   ________,
   ________}  

/*  .  */
 ,{
   ________,
   ________,
   ________,
   ________,
   ________,
   ___XX___,
   ___XX___,
   ________}  

/*  /  */
 ,{
   _____XX_,
   ____XX__,
   ___XX___,
   __XX____,
   _XX_____,
   XX______,
   X_______,
   ________}  

/*  0  */
 ,{
   __XXX___,
   _XX_XX__,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   _XX_XX__,
   __XXX___,
   ________}  

/*  1  */
 ,{
   ___XX___,
   __XXX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   _XXXXXX_,
   ________}  

/*  2  */
 ,{
   _XXXXX__,
   XX___XX_,
   _____XX_,
   ___XXX__,
   __XX____,
   _XX__XX_,
   XXXXXXX_,
   ________}  

/*  3  */
 ,{
   _XXXXX__,
   XX___XX_,
   _____XX_,
   __XXXX__,
   _____XX_,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  4  */
 ,{
   ___XXX__,
   __XXXX__,
   _XX_XX__,
   XX__XX__,
   XXXXXXX_,
   ____XX__,
   ___XXXX_,
   ________}  

/*  5  */
 ,{
   XXXXXXX_,
   XX______,
   XX______,
   XXXXXX__,
   _____XX_,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  6  */
 ,{
   __XXX___,
   _XX_____,
   XX______,
   XXXXXX__,
   XX___XX_,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  7  */
 ,{
   XXXXXXX_,
   XX___XX_,
   ____XX__,
   ___XX___,
   __XX____,
   __XX____,
   __XX____,
   ________}  

/*  8  */
 ,{
   _XXXXX__,
   XX___XX_,
   XX___XX_,
   _XXXXX__,
   XX___XX_,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  9  */
 ,{
   _XXXXX__,
   XX___XX_,
   XX___XX_,
   _XXXXXX_,
   _____XX_,
   ____XX__,
   _XXXX___,
   ________}  

/*  :  */
 ,{
   ________,
   ___XX___,
   ___XX___,
   ________,
   ________,
   ___XX___,
   ___XX___,
   ________}  

/*  ;  */
 ,{
   ________,
   ___XX___,
   ___XX___,
   ________,
   ________,
   ___XX___,
   ___XX___,
   __XX____}  

/*  <  */
 ,{
   _____XX_,
   ____XX__,
   ___XX___,
   __XX____,
   ___XX___,
   ____XX__,
   _____XX_,
   ________}  

/*  =  */
 ,{
   ________,
   ________,
   _XXXXXX_,
   ________,
   ________,
   _XXXXXX_,
   ________,
   ________}  

/*  >  */
 ,{
   _XX_____,
   __XX____,
   ___XX___,
   ____XX__,
   ___XX___,
   __XX____,
   _XX_____,
   ________}  

/*  ?  */
 ,{
   _XXXXX__,
   XX___XX_,
   ____XX__,
   ___XX___,
   ___XX___,
   ________,
   ___XX___,
   ________}  

/*  @  */
 ,{
   _XXXXX__,
   XX___XX_,
   XX_XXXX_,
   XX_XXXX_,
   XX_XXXX_,
   XX______,
   _XXXX___,
   ________}  

/*  A  */
 ,{
   __XXX___,
   _XX_XX__,
   XX___XX_,
   XXXXXXX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   ________}  

/*  B  */
 ,{
   XXXXXX__,
   _XX__XX_,
   _XX__XX_,
   _XXXXX__,
   _XX__XX_,
   _XX__XX_,
   XXXXXX__,
   ________}  

/*  C  */
 ,{
   __XXXX__,
   _XX__XX_,
   XX______,
   XX______,
   XX______,
   _XX__XX_,
   __XXXX__,
   ________}  

/*  D  */
 ,{
   XXXXX___,
   _XX_XX__,
   _XX__XX_,
   _XX__XX_,
   _XX__XX_,
   _XX_XX__,
   XXXXX___,
   ________}  

/*  E  */
 ,{
   XXXXXXX_,
   _XX___X_,
   _XX_X___,
   _XXXX___,
   _XX_X___,
   _XX___X_,
   XXXXXXX_,
   ________}  

/*  F  */
 ,{
   XXXXXXX_,
   _XX___X_,
   _XX_X___,
   _XXXX___,
   _XX_X___,
   _XX_____,
   XXXX____,
   ________}  

/*  G  */
 ,{
   __XXXX__,
   _XX__XX_,
   XX______,
   XX______,
   XX__XXX_,
   _XX__XX_,
   __XXX_X_,
   ________}  

/*  H  */
 ,{
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XXXXXXX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   ________}  

/*  I  */
 ,{
   __XXXX__,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   __XXXX__,
   ________}  

/*  J  */
 ,{
   ___XXXX_,
   ____XX__,
   ____XX__,
   ____XX__,
   XX__XX__,
   XX__XX__,
   _XXXX___,
   ________}  

/*  K  */
 ,{
   XXX__XX_,
   _XX__XX_,
   _XX_XX__,
   _XXXX___,
   _XX_XX__,
   _XX__XX_,
   XXX__XX_,
   ________}  

/*  L  */
 ,{
   XXXX____,
   _XX_____,
   _XX_____,
   _XX_____,
   _XX___X_,
   _XX__XX_,
   XXXXXXX_,
   ________}  

/*  M  */
 ,{
   XX___XX_,
   XXX_XXX_,
   XXXXXXX_,
   XXXXXXX_,
   XX_X_XX_,
   XX___XX_,
   XX___XX_,
   ________}  

/*  N  */
 ,{
   XX___XX_,
   XXX__XX_,
   XXXX_XX_,
   XX_XXXX_,
   XX__XXX_,
   XX___XX_,
   XX___XX_,
   ________}  

/*  O  */
 ,{
   _XXXXX__,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  P  */
 ,{
   XXXXXX__,
   _XX__XX_,
   _XX__XX_,
   _XXXXX__,
   _XX_____,
   _XX_____,
   XXXX____,
   ________}  

/*  Q  */
 ,{
   _XXXXX__,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX__XXX_,
   _XXXXX__,
   ____XXX_}  

/*  R  */
 ,{
   XXXXXX__,
   _XX__XX_,
   _XX__XX_,
   _XXXXX__,
   _XX_XX__,
   _XX__XX_,
   XXX__XX_,
   ________}  

/*  S  */
 ,{
   __XXXX__,
   _XX__XX_,
   __XX____,
   ___XX___,
   ____XX__,
   _XX__XX_,
   __XXXX__,
   ________}  

/*  T  */
 ,{
   _XXXXXX_,
   _XXXXXX_,
   _X_XX_X_,
   ___XX___,
   ___XX___,
   ___XX___,
   __XXXX__,
   ________}  

/*  U  */
 ,{
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  V  */
 ,{
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   _XX_XX__,
   __XXX___,
   ________}  

/*  W  */
 ,{
   XX___XX_,
   XX___XX_,
   XX___XX_,
   XX_X_XX_,
   XX_X_XX_,
   XXXXXXX_,
   _XX_XX__,
   ________}  

/*  X  */
 ,{
   XX___XX_,
   XX___XX_,
   _XX_XX__,
   __XXX___,
   _XX_XX__,
   XX___XX_,
   XX___XX_,
   ________}  

/*  Y  */
 ,{
   _XX__XX_,
   _XX__XX_,
   _XX__XX_,
   __XXXX__,
   ___XX___,
   ___XX___,
   __XXXX__,
   ________}  

/*  Z  */
 ,{
   XXXXXXX_,
   XX___XX_,
   X___XX__,
   ___XX___,
   __XX__X_,
   _XX__XX_,
   XXXXXXX_,
   ________}  

/*  [  */
 ,{
   __XXXX__,
   __XX____,
   __XX____,
   __XX____,
   __XX____,
   __XX____,
   __XXXX__,
   ________}  

/*  \  */
 ,{
   XX______,
   _XX_____,
   __XX____,
   ___XX___,
   ____XX__,
   _____XX_,
   ______X_,
   ________}  

/*  ]  */
 ,{
   __XXXX__,
   ____XX__,
   ____XX__,
   ____XX__,
   ____XX__,
   ____XX__,
   __XXXX__,
   ________}  

/*  ^  */
 ,{
   ___X____,
   __XXX___,
   _XX_XX__,
   XX___XX_,
   ________,
   ________,
   ________,
   ________}  

/*  _  */
 ,{
   ________,
   ________,
   ________,
   ________,
   ________,
   ________,
   ________,
   XXXXXXXX}  

/*  `  */
 ,{
   __XX____,
   ___XX___,
   ____XX__,
   ________,
   ________,
   ________,
   ________,
   ________}  

/*  a  */
 ,{
   ________,
   ________,
   _XXXX___,
   ____XX__,
   _XXXXX__,
   XX__XX__,
   _XXX_XX_,
   ________}  

/*  b  */
 ,{
   XXX_____,
   _XX_____,
   _XXXXX__,
   _XX__XX_,
   _XX__XX_,
   _XX__XX_,
   XX_XXX__,
   ________}  

/*  c  */
 ,{
   ________,
   ________,
   _XXXXX__,
   XX___XX_,
   XX______,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  d  */
 ,{
   ___XXX__,
   ____XX__,
   _XXXXX__,
   XX__XX__,
   XX__XX__,
   XX__XX__,
   _XXX_XX_,
   ________}  

/*  e  */
 ,{
   ________,
   ________,
   _XXXXX__,
   XX___XX_,
   XXXXXXX_,
   XX______,
   _XXXXX__,
   ________}  

/*  f  */
 ,{
   __XXXX__,
   _XX__XX_,
   _XX_____,
   XXXXX___,
   _XX_____,
   _XX_____,
   XXXX____,
   ________}  

/*  g  */
 ,{
   ________,
   ________,
   _XXX_XX_,
   XX__XX__,
   XX__XX__,
   _XXXXX__,
   ____XX__,
   XXXXX___}  

/*  h  */
 ,{
   XXX_____,
   _XX_____,
   _XX_XX__,
   _XXX_XX_,
   _XX__XX_,
   _XX__XX_,
   XXX__XX_,
   ________}  

/*  i  */
 ,{
   ___XX___,
   ________,
   __XXX___,
   ___XX___,
   ___XX___,
   ___XX___,
   __XXXX__,
   ________}  

/*  j  */
 ,{
   _____XX_,
   ________,
   _____XX_,
   _____XX_,
   _____XX_,
   _XX__XX_,
   _XX__XX_,
   __XXXX__}  

/*  k  */
 ,{
   XXX_____,
   _XX_____,
   _XX__XX_,
   _XX_XX__,
   _XXXX___,
   _XX_XX__,
   XXX__XX_,
   ________}  

/*  l  */
 ,{
   __XXX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   __XXXX__,
   ________}  

/*  m  */
 ,{
   ________,
   ________,
   XXX_XX__,
   XXXXXXX_,
   XX_X_XX_,
   XX_X_XX_,
   XX_X_XX_,
   ________}  

/*  n  */
 ,{
   ________,
   ________,
   XX_XXX__,
   _XX__XX_,
   _XX__XX_,
   _XX__XX_,
   _XX__XX_,
   ________}  

/*  o  */
 ,{
   ________,
   ________,
   _XXXXX__,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   _XXXXX__,
   ________}  

/*  p  */
 ,{
   ________,
   ________,
   XX_XXX__,
   _XX__XX_,
   _XX__XX_,
   _XXXXX__,
   _XX_____,
   XXXX____}  

/*  q  */
 ,{
   ________,
   ________,
   _XXX_XX_,
   XX__XX__,
   XX__XX__,
   _XXXXX__,
   ____XX__,
   ___XXXX_}  

/*  r  */
 ,{
   ________,
   ________,
   XX_XXX__,
   _XXX_XX_,
   _XX_____,
   _XX_____,
   XXXX____,
   ________}  

/*  s  */
 ,{
   ________,
   ________,
   _XXXXXX_,
   XX______,
   _XXXXX__,
   _____XX_,
   XXXXXX__,
   ________}  

/*  t  */
 ,{
   __XX____,
   __XX____,
   XXXXXX__,
   __XX____,
   __XX____,
   __XX_XX_,
   ___XXX__,
   ________}  

/*  u  */
 ,{
   ________,
   ________,
   XX__XX__,
   XX__XX__,
   XX__XX__,
   XX__XX__,
   _XXX_XX_,
   ________}  

/*  v  */
 ,{
   ________,
   ________,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   _XX_XX__,
   __XXX___,
   ________}  

/*  w  */
 ,{
   ________,
   ________,
   XX___XX_,
   XX_X_XX_,
   XX_X_XX_,
   XXXXXXX_,
   _XX_XX__,
   ________}  

/*  x  */
 ,{
   ________,
   ________,
   XX___XX_,
   _XX_XX__,
   __XXX___,
   _XX_XX__,
   XX___XX_,
   ________}  

/*  y  */
 ,{
   ________,
   ________,
   XX___XX_,
   XX___XX_,
   XX___XX_,
   _XXXXXX_,
   _____XX_,
   XXXXXX__}  

/*  z  */
 ,{
   ________,
   ________,
   _XXXXXX_,
   _X__XX__,
   ___XX___,
   __XX__X_,
   _XXXXXX_,
   ________} 

/*  {  */
 ,{
   ____XXX_,
   ___XX___,
   ___XX___,
   _XXX____,
   ___XX___,
   ___XX___,
   ____XXX_,
   ________}  

/*  |  */
 ,{
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ___XX___,
   ________}  
   
/*  }  */
 ,{
   _XXX____,
   ___XX___,
   ___XX___,
   ____XXX_,
   ___XX___,
   ___XX___,
   _XXX____,
   ________}  

/*  ~  */
 ,{
   _XXX_XX_,
   XX_XXX__,
   ________,
   ________,
   ________,
   ________,
   ________,
   ________}   

/* 0x7f */
 ,{
   XXXXXX__,
   XXXXXX__,
   XXXXXX__,
   XXXXXX__,
   XXXXXX__,
   XXXXXX__,
   XXXXXX__,
   ________}
   
};

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
uint8_t  gui_put_char8_8(uint16_t x, uint16_t y, uint8_t ch)
{  
   uint8_t   i;

   if( x>(GUI_DISPLAY_X_MAX-8) ) return(0);
   if( y>(GUI_DISPLAY_Y_MAX-8) ) return(0);
   if( (ch<0x20) || (ch>0x7f) ) ch = 0x20;
   
   ch -= 0x20; 
   for(i=0; i<8; i++)
   {  
      gui_point_color(x, y, s_ascii_font8_8[ch][i], 8); 
      y++;                                                                    
   }
   
   return(1);
}


void  gui_put_string8_8(uint16_t x, uint16_t y, char *p_str)
{  while(1)
   {  if( (*p_str)=='\0' ) break;
      if( gui_put_char8_8(x, y, *p_str++)==0 ) break;
      x += 6;                    
   }
}

void gui_put_num8_8(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t mode)
{
    uint8_t t,temp;
    uint8_t enshow = 0;
    for(t=0; t<len; t++)
    {
        temp = (num/gui_pow(10,len-t-1))%10;
        if (enshow==0&&t<(len-1))
        {
            if (temp==0)
            {
                if(mode)
                {
                  gui_put_char8_8(x+8*t, y, '0');
                }
                continue;
            }   else enshow=1;
        }
        gui_put_char8_8(x+8*t, y, temp+'0');
    }
}


#endif

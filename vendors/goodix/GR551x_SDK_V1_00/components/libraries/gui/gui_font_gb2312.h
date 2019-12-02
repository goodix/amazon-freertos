/**
 *****************************************************************************************
 *
 * @file gui_font_gb2312.h
 *
 * @brief Show gb2312 string API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef _GUI_FONT_GB2312_H_
#define _GUI_FONT_GB2312_H_

#include "gui_config.h"

#if FONT_GB2312_EN==1

/**
 *****************************************************************************************
 * @brief Display gb2312 string and 8*16 ascii char.
 * @note  No line feed.
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] p_str: Display string.
 *****************************************************************************************
 */
void gui_put_string(uint16_t x, uint16_t y, char *p_str);

#endif

#endif

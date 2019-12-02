/**
 *****************************************************************************************
 *
 * @file gui_font8_8.h
 *
 * @brief Show 8*8 ascii API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */



#ifndef  FONT8_8_H
#define  FONT8_8_H

#include "gui_config.h"

#if FONT8x8_EN==1

/**
 *****************************************************************************************
 * @brief Display one ascii.
 * @note  Display value between 0x20 to 0x7f, others show ' '.
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] ch: Display ascii.
 *
 * @return 0x01--success, 0x00--fail(the coordinate value is out of range)
 *****************************************************************************************
 */
uint8_t  gui_put_char8_8(uint16_t x, uint16_t y, uint8_t ch);

/**
 *****************************************************************************************
 * @brief Display one ascii string.
 * @note  No line feed.
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] p_str: Display ascii string.
 *****************************************************************************************
 */
void  gui_put_string8_8(uint16_t x, uint16_t y, char *p_str);

/**
 *****************************************************************************************
 * @brief Display num.
 *
 * @param[in] x:    X coordinate.
 * @param[in] y:    Y coordinate.
 * @param[in] num:  Display num.
 * @param[in] len:  Display len.
 * @param[in] mode: 0x01--the 0 in front of the value will be displayed
 *                  0x00--the 0 in front of the value will not be displayed
 *****************************************************************************************
 */
void gui_put_num8_8(uint16_t x, uint16_t y, uint32_t num, uint8_t len, uint8_t mode);

#endif

#endif

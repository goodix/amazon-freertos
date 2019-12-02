/**
 *****************************************************************************************
 *
 * @file gui_font_other.h
 *
 * @brief Show other modulus font API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef _GUI_FONT_OTHER_H_
#define _GUI_FONT_OTHER_H_

#include "gui_config.h"

#if FONT_OTHER_EN==1
/**
 *****************************************************************************************
 * @brief Display one 16x32 Modulus font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:         X coordinate.
 * @param[in] y:         Y coordinate.
 * @param[in] code16_32: Array of font.
 * @param[in] index:     Index of array of font.
 *
 * @return 0x01--success, 0x00--fail(the coordinate value is out of range)
 *****************************************************************************************
 */
uint8_t gui_put_char16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t index);

/**
 *****************************************************************************************
 * @brief Display an Array font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] code16_32: Array of font.
 * @param[in] num:     Show the total number.
 *****************************************************************************************
 */
void gui_put_string16_32(uint16_t x, uint16_t y, const uint8_t(*code16_32)[16], uint16_t num);

/**
 *****************************************************************************************
 * @brief Display one 32x64 Modulus font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:         X coordinate.
 * @param[in] y:         Y coordinate.
 * @param[in] code32_64: Array of font.
 * @param[in] index:     Index of array of font.
 *
 * @return 0x01--success, 0x00--fail(the coordinate value is out of range)
 *****************************************************************************************
 */
uint8_t gui_put_char32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t index);

/**
 *****************************************************************************************
 * @brief Display an Array font.
 * @note  Modulus set: 1--display,Determinant,Big-Endian
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] code32_64: Array of font.
 * @param[in] num:     Show the total number.
 *****************************************************************************************
 */
void gui_put_string32_64(uint16_t x, uint16_t y, const uint8_t(*code32_64)[16], uint16_t num);
#endif
#endif


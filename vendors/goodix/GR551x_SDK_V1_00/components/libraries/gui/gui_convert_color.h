/**
 *****************************************************************************************
 *
 * @file gui_convert_color.h
 *
 * @brief Convert color API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */


#ifndef  GUI_CONVERT_COLOR_H
#define  GUI_CONVERT_COLOR_H

#include "gui_config.h"

#if CONVERT_COLOR_EN==1

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 16-bit index value.
 * @note  The convert value applies to a 64K color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 16-bit index value(64K color, d15-d11 for R value, d10-d5 for G value, d4-d0 for B value)
 *****************************************************************************************
 */
uint16_t  gui_color_to_index_565(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 16-bit index value to a RGB value.
 * @note  The convert value applies to a 64K color LCD screen.
 *
 * @param[in] index:  16 bit index value.
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_565(uint16_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 15-bit index value.
 * @note  The convert value applies to a 32K color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 15-bit index value.
 *****************************************************************************************
 */
uint16_t  gui_color_to_index_555(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 15-bit index value to a RGB value.
 * @note  The convert value applies to a 32K color LCD screen.
 *
 * @param[in] index:  15 bit index value.
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_555(uint16_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 12-bit index value.
 * @note  The convert value applies to a 4096 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 12-bit index value.(RRRRGGGGBBBB)
 *****************************************************************************************
 */
uint16_t  gui_color_to_index_444(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 12-bit index value to a RGB value.
 * @note  The convert value applies to a 4096 color LCD screen.
 *
 * @param[in] index:  12 bit index value.(RRRRGGGGBBBB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_444(uint16_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 8-bit index value.
 * @note  The convert value applies to a 256 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 8-bit index value.(RRRGGGBB)
 *****************************************************************************************
 */
uint8_t   gui_color_to_index_332(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 8-bit index value to a RGB value.
 * @note  The convert value applies to a 256 color LCD screen.
 *
 * @param[in] index:  8 bit index value.(RRRGGGBB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_332(uint8_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 6-bit index value.
 * @note  The convert value applies to a 256 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 6-bit index value.(RRGGBB)
 *****************************************************************************************
 */
uint8_t   gui_color_to_index_222(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 6-bit index value to a RGB value.
 * @note  The convert value applies to a 64K color LCD screen.
 *
 * @param[in] index:  6 bit index value.(RRGGBB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_222(uint8_t index);

/**
 *****************************************************************************************
 * @brief Convert the RGB value to a 3-bit index value.
 * @note  The convert value applies to a 64 color LCD screen.
 *
 * @param[in] color_rgb:  RGB value.
 * 
 * @return 3-bit index value.(RGB)
 *****************************************************************************************
 */
uint8_t   gui_color_to_index_111(uint32_t color_rgb);

/**
 *****************************************************************************************
 * @brief Convert the 3-bit index value to a RGB value.
 * @note  The convert value applies to a 64 color LCD screen.
 *
 * @param[in] index:  3 bit index value.(RGB)
 * 
 * @return RGB value
 *****************************************************************************************
 */
uint32_t  gui_index_to_color_111(uint8_t index);
#endif

#endif




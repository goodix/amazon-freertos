/**
 *****************************************************************************************
 *
 * @file gui_color.h
 *
 * @brief Set gui color API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef  GUI_COLOR_H
#define  GUI_COLOR_H

#include "gui_config.h"

/**
 *****************************************************************************************
 * @brief 
 * @note  Hexadecimal bit conversion.
 *
 * @param[in] dcb:  DCB data.
 *
 * @return Hex data
 *****************************************************************************************
 */
uint8_t gui_dcb_to_hex(uint8_t dcb);

/**
 *****************************************************************************************
 * @brief Set display and background color.
 *
 * @param[in] disp_color:  Display color.
 * @param[in] back_color:  Background color.
 *****************************************************************************************
 */
void gui_set_color(T_COLOR disp_color, T_COLOR back_color);

/**
 *****************************************************************************************
 * @brief Get background color
 *
 * @return Background color
 *****************************************************************************************
 */
T_COLOR  gui_get_back_color(void);

/**
 *****************************************************************************************
 * @brief Get display color
 *
 * @return Display color
 *****************************************************************************************
 */
T_COLOR  gui_get_disp_color(void);

/**
 *****************************************************************************************
 * @brief Swap foreground and background colors for reverse display
 *****************************************************************************************
 */
void  gui_exchange_color(void);

/**
 *****************************************************************************************
 * @brief Based on the current color settings,draw point
 *
 * @param[in] x:  X coordinate.
 * @param[in] y:  Y coordinate.
 * @param[in] font_dat: one byte point.
 * @param[in] num: point count.
 *****************************************************************************************
 */
void gui_point_color(uint8_t x, uint8_t y, uint8_t font_dat, uint8_t num);

#endif

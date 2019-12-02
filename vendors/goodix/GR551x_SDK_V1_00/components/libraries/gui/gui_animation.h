/**
 *****************************************************************************************
 *
 * @file gui_animation.h
 *
 * @brief Animation API.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef _ANIMATION_H__
#define _ANIMATION_H__
#include "gui_config.h"

#if ANIMATION_EN==1

typedef void(*animation_stop_handle_t)(void);/**< animation stop call back dfine. */

/**@brief Animation types. */
typedef enum {
  MOVE_LEFT = 1,
  MOVE_RIGHT,
  MOVE_UP,
  MOVE_DOWN,
}animation_type_t;

/**
 *****************************************************************************************
 * @brief Animation init.
 *
 * @param[in] display_ram:  Pointer of display memory
 * @param[in] display_buffer: Pointer of display buffer.
 *****************************************************************************************
 */
void gui_animation_init(T_COLOR (*display_ram)[GUI_DISPLAY_X_MAX], T_COLOR (*display_buffer)[GUI_DISPLAY_X_MAX]);

/**
 *****************************************************************************************
 * @brief Animation timer task.
 * @note The function should be called in animation timer handler
 *****************************************************************************************
 */
void gui_animation_timer_task(void);

/**
 *****************************************************************************************
 * @brief Start a animation.
 *
 * @param[in] type:  Animation type 
 * @param[in] stop_call: Animation stop callback.
 *****************************************************************************************
 */
void gui_start_animation(animation_type_t type, animation_stop_handle_t stop_call);

#endif
#endif

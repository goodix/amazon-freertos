/**
 ****************************************************************************************
 *
 * @file app_key.c
 *
 * @brief App Key API
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __APP_KEY_H__
#define __APP_KEY_H__

#include "app_key_core.h"
#include "app_gpiote.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup APP_KEY_STRUCT Structures
 * @{
 */
/**@brief App key gpio initialization variables. */
typedef struct
{
    app_io_type_t  gpio_type;            /**< Key gpio type. */
    uint32_t       gpio_pin;             /**< Key gpio pin. */
    app_io_mode_t  trigger_mode;         /**< Specifies the operating mode for the selected pin. */
    uint8_t        key_id;               /**< Key register ID. */
} app_key_gpio_t;
/** @} */

/**
 * @defgroup APP_KEY_TYPEDEF Typedefs
 * @{
 */
/**@brief APP Key event callback.*/
typedef void (*app_key_evt_cb_t)(uint8_t key_id, app_key_click_type_t key_click_type);
/** @} */

/**
 * @defgroup APP_KEY_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief App key initialize.
 *
 * @param[in] key_inst:     The array of key instance.
 * @param[in] key_num:      The number of key instance.
 * @param[in] key_click_cb: App key click event callback.
 *
 * @return Result of app key inlitialization.
 *****************************************************************************************
 */
bool app_key_init(app_key_gpio_t key_inst[], uint8_t key_num, app_key_evt_cb_t key_click_cb);

/**
 *****************************************************************************************
 * @brief App key pressed down handler.
 *****************************************************************************************
 */
void app_key_pressed_handler(app_key_gpio_t *p_app_key_info);
/** @} */

#endif


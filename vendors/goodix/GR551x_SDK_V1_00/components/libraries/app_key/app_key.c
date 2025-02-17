/**
 *****************************************************************************************
 *
 * @file app_key.c
 *
 * @brief App key Implementation.
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
#include "app_key.h"
#include "app_gpiote.h"
#include "app_timer.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_KEY_TIMER_INTERVAL   10    /**< App key polling interval interval (in units of 1ms). */
/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint8_t            s_app_key_info[APP_KEY_REG_COUNT_MAX];
static app_gpiote_param_t s_app_io_cfg[APP_KEY_REG_COUNT_MAX];
static app_key_evt_cb_t   s_app_key_evt_cb;
static uint8_t            s_app_key_reg_num;
static app_timer_id_t     s_app_key_timer_id;
static bool               s_is_timer_enabled;

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Polling app key pressed state.
 *****************************************************************************************
 */
static void app_key_press_state_polling(void)
{
    app_io_pin_state_t   pin_state;
    bool                 is_pressed = false;

    for (uint8_t key_idx = 0; key_idx < s_app_key_reg_num; key_idx++)
    {
        pin_state = app_io_read_pin(s_app_io_cfg[key_idx].type, s_app_io_cfg[key_idx].pin);

        if (APP_IO_PIN_RESET == pin_state)
        {
            is_pressed = true;
        }

        app_key_core_key_pressed_record(key_idx, is_pressed);
    }
}

/**
 *****************************************************************************************
 * @brief App key timing timeout handler.
 *****************************************************************************************
 */
static void app_key_timeout_handler(void *p_arg)
{
    app_key_press_state_polling();
    app_key_core_polling_10ms();
}

/**
 *****************************************************************************************
 * @brief Start app key timer.
 *****************************************************************************************
 */
static void app_key_timer_start(void)
{
    app_timer_create(&s_app_key_timer_id, ATIMER_REPEAT, app_key_timeout_handler);
    app_timer_start(s_app_key_timer_id, APP_KEY_TIMER_INTERVAL, NULL);
    s_is_timer_enabled = true;
}

/**
 *****************************************************************************************
 * @brief Stop app key timer.
 *****************************************************************************************
 */
void app_key_timer_stop(void)
{
    app_timer_delete(&s_app_key_timer_id);
    s_is_timer_enabled = false;
}

/**
 *****************************************************************************************
 * @brief App key core event handler.
 *****************************************************************************************
 */
static void app_key_core_evt_handler(uint8_t key_idx, app_key_click_type_t key_click_type)
{
    if (app_key_core_is_all_release())
    {
        app_key_timer_stop();
    }

    s_app_key_evt_cb(s_app_key_info[key_idx], key_click_type);
}

static void app_gpiote_event_handler(app_gpiote_evt_t *p_evt)
{
    if ( NULL == p_evt)
        return;

    if (!s_is_timer_enabled)
    {
        app_key_timer_start();
    }

    for (uint8_t key_idx = 0; key_idx < s_app_key_reg_num; key_idx++)
    {
        if (APP_IO_TYPE_NORMAL == p_evt->type)
        {
            if ((p_evt->pin == s_app_io_cfg[key_idx].pin))
            {
                app_key_core_key_wait_polling_record(key_idx);
                return;
            }
        }
        else if ((APP_IO_TYPE_AON == p_evt->type) && \
                 (p_evt->pin  == s_app_io_cfg[key_idx].pin))
        {
            app_key_core_key_wait_polling_record(key_idx);
            return;
        }
    }
}



/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool app_key_init(app_key_gpio_t key_inst[], uint8_t key_num, app_key_evt_cb_t key_evt_cb)
{
    if (APP_KEY_REG_COUNT_MAX < key_num || NULL == key_evt_cb)
    {
        return false;
    }

    for (uint8_t key_idx = 0; key_idx < key_num; key_idx++)
    {
        if (key_inst[key_idx].gpio_type != APP_IO_TYPE_NORMAL && key_inst[key_idx].gpio_type != APP_IO_TYPE_AON)
        {
            return false;
        }
        s_app_key_info[key_idx] = key_inst[key_idx].key_id;
        s_app_io_cfg[key_idx].type = key_inst[key_idx].gpio_type;
        s_app_io_cfg[key_idx].pin  = key_inst[key_idx].gpio_pin;
        s_app_io_cfg[key_idx].mode = key_inst[key_idx].trigger_mode;
        s_app_io_cfg[key_idx].pull = APP_IO_PULLUP;
        s_app_io_cfg[key_idx].handle_mode = APP_IO_ENABLE_WAKEUP;
        s_app_io_cfg[key_idx].io_evt_cb = app_gpiote_event_handler;
    }
    
    app_gpiote_init(s_app_io_cfg, key_num); 
    
    s_app_key_reg_num = key_num;
    s_app_key_evt_cb  = key_evt_cb;
    app_key_core_cb_register(app_key_core_evt_handler);

    return true;
}



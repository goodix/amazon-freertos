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

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_key.h"
#include "app_timer.h"

/*
 * DEFINES
 *****************************************************************************************
 */
#define APP_KEY_TIMER_INTERVAL   10    /**< App key polling interval interval (in units of 1ms). */

/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */
/**@breif Information of app key register. */
typedef struct
{
    app_key_gpio_type_t  gpio_type;            /**< Key gpio type. */
    gpio_regs_t         *GPIOx;                /**< GPIOx: Where x can be (0, 1) to select the GPIO peripheral port for GR55xx device. */
    uint16_t             gpio_pin;             /**< Key gpio pin. */
    uint16_t             aon_gpio_pin;         /**< Key ano_gpio type. */
    uint8_t              key_id;               /**< Key id. */
} app_key_info_t;

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_key_info_t   s_app_key_info[APP_KEY_REG_COUNT_MAX];
static app_key_evt_cb_t s_app_key_evt_cb;
static uint8_t          s_app_key_reg_num;
static app_timer_id_t   s_app_key_timer_id;
static bool             s_is_timer_enabled;
static bool             s_is_ext0_enabled;
static bool             s_is_ext1_enabled; 
static bool             s_is_ext2_enabled;


/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize app key gpio.
 *
 * @param[in] GPIOx:        Where x can be (0, 1) to select the GPIO peripheral port for GR55xx device.
 * @param[in] gpio_pin:     Key gpio pin.
 * @param[in] trigger_mode: Specifies the operating mode for the selected pin.
 *****************************************************************************************
 */
static void app_key_gpio_hal_init(gpio_regs_t *GPIOx, uint16_t gpio_pin, uint32_t trigger_mode)
{
    gpio_init_t GPIO_Init = GPIO_DEFAULT_CONFIG;

    GPIO_Init.pull = GPIO_PULLUP;
    GPIO_Init.pin  = gpio_pin;
    GPIO_Init.mode = trigger_mode;
    hal_gpio_init(GPIOx, &GPIO_Init);

    if (GPIO0 == GPIOx && !s_is_ext0_enabled)
    {
        NVIC_ClearPendingIRQ(EXT0_IRQn);
        NVIC_EnableIRQ(EXT0_IRQn);
        s_is_ext0_enabled = true;
    }
    else if (GPIO1 == GPIOx && !s_is_ext1_enabled)
    {
        NVIC_ClearPendingIRQ(EXT1_IRQn);
        NVIC_EnableIRQ(EXT1_IRQn);
        s_is_ext1_enabled = true;
    }

}

/**
 *****************************************************************************************
 * @brief Initialize app key ano_gpio.
 *
 * @param[in] aon_gpio_pin: Key aon_gpio pin.
 * @param[in] trigger_mode: Specifies the operating mode for the selected pin.
 *****************************************************************************************
 */
static void app_key_aon_gpio_hal_init(uint16_t aon_gpio_pin, uint32_t trigger_mode)
{
    aon_gpio_init_t AON_GPIO_Init = AON_GPIO_DEFAULT_CONFIG;

    AON_GPIO_Init.pull = AON_GPIO_PULLUP;
    AON_GPIO_Init.pin  = aon_gpio_pin;
    AON_GPIO_Init.mode = trigger_mode;
    hal_aon_gpio_init(&AON_GPIO_Init);

    if (!s_is_ext2_enabled)
    {
        NVIC_ClearPendingIRQ(EXT2_IRQn);
        NVIC_EnableIRQ(EXT2_IRQn);
        s_is_ext2_enabled = true;
    }
}

/**
 *****************************************************************************************
 * @brief Polling app key pressed state.
 *****************************************************************************************
 */
static void app_key_press_state_polling(void)
{
    gpio_pin_state_t     gpio_pin_level;
    aon_gpio_pin_state_t aon_gpio_pin_level;
    bool                 is_pressed = false;

    for (uint8_t key_idx = 0; key_idx < s_app_key_reg_num; key_idx++)
    {
        if (APP_KEY_GPIO == s_app_key_info[key_idx].gpio_type)
        {
            gpio_pin_level = hal_gpio_read_pin(s_app_key_info[key_idx].GPIOx, s_app_key_info[key_idx].gpio_pin);

            if (GPIO_PIN_RESET == gpio_pin_level)
            {
                is_pressed = true;
            }
        }
        else if (APP_KEY_AON_GPIO == s_app_key_info[key_idx].gpio_type)
        {
            aon_gpio_pin_level = hal_aon_gpio_read_pin(s_app_key_info[key_idx].aon_gpio_pin);

            if (AON_GPIO_PIN_RESET == aon_gpio_pin_level)
            {
                is_pressed = true;
            }
        }

        app_key_core_key_pressed_record(key_idx, is_pressed);
        is_pressed = false;
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
 * @brief App key pressed down handler.
 *****************************************************************************************
 */
static void app_key_pressed_handler(app_key_info_t *p_app_key_info)
{
    if (!s_is_timer_enabled)
    {
        app_key_timer_start();
    }

    for (uint8_t key_idx = 0; key_idx < s_app_key_reg_num; key_idx++)
    {
        if (APP_KEY_GPIO == p_app_key_info->gpio_type)
        {
            if ((p_app_key_info->GPIOx == s_app_key_info[key_idx].GPIOx) && \
                (p_app_key_info->gpio_pin == s_app_key_info[key_idx].gpio_pin))
            {
                app_key_core_key_wait_polling_record(key_idx);
                return;
            }
        }
        else if ((APP_KEY_AON_GPIO == p_app_key_info->gpio_type) && \
                 (p_app_key_info->aon_gpio_pin  == s_app_key_info[key_idx].aon_gpio_pin))
        {
            app_key_core_key_wait_polling_record(key_idx);
            return;
        }
    }
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

    s_app_key_evt_cb(s_app_key_info[key_idx].key_id, key_click_type);
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool app_key_init(app_key_gpio_init_t key_inst[], uint8_t key_num, app_key_evt_cb_t key_evt_cb)
{
    if (APP_KEY_REG_COUNT_MAX < key_num || NULL == key_evt_cb)
    {
        return false;
    }

    for (uint8_t key_idx = 0; key_idx < key_num; key_idx++)
    {
        s_app_key_info[key_idx].gpio_type = key_inst[key_idx].gpio_type;
        s_app_key_info[key_idx].key_id    = key_inst[key_idx].key_id;

        if (APP_KEY_GPIO == key_inst[key_idx].gpio_type)
        {
            app_key_gpio_hal_init(key_inst[key_idx].GPIOx, key_inst[key_idx].gpio_pin, key_inst[key_idx].trigger_mode);

            s_app_key_info[key_idx].GPIOx    = key_inst[key_idx].GPIOx;
            s_app_key_info[key_idx].gpio_pin = key_inst[key_idx].gpio_pin;
        }
        else if (APP_KEY_AON_GPIO == key_inst[key_idx].gpio_type)
        {
            app_key_aon_gpio_hal_init(key_inst[key_idx].aon_gpio_pin, key_inst[key_idx].trigger_mode);

            s_app_key_info[key_idx].aon_gpio_pin  = key_inst[key_idx].aon_gpio_pin;
        }
        else
        {
            return false;
        }
    }

    s_app_key_reg_num = key_num;
    s_app_key_evt_cb  = key_evt_cb;
    app_key_core_cb_register(app_key_core_evt_handler);
    return true;
}

void EXT0_IRQHandler(void)
{
    app_key_info_t  triggered_key_info;

    triggered_key_info.gpio_type = APP_KEY_GPIO;
    triggered_key_info.GPIOx     = GPIO0;
    triggered_key_info.gpio_pin  = __HAL_GPIO_IT_GET_IT(GPIO0, GPIO_PIN_ALL);

    __HAL_GPIO_IT_CLEAR_IT(GPIO0, triggered_key_info.gpio_pin);

    app_key_pressed_handler(&triggered_key_info);
}

void EXT1_IRQHandler(void)
{
    app_key_info_t  triggered_key_info;

    triggered_key_info.gpio_type = APP_KEY_GPIO;
    triggered_key_info.GPIOx     = GPIO1;
    triggered_key_info.gpio_pin  = __HAL_GPIO_IT_GET_IT(GPIO1, GPIO_PIN_ALL);

    __HAL_GPIO_IT_CLEAR_IT(GPIO1, triggered_key_info.gpio_pin);

    app_key_pressed_handler(&triggered_key_info);
}

void EXT2_IRQHandler(void)
{
    app_key_info_t  triggered_key_info;

    triggered_key_info.gpio_type    = APP_KEY_AON_GPIO;
    triggered_key_info.aon_gpio_pin = __HAL_AON_GPIO_IT_GET_IT(AON_GPIO_PIN_ALL);

    __HAL_AON_GPIO_IT_CLEAR_IT(triggered_key_info.aon_gpio_pin);

    app_key_pressed_handler(&triggered_key_info);
}


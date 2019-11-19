/**
 *****************************************************************************************
 *
 * @file bsp.c
 *
 * @brief Board Support Package Implementation.
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
#include "bsp.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void bsp_key_init(app_key_evt_cb_t key_click_cb)
{
    app_key_gpio_init_t app_key_inst[5];

    app_key_inst[0].GPIOx        = GPIO0;  
    app_key_inst[0].gpio_type    = APP_KEY_GPIO;
    app_key_inst[0].gpio_pin     = KEY_UP_PIN;
    app_key_inst[0].trigger_mode = KEY_TRIGGER_MODE;
    app_key_inst[0].key_id       = BSP_KEY_UP_ID;

    app_key_inst[1].GPIOx        = GPIO0;  
    app_key_inst[1].gpio_type    = APP_KEY_GPIO;
    app_key_inst[1].gpio_pin     = KEY_DOWN_PIN;
    app_key_inst[1].trigger_mode = KEY_TRIGGER_MODE;
    app_key_inst[1].key_id       = BSP_KEY_DOWN_ID;

    app_key_inst[2].GPIOx        = GPIO0;  
    app_key_inst[2].gpio_type    = APP_KEY_GPIO;
    app_key_inst[2].gpio_pin     = KEY_RIGHT_PIN;
    app_key_inst[2].trigger_mode = KEY_TRIGGER_MODE;
    app_key_inst[2].key_id       = BSP_KEY_RIGHT_ID;

    app_key_inst[3].GPIOx        = GPIO0;  
    app_key_inst[3].gpio_type    = APP_KEY_GPIO;
    app_key_inst[3].gpio_pin     = KEY_LEFT_PIN;
    app_key_inst[3].trigger_mode = KEY_TRIGGER_MODE;
    app_key_inst[3].key_id       = BSP_KEY_LEFT_ID;

    app_key_inst[4].gpio_type    = APP_KEY_AON_GPIO;
    app_key_inst[4].aon_gpio_pin = KEY_OK_PIN;
    app_key_inst[4].trigger_mode = KEY_TRIGGER_MODE;
    app_key_inst[4].key_id       = BSP_KEY_OK_ID;

    app_key_init(app_key_inst, 5, key_click_cb);
}

void bsp_led_init(void)
{
    msio_init_t MSIO_Init = MSIO_DEFAULT_CONFIG;
    MSIO_Init.pin         = LED_NUM_1_IO;
    MSIO_Init.direction   = MSIO_DIRECTION_OUTPUT;
    MSIO_Init.mode        = MSIO_MODE_DIGITAL;
    hal_msio_init(&MSIO_Init);

    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;
    gpio_config.mode        = GPIO_MODE_OUTPUT;
    gpio_config.pin         = LED_NUM_0_IO;
    hal_gpio_init(LED_NUM_0_GRP, &gpio_config); 
}

void bsp_led_open(bsp_led_num_t led_num)
{
    switch (led_num)
    {
        case BSP_LED_NUM_0:
            ll_gpio_reset_output_pin(LED_NUM_0_GRP, LED_NUM_0_IO);
            break;

        case BSP_LED_NUM_1:
            hal_msio_write_pin(LED_NUM_1_IO, MSIO_PIN_RESET);              
            break;

        default:
            break;
    }
}

void bsp_led_close(bsp_led_num_t led_num)
{
    switch (led_num)
    {
        case BSP_LED_NUM_0:
            ll_gpio_set_output_pin(LED_NUM_0_GRP, LED_NUM_0_IO);
            break;

        case BSP_LED_NUM_1:
            hal_msio_write_pin(LED_NUM_1_IO, MSIO_PIN_SET);
            break;

        default:
            break;
    }
}

 

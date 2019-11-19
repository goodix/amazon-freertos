/**
 *****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief  User Periph Init Function Implementation.
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
//#include "dfu_port.h"
#include "hal_flash.h"
#include "gr551x_spi_flash.h"
#include "gr55xx_dfu.h"
#include "user_app.h"
#include "boards.h"
#include "app_log.h"

static void dfu_program_start_callback(void);
static void dfu_programing_callback(uint8_t pro);
static void dfu_program_end_callback(uint8_t status);
static void dfu_spi_flash_init(uint8_t cs_pin, uint8_t cs_mux, uint8_t spi_group);

static dfu_func_t dfu_func = 
{
    .dfu_ble_send_data      = NULL,
    .dfu_uart_send_data     = NULL,
    .dfu_flash_read         = hal_flash_read,
    .dfu_flash_write        = hal_flash_write,
    .dfu_flash_erase        = hal_flash_erase,
    .dfu_flash_erase_chip   = hal_flash_erase_chip,
    .dfu_flash_set_security = hal_flash_set_security,
    .dfu_flash_get_security = hal_flash_get_security,
    .dfu_flash_get_info     = hal_flash_get_info,
    .dfu_delay_reset_timer_start = dfu_default_reset_dealy_timer_start,
};

static dfu_spi_flash_func_t dfu_spi_flash_func= 
{
    .dfu_spi_flash_init = dfu_spi_flash_init,
    .dfu_spi_flash_read = spi_flash_read,
    .dfu_spi_flash_write = spi_flash_write,
    .dfu_spi_flash_erase = spi_flash_sector_erase,
    .dfu_spi_flash_erase_chip = spi_flash_chip_erase,
    .dfu_spi_flash_get_info = spi_flash_device_info,
};

static dfu_pro_callback_t dfu_pro_call = 
{
    .dfu_program_start_callback = dfu_program_start_callback,
    .dfu_programing_callback = dfu_programing_callback,
    .dfu_program_end_callback = dfu_program_end_callback,
};


static void dfu_spi_flash_init(uint8_t cs_pin, uint8_t cs_mux, uint8_t spi_group)
{
    const spi_flash_io_t spi_io[4] = {DEFAULT_SPIM_GROUP0, DEFAULT_SPIM_GROUP1,\
                                      DEFAULT_SPIM_GROUP2, DEFAULT_SPIM_GROUP3};
    const uint32_t gpio_pin[] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7,\
                                 GPIO_PIN_8,GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15};
    const uint32_t gpio_pin_mux[] = {GPIO_MUX_0,GPIO_MUX_1,GPIO_MUX_2,GPIO_MUX_3,GPIO_MUX_4,GPIO_MUX_5,GPIO_MUX_6,GPIO_MUX_7,\
                                     GPIO_MUX_8};
    
    spi_flash_io_t spi_config_io;
    if(spi_group < 5)
    {
        memcpy(&spi_config_io, &spi_io[spi_group], sizeof(spi_flash_io_t));
    }
    
    if(cs_pin < 32)
    {
        if(cs_pin < 16)
        {
            spi_config_io.spi_cs.gpio = GPIO0;
            spi_config_io.spi_cs.pin = gpio_pin[cs_pin];
        }
        else
        {
            spi_config_io.spi_cs.gpio = GPIO1;
            spi_config_io.spi_cs.pin = gpio_pin[cs_pin-16];
        }
    }
    if(cs_mux < 9)
    {
        spi_config_io.spi_cs.mux = gpio_pin_mux[cs_mux];
    }
    spi_flash_init(&spi_config_io);
}

static void dfu_program_start_callback(void)
{
    APP_LOG_DEBUG("Dfu start program");
}   

static void dfu_programing_callback(uint8_t pro)
{
    APP_LOG_DEBUG("Dfu programing---%d%%", pro);
}

static void dfu_program_end_callback(uint8_t status)
{
    APP_LOG_DEBUG("Dfu program end");
    if(0x01 == status)
    {
        APP_LOG_DEBUG("status: successful");
    }
    else
    {
        APP_LOG_DEBUG("status: error");
    }
}

void dfu_port_init(void)
{
    dfu_init(&dfu_func, &dfu_pro_call);
    dfu_spi_flash_func_config(&dfu_spi_flash_func);
}




/**
 ****************************************************************************************
 *
 * @file uc1701_config.c
 *
 * @brief uc1701 config Implementation.
 *
 ****************************************************************************************
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
#include "st7735_config.h"
#include "gr55xx_delay.h"
#include "gr55xx_hal.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
#ifdef DISPLAY_DRIVER_TYPE_HW_SPI
static spi_handle_t s_SPIMHandle;
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void st7735_init(void)
{
    gpio_init_t GPIO_InitStruct = GPIO_DEFAULT_CONFIG;
    GPIO_InitStruct.mode = GPIO_MODE_OUTPUT;
    GPIO_InitStruct.pin  = DISPLAY_CMD_AND_DATA_PIN | DISPLAY_SPIM_CS0_PIN | DISPLAY_SPIM_CLK_PIN | DISPLAY_SPIM_MOSI_PIN;       
    hal_gpio_init(GPIO0, &GPIO_InitStruct);

    GPIO_InitStruct.pin  = GPIO_PIN_2;
    hal_gpio_init(GPIO0, &GPIO_InitStruct);
    hal_gpio_write_pin(GPIO0, GPIO_PIN_2, GPIO_PIN_SET);
    
#ifdef DISPLAY_DRIVER_TYPE_HW_SPI
    s_SPIMHandle.p_instance              = SPIM;
    s_SPIMHandle.init.data_size          = SPI_DATASIZE_8BIT;
    s_SPIMHandle.init.clock_polarity     = SPI_POLARITY_LOW;
    s_SPIMHandle.init.clock_phase        = SPI_PHASE_1EDGE;
    s_SPIMHandle.init.baudrate_prescaler = SystemCoreClock / 4000000;
    s_SPIMHandle.init.ti_mode            = SPI_TIMODE_DISABLE;
    s_SPIMHandle.init.slave_select       = SPI_SLAVE_SELECT_0;
    
    hal_spi_deinit(&s_SPIMHandle);
    hal_spi_init(&s_SPIMHandle);
#endif
}

#ifdef DISPLAY_DRIVER_TYPE_SW_IO
/*--------------------------------DISPLAY_DRIVER_TYPE_SW_IO------------------------------------*/
void st7735_write_cmd(uint8_t cmd)
{
    uint8_t i;
    
    SEND_CMD;
    CS_LOW;
    for(i=0;i<8;i++)
    {
        if (cmd &0x80)
           SDA_HIGH;
        else 
           SDA_LOW;

        SCK_LOW;
        SCK_HIGH;
        cmd <<= 1;
    }
    CS_HIGH;
}

void st7735_write_data(uint8_t data)
{
    uint8_t i;
    
    SEND_DATA;
    CS_LOW;
    for(i=0;i<8;i++)
    {
    if(data&0x80)
        SDA_HIGH;
    else 
        SDA_LOW;
    SCK_LOW;
    SCK_HIGH;
    data <<= 1;
    }
    CS_HIGH;
}

void st7735_write_buffer(uint8_t *p_data, uint16_t length)
{
    uint16_t i= 0;
    SEND_DATA;
    CS_LOW;
    for (i=0; i<length; i ++) 
    {
        st7735_write_data(p_data[i]);
    }   
    CS_HIGH;
}

#else
/*--------------------------------DISPLAY_DRIVER_TYPE_HW_SPI------------------------------------*/
void st7735_write_cmd(uint8_t cmd)
{
    SEND_CMD;
    CS_LOW;
    hal_spi_transmit(&s_SPIMHandle, &cmd, 1, 5000);
    CS_HIGH;
}

void st7735_write_data(uint8_t data)
{
    SEND_DATA;
    CS_LOW;
    hal_spi_transmit(&s_SPIMHandle, &data, 1, 5000);
    CS_HIGH;
}

void st7735_write_buffer(uint8_t *p_data, uint16_t length)
{
    SEND_DATA;
    CS_LOW;
    hal_spi_transmit(&s_SPIMHandle, p_data, length, 5000);
    CS_HIGH;
}

void hal_spi_msp_init(spi_handle_t *hspi)
{
    gpio_init_t GPIO_InitStructure;
  
    GPIO_InitStructure.mode = GPIO_MODE_MUX;
    GPIO_InitStructure.pin = DISPLAY_SPIM_CLK_PIN | DISPLAY_SPIM_MOSI_PIN;
    GPIO_InitStructure.mux = GPIO_MUX_4;
    hal_gpio_init(DISPLAY_SPIM_GPIO_PORT, &GPIO_InitStructure);

    NVIC_ClearPendingIRQ(SPI_M_IRQn);
    NVIC_EnableIRQ(SPI_M_IRQn);
}

void hal_spi_msp_deinit(spi_handle_t *p_spi)
{
}

#endif

void st7735_delay(uint16_t time)
{
    delay_ms(time);
}



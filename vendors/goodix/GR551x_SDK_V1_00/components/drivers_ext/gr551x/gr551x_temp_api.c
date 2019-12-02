/**
 ****************************************************************************************
 *
 * @file gr551x_temp_api.c
 *
 * @brief GR551x temperature module.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "gr55xx_hal.h"
#include "gr55xx_sys.h"
#include "gr551x_temp_api.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static adc_handle_t gr551x_temp_handle = {0};
static double adc_temp = 0;
static double adc_slope = 0;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_gr551x_temp_init(void)
{
    adc_trim_info_t adc_trim = {0};

    gr551x_temp_handle.init.channel_n  = ADC_INPUT_SRC_TMP;
    gr551x_temp_handle.init.channel_p  = ADC_INPUT_SRC_TMP;
    gr551x_temp_handle.init.input_mode = ADC_INPUT_SINGLE;
    gr551x_temp_handle.init.ref_source = ADC_REF_SRC_BUF_INT;
    gr551x_temp_handle.init.ref_value  = ADC_REF_VALUE_0P8;
    gr551x_temp_handle.init.clock      = ADC_CLK_1P6M;
    hal_adc_init(&gr551x_temp_handle);

    if(SDK_SUCCESS == sys_adc_trim_get(&adc_trim))
    {
        adc_temp = (double)adc_trim.adc_temp;
        adc_slope = (-1) * (double)adc_trim.adc_slope;
    }
    else
    {
        adc_temp = 4979;
        adc_slope = -4932;
    }
    return;
}

double hal_gr551x_temp_read(void)
{
    uint16_t conver_buff[16] = {0};
    uint16_t average = 0;

    /* Got the average of Temp */
    hal_adc_poll_for_conversion(&gr551x_temp_handle, conver_buff, 16);
    for(uint8_t i = 0; i < 8; i++)
    {
        average += conver_buff[8 + i];
    }
    average = average >> 3;
    return (((double)average - adc_temp) / adc_slope) / (-0.00175) + 25.0;
}

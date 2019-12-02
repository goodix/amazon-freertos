/**
 ****************************************************************************************
 *
 * @file gr551x_adc_voltage_api.c
 *
 * @brief GR551x ADC voltage module.
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
#include "gr551x_adc_voltage_api.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

/*
 * STATIC VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static double adc_offset_int = 0;
static double adc_slope_int  = 0;
static double adc_offset_ext = 0;
static double adc_slope_ext  = 0;
static bool adc_trim_read_done = false;

static void load_trim(void);
static double bias_scale[] = {0.635856, 0.757654, 0.879444, 1.000000, 1.122013, 1.243543, 1.364207, 1.485379,\
                              1.607796, 1.730146, 1.851457, 1.974328, 2.093468, 2.210175, 2.331091, 2.452607};

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void hal_gr551x_adc_voltage_intern(adc_handle_t *hadc, uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    double cslope, coffset;

    load_trim();

    if (ADC_INPUT_SINGLE == hadc->init.input_mode)
    {
        coffset = adc_offset_int;
        cslope  = adc_slope_int / bias_scale[hadc->init.ref_value - ADC_REF_VALUE_0P5];
        for (uint32_t i = 0; i < buflen; i++)
        {
            outbuf[i] = ((double)inbuf[i] - coffset) / cslope;
        }
    }
    else
    {
        coffset = 8192 - (adc_offset_int - 8192);
        cslope  = adc_slope_int / bias_scale[hadc->init.ref_value - ADC_REF_VALUE_0P5];
        for (uint32_t i = 0; i < buflen; i++)
        {
            outbuf[i] = (coffset - (double)inbuf[i] * 2) / cslope;
        }
    }
}

void hal_gr551x_adc_voltage_extern(adc_handle_t *hadc, double vref, uint16_t *inbuf, double *outbuf, uint32_t buflen)
{
    double cslope, coffset;

    load_trim();

    if (ADC_INPUT_SINGLE == hadc->init.input_mode)
    {
        coffset = adc_offset_ext;
        cslope  = adc_slope_ext * 1.0f / vref;
        for (uint32_t i = 0; i < buflen; i++)
        {
            outbuf[i] = ((double)inbuf[i] - coffset) / cslope;
        }
    }
    else
    {
        coffset = 8192 - (adc_offset_ext - 8192);
        cslope  = adc_slope_ext * 1.0f / vref;
        for (uint32_t i = 0; i < buflen; i++)
        {
            outbuf[i] = (coffset - (double)inbuf[i] * 2) / cslope;
        }
    }
}

static void load_trim(void)
{
    adc_trim_info_t adc_trim = {0};

    /* Avoid repeated loading during hot start */
    if (false == adc_trim_read_done)
    {
        if(SDK_SUCCESS == sys_adc_trim_get(&adc_trim))
        {
            adc_offset_int = (double)adc_trim.adc_offset;
            adc_slope_int  = (-1) * (double)adc_trim.adc_slope;
            adc_offset_ext = (double)adc_trim.adc_offset_ext;
            adc_slope_ext  = (-1) * (double)adc_trim.adc_slope_ext;
            adc_trim_read_done = true;
        }
        else
        {
            adc_offset_int = 8362;
            adc_slope_int = -4754;
            adc_offset_ext = 8362;
            adc_slope_ext = -4754;
        }
    }
    return;
}

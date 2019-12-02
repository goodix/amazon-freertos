/**
 ****************************************************************************************
 *
 * @file gr551x_adc_voltage_api.h
 *
 * @brief Header file - GR551x ADC voltage module.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */
#ifndef __GR551X_ADC_VOLTAGE_API_H__
#define __GR551X_ADC_VOLTAGE_API_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>
#include "gr55xx_hal_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief  Convert the ADC conversion results to a voltage value(internal reference).
 *
 * @param[in]  hadc: Pointer to a ADC handle which contains the configuration information for
 *                    the specified ADC module.
 * @param[in]  inbuf: Pointer to data buffer which storage ADC conversion results.
 * @param[out] outbuf: Pointer to data buffer which to storage voltage results.
 * @param[in]  length: Length of data buffer,  ranging between 0 and 4095.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
void hal_gr551x_adc_voltage_intern(adc_handle_t *hadc, uint16_t *inbuf, double *outbuf, uint32_t buflen);

/**
 ****************************************************************************************
 * @brief  Convert the ADC conversion results to a voltage value(external reference).
 *
 * @param[in]  hadc: Pointer to a ADC handle which contains the configuration information for
 *                    the specified ADC module.
 * @param[in]  ref: slope of ADC.
 * @param[in]  inbuf: Pointer to data buffer which storage ADC conversion results.
 * @param[out] outbuf: Pointer to data buffer which to storage voltage results.
 * @param[in]  length: Length of data buffer,  ranging between 0 and 4095.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
void hal_gr551x_adc_voltage_extern(adc_handle_t *hadc, double vref, uint16_t *inbuf, double *outbuf, uint32_t buflen);

#ifdef __cplusplus
}
#endif

#endif // __GR551X_ADC_VOLTAGE_API_H__

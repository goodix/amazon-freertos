/**
 ****************************************************************************************
 *
 * @file gr551x_vbat_api.h
 *
 * @brief Header file - GR551x battery module.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */
#ifndef __GR551X_VBAT_API_H__
#define __GR551X_VBAT_API_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief  Initialize ADC battery voltage detection.
 *
 ****************************************************************************************
 */
void hal_gr551x_vbat_init(void);

/**
 ****************************************************************************************
 * @brief  Get the battery voltage.
 *
 * @return The volatge of battery. Unit (volt).
 ****************************************************************************************
 */
double hal_gr551x_vbat_read(void);

#ifdef __cplusplus
}
#endif

#endif // __GR551X_VBAT_API_H__

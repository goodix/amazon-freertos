/**
 ****************************************************************************************
 *
 * @file gr551x_tim_delay.h
 *
 * @brief Header file - GR551x tim delay.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */
#ifndef __GR551X_TIM_DELAY_H__
#define __GR551X_TIM_DELAY_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>
#include "gr55xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief  Initialize the DUAL TIM according to the specified register
 *         in the dual_timer_regs_t.
 ****************************************************************************************
 */
void tim_delay_init(dual_timer_regs_t *timx);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] us:  Microsecond.
 *****************************************************************************************
 */
void tim_delay_us(uint32_t us);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] ms:  Millisecond.
 *****************************************************************************************
 */
void tim_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // __GR551X_TIM_DELAY_H__

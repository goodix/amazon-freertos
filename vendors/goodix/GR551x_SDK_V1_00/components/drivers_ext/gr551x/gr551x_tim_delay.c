/**
 ****************************************************************************************
 *
 * @file gr551x_tim_delay.c
 *
 * @brief GR551x tim delay.
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
#include "gr551x_tim_delay.h"

/*
 * STATIC VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static uint32_t fus = 0;
static uint32_t fms = 0;
static dual_timer_regs_t *tim_regs;

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
void tim_delay_init(dual_timer_regs_t *timx)
{
    fus = SystemCoreClock / 1000000;
    fms = SystemCoreClock / 1000;
    tim_regs = timx;
    tim_regs->RELOAD = 0xFFFFFFFF;
    /* Disable tim, period mode, 32-bit counter, one-shot mdoe */
    tim_regs->CTRL   = 0x43;
}

void tim_delay_us(uint32_t us)
{
    uint32_t load = us * fus - 1;
    tim_regs->RELOAD = load;
    /* Enable tim */
    tim_regs->CTRL  |= 0x80;
    while(tim_regs->VALUE != 0);
    tim_regs->CTRL  &= ~0x80;
    /* Clear flag */
    tim_regs->INTCLR = 1;
}

void tim_delay_ms(uint32_t ms)
{
    uint32_t load = ms * fms - 1;
    tim_regs->RELOAD = load;
    /* Enable tim */
    tim_regs->CTRL  |= 0x80;
    while(tim_regs->VALUE != 0);
    tim_regs->CTRL  &= ~0x80;
    /* Clear flag */
    tim_regs->INTCLR = 1;
}

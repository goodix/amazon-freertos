/**
 *****************************************************************************************
 *
 * @file sensorsim.c
 *
 * @brief sensor simulator function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "sensorsim.h"

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */

void sensorsim_init(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_cfg->start_at_max)
    {
        p_state->current_val   = p_cfg->max;
        p_state->is_increasing = false;
    }
    else
    {
        p_state->current_val   = p_cfg->min;
        p_state->is_increasing = true;
    }
}

int16_t sensorsim_measure(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_state->is_increasing)
    {
        sensorsim_increment(p_state, p_cfg);
    }
    else
    {
        sensorsim_decrement(p_state, p_cfg);
    }

    return p_state->current_val;
}


void sensorsim_increment(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_cfg->max - p_state->current_val > p_cfg->incr)
    {
        p_state->current_val += p_cfg->incr;
    }
    else
    {
        p_state->current_val   = p_cfg->max;
        p_state->is_increasing = false;
    }
}

void sensorsim_decrement(sensorsim_state_t *p_state, const sensorsim_cfg_t *p_cfg)
{
    if (p_state->current_val - p_cfg->min > p_cfg->incr)
    {
        p_state->current_val -= p_cfg->incr;
    }
    else
    {
        p_state->current_val   = p_cfg->min;
        p_state->is_increasing = true;
    }
}


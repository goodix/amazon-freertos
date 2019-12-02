/**
 *****************************************************************************************
 *
 * @file gui_animation_config.c
 *
 * @brief Users should implement the timer-related interface themselves
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

#include "gui_config.h"
#include "gui_animation.h"

#if ANIMATION_EN==1

#include "gr55xx_sys.h"
#define ANIMATION_TIMER_INTERVAL 150
                                                                                  
#ifdef ENV_USE_RTOS
#include "FreeRTOS.h"
#include "timers.h"
static TimerHandle_t timer_handle = NULL;

#else
#include "app_timer.h"
static app_timer_id_t animation_timer_id;
#endif

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static void gui_animation_timerout_handler(void *p_arg)
{
    gui_animation_timer_task();
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 */
void gui_animation_timer_start(void)
{
    #ifndef ENV_USE_RTOS
    app_timer_create(&animation_timer_id, ATIMER_REPEAT, gui_animation_timerout_handler);
    app_timer_start(animation_timer_id, ANIMATION_TIMER_INTERVAL, NULL);
    #else
    timer_handle = xTimerCreate(NULL, (ANIMATION_TIMER_INTERVAL), pdTRUE, NULL, gui_animation_timerout_handler); 
    xTimerStart(timer_handle, 0);
    #endif
}

void gui_animation_timer_stop(void)
{
    #ifndef ENV_USE_RTOS
    app_timer_delete(&animation_timer_id);
    #else
    xTimerDelete(timer_handle, 0);
    #endif
}

#endif


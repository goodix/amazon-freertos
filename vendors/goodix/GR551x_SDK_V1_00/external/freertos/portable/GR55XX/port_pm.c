
/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include <stdio.h>
#include "gr55xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gr55xx_hal.h"
#include "gr55xx_pwr.h"
#include "app_timer.h"
#include "semphr.h"

/*
 * Some kernel aware debuggers require data to be viewed to be global, rather
 * than file scope.
 */
//#define TRACE_IO_TOGGLE

#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
    /* Ensure the SysTick is clocked at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK_BIT  ( 1UL << 2UL )
#else
    /* The way the SysTick is clocked is not modified in case it is not the same
    as the core. */
    #define portNVIC_SYSTICK_CLK_BIT  ( 0 )
#endif


///* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG           ( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG           ( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG  ( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG                ( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
///* ...then bits in the registers. */
#define portNVIC_SYSTICK_INT_BIT            ( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT         ( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT     ( 1UL << 16UL )

static app_timer_id_t aon_timer_handle = 0;
static uint32_t aon_timer_flag = 0x0;
static uint32_t save_curr_count = 0;
static uint8_t  timer_init_flag = 0;
static int trans_ticks;


/**
 *****************************************************************************************
 * @brief vPortExitDeepSleep 
 *
 * @return void
 *****************************************************************************************
 */
void vPortExitDeepSleep(void)
{ 
    /* Restart from whatever is left in the count register to complete
    this tick period. */
    portNVIC_SYSTICK_LOAD_REG = save_curr_count;//portNVIC_SYSTICK_CURRENT_VALUE_REG;

    /* Restart SysTick. */
    portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
    portNVIC_SYSTICK_CTRL_REG |= ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );
    /* Reset the reload register to the value required for normal tick
    periods. */
    unsigned int CountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
    portNVIC_SYSTICK_LOAD_REG = CountsForOneTick - 1UL;
}

/**
 *****************************************************************************************
 * @brief aon_timer_build_handler
 *
 * @param[in] arg: pointer of args
 *
 * @return void
 *****************************************************************************************
 */
static void aon_timer_build_handler(void* p_ctx)
{
    aon_timer_flag = 0x1;
}

/**
 *****************************************************************************************
 * @brief vPortEnterDeepSleep
 *
 * @param[in] xExpectedIdleTime: next task resume time to work
 *
 * @return void
 *****************************************************************************************
 */
int pwr_mgmt_enter_sleep_with_cond(int ticks_to_up)
{  
      if (0<ticks_to_up && ticks_to_up<10)
      {
          return 0;
      }

      pwr_mgmt_save_context();

      if (pwr_mgmt_get_wakeup_flag() == COLD_BOOT)
      {
            trans_ticks = (int)ticks_to_up;
          
            if (trans_ticks>0)
            {
                aon_timer_flag = 0x0;
                if (timer_init_flag == 0)
                {
                    app_timer_create( &aon_timer_handle, ATIMER_ONE_SHOT, aon_timer_build_handler);
                    timer_init_flag = 0x1;
                }
                app_timer_start(aon_timer_handle, trans_ticks, NULL);
            }
            
            __disable_irq();
            #ifdef TRACE_IO_TOGGLE
            point_at(4);
            #endif
            pwr_mgmt_shutdown();
      }
      else //wekeup from deep sleep mode
      {
          //clear wakeup mark ,parpare next time enter-sleep action
          pwr_mgmt_set_wakeup_flag(COLD_BOOT);
          if (aon_timer_flag)
          {
              #ifdef TRACE_IO_TOGGLE
              point_at(6);
              #endif
              return trans_ticks - 1;
          }
          else
          {
              #ifdef TRACE_IO_TOGGLE             
              point_at(7);
              #endif
              int ticks = app_timer_stop_and_ret(aon_timer_handle);
              if (ticks>1000)
              {   
                  ticks /= 1000;
                  return ticks;                  
              }
          }
      }
      return -1;
}

void vPortEnterDeepSleep( TickType_t xExpectedIdleTime )
{
    static int rest_ticks;
    NVIC_SetPendingIRQ(BLE_SDK_IRQn);
    NVIC_EnableIRQ(BLE_SDK_IRQn);
    __disable_irq();
    int ret = pwr_mgmt_get_sleep_mode();
    if (ret == PMR_MGMT_SLEEP_MODE)
    {
       portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;
       save_curr_count = portNVIC_SYSTICK_CURRENT_VALUE_REG;
       rest_ticks = pwr_mgmt_enter_sleep_with_cond(xExpectedIdleTime);
       if (rest_ticks > 0)
       {
           vTaskStepTick( rest_ticks );
       }
       /* Restart from whatever is left in the count register to complete this tick period. */
       portNVIC_SYSTICK_LOAD_REG = save_curr_count;//portNVIC_SYSTICK_CURRENT_VALUE_REG;
       /* Restart SysTick. */
       portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
       portNVIC_SYSTICK_CTRL_REG |= ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );
       /* Reset the reload register to the value required for normal tick periods. */
       uint32_t CountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
       portNVIC_SYSTICK_LOAD_REG = CountsForOneTick - 1UL;
    } 
    __enable_irq();
}

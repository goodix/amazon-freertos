
/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include <stdio.h>
#include "gr55xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gr55xx_hal.h"
#include "gr55xx_sys.h"
#include "app_timer.h"
#include "semphr.h"

static app_timer_id_t s_rtos_timer_hdl = 0;

typedef enum
{
   AON_TIMER_READY = 0,
   AON_TIMER_RUN_FINISH,    
}RTOS_TIMER_RUN_STATE_t;

static volatile RTOS_TIMER_RUN_STATE_t s_aon_timer_build_finish = AON_TIMER_READY;

uint32_t vPortLocker(void)
{
   uint32_t ret_pri = __get_PRIMASK();
   __set_PRIMASK(1); 
   return ret_pri;
}

void vPortUnLocker(uint32_t set_pri)
{
   __set_PRIMASK(set_pri); 
}

static inline void SysTickStop(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

static inline void SysTickStart(void)
{
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void SysTickReload(uint32_t CyclesPerTick)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->LOAD  = (uint32_t)(CyclesPerTick - 1UL);                 /* set reload register */
    SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk \
                  | SysTick_CTRL_CLKSOURCE_Msk \
                  | SysTick_CTRL_TICKINT_Msk;
}

static void aon_timer_build_handler(void* p_ctx)
{
    s_aon_timer_build_finish = AON_TIMER_RUN_FINISH;
}



extern pwr_mgmt_mode_t __attribute__((section("RAM_CODE"))) pwr_mgmt_check_timer(void);
static void pwr_mgmt_enter_sleep_with_cond(int pwr_mgmt_expected_time)
{  
      static int s_total_sleep_ticks = 0;
      
      /* Save the context of RTOS. */
      pwr_mgmt_save_context();
      
      /* Judge the current startup mode mark. */
      if (pwr_mgmt_get_wakeup_flag() == COLD_BOOT)
      {  
            s_total_sleep_ticks = pwr_mgmt_expected_time;
          
            /* Open app timer only when there is a specific next task to execute. */
            if (s_total_sleep_ticks > 0)
            {
                /* Only create the timer once. */
                static uint8_t timer_init_flag = 0;
                if ( !timer_init_flag )
                {
                    app_timer_create( &s_rtos_timer_hdl, ATIMER_ONE_SHOT, aon_timer_build_handler);
                    timer_init_flag = 0x1;
                }
                
                /* Set this flag to ready. */
                s_aon_timer_build_finish = AON_TIMER_READY;
               
                /* To start the app timer. */
                app_timer_start(s_rtos_timer_hdl, s_total_sleep_ticks-1, NULL);
            }

            /* To disbale global IRQ. */
            uint32_t _local_lock = vPortLocker();
            
            /* To stop the systick. */
            SysTickStop();
            
            /* Shutdown all system power and wait some event to wake up. */
            pwr_mgmt_shutdown();
            
            /* To stop the app timer. */
            app_timer_stop(s_rtos_timer_hdl);
           
            /* To start the systick. */
            SysTickStart();
            
            /* To enable global IRQ. */
            vPortUnLocker(_local_lock);

      }
      else //wekeup from deep sleep mode
      {
          /* clear wakeup mark ,parpare next time enter-sleep action. */
          pwr_mgmt_set_wakeup_flag(COLD_BOOT);
          
          /* To disbale global IRQ. */
          uint32_t _local_lock = vPortLocker();
          
          /* Restart SysTick module. */
          SysTickReload( configCPU_CLOCK_HZ / configTICK_RATE_HZ);
          
          /* Check whether it is another wake-up source. */
          if ( AON_TIMER_RUN_FINISH == s_aon_timer_build_finish )
          {
              /* Compensation for system counter of RTOS. */
              vTaskStepTick( s_total_sleep_ticks );
          }
          else
          {
              
              /* To get the elapsed time from app-timer. */
              int ticks =  app_timer_stop_and_ret(s_rtos_timer_hdl);
              if (ticks>1000)  
                  ticks /= 1000;
              else
                  ticks = 1;
              
              /* Compensation for system counter of RTOS. */
              vTaskStepTick( ticks );
          }
          
          /* To enable global IRQ. */
          vPortUnLocker(_local_lock);
      }
      return ;
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
void vPortEnterDeepSleep( TickType_t xExpectedIdleTime )
{

    if (!sys_ke_sleep_check())
    {
       NVIC_SetPendingIRQ(BLE_SDK_IRQn);
       NVIC_EnableIRQ(BLE_SDK_IRQn);
    }
    
    if ( 0 < xExpectedIdleTime && xExpectedIdleTime < 5 )
    {
       pwr_mgmt_wfe_sleep();
       return ;        
    }
    
    uint32_t _local_lock = vPortLocker();
    pwr_mgmt_mode_t ret = pwr_mgmt_get_sleep_mode();
    vPortUnLocker(_local_lock);
    
    if (!pwr_mgmt_check_pend_irq())
    {
        if (ret == PMR_MGMT_SLEEP_MODE)
        {
            pwr_mgmt_enter_sleep_with_cond(xExpectedIdleTime);
        } 
        else
        {
            pwr_mgmt_wfe_sleep();
        }
    } 
}

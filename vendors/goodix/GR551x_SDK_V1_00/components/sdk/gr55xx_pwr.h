/**
 *****************************************************************************************
 *
 * @file gr55xx_pwr.h
 *
 * @brief GR55XX Platform Power Manager Module API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/**
 * @addtogroup SYSTEM
 * @{
 */
 
/**
 * @addtogroup PWR Power Manager
 * @{
 * @brief Definitions and prototypes for the Power Manager interface.
 */


#ifndef __GR55XX_PWR_H_
#define __GR55XX_PWR_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "system_gr55xx.h"

/**
 * @defgroup GR55XX_PWR_TYPEDEF Typedefs
 * @{
 */

/**@brief power manager setting parameter.
 *        Use pwr_mgmt_var_set to transfer the parameters in the structure to PMU, 
 *        and then the pwr_mgmt_mode_set function will use the new parameters for
 *        power management.
 *        Note that this is an advanced API, the wrong setting of parameters may 
 *        lead to abnormal power management, so please use it carefully.
 */
typedef struct
{
    uint32_t  pwr_mgmt_app_timer_thrd;
    uint32_t  pwr_mgmt_ble_core_thrd;
    uint32_t  pwr_mgmt_rtc_timer_thrd;
} pwr_mgmt_var_box_t;

/**@brief power manager boot type. */
typedef enum 
{
    COLD_BOOT = 0,                    /**< Cold boot state. */
    WARM_BOOT,                        /**< Warm boot state. */
} boot_mode_t;

/**@brief power manager model. */
typedef enum
{
    PMR_MGMT_ACTIVE_MODE = 0x0,       /**< Full speed state. */
    PMR_MGMT_IDLE_MODE,               /**< Idle state. */
    PMR_MGMT_SLEEP_MODE,              /**< Deep sleep state. */
} pwr_mgmt_mode_t;

/**@brief power manager device work state. */
typedef enum
{
    DEVICE_BUSY = 0x0,                /**< Device busy state. */
    DEVICE_IDLE,                      /**< Device idle state. */
} pwr_mgmt_dev_state_t;

/**@brief power manager app timer work state. */
typedef enum
{
    EVENT_APP_TIMER_START = 0,        /**< App-timer start state. */
    EVENT_APP_TIMER_STOP,             /**< App-timer stop state. */
} notify_timer_event_t;

/**@brief parameter configuration table. */ 
typedef struct 
{
   uint16_t pwr_dur;
   uint16_t pwr_ext;    
   uint16_t pwr_osc;
   uint8_t  pwr_delay_hslot;
   uint16_t pwr_delay_hus;
   uint16_t pwr_push_hus;
   uint32_t pwr_timer_ths;
   uint32_t pwr_ble_ths;
} pwr_table_t; 

/**@brief Trace function type. */ 
typedef void (*trace_func_t)(uint8_t);

/**@brief Peripheral function type. */ 
typedef void (*periph_func_t)(void);

/**@brief Before sleep function type. */ 
typedef void (*pwr_before_sleep_func_t)(void);

/**@brief Device check function type. */ 
typedef bool (*pwr_dev_check_func_t)(void);

/**@brief pwr table. */ 
extern pwr_table_t pwr_table[];
 
/** @} */

/** @addtogroup GR55XX_PWR_FUNCTIONS Functions
 * @{ */
/**
 *****************************************************************************************
 * @brief This function allows ARM to enter deep sleep mode, but users should not use this 
 *        function directly.
 *        Note that this function is only available in environments where non-RTOS is used, 
 *             and that users can only execute it while in main.c.
 *****************************************************************************************
 */
void pwr_mgmt_shutdown(void);

/**
 ****************************************************************************************
 * @brief  Get the current boot mode.     
 * @retval : cold boot or warm boot.
 ****************************************************************************************
 */
boot_mode_t pwr_mgmt_get_wakeup_flag(void);

/**
 ****************************************************************************************
 * @brief  Mark the mode of next boot, cold boot or warm boot.
 * @param[in] boot_mode : cold boot or warm boot.
 * @retval : void
 ****************************************************************************************
 */
void pwr_mgmt_set_wakeup_flag(boot_mode_t boot_mode);

/**
 ****************************************************************************************
 * @brief  Set the specified sleep mode. When the setting is completed, the system will
 *         automatically enter the specified sleep mode through the strategy.  
 * @param[in] pm_mode : sleep level
 * @retval : void
 ****************************************************************************************
 */
void pwr_mgmt_mode_set(pwr_mgmt_mode_t pm_mode);

/**
 ****************************************************************************************
 * @brief       Get the specified sleep mode.   
 * @retval    : pwr_mgmt_mode_t
 ****************************************************************************************
 */
pwr_mgmt_mode_t pwr_mgmt_mode_get(void);

/**
 ****************************************************************************************
 * @brief  Sleep Policy Scheduling Function.
 *         Note that this function is only available in environments where non-RTOS is used, 
           and that users can only execute it while in main.c.
 * @retval : void
 ****************************************************************************************
 */
void pwr_mgmt_schedule(void);

/**
 ****************************************************************************************
 * @brief       Wake the BLE core via an external request.
 * @return      status
 * @retval      The status of the requested operation.
 *              
 *              false, if the BLE core is not sleeping.
 *              true,  if the BLE core was woken up successfully.
 *              
 ****************************************************************************************
 */
bool pwr_mgmt_ble_wakeup(void);

/**
 ****************************************************************************************
 * @brief  This function is used to push startup information in app timer. 
 *         This information will optimize power management strategy. 
 *         Note that this function is an advanced API and users should not use it directly.
 * @param[in] timer_event :  EVENT_APP_TIMER_START or EVENT_APP_TIMER_STOP
 * @retval : void
 ****************************************************************************************
 */
void pwr_mgmt_notify_timer_event(notify_timer_event_t timer_event);

/**
 ****************************************************************************************
 * @brief  Query the sleep mode that the current system can access.
 * @retval : void
 ****************************************************************************************
 */
pwr_mgmt_mode_t pwr_mgmt_get_sleep_mode(void);


/**
 ****************************************************************************************
 * @brief  Update wakeup param.
 * @retval : void
 ****************************************************************************************
 */
void pwr_mgmt_update_wkup_param(void);

/**
 ****************************************************************************************
 * @brief  Execution of this function allows ARM to enter the WFE state and exit the WFE 
 *         state when an event or interrupt occurs.
 * @retval : void
 ****************************************************************************************
 */
void pwr_mgmt_wfe_sleep(void);

/**
 ****************************************************************************************
 * @brief    PMU Initialization Function.
 * @param    p_pwr_table      : PMU parameter configuration table.
 * @param    sys_clk          : the clock of system
 * @return   void
 ****************************************************************************************
 */
void pwr_mgmt_init( pwr_table_t *p_pwr_table, mcu_clock_type_t sys_clk);

/**
 ****************************************************************************************
 * @brief    Peripheral Controller Initialization Register interface.
 * @param    p_periph_init      :  the pointer of device init function.
 * @return   void
 ****************************************************************************************
 */
void pwr_mgmt_dev_init(periph_func_t p_periph_init);

/**
 ****************************************************************************************
 * @brief    Device config resume interface.
 * @return   void
 ****************************************************************************************
 */
void pwr_mgmt_dev_resume(void);

/**
 ****************************************************************************************
 * @brief    Mem state control under deep sleep & work state.
 * @param    mem_sleep_state  : control in deep sleep.
 * @param    mem_work_state   : control in work state.
 * @return   void
 ****************************************************************************************
 */
void pwr_mgmt_mem_ctl_set(uint32_t mem_sleep_state, uint32_t mem_work_state);

/**
 ****************************************************************************************
 * @brief    Set PMU callback function.
 * @param    dev_check_fun    : Device check callback function.
 * @param    before_sleep_fun : Pre-execution callback function for deep sleep.
 * @return   void
 ****************************************************************************************
 */
void pwr_mgmt_set_callback(pwr_dev_check_func_t dev_check_fun, pwr_before_sleep_func_t before_sleep_fun);

 /**
 ****************************************************************************************
 * @brief  Set the wakeup source.
 * @param[in] wakeup_source : 
 *            PWR_WKUP_COND_EXT      
 *            PWR_WKUP_COND_TIMER  
 *            PWR_WKUP_COND_BLE 
 *            PWR_WKUP_COND_CALENDAR 
 * @retval :  void
 ****************************************************************************************
 */
void pwr_mgmt_wakeup_source_setup(uint32_t wakeup_source);

 /**
 ****************************************************************************************
 * @brief  Clear the wakeup source.
 * @param[in] wakeup_source : 
 *            PWR_WKUP_COND_EXT      
 *            PWR_WKUP_COND_TIMER  
 *            PWR_WKUP_COND_BLE 
 *            PWR_WKUP_COND_CALENDAR 
 * @retval :  void
 ****************************************************************************************
 */
void pwr_mgmt_wakeup_source_clear(uint32_t wakeup_source);

 /**
 ****************************************************************************************
 * @brief  Save context function.
 * @retval :  void
 ****************************************************************************************
 */
void pwr_mgmt_save_context(void);

 /**
 ****************************************************************************************
 * @brief  Load context function.
 * @retval :  void
 ****************************************************************************************
 */
void pwr_mgmt_load_context(void);

 /**
 ****************************************************************************************
 * @brief  Disable nvic irq.
 * @retval :  void
 ****************************************************************************************
 */
void pwr_mgmt_disable_nvic_irq(void);

/**
 ****************************************************************************************
 * @brief  Enable nvic irq.
 * @retval :  void
 ****************************************************************************************
 */
void pwr_mgmt_enable_nvic_irq(void);

/**
 ****************************************************************************************
 * @brief  Check nvic irq.
 * @retval :  void
 ****************************************************************************************
 */
bool pwr_mgmt_check_pend_irq(void);


 /**
 ****************************************************************************************
 * @brief  PMU Tracking Function
 ****************************************************************************************
 */
enum
{
   TRC_PWR_WFE_MODE = 0,
   TRC_PWR_DSLEEP_MODE,
   TRC_PWR_ACTIVE_MODE,
   TRC_PWR_BLE_RET_DSLEEP,
   TRC_PWR_APP_TIMER_REFUSE,
   TRC_PWR_APP_TIMER_PASS,    
   TRC_PWR_BLE_TIMER_PASS,    
};
void pwr_mgmt_register_trace_func(trace_func_t trace_func);

/** @} */

#endif
/** @} */
/** @} */

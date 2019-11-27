/**
 ******************************************************************************
 *
 * @file platform_sdk.h
 *
 * @brief NVDS API
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix  Technology Co., Ltd
 * All Rights Reserved
 *
 ******************************************************************************
 */
 
 
/**
 * @addtogroup SYSTEM
 * @{
 */
 /**
  @addtogroup Plat_SDK Platform SDK
  @{
  @brief Definitions and prototypes for the Platform SDK
 */

#ifndef _PLATFORM_SDK_H
#define _PLATFORM_SDK_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "system_gr55xx.h"

/**
 * @defgroup  PlAT_SDK_DEFINES Defines
 * @{
 */

/**@brief IO PINs for leakage protection. */
#define GPIO_PIN(x)                 ( 1<< (x) )
#define AON_IO_PIN(x)               ( 1<< (x) )
#define MSIO_PIN(x)                 ( 1<< (x) )

 /** @} */

/**@addtogroup PlAT_SDK_ENUM Enumerations
 * @{ */

/**@brief system clock and run mode. */
typedef enum
{
   XIP_64M = 0,
   XIP_48M,   
   XIP_XO16M,  
   XIP_24M, 
   XIP_16M, 
   XIP_32M,
   MIRROR_64M,
   MIRROR_48M,    
   MIRROR_XO16M,
   MIRROR_24M, 
   MIRROR_16M, 
   MIRROR_32M,
} table_idx_t;

/**@brief sdk clock type. */
typedef enum
{
    RNG_OSC_CLK = 0,
    RTC_OSC_CLK,
    RNG_OSC_CLK2,
} sdk_clock_type_t;


/**@brief memory power setting mode. */
typedef enum
{
   MEM_POWER_FULL_MODE = 0,
   MEM_POWER_AUTO_MODE,    
} mem_power_t;
 /** @} */
 
/**@addtogroup PlAT_SDK_STRUCT Structures
 * @{ */ 
/**
 ****************************************************************************************
 * @brief   IO leakage protection config table.
 * The three fields are defined in bitmap format, each bit is corresponding to each IO.
 * The configuration rules are as below.
 * 1. input to GR551x with on-board PU or PD resistors.
 * 2. output from GR551x.
 * IO setting must be set properly according to board configurations to avoid leackage
 * during sleep.
 ****************************************************************************************
 */
typedef struct
{
   uint32_t gpio;
   uint8_t  aon_gpio;
   uint8_t  msio;
} io_table_t;

 /** @} */

/** @addtogroup PLAT_SDK_FUNCTIONS Functions
 * @{ */

/**
 ****************************************************************************************
 * @brief   platform sdk init function.
 * @retval :  void
 ****************************************************************************************
 */
void platform_sdk_init(void);

/**
 ****************************************************************************************
 * @brief  set the memory power state.
 * @param[in] mem_pwr_mode : FULL POWER setting or AUTO POWER setting.
 * @retval : void
 ****************************************************************************************
 */
void system_lp_mem_mode_set(mem_power_t mem_pwr_mode);

/**
 ****************************************************************************************
 * @brief  enable the power status of specified memory address.
 * @param[in] addr : memory address.
 * @param[in] length : memory length.
 * @retval : void
 ****************************************************************************************
 */
void system_mem_power_enable(uint32_t addr, uint32_t length);

/**
 ****************************************************************************************
 * @brief  update the counter A and counter B.
 * @param[in] cnt_a :  Start Index Number.
 * @param[in] cnt_b :  End Index Number.
 * @retval :  void
 ****************************************************************************************
 */
void system_lp_counter_set(uint8_t cnt_a, uint8_t cnt_b);

/**
 ****************************************************************************************
 * @brief  Enable patch function.
 * @param[in] table_idx :  Start Index Number.
 * @param[in] dur_offset   :  duration setting.
 * @param[in] ext_offset   :  ext wakeup setting.
 * @param[in] osc_offset   :  pre-wakeup setting.
 * @retval :  void
 ****************************************************************************************
 */
void system_lp_table_update_twval(table_idx_t table_idx, int16_t dur_offset, int16_t ext_offset, int16_t osc_offset);

/**
 ****************************************************************************************
 * @brief  Leakage Protection for User's IO.
 *         The leakage protection only acts on the outside without pull-up and pull-down,
 *         and only for input mode IO. By default, flash-related IO has been internally processed.
 *         For instance : 
 *         io_table_t io_table =
 *         {
 *             .gpio     = GPIO_PIN(0) | GPIO_PIN(1),
 *             .aon_gpio = AON_GPIO_PIN(0) | AON_GPIO_PIN(1),
 *             .msio     = MSIO_PIN(0) | MSIO_PIN(1),
 *         };
 * @param[in] p_io_table : the potiner of io setting table.
 * @retval :  void
 ****************************************************************************************
 */
void system_io_leakage_protect(io_table_t *p_io_table);

/**
 ****************************************************************************************
 * @brief  Platform low power clock init function.
 * @param[in] clock     :  External RTC setting or internal RNG/RNG2 setting.
 * @param[in] accuracy  :  Low speed clock accuracy.
 * @param[in] xo_offset :  Clock calibration parameter.
 * @retval :  void
 ****************************************************************************************
 */
void platform_clock_init(mcu_clock_type_t sys_clock, sdk_clock_type_t clock, uint16_t accuracy, uint16_t xo_offset);

/**
 ****************************************************************************************
 * @brief  Platform rc calibration function.
 * @retval :  void
 ****************************************************************************************
 */
void platform_rc_calibration(void);

/**
 ****************************************************************************************
 * @brief  Platform init function.
 * @retval :  void
 ****************************************************************************************
 */
void platform_init(void);

/**
 ****************************************************************************************
 * @brief  PMU init function.
 * @param[in] clock_type :  clock type to be configured.
 * @retval : void
 ****************************************************************************************
 */
void system_pmu_init(mcu_clock_type_t clock_type);

/**
 ****************************************************************************************
 * @brief  PMU deinit function.
 * @retval : void
 ****************************************************************************************
 */
void system_pmu_deinit(void);

/**
 ****************************************************************************************
 * @brief  Warm boot process.
 * @retval :  void
 ****************************************************************************************
 */
void warm_boot(void);

/**
 ****************************************************************************************
 * @brief  PMU calibration handler.     
 * @param[in] p_arg : no args.
 * @retval :  void
 ****************************************************************************************
 */
void pmu_calibration_handler(void* p_arg);


/**
 ****************************************************************************************
 * @brief  start calibration.     
 * @param[in] interval : the interval of calibration process.
 * @retval :  void
 ****************************************************************************************
 */
void system_pmu_calibration_start(uint32_t interval);

/**
 ****************************************************************************************
 * @brief  stop calibration.     
 * @retval :  void
 ****************************************************************************************
 */
void system_pmu_calibration_stop(void);


/** @} */

#endif

/** @} */
/** @} */


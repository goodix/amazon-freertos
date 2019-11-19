/**
 ****************************************************************************************
 *
 * @file gr55xx_sys_cfg.h
 *
 * @brief Define the chip configuration
 *
 * Copyright(C) 2016-2018, Shenzhen Huiding Technology Co., Ltd
 * All Rights Reserved
 *
 ****************************************************************************************
 */

#ifndef __GR55XX_SYS_CFG_H__
#define __GR55XX_SYS_CFG_H__

#include <cmsis_compiler.h>
#define __ARRAY_EMPTY

/*
 *****************************************************************************
 * Gr55xx Chip configure defination BEGIN
 *****************************************************************************
 */
// BLE Sleep configure defination
typedef struct
{
    /// Sleep enable flag
    uint8_t sleep_enable;

    /// External wake-up support
    uint8_t ext_wakeup_enable;

    /// Twosc delay
    uint16_t twosc;

    /// Twext delay
    uint16_t twext;

    /// Twrm delay
    uint16_t twrm;

    /// Duration of sleep and wake-up algorithm (depends on CPU speed) expressed in half us.
    uint16_t sleep_algo_dur;

} ble_slp_config_t ;

// BLE Scheduler configure defination
typedef struct
{
    /// Programme delay
    uint8_t prog_delay;

} ble_sch_config_t;

/// GR55XX Chip configure defination
typedef struct
{
    /// BLE Sleep configure
    ble_slp_config_t ble_slp_cfg;

    /// BLE Sch configure
    ble_sch_config_t ble_sch_cfg;

} gr55xx_chip_config_t;

/*
 *****************************************************************************
 * Gr55xx Chip configure defination END
 *****************************************************************************
 */

#endif

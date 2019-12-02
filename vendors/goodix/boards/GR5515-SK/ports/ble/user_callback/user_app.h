/**
 *****************************************************************************************
 *
 * @file user_app.h
 *
 * @brief Header file - User Function
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */
#ifndef __USER_APP_H__
#define __USER_APP_H__

#include "gr55xx_sys.h"
//#include "ble_prf_types.h"
#include "custom_config.h"

/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief This callback will be called when ble stack initialized completely 
 *****************************************************************************************
 */
void ble_init_cmp_callback(void);




#if defined (__CC_ARM)


#define RW_RAM_ENV_HEAP_ADDR          &Image$$RW_RAM_ENV_HEAP$$Base
#define RW_RAM_ATT_DB_HEAP_ADDR       &Image$$RW_RAM_ATT_DB_HEAP$$Base
#define RW_RAM_MSG_HEAP_ADDR          &Image$$RW_RAM_MSG_HEAP$$Base
#define RW_RAM_NON_RET_HEAP_ADDR      &Image$$RW_RAM_NON_RET_HEAP$$Base

#elif defined ( __GNUC__ )

extern char RW_RAM_ENV_HEAP[];
extern char RW_RAM_ATT_DB_HEAP[];
extern char RW_RAM_MSG_HEAP[];
extern char RW_RAM_NON_RET_HEAP[];

#define RW_RAM_ENV_HEAP_ADDR          &RW_RAM_ENV_HEAP
#define RW_RAM_ATT_DB_HEAP_ADDR       &RW_RAM_ATT_DB_HEAP
#define RW_RAM_MSG_HEAP_ADDR          &RW_RAM_MSG_HEAP
#define RW_RAM_NON_RET_HEAP_ADDR      &RW_RAM_NON_RET_HEAP

#elif  defined ( __ICCARM__ )

#pragma section = "RW_RAM_ENV_HEAP"
#pragma section = "RW_RAM_ATT_DB_HEAP"
#pragma section = "RW_RAM_MSG_HEAP"
#pragma section = "RW_RAM_NON_RET_HEAP"

#define RW_RAM_ENV_HEAP_ADDR          __section_begin("RW_RAM_ENV_HEAP")
#define RW_RAM_ATT_DB_HEAP_ADDR       __section_begin("RW_RAM_ATT_DB_HEAP")
#define RW_RAM_MSG_HEAP_ADDR          __section_begin("RW_RAM_MSG_HEAP")
#define RW_RAM_NON_RET_HEAP_ADDR      __section_begin("RW_RAM_NON_RET_HEAP")

#endif

#endif


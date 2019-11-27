/**
 ****************************************************************************************
 *
 * @file custom_config.h
 *
 * @brief Custom configuration file for applications.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the name of GOODIX nor the names of its contributors may be used
    to endorse or promote products derived from this software without
    specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************************
 */

/*
 * DEFINES
 *****************************************************************************************
 */
#ifndef __CUSTOM_CONFIG_H__
#define __CUSTOM_CONFIG_H__

// <<< Use Configuration Wizard in Context Menu >>>

// <h> Basic configuration

// <o> Enable hardfault callstack info print
// <0=> DISABLE
// <1=> ENABLE
#ifndef HARDFAULT_TRACE_ENABLE
#define HARDFAULT_TRACE_ENABLE  1
#endif

// <o> Enable app driver module
// <0=> DISABLE
// <1=> ENABLE
#ifndef APP_DRIVER_USE_ENABLE
#define APP_DRIVER_USE_ENABLE   1
#endif

// <o> Eanble APP log module
// <0=> DISABLE
// <1=> ENABLE
#ifndef APP_LOG_ENABLE
#define APP_LOG_ENABLE          1
#endif

// <o> Enable SK GUI module
// <0=> DISABLE
// <1=> ENABLE
#ifndef SK_GUI_ENABLE
#define SK_GUI_ENABLE           1
#endif

// <o> Enable DTM test support
// <0=> DISABLE
// <1=> ENABLE
#ifndef DTM_TEST_ENABLE
#define DTM_TEST_ENABLE         0
#endif

// <o> Enable BLE DFU support
// <0=> DISABLE
// <1=> ENABLE
#ifndef DFU_ENABLE
#define DFU_ENABLE              0
#endif

// <o> The start address of NVDS
//  <i>Default: 0x0107E000
#define NVDS_START_ADDR         0x010FC000

// <o> The Number of sectors for NVDS
// <i> Default:  1
#ifndef NVDS_NUM_SECTOR
#define NVDS_NUM_SECTOR         1
#endif

// <o> Call Stack Size
// <i> Default: 0x8000
#ifndef CSTACK_HEAP_SIZE
#define CSTACK_HEAP_SIZE        0x8000
#endif

// <o> RAM size of Application
// <i> Default: 0x00030000
#ifndef APP_RAM_SIZE
#define APP_RAM_SIZE            0x00030000
#endif

// <o> Code size of Application
// <i> Default: 0x00800000
#ifndef APP_MAX_CODE_SIZE
#define APP_MAX_CODE_SIZE       0x00800000
#endif
// </h>

// <h> Boot info configuration

// <o> Code load address
// <i> Default:  0x01002000
#define APP_CODE_LOAD_ADDR      0x01002000

// <o> Code run address
// <i> Default:  0x01002000
#define APP_CODE_RUN_ADDR       0x01002000

// <ol.0..5> System clock
// <0=> 64MHZ
// <1=> 48MHZ
// <2=> 16MHZ-XO
// <3=> 24MHZ
// <4=> 16MHZ
// <5=> 32MHZ-CPLL
#define SYSTEM_CLOCK            0

// <o> External clock accuracy used in the LL to compute timing  <1-500>
// <i> Range: 1-500
#define CFG_LF_ACCURACY_PPM     500

// <o> Delay time for Boot startup
// <0=> Not Delay
// <1=> Delay 1s
#define BOOT_LONG_TIME          1

// <o> Code version.16bits
#define VERSION                 1

// <o> DAP boot mode
// <0=> DISABLE
// <1=> ENABLE
#define DAP_BOOT_ENABLE         1
// </h>

// <h> ble resource configuration

// <o> Support maximum number of BLE profile <1-64>
// <i> Range: 1-64
#ifndef CFG_MAX_PRF_NB
#define CFG_MAX_PRF_NB          10
#endif

// <o> Support maximum number of bonded devices <1-10>
// <i> Range: 1-10
#ifndef CFG_MAX_BOND_DEV_NUM
#define CFG_MAX_BOND_DEV_NUM    4
#endif

// <o> Support maximum number of BLE link <1-10>
// <i> Range: 1-10
#ifndef CFG_MAX_CONNECTIONS
#define CFG_MAX_CONNECTIONS     10
#endif
// </h>

// <h> mesh support configuration
// <o> mesh support
// <0=> NOT SUPPORT
// <1=> SUPPORT
#ifndef CFG_MESH_SUPPORT
#define CFG_MESH_SUPPORT        0
#endif
// </h>

// <<< end of configuration section >>>
#endif //__CUSTOM_CONFIG_H__

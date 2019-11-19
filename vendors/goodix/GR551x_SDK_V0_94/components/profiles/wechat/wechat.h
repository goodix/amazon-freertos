/**
 *******************************************************************************
 *
 * @file wechat.h
 *
 * @brief wechat Service API.
 *
 *******************************************************************************
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

/**
 * @addtogroup BLE_SRV BLE Services
 * @{
 * @brief Definitions and prototypes for the BLE Service interface.
 */
/**
 * @defgroup BLE_SDK_WECHAT Wechat (WECHAT)
 * @{
 * @brief WECHAT Interface module.
 *
 * @details 
 */

#ifndef __WECHART_H__
#define __WECHART_H__

#include "gr55xx_sys.h"
#include "custom_config.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup WECHAT_MACRO Defines
 * @{
 */
#define WECHAT_CONNECTION_MAX               (10 < CFG_MAX_CONNECTIONS ?\
                                             10 : CFG_MAX_CONNECTIONS)   /**< Maximum number of WECHAT connections.*/
/** @} */

/**
 * @defgroup WECHAT_ENUM Enumerations
 * @{
 */
/**@brief wechat Service environment variable. */
typedef struct
{
    uint8_t  char_mask;
    uint16_t start_hdl;
    uint16_t pedo_ntf_cfg;
    uint16_t target_ntf_cfg;
    uint8_t  device_mac[6];
} wechat_env_t;

/**@brief wechat pedometer environment variable. */
typedef struct
{
    char flag;
    char step_count[3];
    char step_dist[3];
    char step_calorie[3];

} CURR_PEDO_t;

/**@brief wechat target environment variable. */
typedef struct
{
    char flag;
    char step_count[3];
} TARGET_t;

/**@brief wechat data environment variable. */
typedef struct
{
    uint8_t *data;
    uint16_t len;
    uint16_t offset;
} data_info;

/** @} */

/**
 * @defgroup wechat Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief Add a wechat Service instance in the DB.
 *
 * @param[in] wechat_env_t Pointer to a wechat Service environment variable.
 *
 * @return Result of service initialization.
 *****************************************************************************************
 */
sdk_err_t wechat_service_add(wechat_env_t *p_wechat_env);

/**
 *****************************************************************************************
 * @brief The interface sends several subpackages that are split up to the client in succession.
 *
 * @param[in] p_data : Pointer to the parameters of the write request.
 * @param[in] length : data length
 *
 * @return If the request was consumed or not.
 *****************************************************************************************
 */
int ble_wechat_indicate_data(uint8_t *p_data, uint32_t length);
/** @} */

#endif
/** @} */
/** @} */

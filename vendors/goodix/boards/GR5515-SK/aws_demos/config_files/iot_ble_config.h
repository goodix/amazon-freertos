/*
 * Amazon FreeRTOS V1.4.2
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/**
 * @file iot_ble_config.h
 * @brief BLE configuration overrides for GR5515-sk board.
 */


#ifndef _AWS_BLE_CONFIG_H_
#define _AWS_BLE_CONFIG_H_


/* Device complete name for this peripheral device */
#define IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME                      "GR5515"
#define IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME_SIZE                 (sizeof(IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME) - 1)
#define IOT_BLE_DEVICE_SHORT_LOCAL_NAME                         "GR5x"
#define IOT_BLE_DEVICE_SHORT_LOCAL_NAME_SIZE                    (sizeof(IOT_BLE_DEVICE_SHORT_LOCAL_NAME) - 1)

#define AWS_IOT_MQTT_ENABLE_METRICS                             1
#define IOT_BLE_ADVERTISING_INTERVAL                            80
#define IOT_BLE_ENABLE_NUMERIC_COMPARISON                       1    //set 0 in test and set 1 in demo
#define IOT_BLE_ENABLE_BONDING                                  1
#define IOT_BLE_ENABLE_SECURE_CONNECTION                        1
#define IOT_BLE_MAX_BONDED_DEVICES                              10
#define IOT_BLE_PREFERRED_MTU_SIZE                              512u
#define IOT_BLE_ENABLE_WIFI_PROVISIONING                        0           /* Enable WIFI provisioning GATT service */
#define IOT_BLE_ADD_CUSTOM_SERVICES                             0
#define GR_BLE_MAX_NB_LECB_DEFUALT                              10u


/* Include BLE default config at bottom to set the default values for the configurations which are not overridden */
#include "iot_ble_config_defaults.h"

#endif /* _AWS_BLE_CONFIG_H_ */

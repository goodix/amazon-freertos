/*
 * Amazon FreeRTOS
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
 * @file iot_ble_hal_internals.h
 * @brief Internally shared functions and variable for HAL ble stack.
 */

#ifndef _AWS_BLE_INTERNALS_H_
#define _AWS_BLE_INTERNALS_H_

#include "iot_ble_config.h"
#include "iot_ble_gap_config.h"
#include "bt_hal_manager.h"
#include "bt_hal_manager_adapter_ble.h"
#include "bt_hal_gatt_server.h"
#include "ble_gatt.h"
#include "ble_gatts.h"
#include "ble_gapc.h"
#include "ble_sec.h"


typedef enum {
    BLE_GATTS_TYPE_INVALID = 0,
    BLE_GATTS_TYPE_READ,
    BLE_GATTS_TYPE_WRITE,
} BLE_GATTS_TYPE;

typedef struct
{
    bool                bBondable;
    bool                bOnlySecure;
    gap_bdaddr_t        xDeviceAddress;
    uint32_t            xLocalMtu;
    uint32_t            xExchangedMtu;
    uint8_t *           puDeviceName;
    uint16_t            usDeviceNameLength;
    BTDeviceType_t      xDeviceType;
    BTIOtypes_t         xIOTypes;    
} BTProperties_t;

typedef struct
{
    const BTBdaddr_t *  pxBdAddr;
    uint32_t            ulMinInterval;
    uint32_t            ulMaxInterval;
    uint32_t            ulLatency;
    uint32_t            ulTimeout;
} BTConnectionParams_t;

typedef struct
{
    BTuuidType_t        uuid_type;
    union
    {
        attm_desc_t     attm;
        attm_desc_128_t attm128;
    } properties;
    uint16_t            service_handle;     //save service handle
    uint16_t            parent_handle;      //save parent handle
    uint16_t            handle;             //save my onw handle, service handle's parent is 0, if service_handle == handle, it's service entity
    BTDbAttributeType_t type;    
    BTCharProperties_t  raw_properties;
    BTCharPermissions_t raw_permissions;
} BTGattEntity_t;

typedef struct{
    bool                isUsed;             //record this posit
    uint16_t            mServiceHandle;     //service handlle
    uint16_t            mGattNum;           //number of attribute
    BTuuidType_t        mUuidType;          //just support 16b & 128b
    void *              pAttTable;          //pointer to att table, real type is attm_desc_t * or attm_desc_128_t *    
}BTGattServiceList_t;

/*
 * save Att Value for handles, for read & write
 */
typedef struct {
    bool                is_used;
    bool                is_wrote;           // is wrote already ? if yes ,can read , orelse, cannot read!
    uint16_t            service_handle;     // service handle (porting layer handle)
    uint16_t            mem_size;           // alloced mem size of this handle
    uint16_t            cur_size;           // current data size , if == 0, data pointer is invalid
    uint8_t             * data;             // data pointer, point to alloc memory
} BTCacheGattValue_t;

typedef struct {
    uint16_t            service_handle;                            //save service handle
    uint16_t            family_handle_count;                       //count of entire service, including service_handle
    uint16_t            family_handle[GR_BLE_GATT_MAX_ENTITIES];   //save all family handles, including service_handle    
} BTGattFamilyHandle_t;


extern BTBleAdapterCallbacks_t      xBTBleAdapterCallbacks;
extern BTGattServerCallbacks_t      xGattServerCb;
extern BTCallbacks_t                xBTCallbacks;
extern BTProperties_t               xProperties;
extern BTUuid_t                     xAppUuid;
extern uint8_t                      pucBondedAddresses[ 6 * IOT_BLE_MAX_BONDED_DEVICES ];
extern uint32_t                     ulGattServerIFhandle;
extern sec_param_t                  xGRSecurityParameters;

const void *            prvBTGetGattServerInterface(void);
const void *            prvBTGetLeAdapter(void);

BTStatus_t              prvAFRUUIDtoGoodix( BTUuid_t * pxAFRUuid, ble_uuid_t * pxGoodixUUID);
BTStatus_t              prvGetServiceAttmTable(uint16_t usServiceHandle, bool * isUUID128, void ** ptable, uint32_t * att_num);

void                    prvBTGattServiceListInit(void);
BTStatus_t              prvBTGattServiceListPut(const BTGattServiceList_t srv);
BTGattServiceList_t *   prvBTGattServiceListGet(uint16_t serviceHandle);
BTGattServiceList_t *   prvBTGattServiceListGetHead(void);
void                    prvBTGattServiceListDelete(uint16_t serviceHandle);

void                    prvBTGattValueHandleInit(void);
void                    prvBTGattValueHandlePush(uint16_t porting_handle, uint16_t mem_size);
void                    prvBTGattValueHandleDeleteService(uint16_t service_handle);
void                    prvBTGattValueHandleDeleteAll(void);
BTCacheGattValue_t *    prvBTGattValueHandleGet(uint16_t porting_handle);

BTGattEntity_t *        prvBTGattEntityGet(uint16_t port_handle);


#endif /* ifndef _AWS_BLE_INTERNALS_H_ */

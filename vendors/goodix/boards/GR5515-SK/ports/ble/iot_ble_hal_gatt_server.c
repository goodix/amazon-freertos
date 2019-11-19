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
 * @file iot_ble_hal_gatt_server.c
 * @brief Hardware Abstraction Layer for GATT server ble stack.
 */

/* Standard Includes. */
#include <string.h>

/* FreeRTOS Includes. */
#include "FreeRTOS.h"

/* AWS BLE Lib Includes. */
#include "bt_hal_manager_adapter_ble.h"
#include "bt_hal_manager.h"
#include "bt_hal_gatt_server.h"
#include "iot_ble_gap_config.h"
#include "iot_ble_hal_internals.h"

#include "user_app.h"
#include "ble.h"
#include "gr_porting.h"
#include "gr_utils.h"

uint8_t                     xGattTableSize = 0;
uint32_t                    ulGattServerIFhandle = 25;       //actually, this handle is not used for gr porting, set magic number 25
BTGattServiceList_t         xGattSrvList[GR_BLE_MAX_SERVICES];
BTGattEntity_t              xGattTable[ GR_BLE_GATT_MAX_ENTITIES ];
BTCacheGattValue_t          xGattCacheValueTable[GR_BLE_GATT_MAX_ENTITIES];

const uint8_t               iot_ble_hal_gatt_serverCCCD_UUID[] = { 0, 0, 0x29, 0x02, 0x79, 0xE6, 0xB5, 0x83, 0xFB, 0x4E, 0xAF, 0x48, 0x68, 0x11, 0x7F, 0x8A };
const uint16_t              iot_ble_hal_gatt_serverCCCD_UUID_2BYTES = 0x2902;
BTGattServerCallbacks_t     xGattServerCb;

gr_ble_gatt_params_t        s_gr_ble_gatt_params_ins = {
    .is_gatt_initialized    = false,
};

static BTGattFamilyHandle_t xGrGattFamilyHandle;

static BTStatus_t prvBTRegisterServer( BTUuid_t * pxUuid );
static BTStatus_t prvBTUnregisterServer( uint8_t ucServerIf );
static BTStatus_t prvBTGattServerInit( const BTGattServerCallbacks_t * pxCallbacks );
static BTStatus_t prvBTConnect( uint8_t ucServerIf,
                                const BTBdaddr_t * pxBdAddr,
                                bool bIsDirect,
                                BTTransport_t xTransport );
static BTStatus_t prvBTDisconnect( uint8_t ucServerIf,
                                   const BTBdaddr_t * pxBdAddr,
                                   uint16_t usConnId );
static BTStatus_t prvBTAddService( uint8_t ucServerIf,
                                   BTGattSrvcId_t * pxSrvcId,
                                   uint16_t usNumHandles );
static BTStatus_t prvBTAddIncludedService( uint8_t ucServerIf,
                                           uint16_t usServiceHandle,
                                           uint16_t usIncludedHandle );
static BTStatus_t prvBTAddCharacteristic( uint8_t ucServerIf,
                                          uint16_t usServiceHandle,
                                          BTUuid_t * pxUuid,
                                          BTCharProperties_t xProperties,
                                          BTCharPermissions_t xPermissions );
static BTStatus_t prvBTSetVal( BTGattResponse_t * pxValue );
static BTStatus_t prvBTAddDescriptor( uint8_t ucServerIf,
                                      uint16_t usServiceHandle,
                                      BTUuid_t * pxUuid,
                                      BTCharPermissions_t xPermissions );
static BTStatus_t prvBTStartService( uint8_t ucServerIf,
                                     uint16_t usServiceHandle,
                                     BTTransport_t xTransport );
static BTStatus_t prvBTStopService( uint8_t ucServerIf,
                                    uint16_t usServiceHandle );
static BTStatus_t prvBTDeleteService( uint8_t ucServerIf,
                                      uint16_t usServiceHandle );
static BTStatus_t prvBTSendIndication( uint8_t ucServerIf,
                                       uint16_t usAttributeHandle,
                                       uint16_t usConnId,
                                       size_t xLen,
                                       uint8_t * pucValue,
                                       bool bConfirm );
static BTStatus_t prvBTSendResponse( uint16_t usConnId,
                                     uint32_t ulTransId,
                                     BTStatus_t xStatus,
                                     BTGattResponse_t * pxResponse );
static BTStatus_t prvAddServiceBlob( uint8_t ucServerIf,
                                     BTService_t * pxService );

static BTStatus_t prvAFRPermtoGoodixPerm(const BTCharProperties_t xProperties, const BTCharPermissions_t xPermissions, uint16_t *perm);

static BTGattServerInterface_t xGATTserverInterface =
{
    .pxRegisterServer     = prvBTRegisterServer,
    .pxUnregisterServer   = prvBTUnregisterServer,
    .pxGattServerInit     = prvBTGattServerInit,
    .pxConnect            = prvBTConnect,
    .pxDisconnect         = prvBTDisconnect,
    .pxAddServiceBlob     = prvAddServiceBlob,
    .pxAddService         = prvBTAddService,
    .pxAddIncludedService = prvBTAddIncludedService,
    .pxAddCharacteristic  = prvBTAddCharacteristic,
    .pxSetVal             = prvBTSetVal,
    .pxAddDescriptor      = prvBTAddDescriptor,
    .pxStartService       = prvBTStartService,
    .pxStopService        = prvBTStopService,
    .pxDeleteService      = prvBTDeleteService,
    .pxSendIndication     = prvBTSendIndication,
    .pxSendResponse       = prvBTSendResponse
};



/*-----------------------------------------------------------*/

BTStatus_t prvBTRegisterServer( BTUuid_t * pxUuid )
{
    BTStatus_t xStatus = eBTStatusSuccess;

    xGattServerCb.pxRegisterServerCb( eBTStatusSuccess, ulGattServerIFhandle, pxUuid );
    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTUnregisterServer( uint8_t ucServerIf )
{
    BTStatus_t xStatus = eBTStatusSuccess;

    xGattServerCb.pxUnregisterServerCb( eBTStatusSuccess, ulGattServerIFhandle );
    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTGattServerInit( const BTGattServerCallbacks_t * pxCallbacks )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    
    xGattTableSize = 0;
    memset(&xGattTable[0], 0, sizeof(BTGattEntity_t) * GR_BLE_GATT_MAX_ENTITIES);
    gr_gatt_service_reset();
    prvBTGattServiceListInit();
    prvBTGattValueHandleInit();        

    if( pxCallbacks != NULL )
    {
        xGattServerCb = *pxCallbacks;
    }
    else
    {
        xStatus = eBTStatusParamInvalid;
    }

    return xStatus;       
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTConnect( uint8_t ucServerIf,
                         const BTBdaddr_t * pxBdAddr,
                         bool bIsDirect,
                         BTTransport_t xTransport )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTDisconnect( uint8_t ucServerIf,
                            const BTBdaddr_t * pxBdAddr,
                            uint16_t usConnId )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTAddService( uint8_t ucServerIf,
                            BTGattSrvcId_t * pxSrvcId,
                            uint16_t usNumHandles )
{
    BTStatus_t rStatus = eBTStatusSuccess;
    attm_desc_t     attm;
    attm_desc_128_t attm128;
    
    if( rStatus == eBTStatusSuccess )
    {
        if( xGattTableSize == GR_BLE_GATT_MAX_ENTITIES - 1 )
        {
            return eBTStatusNoMem;
        }
        else
        {            
            if( eBTServiceTypePrimary == pxSrvcId->xServiceType ){
                xGattTable[ xGattTableSize ].type = eBTDbPrimaryService;
            } else {
                xGattTable[ xGattTableSize ].type = eBTDbSecondaryService;
            }
            
            if(pxSrvcId->xId.xUuid.ucType == eBTuuidType16){

                attm.perm           = 0;
                attm.ext_perm       = 0;
                attm.max_size       = 0;
                attm.uuid           = pxSrvcId->xId.xUuid.uu.uu16;
                
                xGattTable[ xGattTableSize ].uuid_type          = eBTuuidType16;
                xGattTable[ xGattTableSize ].properties.attm    = attm;
            } else if(pxSrvcId->xId.xUuid.ucType == eBTuuidType128){                
                attm128.perm        = 0;
                attm128.ext_perm    = 0;
                attm128.max_size    = 0;                
                memcpy(&attm128.uuid[0], &pxSrvcId->xId.xUuid.uu.uu128[0], bt128BIT_UUID_LEN);
                
                xGattTable[ xGattTableSize ].uuid_type          = eBTuuidType128;
                xGattTable[ xGattTableSize ].properties.attm128 = attm128;
            }
            
            xGattTable[ xGattTableSize ].parent_handle  = 0;
            xGattTable[ xGattTableSize ].handle = xGattTableSize == 0 ? GR_BLE_GATT_PORTING_LAYER_START_HANDLE : xGattTable[ xGattTableSize - 1 ].handle + 1;            
            xGattTable[ xGattTableSize ].service_handle = xGattTable[ xGattTableSize ].handle;
            xGattTable[ xGattTableSize ].raw_properties = 0;
            xGattTable[ xGattTableSize ].raw_permissions= 0;
            
            prvBTGattValueHandlePush(xGattTable[ xGattTableSize ].handle, GR_BLE_GATTS_VAR_ATTR_LEN_DEFAULT);
            xGattTableSize += 1;
        }
        
        if( xGattServerCb.pxServiceAddedCb )
        {
            //notify service handle by callback
            xGattServerCb.pxServiceAddedCb( rStatus, ulGattServerIFhandle, pxSrvcId, xGattTable[ xGattTableSize - 1 ].handle );
        }

        GRH_LOG(INFO,( "Service Added to HAL table with handle %x ", xGattTable[ xGattTableSize - 1 ].handle ));
    }

    return rStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTAddIncludedService( uint8_t ucServerIf,
                                    uint16_t usServiceHandle,
                                    uint16_t usIncludedHandle )
{
    BTStatus_t xStatus = eBTStatusSuccess;

    return xStatus;
}


/*-----------------------------------------------------------*/


BTStatus_t prvAddServiceBlob( uint8_t ucServerIf,
                              BTService_t * pxService )
{
    uint16_t usAttrIndex = 0;
    BTStatus_t xStatus = eBTStatusSuccess;
    uint16_t usServiceHandle = pxService->pusHandlesBuffer[0];  //first elem must be service handle

    for( usAttrIndex = 1; usAttrIndex < pxService->xNumberOfAttributes - 1; usAttrIndex++ )
    {
        switch( pxService->pxBLEAttributes[ usAttrIndex ].xAttributeType )
        {
            case eBTDbPrimaryService:
                xStatus = eBTStatusFail;
                break;

            case eBTDbSecondaryService:
                xStatus = eBTStatusFail;
                break;

            case eBTDbIncludedService:
                //
                break;

            case eBTDbCharacteristicDecl:
                xStatus = eBTStatusSuccess;  //auto add
                break;

            case eBTDbCharacteristic:
                xStatus = prvBTAddCharacteristic( ucServerIf,
                                        usServiceHandle,
                                        &pxService->pxBLEAttributes[ usAttrIndex ].xCharacteristic.xUuid,
                                        pxService->pxBLEAttributes[ usAttrIndex ].xCharacteristic.xProperties,
                                        pxService->pxBLEAttributes[ usAttrIndex ].xCharacteristic.xPermissions );
                break;

            case eBTDbDescriptor:
                xStatus = prvBTAddDescriptor(ucServerIf,
                               usServiceHandle,
                               &pxService->pxBLEAttributes[ usAttrIndex ].xCharacteristic.xUuid,
                               pxService->pxBLEAttributes[ usAttrIndex ].xCharacteristic.xPermissions );
                break;

            default:
                xStatus = eBTStatusUnsupported;
                break;
        }
    }
    
    return xStatus;
}


//Characteristic means Characteristic value
BTStatus_t prvBTAddCharacteristic( uint8_t ucServerIf,
                                   uint16_t usServiceHandle,
                                   BTUuid_t * pxUuid,
                                   BTCharProperties_t xProperties,
                                   BTCharPermissions_t xPermissions )
{
    BTStatus_t      rStatus = eBTStatusSuccess;
    attm_desc_t     attm;
    attm_desc_128_t attm128;    
    uint16_t        perm = 0, 
                    ext_perm = 0;

    if( rStatus == eBTStatusSuccess )
    {
        if( xGattTableSize == GR_BLE_GATT_MAX_ENTITIES - 1 )
        {
            rStatus = eBTStatusNoMem;
        }
        else
        {
            prvAFRPermtoGoodixPerm(xProperties, xPermissions, &perm);
        
            ext_perm |= ATT_VAL_LOC_USER;
            if(pxUuid->ucType == eBTuuidType16){
                ext_perm |= ATT_UUID_TYPE_SET(UUID_TYPE_16);            

                attm.perm           = perm;
                attm.ext_perm       = ext_perm;
                attm.max_size       = GR_BLE_GATTS_VAR_ATTR_LEN_MAX;
                attm.uuid           = pxUuid->uu.uu16;
                
                xGattTable[ xGattTableSize ].uuid_type          = eBTuuidType16;
                xGattTable[ xGattTableSize ].properties.attm    = attm;
            } else if(pxUuid->ucType == eBTuuidType128){
                ext_perm |= ATT_UUID_TYPE_SET(UUID_TYPE_128);
                                
                attm128.perm        = perm;
                attm128.ext_perm    = ext_perm;
                attm128.max_size    = GR_BLE_GATTS_VAR_ATTR_LEN_MAX;                
                memcpy(&attm128.uuid[0], &pxUuid->uu.uu128[0], bt128BIT_UUID_LEN);
                
                xGattTable[ xGattTableSize ].uuid_type          = eBTuuidType128;
                xGattTable[ xGattTableSize ].properties.attm128 = attm128;
            }
            
            xGattTable[ xGattTableSize ].type = eBTDbCharacteristic;
            xGattTable[ xGattTableSize ].parent_handle  = UINT16_MAX;
            xGattTable[ xGattTableSize ].service_handle = usServiceHandle;
            xGattTable[ xGattTableSize ].raw_properties = xProperties;
            xGattTable[ xGattTableSize ].raw_permissions= xPermissions;

            for( int i = 0; i < xGattTableSize; ++i )
            {
                if( ( (xGattTable[i].type == eBTDbPrimaryService) || (xGattTable[i].type == eBTDbSecondaryService)) && ( xGattTable[ i ].handle == usServiceHandle ) )
                {
                    xGattTable[ xGattTableSize ].parent_handle = xGattTable[ i ].handle;
                    break;
                }
            }

            if( xGattTable[ xGattTableSize ].parent_handle == UINT16_MAX )
            {
                rStatus = eBTStatusParamInvalid;
            }
            else
            {
                //xGattTable[ xGattTableSize ].uuid = ble_uuid;
                xGattTable[ xGattTableSize ].handle = xGattTableSize == 0 ? GR_BLE_GATT_PORTING_LAYER_START_HANDLE : xGattTable[ xGattTableSize - 1 ].handle + 2;   //handle + 1 left for char decl
                prvBTGattValueHandlePush(xGattTable[ xGattTableSize ].handle - 1, GR_BLE_GATTS_VAR_ATTR_LEN_DEFAULT);  //for char decl
                prvBTGattValueHandlePush(xGattTable[ xGattTableSize ].handle, GR_BLE_GATTS_VAR_ATTR_LEN_DEFAULT);      //for char value
                xGattTableSize += 1;
                
                rStatus = eBTStatusSuccess;
                GRH_LOG(INFO,( "Charachtersitics Added to HAL table with handle %d   ", xGattTable[ xGattTableSize - 1 ].handle ));
            }
        }
    }

    if( xGattServerCb.pxCharacteristicAddedCb )
    {
        GRH_LOG(INFO,( "Charachtersitics Added status: %d  ", rStatus));
        xGattServerCb.pxCharacteristicAddedCb( rStatus,
                                               ulGattServerIFhandle,
                                               pxUuid,
                                               xGattTable[ xGattTableSize - 1 ].parent_handle,
                                               xGattTableSize == 0 ? 0 : xGattTable[ xGattTableSize - 1 ].handle );
    }

    return rStatus;    
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTSetVal( BTGattResponse_t * pxValue )
{
    uint16_t handle = 0;
    uint16_t rStatus = SDK_SUCCESS;

    handle = gr_gatt_transto_ble_stack_handle(pxValue->usHandle);
    
    rStatus = ble_gatts_value_set(handle, pxValue->xAttrValue.xLen, pxValue->xAttrValue.usOffset, (const uint8_t*) pxValue->xAttrValue.pucValue);    
    if( rStatus != SDK_SUCCESS )
    {
        GRH_LOG(ERROR, ( "prvBTSetVal Error %d HVX  ", rStatus ));
        
        return eBTStatusFail;
    }
    
    return eBTStatusSuccess;    
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTAddDescriptor( uint8_t ucServerIf,
                               uint16_t usServiceHandle,
                               BTUuid_t * pxUuid,
                               BTCharPermissions_t xPermissions )
{
    BTStatus_t      rStatus = eBTStatusSuccess;
    attm_desc_t     attm;
    attm_desc_128_t attm128;    
    uint16_t        perm = 0, 
                    ext_perm = 0;    
    BTCharProperties_t xProperties = eBTPropRead | eBTPropWrite | eBTPropWriteNoResponse | eBTPropSignedWrite;    

    if( rStatus == eBTStatusSuccess )
    {
        if( xGattTableSize == GR_BLE_GATT_MAX_ENTITIES - 1 )
        {
            rStatus = eBTStatusNoMem;
        }
        else
        {
            prvAFRPermtoGoodixPerm(xProperties, xPermissions, &perm);
            
            ext_perm |= ATT_VAL_LOC_USER;
            
            if( ( ( pxUuid->ucType == eBTuuidType128 ) && ( memcmp( pxUuid->uu.uu128, iot_ble_hal_gatt_serverCCCD_UUID, 16 ) == 0 ) ) ||
                ( ( pxUuid->ucType == eBTuuidType16 ) && ( pxUuid->uu.uu16 == iot_ble_hal_gatt_serverCCCD_UUID_2BYTES ) ) ){
                attm.perm           = perm;
                attm.ext_perm       = ATT_VAL_LOC_USER | ATT_UUID_TYPE_SET(UUID_TYPE_16);
                attm.max_size       = GR_BLE_GATTS_VAR_ATTR_LEN_MAX;
                attm.uuid           = iot_ble_hal_gatt_serverCCCD_UUID_2BYTES;
                    
                xGattTable[ xGattTableSize ].uuid_type          = eBTuuidType16;
                xGattTable[ xGattTableSize ].properties.attm    = attm;
            } else if(pxUuid->ucType == eBTuuidType16){
                ext_perm |= ATT_UUID_TYPE_SET(UUID_TYPE_16);            

                attm.perm           = perm;
                attm.ext_perm       = ext_perm;
                attm.max_size       = GR_BLE_GATTS_VAR_ATTR_LEN_MAX;
                attm.uuid           = pxUuid->uu.uu16;
                
                xGattTable[ xGattTableSize ].uuid_type          = eBTuuidType16;
                xGattTable[ xGattTableSize ].properties.attm    = attm;
            } else if(pxUuid->ucType == eBTuuidType128){
                ext_perm |= ATT_UUID_TYPE_SET(UUID_TYPE_128);

                attm128.perm        = perm;
                attm128.ext_perm    = ext_perm;
                attm128.max_size    = GR_BLE_GATTS_VAR_ATTR_LEN_MAX;
                memcpy(&attm128.uuid[0], &pxUuid->uu.uu128[0], bt128BIT_UUID_LEN);
                
                xGattTable[ xGattTableSize ].uuid_type          = eBTuuidType128;
                xGattTable[ xGattTableSize ].properties.attm128 = attm128;
            }
            
            xGattTable[ xGattTableSize ].type = eBTDbDescriptor;
            xGattTable[ xGattTableSize ].parent_handle  = UINT16_MAX;
            xGattTable[ xGattTableSize ].service_handle = usServiceHandle;
            xGattTable[ xGattTableSize ].raw_properties = 0;
            xGattTable[ xGattTableSize ].raw_permissions= xPermissions;

            for( int16_t i = ( int16_t ) xGattTableSize - 1; i >= 0; --i )
            {
                //find parent Characteristic, must find in reverse order
                if( ( xGattTable[ i ].type == eBTDbCharacteristic ) && ( xGattTable[ i ].parent_handle == usServiceHandle ) )
                {
                    //set Characteristic's handle as descr's parent handle 
                    xGattTable[ xGattTableSize ].parent_handle = xGattTable[ i ].handle;
                    break;
                }
            }

            if( xGattTable[ xGattTableSize ].parent_handle == UINT16_MAX )
            {
                rStatus = eBTStatusParamInvalid;
            }
            else
            {
                //xGattTable[ xGattTableSize ].uuid = ble_uuid;
                xGattTable[ xGattTableSize ].handle = xGattTableSize == 0 ? GR_BLE_GATT_PORTING_LAYER_START_HANDLE : xGattTable[ xGattTableSize - 1 ].handle + 1;   //handle + 1 left for char value?                
                prvBTGattValueHandlePush(xGattTable[ xGattTableSize ].handle, GR_BLE_GATTS_VAR_ATTR_LEN_MAX);
                xGattTableSize += 1;
                
                rStatus = eBTStatusSuccess;
                GRH_LOG(INFO, ( "Descriptor Added to HAL table with handle %d   ", xGattTable[ xGattTableSize - 1 ].handle ));
            }
        }
    }
    
    if( xGattServerCb.pxDescriptorAddedCb )
    {
        GRH_LOG(INFO, ( "Descriptor Added status: %d  ", rStatus));
        xGattServerCb.pxDescriptorAddedCb( rStatus,
                                           ulGattServerIFhandle,
                                           pxUuid,
                                           usServiceHandle,
                                           xGattTableSize == 0 ? 0 : xGattTable[ xGattTableSize - 1 ].handle );
    }
    
    return rStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvGetServiceAttmTable(uint16_t usServiceHandle, bool * isUUID128, void ** ptable, uint32_t * att_num){
    bool        is_128b    = false;    
    uint32_t    gatt_count = 0, i =0;
    //attm_desc_128_t * pdesc128 = NULL;
    //attm_desc_t * pdesc = NULL;
    void * pattm = NULL;
    
    gatt_count = 0;
    for( i= 0; i < xGattTableSize; i++)
    {        
        if(xGattTable[ i ].service_handle == usServiceHandle){
            if( ( xGattTable[ i ].type == eBTDbCharacteristic ) && ( xGattTable[ i ].parent_handle == usServiceHandle ) ){ //find characteristic value
                gatt_count += 2;  //extra 1 for characteristic decl
            } else {
                gatt_count++;
            }
            
            if(xGattTable[ i ].uuid_type == eBTuuidType128){
                is_128b = true;
            }
        }        
    }
    
    *isUUID128 = is_128b;
    
    if(gatt_count == 0){
        return eBTStatusUnHandled;
    }
    
    uint32_t temp_count = 0;
    if(is_128b){        
        attm_desc_128_t char_decl_128 = {
                    BLE_ATT_16_TO_128_ARRAY(BLE_ATT_DECL_CHARACTERISTIC), READ_PERM_UNSEC, 0, 0
        };
        
        pattm = pvPortMalloc(gatt_count * sizeof(attm_desc_128_t));
        if(pattm == NULL){
            return eBTStatusNoMem;
        }
        memset(pattm, 0, gatt_count * sizeof(attm_desc_128_t));
        
        temp_count = 0;
        for(i = 0; i < xGattTableSize; i++){
            if(xGattTable[ i ].service_handle == usServiceHandle){
                //add char decl
                if( ( xGattTable[ i ].type == eBTDbCharacteristic ) && ( xGattTable[ i ].parent_handle == usServiceHandle ) ){
                    memcpy(((attm_desc_128_t*)pattm) + temp_count, &char_decl_128, sizeof(attm_desc_128_t));
                    temp_count++;
                } 
                
                if(xGattTable[ i ].uuid_type == eBTuuidType128){
                    memcpy( (((attm_desc_128_t*)pattm) + temp_count), &(xGattTable[ i ].properties.attm128), sizeof(attm_desc_128_t));
                } else {
                    uint8_t uuid128[16] = BLE_ATT_16_TO_128_ARRAY(xGattTable[ i ].properties.attm.uuid);
                    
                    memcpy( (((attm_desc_128_t*)pattm) + temp_count)->uuid, &uuid128[0], 16);
                    
                    (((attm_desc_128_t*)pattm) + temp_count)->perm      = xGattTable[ i ].properties.attm.perm;
                    (((attm_desc_128_t*)pattm) + temp_count)->ext_perm  = xGattTable[ i ].properties.attm.ext_perm;
                    (((attm_desc_128_t*)pattm) + temp_count)->max_size  = xGattTable[ i ].properties.attm.max_size;                        
                }
                temp_count++;
            }
            
            if(temp_count == gatt_count){
                break;
            }
        }
        
    } else {
        attm_desc_t char_decl = {
                    BLE_ATT_DECL_CHARACTERISTIC, READ_PERM_UNSEC, 0, 0
        };
        
        pattm = pvPortMalloc(gatt_count * sizeof(attm_desc_t));
        if(pattm == NULL){
            return eBTStatusNoMem;
        }
        memset(pattm, 0, gatt_count * sizeof(attm_desc_t));
        
        temp_count = 0;
        for(i = 0; i < xGattTableSize; i++){
            if(xGattTable[ i ].service_handle == usServiceHandle){
                //add char decl
                if( ( xGattTable[ i ].type == eBTDbCharacteristic ) && ( xGattTable[ i ].parent_handle == usServiceHandle ) ){
                    memcpy(((attm_desc_t*)pattm) + temp_count, &char_decl, sizeof(attm_desc_t));
                    temp_count++;
                }
                
                memcpy(((attm_desc_t*)pattm) + temp_count, &(xGattTable[ i ].properties.attm), sizeof(attm_desc_t));
                temp_count++;
            }
            
            if(temp_count == gatt_count){
                break;
            }
        }
    }
    
    * ptable = pattm;
    * att_num = gatt_count;
    
    return eBTStatusSuccess;
}


BTStatus_t prvBTStartService( uint8_t ucServerIf,
                              uint16_t usServiceHandle,
                              BTTransport_t xTransport )
{
    
    BTStatus_t rStatus = eBTStatusSuccess;
    uint32_t att_num = 0;
    void * ptable = NULL;
    bool isUuid128 = false;
    BTGattServiceList_t srvlist;
    
    rStatus = prvGetServiceAttmTable(usServiceHandle, &isUuid128, (void*) &ptable, &att_num);

    if(rStatus == eBTStatusSuccess){
        
        srvlist.mGattNum        = att_num;        
        srvlist.mServiceHandle  = usServiceHandle;
        srvlist.mUuidType       = isUuid128 ? eBTuuidType128 : eBTuuidType16;
        srvlist.pAttTable       = ptable;
        
        rStatus = prvBTGattServiceListPut(srvlist);
        
        if(rStatus == eBTStatusSuccess){
            rStatus = gr_gatt_service_register(usServiceHandle);
        }
    }
    GRH_LOG(INFO, ("prvBTStartService : %d  ", rStatus));
    if(xGattServerCb.pxServiceStartedCb != NULL){
        xGattServerCb.pxServiceStartedCb( rStatus, ucServerIf, usServiceHandle );
    }       
    
    return rStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTStopService( uint8_t ucServerIf,
                             uint16_t usServiceHandle )
{
    BTStatus_t xStatus = eBTStatusSuccess;

    /* It is not supported to stop a GATT service, so we just return success.
     */
    if( xGattServerCb.pxServiceStoppedCb )
    {
        xGattServerCb.pxServiceStoppedCb( eBTStatusSuccess, ucServerIf, usServiceHandle );
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTDeleteService( uint8_t ucServerIf,
                               uint16_t usServiceHandle )
{
    BTStatus_t xStatus = eBTStatusSuccess;    
    BTGattServiceList_t * psrv = prvBTGattServiceListGet(usServiceHandle);
    
    if(psrv != NULL){
        if(psrv->pAttTable != NULL)
            vPortFree(psrv->pAttTable);
        
        prvBTGattServiceListDelete(usServiceHandle);
        prvBTGattValueHandleDeleteService(usServiceHandle);
    } else {
        xStatus = eBTStatusUnHandled;
    }

    if( xGattServerCb.pxServiceDeletedCb )
    {
        xGattServerCb.pxServiceDeletedCb( xStatus, ucServerIf, usServiceHandle );
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTSendIndication( uint8_t ucServerIf,
                                uint16_t usAttributeHandle,
                                uint16_t usConnId,
                                size_t xLen,
                                uint8_t * pucValue,
                                bool bConfirm )
{
    BTStatus_t  xStatus = eBTStatusSuccess;
    sdk_err_t   error_code ;    
    gatts_noti_ind_t gr_ind;
    
    if(bConfirm){
        gr_ind.type = BLE_GATT_INDICATION;
    } else {
        gr_ind.type = BLE_GATT_NOTIFICATION;
    }
    
    gr_ind.handle = gr_gatt_transto_ble_stack_handle(usAttributeHandle);
    gr_ind.length = xLen;
    gr_ind.value  = pucValue;
    
    error_code = ble_gatts_noti_ind(usConnId, &gr_ind);
    
    GRH_LOG(INFO, ("prvBTSendIndication handle: %d , confirm:%d  ", usAttributeHandle, bConfirm));
    //gr_util_print_buffer(BLE_GATTS_TYPE_INVALID, usAttributeHandle, pucValue, xLen);
    
    xStatus = gr_util_to_afr_status_code(error_code);
    
    if(xStatus != eBTStatusSuccess) {
        if(/*bConfirm && */xGattServerCb.pxIndicationSentCb){
            xGattServerCb.pxIndicationSentCb(usConnId, xStatus);
        }
    } else {
        //called in callback
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTSendResponse( uint16_t usConnId,
                              uint32_t ulTransId,
                              BTStatus_t xStatus,
                              BTGattResponse_t * pxResponse )
{
    BTStatus_t rStatus = eBTStatusSuccess;
    
    //auto response in ble stack
    GRH_LOG(INFO, ("prvBTSendResponse calledd(%d, %d, %d)...", usConnId,  ulTransId, xStatus));   
    
    switch(ulTransId){
        case BLE_GATTS_TYPE_READ:
        {
            gatts_read_cfm_t    cfm;            
            uint16_t            stack_handle = gr_gatt_transto_ble_stack_handle(pxResponse->xAttrValue.usHandle);

            cfm.handle = stack_handle;
            cfm.status = (xStatus == eBTStatusSuccess) ? BLE_SUCCESS : BLE_ATT_ERR_READ_NOT_PERMITTED;            
            cfm.length = pxResponse->xAttrValue.xLen;
            cfm.value  = pxResponse->xAttrValue.pucValue;
            
            GRH_LOG(DEBUG,(">>> Read Attr port handle: %d  ", pxResponse->xAttrValue.usHandle));
            //gr_util_print_buffer(BLE_GATTS_TYPE_READ, pxResponse->xAttrValue.usHandle, cfm.value, cfm.length);
            
            ble_gatts_read_cfm(usConnId, &cfm);
        }
        break;
        
        case BLE_GATTS_TYPE_WRITE:
        {            
            gatts_write_cfm_t   cfm;
            uint16_t            stack_handle = gr_gatt_transto_ble_stack_handle(pxResponse->xAttrValue.usHandle);
                    
            cfm.handle = stack_handle;
            
            if(stack_handle == GR_BLE_GATT_INVALID_HANDLE){
                cfm.status = BLE_ATT_ERR_INVALID_HANDLE;
            } else if(xStatus == eBTStatusSuccess){
                cfm.status = BLE_SUCCESS;
            } else {
                cfm.status = BLE_ATT_ERR_READ_NOT_PERMITTED;
            }
            ble_gatts_write_cfm(usConnId, &cfm);
        }
        break;
        
        default:
        {
            
        }
        break;
    }
        
    if( xGattServerCb.pxResponseConfirmationCb != NULL )
    {
        xGattServerCb.pxResponseConfirmationCb( rStatus, pxResponse->usHandle );
    }

    return rStatus;
}

/*-----------------------------------------------------------*/

const void * prvBTGetGattServerInterface(void)
{
    return &xGATTserverInterface;
}

/*-----------------------------------------------------------*/

BTStatus_t prvAFRPermtoGoodixPerm(const BTCharProperties_t xProperties, const BTCharPermissions_t xPermissions, uint16_t *perm){

    *perm = 0;
    
    if(xProperties & eBTPropBroadcast){
        *perm |= BROADCAST_ENABLE;
    }
    
    if(xProperties & eBTPropRead){
        if(xPermissions & eBTPermRead){
            *perm |= READ_PERM(NOAUTH);
        } else if(xPermissions & eBTPermReadEncrypted){
            *perm |= READ_PERM(UNAUTH);
        } else if(xPermissions & eBTPermReadEncryptedMitm){
            *perm |= READ_PERM(AUTH);
        }        
    }
    
    if(xProperties & eBTPropWriteNoResponse){
        if(xPermissions & eBTPermWrite){
            *perm |= WRITE_CMD_PERM(NOAUTH);
        } else if(xPermissions & eBTPermWriteEncrypted){
            *perm |= WRITE_CMD_PERM(UNAUTH);
        } else if(xPermissions & eBTPermWriteEncryptedMitm){
            *perm |= WRITE_CMD_PERM(AUTH);
        }
    }
    
    if(xProperties & eBTPropWrite){
        if(xPermissions & eBTPermWrite){
            *perm |= WRITE_REQ_PERM(NOAUTH);
        } else if(xPermissions & eBTPermWriteEncrypted){
            *perm |= WRITE_REQ_PERM(UNAUTH);
        } else if(xPermissions & eBTPermWriteEncryptedMitm){
            *perm |= WRITE_REQ_PERM(AUTH);
        }
    }
    
    if(xProperties & eBTPropNotify){
        if(xPermissions & (eBTPermWrite | eBTPermRead)){
            *perm |= NOTIFY_PERM(NOAUTH);
        } else if(xPermissions & (eBTPermWriteEncrypted | eBTPermReadEncrypted)){
            *perm |= NOTIFY_PERM(UNAUTH);
        } else if(xPermissions & (eBTPermWriteEncryptedMitm | eBTPermReadEncryptedMitm)){
            *perm |= NOTIFY_PERM(AUTH);
        }        
    }
    
    if(xProperties & eBTPropIndicate){
        if(xPermissions & (eBTPermWrite | eBTPermRead)){
            *perm |= INDICATE_PERM(NOAUTH);
        } else if(xPermissions & (eBTPermWriteEncrypted | eBTPermReadEncrypted)){
            *perm |= INDICATE_PERM(UNAUTH);
        } else if(xPermissions & (eBTPermWriteEncryptedMitm | eBTPermReadEncryptedMitm)){
            *perm |= INDICATE_PERM(AUTH);
        }         
    }
    
    if(xProperties & eBTPropSignedWrite){
        if(xPermissions & eBTPermWriteSigned){
            *perm |= WRITE_SIGNED_PERM(UNAUTH);
        } else if(xPermissions & eBTPermWriteSignedMitm){
            *perm |= WRITE_SIGNED_PERM(AUTH);
        } else {
            *perm |= WRITE_SIGNED_PERM(NOAUTH);
        }
    }
    
    if(xProperties & eBTPropExtendedProps){
        *perm |= EXT_PROP_ENABLE;
    }
    
    return eBTStatusSuccess;
}


//

void prvBTGattServiceListInit(void){
    memset(&xGattSrvList[0], 0, sizeof(BTGattServiceList_t) * GR_BLE_MAX_SERVICES);
}

BTStatus_t prvBTGattServiceListPut(const BTGattServiceList_t srv){
    bool isPut = false;
    
    for(int i = 0; i< GR_BLE_MAX_SERVICES; i++){
        if(!xGattSrvList[i].isUsed){
            memcpy(&xGattSrvList[i], &srv, sizeof(BTGattServiceList_t));
            xGattSrvList[i].isUsed = true;
            isPut = true;
            break;
        }
    }
    
    if(isPut){
        return eBTStatusSuccess;
    } else {
        return eBTStatusNoMem;
    }
}

BTGattServiceList_t * prvBTGattServiceListGet(uint16_t serviceHandle){
    
    for(int i = 0; i< GR_BLE_MAX_SERVICES; i++){
        if(xGattSrvList[i].isUsed && xGattSrvList[i].mServiceHandle == serviceHandle){
            return &xGattSrvList[i];
        }
    }
    
    return NULL;
}

BTGattServiceList_t * prvBTGattServiceListGetHead(void){
    return &xGattSrvList[0];
}

void prvBTGattServiceListDelete(uint16_t serviceHandle){
    
    for(int i = 0; i< GR_BLE_MAX_SERVICES; i++){
        if(xGattSrvList[i].isUsed && xGattSrvList[i].mServiceHandle == serviceHandle){
            //must free ptable before delete
            memset(&xGattSrvList[i], 0 , sizeof(BTGattServiceList_t));
            break;
        }
    }
}

static BTGattFamilyHandle_t * prvBTGattValueHandleFindAllChildrenHandles(uint16_t service_handle) {
    uint8_t count = 0;
    
    memset(&xGrGattFamilyHandle, 0 ,sizeof(BTGattFamilyHandle_t));
    
    xGrGattFamilyHandle.service_handle = service_handle;
    
    if((service_handle < GR_BLE_GATT_PORTING_LAYER_START_HANDLE) || (service_handle == GR_BLE_GATT_INVALID_HANDLE))
    {
        return &xGrGattFamilyHandle;
    }
    
    for(int i = 0; i < GR_BLE_GATT_MAX_ENTITIES; i++){
        if(service_handle == xGattTable[i].service_handle){
            xGrGattFamilyHandle.family_handle[count++] = xGattTable[i].handle;
        }
    }
    
    xGrGattFamilyHandle.family_handle_count = count;
    
    return &xGrGattFamilyHandle;
}


void prvBTGattValueHandleInit(void){
    memset(&xGattCacheValueTable[0], 0 , sizeof(BTCacheGattValue_t)*GR_BLE_GATT_MAX_ENTITIES);        
}

void prvBTGattValueHandlePush(uint16_t porting_handle, uint16_t mem_size){
    
    for(int i=0; i < GR_BLE_GATT_MAX_ENTITIES; i++) {
        if(!xGattCacheValueTable[i].is_used) {
            xGattCacheValueTable[i].is_used          = true;
            xGattCacheValueTable[i].is_wrote         = false;
            xGattCacheValueTable[i].service_handle   = porting_handle;
            xGattCacheValueTable[i].cur_size         = 0;            
            xGattCacheValueTable[i].data             = (uint8_t*) pvPortMalloc(mem_size);
            
            if(xGattCacheValueTable[i].data != NULL){
                memset(xGattCacheValueTable[i].data, 0 , mem_size);
                xGattCacheValueTable[i].mem_size     = mem_size;
            } else {
                xGattCacheValueTable[i].mem_size     = 0;
            }
            
            break;
        }
    }
}

BTCacheGattValue_t * prvBTGattValueHandleGet(uint16_t porting_handle){
    if((porting_handle < GR_BLE_GATT_PORTING_LAYER_START_HANDLE) || (porting_handle == GR_BLE_GATT_INVALID_HANDLE))
    {
        return NULL;
    }
    
    for(int i = 0; i < GR_BLE_GATT_MAX_ENTITIES; i++) {
        if((xGattCacheValueTable[i].is_used) && (xGattCacheValueTable[i].service_handle == porting_handle)){
            return &xGattCacheValueTable[i];
        }
    }
    
    return NULL;
}

static void prvBTGattValueHandleDelete(uint16_t porting_handle){
    if((porting_handle < GR_BLE_GATT_PORTING_LAYER_START_HANDLE) || (porting_handle == GR_BLE_GATT_INVALID_HANDLE))
    {
        return ;
    }
    
    for(int i = 0; i < GR_BLE_GATT_MAX_ENTITIES; i++){
        if(xGattCacheValueTable[i].service_handle == porting_handle){
            if((xGattCacheValueTable[i].mem_size > 0) && (xGattCacheValueTable[i].data != NULL)){
                vPortFree(xGattCacheValueTable[i].data);
            }            
            memset(&xGattCacheValueTable[i], 0, sizeof(BTCacheGattValue_t));
            break;
        }
    }    
}


void prvBTGattValueHandleDeleteService(uint16_t service_handle){
    if((service_handle < GR_BLE_GATT_PORTING_LAYER_START_HANDLE) || (service_handle == GR_BLE_GATT_INVALID_HANDLE))
    {
        return ;
    }
    
    BTGattFamilyHandle_t * family_handles = prvBTGattValueHandleFindAllChildrenHandles(service_handle);
    
    configASSERT(family_handles != NULL);
    configASSERT(family_handles->service_handle == service_handle);
    
    for(int i = 0; i < family_handles->family_handle_count; i++){
        prvBTGattValueHandleDelete(family_handles->family_handle[i]);
    }
}


void prvBTGattValueHandleDeleteAll(void){
    int i;
    for(i = 0; i < GR_BLE_GATT_MAX_ENTITIES; i++){
        if((xGattCacheValueTable[i].mem_size > 0) && (xGattCacheValueTable[i].data != NULL)){
            vPortFree(xGattCacheValueTable[i].data);
        }
        memset(&xGattCacheValueTable[i], 0, sizeof(BTCacheGattValue_t));        
    }    
}

BTGattEntity_t * prvBTGattEntityGet(uint16_t port_handle){
    if(port_handle < GR_BLE_GATT_PORTING_LAYER_START_HANDLE || port_handle == GR_BLE_GATT_INVALID_HANDLE){
        return NULL;
    }
    
    for(int i=0; i< GR_BLE_GATT_MAX_ENTITIES; i++){
        if(xGattTable[i].handle == port_handle){
            return &xGattTable[i];
        }
    }
    
    return NULL;
}

static char * prvFormatUUID(BTGattEntity_t gatt){
    static char tbuff[64];
    uint8_t len = 0;
    memset(&tbuff[0], 0 , 64);
    
    if(gatt.uuid_type == eBTuuidType128){
        len = 0;
        for(int i = 15;i >= 0; i--){
            len += sprintf(&tbuff[len], "%02x", gatt.properties.attm128.uuid[i]);
        }
        
    } else if(gatt.uuid_type == eBTuuidType16){
        sprintf(&tbuff[0], "%04x", gatt.properties.attm.uuid);
    }
    
    return &tbuff[0];
}

//can be called after service added finished
void gr_util_gatt_handle_map_print(void){
#if GR_BLE_HAL_TRACE_ENABLE > 0u    
    uint32_t max = xGattTableSize > GR_BLE_GATT_MAX_ENTITIES ? GR_BLE_GATT_MAX_ENTITIES : xGattTableSize;
    uint16_t stack_handle = 0;
    
    GRH_LOG(INFO, ("\r\n+++++ Gatt Service Handle Map +++++") );
    GRH_LOG(INFO, ("+++ Port  +++  Stack  +++  UUID +++") );
    
    for (int i=0; i< max; i++){
        stack_handle = gr_gatt_transto_ble_stack_handle(xGattTable[i].handle);
        GRH_LOG(INFO, ("+++ %-4d  +++  %-6d +++ %s ", xGattTable[i].handle, stack_handle, prvFormatUUID(xGattTable[i])) );
    }
    GRH_LOG(INFO, ("++++++++++++++++++++++++++++++++++++\r\n") );
#endif
    return;
}

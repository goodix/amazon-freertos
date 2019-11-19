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
 * @file iot_ble_hal_gap.c
 * @brief Hardware Abstraction Layer for GAP ble stack.
 */

#include <stddef.h>
#include <string.h>
#include "FreeRTOS.h"
#include "portable.h"
#include "bt_hal_manager_adapter_ble.h"
#include "bt_hal_manager.h"
#include "bt_hal_gatt_server.h"
#include "iot_ble_hal_internals.h"
#include "iot_ble_gap_config.h"
#include "aws_clientcredential.h"
#include "bt_hal_manager_adapter_ble.h"

#include "ble_gatt.h"
#include "gr_utils.h"
#include "gr_porting.h"


gr_ble_gap_params_t s_gr_ble_gap_params_ins = {
    .is_connected       = false,
    .is_adv_started     = false,
    .is_mtu_exchanged   = false,
    .cur_connect_id     = 0,
    .is_need_sec_cfm    = false,
};

BTBleAdapterCallbacks_t xBTBleAdapterCallbacks;


static BTStatus_t prvBTBleAdapterInit( const BTBleAdapterCallbacks_t * pxCallbacks );
static BTStatus_t prvBTRegisterBleApp( BTUuid_t * pxAppUuid );
static BTStatus_t prvBTUnregisterBleApp( uint8_t ucAdapterIf );
static BTStatus_t prvBTGetBleAdapterProperty( BTBlePropertyType_t xType );
static BTStatus_t prvBTSetBleAdapterProperty( const BTBleProperty_t * pxProperty );
static BTStatus_t prvBTGetallBleRemoteDeviceProperties( BTBdaddr_t * pxRremoteAddr );
static BTStatus_t prvBTGetBleRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                                   BTBleProperty_t xType );
static BTStatus_t prvBTSetBleRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                                   const BTBleProperty_t * pxProperty );
static BTStatus_t prvBTScan( bool bStart );
static BTStatus_t prvBTConnect( uint8_t ucAdapterIf,
                                const BTBdaddr_t * pxBdAddr,
                                bool bIsDirect,
                                BTTransport_t ulTransport );
static BTStatus_t prvBTDisconnect( uint8_t ucAdapterIf,
                                   const BTBdaddr_t * pxBdAddr,
                                   uint16_t usConnId );
static BTStatus_t prvBTStartAdv( uint8_t ucAdapterIf );
static BTStatus_t prvBTStopAdv( uint8_t ucAdapterIf );
static BTStatus_t prvBTReadRemoteRssi( uint8_t ucAdapterIf,
                                       const BTBdaddr_t * pxBdAddr );
static BTStatus_t prvBTScanFilterParamSetup( BTGattFiltParamSetup_t xFiltParam );
static BTStatus_t prvBTScanFilterAddRemove( uint8_t ucAdapterIf,
                                            uint32_t ulAction,
                                            uint32_t ulFiltType,
                                            uint32_t ulFiltIndex,
                                            uint32_t ulCompanyId,
                                            uint32_t ulCompanyIdMask,
                                            const BTUuid_t * pxUuid,
                                            const BTUuid_t * pxUuidMask,
                                            const BTBdaddr_t * pxBdAddr,
                                            char cAddrType,
                                            size_t xDataLen,
                                            char * pcData,
                                            size_t xMaskLen,
                                            char * pcMask );
static BTStatus_t prvBTScanFilterEnable( uint8_t ucAdapterIf,
                                         bool bEnable );
static BTTransport_t prvBTGetDeviceType( const BTBdaddr_t * pxBdAddr );
static BTStatus_t prvBTSetAdvData( uint8_t ucAdapterIf,
                                   BTGattAdvertismentParams_t * pxParams,
                                   uint16_t usManufacturerLen,
                                   char * pcManufacturerData,
                                   uint16_t usServiceDataLen,
                                   char * pcServiceData,
                                   BTUuid_t * pxServiceUuid,
                                   size_t xNbServices );
static BTStatus_t prvBTSetAdvRawData( uint8_t ucAdapterIf,
                                      uint8_t * pucData,
                                      uint8_t ucLen );
static BTStatus_t prvBTConnParameterUpdateRequest( const BTBdaddr_t * pxBdAddr,
                                                   uint32_t ulMinInterval,
                                                   uint32_t ulMaxInterval,
                                                   uint32_t ulLatency,
                                                   uint32_t ulTimeout );
static BTStatus_t prvBTSetScanParameters( uint8_t ucAdapterIf,
                                          uint32_t ulScanInterval,
                                          uint32_t ulScanWindow );
static BTStatus_t prvBTMultiAdvEnable( uint8_t ucAdapterIf,
                                       BTGattAdvertismentParams_t * xAdvParams );
static BTStatus_t prvBTMultiAdvUpdate( uint8_t ucAdapterIf,
                                       BTGattAdvertismentParams_t * advParams );
static BTStatus_t prvBTMultiAdvSetInstData( uint8_t ucAdapterIf,
                                            bool bSetScanRsp,
                                            bool bIncludeName,
                                            bool bInclTxpower,
                                            uint32_t ulAppearance,
                                            size_t xManufacturerLen,
                                            char * pcManufacturerData,
                                            size_t xServiceDataLen,
                                            char * pcServiceData,
                                            BTUuid_t * pxServiceUuid,
                                            size_t xNbServices );
static BTStatus_t prvBTMultiAdvDisable( uint8_t ucAdapterIf );
static BTStatus_t prvBTBatchscanCfgStorage( uint8_t ucAdapterIf,
                                            uint32_t ulBatchScanFullMax,
                                            uint32_t ulBatchScanTruncMax,
                                            uint32_t ulBatchScanNotifyThreshold );
static BTStatus_t prvBTBatchscanEndBatchScan( uint8_t ucAdapterIf,
                                              uint32_t ulScanMode,
                                              uint32_t ulScanInterval,
                                              uint32_t ulScanWindow,
                                              uint32_t ulAddrType,
                                              uint32_t ulDiscardRule );
static BTStatus_t prvBTBatchscanDisBatchScan( uint8_t ucAdapterIf );
static BTStatus_t prvBTBatchscanReadReports( uint8_t ucAdapterIf,
                                             uint32_t ulScanMode );
static BTStatus_t prvBTSetPreferredPhy( uint16_t usConnId,
                                        uint8_t ucTxPhy,
                                        uint8_t ucRxPhy,
                                        uint16_t usPhyOptions );
static BTStatus_t prvBTReadPhy( uint16_t usConnId,
                                BTReadClientPhyCallback_t xCb );
static const void * prvBTGetGattClientInterface( void );

BTBleAdapter_t xBTLeAdapter =
{
    .pxBleAdapterInit                  = prvBTBleAdapterInit,
    .pxRegisterBleApp                  = prvBTRegisterBleApp,
    .pxUnregisterBleApp                = prvBTUnregisterBleApp,
    .pxGetBleAdapterProperty           = prvBTGetBleAdapterProperty,
    .pxSetBleAdapterProperty           = prvBTSetBleAdapterProperty,
    .pxGetallBleRemoteDeviceProperties = prvBTGetallBleRemoteDeviceProperties,
    .pxGetBleRemoteDeviceProperty      = prvBTGetBleRemoteDeviceProperty,
    .pxSetBleRemoteDeviceProperty      = prvBTSetBleRemoteDeviceProperty,
    .pxScan                            = prvBTScan,
    .pxConnect                         = prvBTConnect,
    .pxDisconnect                      = prvBTDisconnect,
    .pxStartAdv                        = prvBTStartAdv,
    .pxStopAdv                         = prvBTStopAdv,
    .pxReadRemoteRssi                  = prvBTReadRemoteRssi,
    .pxScanFilterParamSetup            = prvBTScanFilterParamSetup,
    .pxScanFilterAddRemove             = prvBTScanFilterAddRemove,
    .pxScanFilterEnable                = prvBTScanFilterEnable,
    .pxGetDeviceType                   = prvBTGetDeviceType,
    .pxSetAdvData                      = prvBTSetAdvData,
    .pxSetAdvRawData                   = prvBTSetAdvRawData,
    .pxConnParameterUpdateRequest      = prvBTConnParameterUpdateRequest,
    .pxSetScanParameters               = prvBTSetScanParameters,
    .pxMultiAdvEnable                  = prvBTMultiAdvEnable,
    .pxMultiAdvUpdate                  = prvBTMultiAdvUpdate,
    .pxMultiAdvSetInstData             = prvBTMultiAdvSetInstData,
    .pxMultiAdvDisable                 = prvBTMultiAdvDisable,
    .pxBatchscanCfgStorage             = prvBTBatchscanCfgStorage,
    .pxBatchscanEndBatchScan           = prvBTBatchscanEndBatchScan,
    .pxBatchscanDisBatchScan           = prvBTBatchscanDisBatchScan,
    .pxBatchscanReadReports            = prvBTBatchscanReadReports,
    .pxSetPreferredPhy                 = prvBTSetPreferredPhy,
    .pxReadPhy                         = prvBTReadPhy,

    .ppvGetGattClientInterface         = prvBTGetGattClientInterface,
    .ppvGetGattServerInterface         = prvBTGetGattServerInterface,
};


/*-----------------------------------------------------------*/



BTStatus_t prvBTBleAdapterInit( const BTBleAdapterCallbacks_t * pxCallbacks )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    
    memset(&s_gr_ble_gap_params_ins, 0 , sizeof(gr_ble_gap_params_t));
    
    if( pxCallbacks != NULL )
    {
        xBTBleAdapterCallbacks = *pxCallbacks;
    }
    else
    {
        xStatus = eBTStatusFail;
    }

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTRegisterBleApp( BTUuid_t * pxAppUuid )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    
    memcpy(&s_gr_ble_gap_params_ins.register_app_uuid, pxAppUuid, sizeof(BTUuid_t));
    
    if( xBTBleAdapterCallbacks.pxRegisterBleAdapterCb )
    {
        xBTBleAdapterCallbacks.pxRegisterBleAdapterCb( xStatus, 0, &s_gr_ble_gap_params_ins.register_app_uuid );
    }
    
    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTUnregisterBleApp( uint8_t ucAdapterIf )
{
    
    BTStatus_t xStatus = eBTStatusSuccess;

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTGetBleAdapterProperty( BTBlePropertyType_t xType )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTSetBleAdapterProperty( const BTBleProperty_t * pxProperty )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTGetallBleRemoteDeviceProperties( BTBdaddr_t * pxRremoteAddr )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTGetBleRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                            BTBleProperty_t xType )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTSetBleRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                            const BTBleProperty_t * pxProperty )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTScan( bool bStart )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTConnect( uint8_t ucAdapterIf,
                         const BTBdaddr_t * pxBdAddr,
                         bool bIsDirect,
                         BTTransport_t ulTransport )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/


BTStatus_t prvBTDisconnect( uint8_t ucAdapterIf,
                            const BTBdaddr_t * pxBdAddr,
                            uint16_t usConnId )
{
    sdk_err_t ret = ble_gap_disconnect((uint8_t)usConnId);
    GRH_LOG(INFO, ("prvBTDisconnect result: %d  ", ret));
    return gr_util_to_afr_status_code(ret);
}


BTStatus_t prvBTSetAdvData( uint8_t ucAdapterIf,
                            BTGattAdvertismentParams_t * pxParams,
                            uint16_t usManufacturerLen,
                            char * pcManufacturerData,
                            uint16_t usServiceDataLen,
                            char * pcServiceData,
                            BTUuid_t * pxServiceUuid,
                            size_t xNbServices )
{    
    BTStatus_t xStatus = eBTStatusSuccess;
    uint8_t ucSlaveConnectInterval[ 4 ];
    uint8_t ucIndex, ucMessageIndex = 0;
    uint8_t ucMessageRaw[ GR_BLE_SCAN_RSP_DATA_LEN_MAX ];
    uint8_t ucFlags;
    
    sdk_err_t x_err = SDK_SUCCESS;
    gap_adv_data_type_t advType;
    

    advType = pxParams->bSetScanRsp ? BLE_GAP_ADV_DATA_TYPE_SCAN_RSP : BLE_GAP_ADV_DATA_TYPE_DATA;
    
    if( ( pxParams->ucNameType == BTGattAdvNameComplete ) || ( IOT_BLE_DEVICE_SHORT_LOCAL_NAME_SIZE >= sizeof( IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME ) - 1 ) )
    {
        if( (xProperties.puDeviceName != NULL) && (xProperties.usDeviceNameLength > 0) ){
            xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_COMPLETE_NAME, ( uint8_t * ) xProperties.puDeviceName, xProperties.usDeviceNameLength ,advType);
        } else {
            xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_COMPLETE_NAME, ( uint8_t * ) IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME, strlen( IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME ) ,advType);
        }
    }
    else if( pxParams->ucNameType == BTGattAdvNameShort )
    {
        if( (xProperties.puDeviceName != NULL) && (xProperties.usDeviceNameLength > 0) ){
            xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_SHORTENED_NAME, ( uint8_t * ) xProperties.puDeviceName, xProperties.usDeviceNameLength ,advType);
        } else {
            xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_SHORTENED_NAME, ( uint8_t * ) IOT_BLE_DEVICE_SHORT_LOCAL_NAME, strlen(IOT_BLE_DEVICE_SHORT_LOCAL_NAME) ,advType);
        }
    }
    
    if( ( xNbServices != 0 ) && ( xStatus == eBTStatusSuccess ) )
    {
        for( ucIndex = 0; ucIndex < xNbServices; ucIndex++ )
        {
            switch( pxServiceUuid[ ucIndex ].ucType )
            {
                case eBTuuidType16:
                    xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_MORE_16_BIT_UUID, ( uint8_t * ) &pxServiceUuid[ ucIndex ].uu.uu16, 2 ,advType);
                    break;

                case eBTuuidType32:
                    xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_MORE_32_BIT_UUID, ( uint8_t * ) &pxServiceUuid[ ucIndex ].uu.uu32, 4 ,advType);
                    break;

                case eBTuuidType128:
                    xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_MORE_128_BIT_UUID, pxServiceUuid[ ucIndex ].uu.uu128, bt128BIT_UUID_LEN ,advType);
                    break;
            }

            if( xStatus != eBTStatusSuccess )
            {
                break;
            }
        }
    }

    if( ( xStatus == eBTStatusSuccess ) && ( pxParams->ulMinInterval != 0 ) && ( pxParams->ulMaxInterval != 0 ) )
    {
        ucSlaveConnectInterval[ 0 ] = ( pxParams->ulMinInterval ) & 0xFF;
        ucSlaveConnectInterval[ 1 ] = ( pxParams->ulMinInterval >> 8 ) & 0xFF;
        ucSlaveConnectInterval[ 2 ] = ( pxParams->ulMaxInterval ) & 0xFF;
        ucSlaveConnectInterval[ 3 ] = ( pxParams->ulMaxInterval >> 8 ) & 0xFF;
        xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_SLAVE_CONN_INT_RANGE, ucSlaveConnectInterval, 4 ,advType);
    }
#if 0 
    //not support set
    if( ( xStatus == eBTStatusSuccess ) && ( pxParams->bSetScanRsp == false ) )
    {
        ucFlags = ( GAP_ADV_FLAG_LE_GENERAL_DISC_MODE | GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED );
        xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_FLAGS, &ucFlags, 1 ,advType);
    }
#else    
    //avoid compile warning
    ucFlags = ucFlags;
#endif     
    if( ( pxParams->bIncludeTxPower == true ) && ( xStatus == eBTStatusSuccess ) )
    {
        int8_t txpwr_dbm = 0;
        ble_gap_tx_power_get(GAP_TX_POWER_ROLE_ADV, 0, &txpwr_dbm);
        xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_TRANSMIT_POWER, (uint8_t*)&txpwr_dbm, 1 ,advType);
    }

    if( ( xStatus == eBTStatusSuccess ) && ( pxParams->ulAppearance != 0 ) )
    {
        xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_APPEARANCE, ( uint8_t * ) &pxParams->ulAppearance, 4 ,advType);
    }

    if( ( pcManufacturerData != NULL ) && ( xStatus == eBTStatusSuccess ) )
    {
        xStatus = gr_util_adv_data_encode( ucMessageRaw, &ucMessageIndex, BLE_GAP_AD_TYPE_MANU_SPECIFIC_DATA, ( uint8_t * ) pcManufacturerData, usManufacturerLen ,advType);
    }

    if( ( pcServiceData != NULL ) && ( xStatus == eBTStatusSuccess ) )
    {
        //noot support 128_service_data        
        //pcServiceData
    }

    if(eBTStatusNoMem == xStatus){
        GRH_LOG(ERROR, ( "+++ prvBTSetAdvData, some params encode failed... "));
        //let it be successful
        xStatus = eBTStatusSuccess;
    }    

    if( xStatus == eBTStatusSuccess )
    {
        s_gr_ble_gap_params_ins.gap_adv_param.chnl_map     = GAP_ADV_CHANNEL_37_38_39;
        s_gr_ble_gap_params_ins.gap_adv_param.filter_pol   = GAP_ADV_ALLOW_SCAN_ANY_CON_ANY;          
        s_gr_ble_gap_params_ins.gap_adv_param.adv_intv_max = IOT_BLE_ADVERTISING_INTERVAL * 2;
        s_gr_ble_gap_params_ins.gap_adv_param.adv_intv_min = IOT_BLE_ADVERTISING_INTERVAL;
        s_gr_ble_gap_params_ins.gap_adv_param.disc_mode    = GAP_DISC_MODE_GEN_DISCOVERABLE;

        if( pxParams->usAdvertisingEventProperties == BTAdvInd ) {
            s_gr_ble_gap_params_ins.gap_adv_param.adv_mode = GAP_ADV_TYPE_ADV_IND;
        } else if( pxParams->usAdvertisingEventProperties == BTAdvNonconnInd ) {        
            s_gr_ble_gap_params_ins.gap_adv_param.adv_mode = GAP_ADV_TYPE_ADV_NONCONN_IND;
        }
        
        x_err = ble_gap_adv_param_set(0, BLE_GAP_OWN_ADDR_STATIC, &s_gr_ble_gap_params_ins.gap_adv_param);
        GRH_LOG(INFO,( "ble_gap_adv_param_set: ret: %d. ", x_err));
        xStatus = gr_util_to_afr_status_code(x_err);
    }
    
    if( xStatus == eBTStatusSuccess ){
        if( pxParams->bSetScanRsp == true )
        {
            memcpy(&s_gr_ble_gap_params_ins.scan_rsp_data[0], ucMessageRaw, ucMessageIndex);
            s_gr_ble_gap_params_ins.scan_rsp_data_len = ucMessageIndex;
            x_err = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, ucMessageRaw, ucMessageIndex);
            if( x_err != SDK_SUCCESS)
            {
                GRH_LOG(ERROR,( "Failed to configure scan response: %d. ", x_err));
            }            
            xStatus = gr_util_to_afr_status_code(x_err);
        }
        else
        {
            memcpy(&s_gr_ble_gap_params_ins.adv_data[0], ucMessageRaw, ucMessageIndex);
            s_gr_ble_gap_params_ins.adv_data_len = ucMessageIndex;
        	x_err = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, ucMessageRaw, ucMessageIndex);
            if( x_err != SDK_SUCCESS)
            {
                GRH_LOG(ERROR,( "Failed to configure scan response: %d. ", x_err));
            }            
            xStatus = gr_util_to_afr_status_code(x_err);
        }
    }
    
    GRH_LOG(INFO,( "prvBTSetAdvData result: %d. ", xStatus));
    
    if(xBTBleAdapterCallbacks.pxSetAdvDataCb != NULL){
        xBTBleAdapterCallbacks.pxSetAdvDataCb( xStatus );
    }

    return xStatus;    
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTSetAdvRawData( uint8_t ucAdapterIf,
                               uint8_t * pucData,
                               uint8_t ucLen )
{
    BTStatus_t xStatus = eBTStatusUnsupported;
    sdk_err_t x_err = SDK_SUCCESS;
    
    
    if(ucLen <= GR_BLE_ADV_DATA_LEN_MAX){
        x_err = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_DATA, pucData, ucLen);
    }
    
    if(x_err == SDK_SUCCESS){
        if(ucLen <= GR_BLE_SCAN_RSP_DATA_LEN_MAX){
            x_err = ble_gap_adv_data_set(0, BLE_GAP_ADV_DATA_TYPE_SCAN_RSP, pucData, ucLen);
        }
    }
    
    xStatus = gr_util_to_afr_status_code(x_err);
    GRH_LOG(INFO,( "prvBTSetAdvRawData result: %d. ", xStatus));

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTStartAdv( uint8_t ucAdapterIf )
{
    BTStatus_t xStatus;
    sdk_err_t x_err;
    
    
    s_gr_ble_gap_params_ins.gap_adv_time_param.duration     = 0;
    s_gr_ble_gap_params_ins.gap_adv_time_param.max_adv_evt  = 0;
    
    x_err = ble_gap_adv_start(0, &s_gr_ble_gap_params_ins.gap_adv_time_param);    
    xStatus = gr_util_to_afr_status_code(x_err);
    GRH_LOG(INFO,( "prvBTStartAdv result: %d. ", xStatus));
    if(xStatus != eBTStatusSuccess){
        //if fail happens, call callback directly
        if( xBTBleAdapterCallbacks.pxAdvStartCb )
        {
            xBTBleAdapterCallbacks.pxAdvStartCb( xStatus, ucAdapterIf );
        }
    } else {
        //if not failed here, it will call the callback when adv started over
        //nothing to do
    }
    
    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTStopAdv( uint8_t ucAdapterIf )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    sdk_err_t  x_err   = SDK_SUCCESS;
    
    if(!s_gr_ble_gap_params_ins.is_adv_started){
        xStatus = eBTStatusSuccess;
    } else {
        x_err = ble_gap_adv_stop(0);    
        xStatus = gr_util_to_afr_status_code(x_err);        
    }
    GRH_LOG(INFO,( "prvBTStopAdv result: %d. err: %d  ", xStatus, x_err));
    
    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTReadRemoteRssi( uint8_t ucAdapterIf,
                                const BTBdaddr_t * pxBdAddr )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTScanFilterParamSetup( BTGattFiltParamSetup_t xFiltParam )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTScanFilterAddRemove( uint8_t ucAdapterIf,
                                     uint32_t ulAction,
                                     uint32_t ulFiltType,
                                     uint32_t ulFiltIndex,
                                     uint32_t ulCompanyId,
                                     uint32_t ulCompanyIdMask,
                                     const BTUuid_t * pxUuid,
                                     const BTUuid_t * pxUuidMask,
                                     const BTBdaddr_t * pxBdAddr,
                                     char cAddrType,
                                     size_t xDataLen,
                                     char * pcData,
                                     size_t xMaskLen,
                                     char * pcMask )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTScanFilterEnable( uint8_t ucAdapterIf,
                                  bool bEnable )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTTransport_t prvBTGetDeviceType( const BTBdaddr_t * pxBdAddr )
{
    //BTStatus_t xStatus = eBTStatusUnsupported;
    return BTTransportLe;
}

/*-----------------------------------------------------------*/

BTStatus_t prvAFRUUIDtoGoodix( BTUuid_t * pxAFRUuid, ble_uuid_t * pxGoodixUUID)
{
    configASSERT(pxGoodixUUID != NULL);
    
    switch( pxAFRUuid->ucType )
    {
        case eBTuuidType16:
            pxGoodixUUID->uuid_len = 2;
            break;

        case eBTuuidType32:
            //pxGoodixUUID->uuid_len = 4;
            //not supported
            return eBTStatusParamInvalid;
            //break;
        
        case eBTuuidType128:
            pxGoodixUUID->uuid_len = 16;            
            break;

        default:
            return eBTStatusParamInvalid;
    }
    /*
    * IMPORTANT: find where to free this pointer to avoid memory leak!
     */
    pxGoodixUUID->uuid = pvPortMalloc(pxGoodixUUID->uuid_len);
    
    if(pxGoodixUUID->uuid == NULL){
        return eBTStatusNoMem;
    }
    
    memset(pxGoodixUUID->uuid, 0 ,pxGoodixUUID->uuid_len);
    
    switch( pxAFRUuid->ucType )
    {
        case eBTuuidType16:
            memcpy(pxGoodixUUID->uuid, &pxAFRUuid->uu.uu16, 2);
            break;

        case eBTuuidType32:
            memcpy(pxGoodixUUID->uuid, &pxAFRUuid->uu.uu32, 4);
            break;
        
        case eBTuuidType128:
            for( int i = 0; i < 16; ++i )
            {
                pxGoodixUUID->uuid[ i ] = pxAFRUuid->uu.uu128[ i ];
            }            
            break;

        default:
            break;
    }

    return eBTStatusSuccess;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTSetScanParameters( uint8_t ucAdapterIf,
                                   uint32_t ulScanInterval,
                                   uint32_t ulScanWindow )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTMultiAdvEnable( uint8_t ucAdapterIf,
                                BTGattAdvertismentParams_t * xAdvParams )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTMultiAdvUpdate( uint8_t ucAdapterIf,
                                BTGattAdvertismentParams_t * advParams )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTMultiAdvSetInstData( uint8_t ucAdapterIf,
                                     bool bSetScanRsp,
                                     bool bIncludeName,
                                     bool bInclTxpower,
                                     uint32_t ulAppearance,
                                     size_t xManufacturerLen,
                                     char * pcManufacturerData,
                                     size_t xServiceDataLen,
                                     char * pcServiceData,
                                     BTUuid_t * pxServiceUuid,
                                     size_t xNbServices )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTMultiAdvDisable( uint8_t ucAdapterIf )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTBatchscanCfgStorage( uint8_t ucAdapterIf,
                                     uint32_t ulBatchScanFullMax,
                                     uint32_t ulBatchScanTruncMax,
                                     uint32_t ulBatchScanNotifyThreshold )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTBatchscanEndBatchScan( uint8_t ucAdapterIf,
                                       uint32_t ulScanMode,
                                       uint32_t ulScanInterval,
                                       uint32_t ulScanWindow,
                                       uint32_t ulAddrType,
                                       uint32_t ulDiscardRule )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTBatchscanDisBatchScan( uint8_t ucAdapterIf )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTBatchscanReadReports( uint8_t ucAdapterIf,
                                      uint32_t ulScanMode )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTSetPreferredPhy( uint16_t usConnId,
                                 uint8_t ucTxPhy,
                                 uint8_t ucRxPhy,
                                 uint16_t usPhyOptions )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTReadPhy( uint16_t usConnId,
                         BTReadClientPhyCallback_t xCb )
{
    BTStatus_t xStatus = eBTStatusUnsupported;

    return xStatus;
}

/*-----------------------------------------------------------*/

const void * prvBTGetGattClientInterface()
{
    return NULL;
}

/*-----------------------------------------------------------*/

const void * prvBTGetLeAdapter(void)
{
    return &xBTLeAdapter;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTConnParameterUpdateRequest( const BTBdaddr_t * pxBdAddr,
                                            uint32_t ulMinInterval,
                                            uint32_t ulMaxInterval,
                                            uint32_t ulLatency,
                                            uint32_t ulTimeout )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    sdk_err_t  x_err   = SDK_SUCCESS;
    gap_conn_update_param_t gap_conn_param;
    
    s_gr_ble_gap_params_ins.gap_conn_param.pxBdAddr         = pxBdAddr;
    s_gr_ble_gap_params_ins.gap_conn_param.ulMinInterval    = ulMinInterval;
    s_gr_ble_gap_params_ins.gap_conn_param.ulMaxInterval    = ulMaxInterval;
    s_gr_ble_gap_params_ins.gap_conn_param.ulLatency        = ulLatency;
    s_gr_ble_gap_params_ins.gap_conn_param.ulTimeout        = ulTimeout;
    
    gap_conn_param.interval_min     =  (uint16_t)ulMinInterval;
    gap_conn_param.interval_max     =  (uint16_t)ulMaxInterval;
    gap_conn_param.slave_latency    =  (uint16_t)ulLatency;
    gap_conn_param.sup_timeout      =  (uint16_t)ulTimeout;
    
    x_err = ble_gap_conn_param_update(0, &gap_conn_param);        
    xStatus = gr_util_to_afr_status_code(x_err);
    
    GRH_LOG(INFO, ("ble_gap_conn_param_update ret: %d  ", xStatus));
    
    if(xStatus != eBTStatusSuccess){
        if(xBTBleAdapterCallbacks.pxConnParameterUpdateCb != NULL){
            xBTBleAdapterCallbacks.pxConnParameterUpdateCb( xStatus,
                                                                pxBdAddr,
                                                                ulMinInterval,
                                                                ulMaxInterval,
                                                                ulLatency,
                                                                ulMaxInterval,
                                                                ulTimeout );
        }
    } else {
        //called in gr gap's callback
    }
    
    return xStatus;    
}


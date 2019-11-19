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
 * @file iot_ble_hal_common_gap.c
 * @brief Hardware Abstraction Layer for GAP ble stack.
 */

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"

#include "bt_hal_manager_adapter_ble.h"
#include "bt_hal_manager.h"
#include "bt_hal_gatt_server.h"
#include "iot_ble_config.h"
#include "iot_ble_hal_internals.h"
#include "iot_ble_gap_config.h"

//GR5515 include
#include "ble_gapc.h"
#include "ble_gapm.h"
#include "ble_sec.h"

#include "gr_utils.h"
#include "gr_porting.h"
#include "gr_message.h"


/*************************************************************************************************
 ******************     GAP wrapper APIs need to be implemented                         **********
 *************************************************************************************************/

BTStatus_t prvBTManagerInit( const BTCallbacks_t * pxCallbacks );
BTStatus_t prvBtManagerCleanup( void );
BTStatus_t prvBTEnable( uint8_t ucGuestMode );
BTStatus_t prvBTDisable(void);
BTStatus_t prvBTGetAllDeviceProperties(void);
BTStatus_t prvBTGetDeviceProperty( BTPropertyType_t xType );
BTStatus_t prvBTSetDeviceProperty( const BTProperty_t * pxProperty );
BTStatus_t prvBTGetAllRemoteDeviceProperties( BTBdaddr_t * pxRemoteAddr );
BTStatus_t prvBTGetRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                         BTPropertyType_t type );
BTStatus_t prvBTSetRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                         const BTProperty_t * property );
BTStatus_t prvBTPair( const BTBdaddr_t * pxBdAddr,
                      BTTransport_t xTransport,
                      bool bCreateBond );
BTStatus_t prvBTCreateBondOutOfBand( const BTBdaddr_t * pxBdAddr,
                                     BTTransport_t xTransport,
                                     const BTOutOfBandData_t * pxOobData );
BTStatus_t prvBTSendSlaveSecurityRequest( const BTBdaddr_t * pxBdAddr,
                                                 BTSecurityLevel_t xSecurityLevel,
                                                 bool bBonding );
BTStatus_t prvBTCancelBond( const BTBdaddr_t * pxBdAddr );
BTStatus_t prvBTRemoveBond( const BTBdaddr_t * pxBdAddr );
BTStatus_t prvBTGetConnectionState( const BTBdaddr_t * pxBdAddr,
                                    bool * bConnectionState );
BTStatus_t prvBTPinReply( const BTBdaddr_t * pxBdAddr,
                          uint8_t ucAccept,
                          uint8_t ucPinLen,
                          BTPinCode_t * pxPinCode );
BTStatus_t prvBTSspReply( const BTBdaddr_t * pxBdAddr,
                          BTSspVariant_t xVariant,
                          uint8_t ucAccept,
                          uint32_t ulPasskey );
BTStatus_t prvBTReadEnergyInfo(void);
BTStatus_t prvBTDutModeConfigure( bool bEnable );
BTStatus_t prvBTDutModeSend( uint16_t usOpcode,
                             uint8_t * pucBuf,
                             size_t xLen );
BTStatus_t prvBTLeTestMode( uint16_t usOpcode,
                            uint8_t * pucBuf,
                            size_t xLen );
BTStatus_t prvBTConfigHCISnoopLog( bool bEnable );
BTStatus_t prvBTConfigClear(void);
BTStatus_t prvBTReadRssi( const BTBdaddr_t * pxBdAddr );
BTStatus_t prvBTGetTxpower( const BTBdaddr_t * pxBdAddr,
                            BTTransport_t xTransport );
const void * prvBTGetClassicAdapter(void);
const void * prvBTGetLeAdapter(void);
//prvBTGetLeAdapter is implemented in another file
uint32_t     prvBTGetLastError(void);


static BTInterface_t xBTinterface =
{
    .pxBtManagerInit                = prvBTManagerInit,
    .pxBtManagerCleanup             = prvBtManagerCleanup,
    .pxEnable                       = prvBTEnable,
    .pxDisable                      = prvBTDisable,
    .pxGetAllDeviceProperties       = prvBTGetAllDeviceProperties,
    .pxGetDeviceProperty            = prvBTGetDeviceProperty,
    .pxSetDeviceProperty            = prvBTSetDeviceProperty,
    .pxGetAllRemoteDeviceProperties = prvBTGetAllRemoteDeviceProperties,
    .pxGetRemoteDeviceProperty      = prvBTGetRemoteDeviceProperty,
    .pxSetRemoteDeviceProperty      = prvBTSetRemoteDeviceProperty,
    .pxPair                         = prvBTPair,
    .pxCreateBondOutOfBand          = prvBTCreateBondOutOfBand,
    .pxSendSlaveSecurityRequest     = prvBTSendSlaveSecurityRequest,
    .pxCancelBond                   = prvBTCancelBond,
    .pxRemoveBond                   = prvBTRemoveBond,
    .pxGetConnectionState           = prvBTGetConnectionState,
    .pxPinReply                     = prvBTPinReply,
    .pxSspReply                     = prvBTSspReply,
    .pxReadEnergyInfo               = prvBTReadEnergyInfo,
    .pxDutModeConfigure             = prvBTDutModeConfigure,
    .pxDutModeSend                  = prvBTDutModeSend,
    .pxLeTestMode                   = prvBTLeTestMode,
    .pxConfigHCISnoopLog            = prvBTConfigHCISnoopLog,
    .pxConfigClear                  = prvBTConfigClear,
    .pxReadRssi                     = prvBTReadRssi,
    .pxGetTxpower                   = prvBTGetTxpower,
    .pxGetClassicAdapter            = prvBTGetClassicAdapter,
    .pxGetLeAdapter                 = prvBTGetLeAdapter,
    .pxGetLastError                 = prvBTGetLastError,
};

/*-----------------------------------------------------------*/
// PUBLIC

const BTInterface_t * BTGetBluetoothInterface(void)
{
    return &xBTinterface;
}

/******************************************************************************************************/

BTCallbacks_t       xBTCallbacks;
BTProperties_t      xProperties;
uint8_t             pucBondedAddresses[ GAP_ADDR_LEN * MAX_BOND_NUM ];

sec_param_t         xGRSecurityParameters = {
    .level          = SEC_MODE1_LEVEL3,
    .io_cap         = IO_DISPLAY_YES_NO,//IO_NO_INPUT_NO_OUTPUT,
    .oob            = pdFALSE,
    .auth           = AUTH_BOND | AUTH_MITM | AUTH_SEC_CON,
    .key_size       = 16,
    .ikey_dist      = KDIST_ENCKEY | KDIST_IDKEY,
    .rkey_dist      = KDIST_ENCKEY | KDIST_IDKEY,
};

gap_sec_key_t       xGRGapSecKey = {
    .key            = {0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xef, \
                       0xfe,0xdc,0xba,0x98,0x76,0x54,0x32,0x10}
};


static void prvGrBTPrintSecParams(void){
    GRH_LOG(INFO, ("Sec Param: level : %d, io_cap:%d, auth:%d, ikey:%d, rkey:%d ", 
                                            xGRSecurityParameters.level, 
                                            xGRSecurityParameters.io_cap, 
                                            xGRSecurityParameters.auth, 
                                            xGRSecurityParameters.ikey_dist, 
                                            xGRSecurityParameters.rkey_dist));    
}

static uint16_t prvGrBTUpdateSecParams(void){
    if(xProperties.bBondable){
        xGRSecurityParameters.auth |= AUTH_BOND;        
        xGRSecurityParameters.ikey_dist |= KDIST_ENCKEY | KDIST_IDKEY;
        xGRSecurityParameters.rkey_dist |= KDIST_ENCKEY | KDIST_IDKEY;
    } else {
        xGRSecurityParameters.auth &= ~AUTH_BOND;
        xGRSecurityParameters.ikey_dist &= ~(KDIST_ENCKEY | KDIST_IDKEY);
        xGRSecurityParameters.rkey_dist &= ~(KDIST_ENCKEY | KDIST_IDKEY);
    } 
    
    if(xProperties.bOnlySecure){
        xGRSecurityParameters.auth |= AUTH_SEC_CON;
    } else {
        xGRSecurityParameters.auth &= ~AUTH_SEC_CON;
    }
    
    xGRSecurityParameters.level = gr_util_convert_to_goodix_security_level(&xGRSecurityParameters);
    
    prvGrBTPrintSecParams();
    
    return ble_sec_params_set(&xGRSecurityParameters);
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTManagerInit( const BTCallbacks_t * pxCallbacks )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    uint8_t     count = 10;
    
    while(!s_gr_ble_common_params_ins.is_ble_initialized && (count > 0)){
        GRH_LOG(INFO, (">>> wait ble stack init..."));
        count --;
        vTaskDelay(500);        
    }
    
    //gr_mm_init();
    gr_ble_cb_msg_task_startup(NULL);

    memset( &xProperties, 0, sizeof( xProperties ) );
    xProperties.xDeviceType         = eBTdeviceDevtypeBle;
    /* Set the device name from the iot_ble_config.h. We store it without a trailing zero. */
    xProperties.usDeviceNameLength  = strlen( IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME );
    xProperties.puDeviceName        = ( uint8_t * ) pvPortMalloc( xProperties.usDeviceNameLength );
    xProperties.xLocalMtu           = IOT_BLE_PREFERRED_MTU_SIZE;
    xProperties.xExchangedMtu       = IOT_BLE_PREFERRED_MTU_SIZE;
    xProperties.bOnlySecure         = IOT_BLE_ENABLE_SECURE_CONNECTION;     //support Security connection in default
    xProperties.bBondable           = IOT_BLE_ENABLE_BONDING;     //support bondable prop in default
        
    ble_gap_pair_enable(true);
    ble_gap_privacy_params_set(900, true);
    ble_gap_irk_set(&xGRGapSecKey);
    prvGrBTUpdateSecParams();
    ble_gap_l2cap_params_set(IOT_BLE_PREFERRED_MTU_SIZE, IOT_BLE_PREFERRED_MTU_SIZE, GR_BLE_MAX_NB_LECB_DEFUALT);

    if( xProperties.puDeviceName != NULL )
    {
        memcpy( xProperties.puDeviceName, IOT_BLE_DEVICE_COMPLETE_LOCAL_NAME, xProperties.usDeviceNameLength );
    }
    else
    {
        xStatus = eBTStatusNoMem;
    }

    if( pxCallbacks != NULL )
    {
        xBTCallbacks = *pxCallbacks;
    }
    else
    {
        xStatus = eBTStatusFail;
    }

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBtManagerCleanup()
{
    BTStatus_t xStatus = eBTStatusSuccess;

    if(xProperties.puDeviceName != NULL)
        vPortFree( xProperties.puDeviceName );

    gr_ble_cb_msg_task_deinit();
    prvBTGattValueHandleDeleteAll();
    
    //gr_mm_report();
    
    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTEnable( uint8_t ucGuestMode )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    
    /** If status is ok and callback is set, trigger the callback.
     *  If status is fail, not need to trig a callback as original call failed.
     **/
    if( ( xStatus == eBTStatusSuccess ) && ( xBTCallbacks.pxDeviceStateChangedCb != NULL ) )
    {
        xBTCallbacks.pxDeviceStateChangedCb( eBTstateOn );
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTDisable(void)
{
    BTStatus_t xStatus = eBTStatusSuccess;

    /** If status is ok and callback is set, trigger the callback.
     *  If status is fail, not need to trig a callback as original call failed.
     **/
    if( ( xStatus == eBTStatusSuccess ) && ( xBTCallbacks.pxDeviceStateChangedCb != NULL ) )
    {
        xBTCallbacks.pxDeviceStateChangedCb( eBTstateOff );
    }

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTGetAllDeviceProperties(void)
{
    return eBTStatusUnsupported;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTGetDeviceProperty( BTPropertyType_t xType )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    BTProperty_t xReturnedProperty;
    uint16_t rStatus = SDK_SUCCESS;

    if( xBTCallbacks.pxAdapterPropertiesCb != NULL )
    {
        xReturnedProperty.xType = xType;

        switch( xType )
        {
            case eBTpropertyBdname:
                xReturnedProperty.pvVal = xProperties.puDeviceName;
                xReturnedProperty.xLen = xProperties.usDeviceNameLength;
                xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                break;

            case eBTpropertyBdaddr:
                rStatus = ble_gap_addr_get( &xProperties.xDeviceAddress );
                if( rStatus == SDK_SUCCESS )
                {
                    memcpy( ( ( BTBdaddr_t * ) xReturnedProperty.pvVal )->ucAddress, xProperties.xDeviceAddress.gap_addr.addr, GAP_ADDR_LEN );
                    xReturnedProperty.xLen = GAP_ADDR_LEN;
                    xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                }
                else
                {
                    xStatus = eBTStatusFail;
                    xBTCallbacks.pxAdapterPropertiesCb( eBTStatusFail, 1, &xReturnedProperty);
                }                
                break;

            case eBTpropertyTypeOfDevice:
                xReturnedProperty.pvVal = &xProperties.xDeviceType;
                xReturnedProperty.xLen = sizeof( xProperties.xDeviceType );
                xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                break;

            case eBTpropertyLocalMTUSize:
                xReturnedProperty.pvVal = &xProperties.xLocalMtu;
                xReturnedProperty.xLen = sizeof( xProperties.xLocalMtu );
                xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                break;

            case eBTpropertyBondable:
                xReturnedProperty.pvVal = &xProperties.bBondable;
                xReturnedProperty.xLen = sizeof( bool );
                xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                break;

            case eBTpropertyIO:
                switch( xGRSecurityParameters.io_cap )
                {
                    case IO_DISPLAY_ONLY:
                        xProperties.xIOTypes = eBTIODisplayOnly;
                        break;

                    case IO_DISPLAY_YES_NO:
                        xProperties.xIOTypes = eBTIODisplayYesNo;
                        break;

                    case IO_KEYBOARD_ONLY:
                        xProperties.xIOTypes = eBTIOKeyboardOnly;
                        break;

                    case IO_KEYBOARD_DISPLAY:
                        xProperties.xIOTypes = eBTIOKeyboardDisplay;
                        break;

                    default:
                        xProperties.xIOTypes = eBTIONone;
                        break;
                }
                xReturnedProperty.pvVal = &xProperties.xIOTypes;
                xReturnedProperty.xLen = sizeof( xProperties.xIOTypes );
                xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                break;

            case eBTpropertySecureConnectionOnly:
                xReturnedProperty.pvVal = &xProperties.bOnlySecure;
                xReturnedProperty.xLen = 1;
                xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                break;
            
            case eBTpropertyAdapterBondedDevices:
            {
                // get bond list
                bond_dev_list_t bond_list;
                uint8_t i = 0;
                memset(&bond_list, 0, sizeof(bond_dev_list_t));
                rStatus = ble_gap_bond_devs_get(&bond_list);
                if( rStatus == SDK_SUCCESS ){
                    for(i = 0; i < bond_list.num; i++) {
                        memcpy(&pucBondedAddresses[i*GAP_ADDR_LEN], &(bond_list.items[i].gap_addr.addr)[0], GAP_ADDR_LEN);
                    }
                    
                    xReturnedProperty.pvVal = &pucBondedAddresses[0];
                    xReturnedProperty.xLen = bond_list.num;
                    xBTCallbacks.pxAdapterPropertiesCb( eBTStatusSuccess, 1, &xReturnedProperty );
                } else {
                    xReturnedProperty.pvVal = &pucBondedAddresses[0];
                    xReturnedProperty.xLen = 0;
                    xBTCallbacks.pxAdapterPropertiesCb( eBTStatusFail, 1, &xReturnedProperty );
                    xStatus = eBTStatusFail;
                }                                 
            }
            break;

            case eBTpropertyRemoteFriendlyName:                
            case eBTpropertyRemoteRssi:
            case eBTpropertyRemoteVersionInfo:
            case eBTpropertyConnectable:
            default:
                xStatus = eBTStatusUnsupported;
                xBTCallbacks.pxAdapterPropertiesCb( eBTStatusUnsupported, 0, NULL );
                break;            
        }
    }
    return xStatus;
}



/*-----------------------------------------------------------*/

BTStatus_t prvBTSetDeviceProperty( const BTProperty_t * pxProperty )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    uint16_t rStatus = SDK_SUCCESS;    

    switch( pxProperty->xType )
    {
        case eBTpropertyBdname:
            xProperties.usDeviceNameLength = pxProperty->xLen;
            if(xProperties.puDeviceName != NULL)
                vPortFree( xProperties.puDeviceName );
            xProperties.puDeviceName = ( uint8_t * ) pvPortMalloc( xProperties.usDeviceNameLength );

            if( xProperties.puDeviceName != NULL )
            {
                memcpy( xProperties.puDeviceName, ( pxProperty->pvVal ), xProperties.usDeviceNameLength );
                rStatus = ble_gap_device_name_set(BLE_GAP_WRITE_PERM_NOAUTH, xProperties.puDeviceName, xProperties.usDeviceNameLength);
                if(rStatus != SDK_SUCCESS){
                    xStatus = eBTStatusFail;
                }
            }
            else
            {
                xStatus = eBTStatusNoMem;
            }

            if( ( xBTCallbacks.pxAdapterPropertiesCb != NULL ) && ( xStatus == eBTStatusSuccess ) )
            {
                xBTCallbacks.pxAdapterPropertiesCb( xStatus, 1, ( BTProperty_t * ) pxProperty );
            }
            break;

            
        case eBTpropertyBdaddr:            
        {
            gap_bdaddr_t bd_addr;        
            
            if(pxProperty->pvVal != NULL)
            {
                configASSERT(GAP_ADDR_LEN == pxProperty->xLen);
                
                memset(&bd_addr, 0 ,sizeof(gap_bdaddr_t));            
                bd_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
                memcpy(bd_addr.gap_addr.addr, pxProperty->pvVal, GAP_ADDR_LEN);
            
                rStatus = ble_gap_addr_set(&bd_addr);
                
                if(rStatus != SDK_SUCCESS){
                    xStatus = eBTStatusFail;
                }
            }
            else
            {
                xStatus = eBTStatusParamInvalid;
            }
        
            if( ( xBTCallbacks.pxAdapterPropertiesCb != NULL ) && ( xStatus == eBTStatusSuccess ) )
            {
                xBTCallbacks.pxAdapterPropertiesCb( xStatus, 1, ( BTProperty_t * ) pxProperty );
            }
        }
        break;

        case eBTpropertyTypeOfDevice:
            xStatus = eBTStatusUnsupported;
            break;        

        case eBTpropertyIO:
            switch( *( ( BTIOtypes_t * ) pxProperty->pvVal ) )
            {
                case eBTIODisplayOnly:
                    xGRSecurityParameters.io_cap = IO_DISPLAY_ONLY ;
                    break;

                case eBTIODisplayYesNo:
                    xGRSecurityParameters.io_cap = IO_DISPLAY_YES_NO ;
                    break;

                case eBTIOKeyboardOnly:
                    xGRSecurityParameters.io_cap = IO_KEYBOARD_ONLY ;
                    break;

                case eBTIOKeyboardDisplay:
                    xGRSecurityParameters.io_cap = IO_KEYBOARD_DISPLAY ;
                    break;

                default:
                    xGRSecurityParameters.io_cap = IO_NO_INPUT_NO_OUTPUT ;
                    xGRSecurityParameters.auth &= ~AUTH_MITM;
                    break;
            }

            rStatus = prvGrBTUpdateSecParams();

            if( rStatus != SDK_SUCCESS )
            {
                xStatus = eBTStatusFail;
            }

            if( ( xBTCallbacks.pxAdapterPropertiesCb != NULL ) && ( xStatus == eBTStatusSuccess ) )
            {
                xBTCallbacks.pxAdapterPropertiesCb( xStatus, 1, ( BTProperty_t * ) pxProperty );
            }

            break;

        case eBTpropertySecureConnectionOnly:

            if( *( ( bool * ) pxProperty->pvVal ) )
            {
                if( xGRSecurityParameters.io_cap == IO_NO_INPUT_NO_OUTPUT )
                {
                    xStatus = eBTStatusUnsupported;
                }
                else
                {
                    xProperties.bOnlySecure = true;
                }
            }
            else
            {
                xProperties.bOnlySecure = false;
            }

            if( ( xBTCallbacks.pxAdapterPropertiesCb != NULL ) && ( xStatus == eBTStatusSuccess ) )
            {
                xBTCallbacks.pxAdapterPropertiesCb( xStatus, 1, ( BTProperty_t * ) pxProperty );
            }

            break;


        case eBTpropertyLocalMTUSize:
        {
            sdk_err_t err;
            xProperties.xLocalMtu = *( ( uint32_t * ) pxProperty->pvVal );
        
            if(xProperties.xLocalMtu > IOT_BLE_PREFERRED_MTU_SIZE){
                xProperties.xLocalMtu = IOT_BLE_PREFERRED_MTU_SIZE;
            } else if(xProperties.xLocalMtu < 23){
                xProperties.xLocalMtu = 23;
            }
            
            err = ble_gap_l2cap_params_set(IOT_BLE_PREFERRED_MTU_SIZE, xProperties.xLocalMtu, GR_BLE_MAX_NB_LECB_DEFUALT);
            
            xStatus = gr_util_to_afr_status_code(err);

            if( ( xBTCallbacks.pxAdapterPropertiesCb != NULL ) && ( xStatus == eBTStatusSuccess ) )
            {
                // trigger mtu exchange:
                //ble_gattc_mtu_exchange(s_gr_ble_gap_params_ins.cur_connect_id);
                xBTCallbacks.pxAdapterPropertiesCb( xStatus, 1, ( BTProperty_t * ) pxProperty );
            }
        }
            break;

        case eBTpropertyBondable:
        {
            xProperties.bBondable = *( ( bool * ) pxProperty->pvVal );
            
            prvGrBTUpdateSecParams();
   
            if( ( xBTCallbacks.pxAdapterPropertiesCb != NULL ) && ( xStatus == eBTStatusSuccess ) )
            {
                xBTCallbacks.pxAdapterPropertiesCb( xStatus, 1, ( BTProperty_t * ) pxProperty );
            }
        }
        break;
            
        default:
            xStatus = eBTStatusUnsupported;
    }

    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTGetAllRemoteDeviceProperties( BTBdaddr_t * pxRemoteAddr )
{
    return eBTStatusUnsupported;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTGetRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                         BTPropertyType_t type )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTSetRemoteDeviceProperty( BTBdaddr_t * pxRemoteAddr,
                                         const BTProperty_t * property )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTPair( const BTBdaddr_t * pxBdAddr,
                      BTTransport_t xTransport,
                      bool bCreateBond )
{
    xTransport = xTransport; /* We use BLE anyway */
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTCreateBondOutOfBand( const BTBdaddr_t * pxBdAddr,
                                     BTTransport_t xTransport,
                                     const BTOutOfBandData_t * pxOobData )
{
    return eBTStatusUnsupported;
}

BTStatus_t prvBTSendSlaveSecurityRequest( const BTBdaddr_t * pxBdAddr,
                                                 BTSecurityLevel_t xSecurityLevel,
                                                 bool bBonding ){
    return eBTStatusUnsupported;
}
                                                 
/*-----------------------------------------------------------*/

BTStatus_t prvBTCancelBond( const BTBdaddr_t * pxBdAddr )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTRemoveBond( const BTBdaddr_t * pxBdAddr )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    gap_bdaddr_t bd_addr;    
    uint16_t rStatus = SDK_SUCCESS;

    memset(&bd_addr, 0 , sizeof(gap_bdaddr_t));    
    memcpy(&(bd_addr.gap_addr.addr)[0], &pxBdAddr->ucAddress[0], btADDRESS_LEN);
    bd_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    
    rStatus = ble_gap_bond_dev_del(&bd_addr);
    
    if(rStatus == SDK_SUCCESS)
    {
        xStatus = eBTStatusSuccess;
        
        if( ( xBTCallbacks.pxBondedCb != NULL ) )
        {
            xBTCallbacks.pxBondedCb( xStatus, ( BTBdaddr_t * ) pxBdAddr, false );
        }
    } else {
        xStatus = eBTStatusFail;
    }
    
    return xStatus;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTGetConnectionState( const BTBdaddr_t * pxBdAddr,
                                    bool * bConnectionState )
{    
    return s_gr_ble_gap_params_ins.is_connected ? eBTStatusSuccess : eBTStatusFail;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTPinReply( const BTBdaddr_t * pxBdAddr,
                          uint8_t ucAccept,
                          uint8_t ucPinLen,
                          BTPinCode_t * pxPinCode )
{
    return eBTStatusUnsupported;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTSspReply( const BTBdaddr_t * pxBdAddr,
                          BTSspVariant_t xVariant,
                          uint8_t ucAccept,
                          uint32_t ulPasskey )
{
    BTStatus_t xStatus = eBTStatusSuccess;
    sdk_err_t err;

    if(s_gr_ble_gap_params_ins.is_need_sec_cfm){
        s_gr_ble_gap_params_ins.sec_cfm.accept = ucAccept;
        err = ble_sec_enc_cfm(s_gr_ble_gap_params_ins.cur_connect_id, &s_gr_ble_gap_params_ins.sec_cfm);
        xStatus = gr_util_to_afr_status_code(err);
        s_gr_ble_gap_params_ins.is_need_sec_cfm = false;
        
        GRH_LOG(INFO, (">>> prvBTSspReply, accept: %d, req_type:%d, ssp:%d, pass:%d ", s_gr_ble_gap_params_ins.sec_cfm.accept, 
                                                                                         s_gr_ble_gap_params_ins.sec_cfm.req_type,
                                                                                         xVariant, ulPasskey));
    }

    return xStatus;
}


/*-----------------------------------------------------------*/

BTStatus_t prvBTReadEnergyInfo(void)
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTDutModeConfigure( bool bEnable )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTDutModeSend( uint16_t usOpcode,
                             uint8_t * pucBuf,
                             size_t xLen )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTLeTestMode( uint16_t usOpcode,
                            uint8_t * pucBuf,
                            size_t xLen )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTConfigHCISnoopLog( bool bEnable )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTConfigClear(void)
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTReadRssi( const BTBdaddr_t * pxBdAddr )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

BTStatus_t prvBTGetTxpower( const BTBdaddr_t * pxBdAddr,
                            BTTransport_t xTransport )
{
    return eBTStatusUnsupported;
}

/*-----------------------------------------------------------*/

const void * prvBTGetClassicAdapter(void)
{
    return NULL;
}


uint32_t     prvBTGetLastError(void){
    return 0;
}


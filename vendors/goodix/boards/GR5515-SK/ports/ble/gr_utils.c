#include "user_app.h"
#include "gr_utils.h"
#include "bt_hal_manager_types.h"
#include "bt_hal_manager.h"
#include "ble_gapm.h"
#include "FreeRTOSConfig.h"
#include "iot_ble_hal_internals.h"


BTStatus_t gr_util_to_afr_status_code(sdk_err_t err){
    BTStatus_t status = eBTStatusFail;
    
    switch(err){
        case SDK_SUCCESS:   
            status = eBTStatusSuccess;  
            break;
        
        case SDK_ERR_POINTER_NULL:
        case SDK_ERR_INVALID_PARAM: 
        case SDK_ERR_INVALID_CONN_IDX:            
        case SDK_ERR_INVALID_HANDLE:
        case SDK_ERR_INVALID_BUFF_LENGTH:
        case SDK_ERR_INVALID_DATA_LENGTH:
        case SDK_ERR_INVALID_OFFSET:                   
        case SDK_ERR_INVALID_ATT_VAL_LEN:
        case SDK_ERR_INVALID_PERM:             
        case SDK_ERR_INVALID_ADV_IDX:
        case SDK_ERR_INVALID_ADV_DATA_TYPE:
        case SDK_ERR_INVALID_PSM_NUM:
        case SDK_ERR_INVALID_PSM_ALREADY_REGISTERED:
        case SDK_ERR_INVALID_PSM_EXCEEDED_MAX_PSM_NUM:
        case SDK_ERR_INVALID_ADDRESS:
        case SDK_ERR_INVALID_ADV_INTERVAL:
        case SDK_ERR_INVALID_DISVCOVERY_MODE:
        case SDK_ERR_INVALID_ADV_PARAM:
        case SDK_ERR_INVALID_ADV_PEER_ADDR:
        case SDK_ERR_INVALID_DURATION_PARAM:
            status = eBTStatusParamInvalid;  
            break;
        
        case SDK_ERR_BUSY:   
            status = eBTStatusBusy;  
            break;
        
        case SDK_ERR_NO_RESOURCES:   
            status = eBTStatusNoMem;  
            break;
        
        case SDK_ERR_REQ_NOT_SUPPORTED:
            status = eBTStatusUnsupported;  
            break;
        
        case SDK_ERR_DISCONNECTED:   
            status = eBTStatusLinkLoss;  
            break;
        
        default:   
            status = eBTStatusFail;  
            break;            
    }
    
    return status;
}

/*secp must not be null*/
BTSecurityLevel_t gr_util_adjust_to_aws_security_level(sec_mode_level_t gr_level){
    BTSecurityLevel_t aws_lev = eBTSecLevelNoSecurity;    
    
    switch(gr_level){
        case SEC_MODE1_LEVEL1:      aws_lev = eBTSecLevelNoSecurity;                break;
        case SEC_MODE1_LEVEL2:      aws_lev = eBTSecLevelUnauthenticatedPairing;    break;
        case SEC_MODE1_LEVEL3:      aws_lev = eBTSecLevelAuthenticatedPairing;      break;
        case SEC_MODE1_LEVEL4:      aws_lev = eBTSecLevelSecureConnect;             break;
        
        case SEC_MODE2_LEVEL1:
        case SEC_MODE2_LEVEL2:
        default:
            break;
    }
    
    return aws_lev;
}

/*secp must not be null*/
BTSecurityLevel_t gr_util_convert_to_aws_security_level(sec_param_t * secp){
    BTSecurityLevel_t lev = eBTSecLevelNoSecurity;
    
    if((secp->auth & AUTH_SEC_CON) && (secp->auth & AUTH_MITM)) { 
        lev = eBTSecLevelSecureConnect;             /**< Encrypted link is required. Necessary: MITM and SC. */
    } else if ( (!(secp->auth & AUTH_SEC_CON)) && (secp->auth & AUTH_MITM)) {
        lev = eBTSecLevelAuthenticatedPairing;      /**< Encrypted link is required. Necessary: MITM; unnecessary: SC. */
    } else if (  (!(secp->auth & AUTH_SEC_CON)) && 
                 (!(secp->auth & AUTH_MITM)) &&
                 (secp->auth & AUTH_BOND)
    ){
        lev = eBTSecLevelUnauthenticatedPairing;    //bondable
    } else {
        lev = eBTSecLevelNoSecurity;
    }
    
    return lev;
}

/*secp must not be null*/
sec_mode_level_t gr_util_convert_to_goodix_security_level(sec_param_t * secp){
    sec_mode_level_t lev = SEC_MODE1_LEVEL1;
    
    if(secp->ikey_dist & KDIST_SIGNKEY){
        if ( (!(secp->auth & AUTH_SEC_CON)) && (secp->auth & AUTH_MITM)){
            lev = SEC_MODE2_LEVEL2;                 /**< Data signing is required. Necessary: MITM; unnecessary: SC. */
        } else if( (!(secp->auth & AUTH_SEC_CON)) && 
                   (!(secp->auth & AUTH_MITM))
        ) {
            lev = SEC_MODE2_LEVEL1;                 /**< Data signing is required. Unnecessary: MITM and SC. */
        }
    } else {
        if((secp->auth & AUTH_SEC_CON) && (secp->auth & AUTH_MITM)) { 
            lev = SEC_MODE1_LEVEL4;             /**< Encrypted link is required. Necessary: MITM and SC. */
        } else if ( (!(secp->auth & AUTH_SEC_CON)) && (secp->auth & AUTH_MITM)) {
            lev = SEC_MODE1_LEVEL3;      /**< Encrypted link is required. Necessary: MITM; unnecessary: SC. */
        } else if (  (!(secp->auth & AUTH_SEC_CON)) && 
                     (!(secp->auth & AUTH_MITM)) &&
                     (secp->auth & AUTH_BOND)
        ){
            lev = SEC_MODE1_LEVEL2;    //bondable
        } else {
            lev = SEC_MODE1_LEVEL1;
        }
    }
    
    return lev;
}

/**
 * @Brief encode the adv data buffer, fill up the pucAdvMsg array. Return eBTStatusSuccess in case of success.
 */
BTStatus_t gr_util_adv_data_encode( uint8_t * pucAdvMsg,
                                    uint8_t * pucIndex,
                                    gap_ad_type_t xDType,
                                    uint8_t * pucData,
                                    const uint8_t ucDataLength, 
                                    gap_adv_data_type_t advType)
{    
    uint8_t max_len = (advType == BLE_GAP_ADV_DATA_TYPE_SCAN_RSP) ? GR_BLE_SCAN_RSP_DATA_LEN_MAX : GR_BLE_ADV_DATA_LEN_MAX;
    BTStatus_t xStatus = eBTStatusSuccess;
    
    GRH_LOG(INFO, ( "adv_data_encode, index: %d, data_len: %d, max_len:%d  ", *pucIndex, ucDataLength, max_len));

    if( ( *pucIndex ) + ucDataLength + 1 <= max_len )
    {
        pucAdvMsg[ ( *pucIndex )++ ] = ucDataLength + 1;
        pucAdvMsg[ ( *pucIndex )++ ] = xDType;
        memcpy( &pucAdvMsg[ *pucIndex ], pucData, ucDataLength );
        ( *pucIndex ) += ucDataLength;
    }
    else
    {
        GRH_LOG(INFO, ( "Advertising data can't fit in advertisement message.\n" ));
        xStatus = eBTStatusNoMem;
    }

    return xStatus;
}

bool gr_util_is_dev_bonded(gap_addr_t addr){
    bool            bonded = false;
    sdk_err_t       err; 
    bond_dev_list_t bond_list;
    err = ble_gap_bond_devs_get(&bond_list);

    if(err == SDK_SUCCESS){
        for(int i=0; i < bond_list.num; i++){
            if(memcmp(&addr.addr[0], &bond_list.items[i].gap_addr.addr[0], GAP_ADDR_LEN) == 0){
                bonded = true;
                break;
            }
        }
    } else {
        bonded = false;
    }
    
    return bonded;
}

void gr_util_print_buffer(BLE_GATTS_TYPE action, uint16_t handle, uint8_t * buff, uint32_t length){
    static uint8_t tbuff[512];
    uint32_t len = 0, i, m,n;
    
    
    memset(&tbuff[0], 0 , 512);
    
    len = strlen((char*) buff);    
    
    if(len == length) { //judge as printable string
        len = sprintf((char*)&tbuff[0], "%s\r\n", buff);
    } else {
        len = 0;
    }
    
    //print 64 bytes in max
    if(length > 64){
        length = 64;
    }
    
    if(true)
    {
        m = length / 10;
        n = length % 10;
        
        for(i=0; i < m; i++){
            len += sprintf((char*)&tbuff[len], "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", buff[i*10 + 0], buff[i*10 + 1], buff[i*10 + 2], buff[i*10 + 3], 
                                                                                                  buff[i*10 + 4], buff[i*10 + 5], buff[i*10 + 6], buff[i*10 + 7], 
                                                                                                  buff[i*10 + 8], buff[i*10 +9]);
        }
        
        if(n > 0){
            for(i=0; i<n ;i++){
                len += sprintf((char*)&tbuff[len], "%02x ", buff[m*10 + i]);
            }
            len += sprintf((char*)&tbuff[len], "\r\n");
        }
    }
    if(action == BLE_GATTS_TYPE_READ){
        GRH_LOG(DEBUG, ("+++++\r\n Read attr port handle: %d,len:%d \r\n %s +++++\r\n", handle, length, &tbuff[0]));
    } else if(action == BLE_GATTS_TYPE_WRITE){
        GRH_LOG(DEBUG,("+++++\r\n Write attr port handle: %d,len:%d \r\n %s +++++\r\n", handle, length, &tbuff[0]));
    } else {
        GRH_LOG(DEBUG,("+++++\r\n port handle: %d,len:%d \r\n %s +++++\r\n", handle, length, &tbuff[0]));
    }
}


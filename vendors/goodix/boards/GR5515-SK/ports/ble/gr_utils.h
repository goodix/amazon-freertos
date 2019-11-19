#ifndef __GR_UTILS_H__
#define __GR_UTILS_H__

#include "user_app.h"
#include "bt_hal_manager.h"
#include "ble_gapc.h"
#include "iot_ble_hal_internals.h"
#include "gr_utils.h"
#include "gr_config.h"

BTStatus_t gr_util_to_afr_status_code(sdk_err_t err);

/**
 * @Brief encode the adv data buffer, fill up the pucAdvMsg array. Return eBTStatusSuccess in case of success.
 */
BTStatus_t gr_util_adv_data_encode( uint8_t * pucAdvMsg,
                                    uint8_t * pucIndex,
                                    gap_ad_type_t xDType,
                                    uint8_t * pucData,
                                    const uint8_t ucDataLength, 
                                    gap_adv_data_type_t advType);

BTSecurityLevel_t gr_util_adjust_to_aws_security_level(sec_mode_level_t gr_level);
                                    
/*secp must not be null*/
BTSecurityLevel_t gr_util_convert_to_aws_security_level(sec_param_t * secp);

/*secp must not be null*/
sec_mode_level_t gr_util_convert_to_goodix_security_level(sec_param_t * secp);
                                    
void gr_util_print_buffer(BLE_GATTS_TYPE action, uint16_t handle, uint8_t * buff, uint32_t length);
                                    
//can be called after service added finished
void gr_util_gatt_handle_map_print(void);

bool gr_util_is_dev_bonded(gap_addr_t addr);

#endif /*__GR_UTILS_H__*/

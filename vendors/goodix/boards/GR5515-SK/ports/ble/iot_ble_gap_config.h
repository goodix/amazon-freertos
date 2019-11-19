#ifndef __AWS_BLE_GAP_CONFIG_H__
#define __AWS_BLE_GAP_CONFIG_H__

#include "custom_config.h"

#define GR_BLE_GATT_MAX_ENTITIES                            (40)
#define GR_BLE_ATTR_MASK_LEN                                (GR_BLE_GATT_MAX_ENTITIES/8+1)
#define GR_BLE_GATTS_VAR_ATTR_LEN_DEFAULT                   (64)
#define GR_BLE_GATTS_VAR_ATTR_LEN_MAX                       (512)   /**< Maximum length for variable length Attribute Values. */
#define GR_BLE_SRV_CONNECT_MAX                              (10 < CFG_MAX_CONNECTIONS ? 10 : CFG_MAX_CONNECTIONS)    /**< Maximum number of connections. */
#define GR_BLE_MAX_SERVICES                                 (10)
#define GR_BLE_GATT_PORTING_LAYER_START_HANDLE              (1)
#define GR_BLE_GATT_INVALID_HANDLE                          0xFFFF

#endif /* __AWS_BLE_GAP_CONFIG_H__ */

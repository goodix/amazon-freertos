#ifndef __GR_PORTING_H__
#define __GR_PORTING_H__

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "gr55xx_sys.h"
#include "iot_ble_gap_config.h"
#include "iot_ble_hal_internals.h"
#include "bt_hal_manager_types.h"
#include "user_app.h"
#include "gr_config.h"

typedef struct {
    bool                    is_connected;
    bool                    is_adv_started;
    bool                    is_mtu_exchanged;
    uint8_t                 cur_connect_id;
    uint8_t                 adv_data[GR_BLE_ADV_DATA_LEN_MAX];
    uint8_t                 adv_data_len;    
    uint8_t                 scan_rsp_data[GR_BLE_SCAN_RSP_DATA_LEN_MAX];
    uint8_t                 scan_rsp_data_len;    
    gap_adv_param_t         gap_adv_param;
    gap_adv_time_param_t    gap_adv_time_param;
    gap_conn_cmp_t          gap_conn_cmp_param;
    BTConnectionParams_t    gap_conn_param;
    
    BTUuid_t                register_app_uuid;
    
    bool                    is_need_sec_cfm;
    sec_cfm_enc_t           sec_cfm;
}gr_ble_gap_params_t;

typedef struct {
    bool                    is_gatt_initialized;
}gr_ble_gatt_params_t;

typedef struct {
    bool                    is_ble_initialized;         //ble stack inited ?
    gap_bdaddr_t            local_bd_addr;
}gr_ble_common_params_t;

typedef struct{
    uint16_t    cur_start_srv_index;
    uint16_t    register_srv_num;
    uint16_t    register_srv_handle[GR_BLE_MAX_SERVICES];
    uint16_t    start_handle;    
}gr_srv_env_t;

extern gr_ble_common_params_t       s_gr_ble_common_params_ins;
extern gr_ble_gap_params_t          s_gr_ble_gap_params_ins;
extern gr_ble_gatt_params_t         s_gr_ble_gatt_params_ins;
extern gr_srv_env_t                 s_gattsp_instance;
extern BTBleAdapterCallbacks_t      xBTBleAdapterCallbacks;
extern BTGattServerCallbacks_t      xGattServerCb;


void gr_ble_stack_init(void);
void gr_gatt_service_reset(void);

BTStatus_t gr_gatt_service_register_all(void) ;
BTStatus_t gr_gatt_service_register(uint16_t serviceHandle);

/*
 * transfer porting layer handle to gatt handle in ble stack
 * JUST Be called when connected
 */
uint16_t gr_gatt_transto_ble_stack_handle(uint16_t porting_handle);

/*
 * transfer gatt handle in ble stack to porting layer handle
 * JUST Be called when connected
 */
uint16_t gr_gatt_transto_porting_layer_handle(uint16_t stack_handle);


/****************************************************************************/
typedef struct
{
    SemaphoreHandle_t mutex;
    bool    is_valid;       //set TRUE when mutex is alloc ok
    bool    is_actived;     //set TRUE when the FreeRTOS is scheduled
} gr_mutex_t;

/**
 * @brief Implementation of gr_mutex_init
 *
 */
void gr_mutex_init( gr_mutex_t * mutex );

/**
 * @brief Implementation of gr_mutex_free for thread-safety.
 *
 */
void gr_mutex_free( gr_mutex_t * mutex );

/**
 * @brief Implementation of gr_mutex_lock
 *
 * @return true if successful, false if timeout,
 *
 */
bool gr_mutex_lock( gr_mutex_t * mutex );

/**
 * @brief Implementation of gr_mutex_unlock
 *
 * @return true if successful, false if timeout,
 *
 */
bool gr_mutex_unlock( gr_mutex_t * mutex );


/**
 * @brief Implementation of gr_mutex_activate
 *
 * @return true if successful, false if timeout,
 *
 */
void gr_mutex_set_activate( gr_mutex_t * mutex , bool active);

void gr_bsp_uart_init(void);

void gr_bsp_uart_send(uint8_t *p_data, uint16_t length);

void gr_bsp_uart_flush(void);

#endif /*__GR_PORTING_H__*/

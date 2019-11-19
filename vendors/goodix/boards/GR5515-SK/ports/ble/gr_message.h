#ifndef __GR_MESSAGE_H__
#define __GR_MESSAGE_H__

#include "gr55xx_sys.h"
#include "iot_ble_hal_internals.h"
#include "bt_hal_manager_types.h"
#include "user_app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ble_gapc.h"
#include "gr_config.h"

#define   GR_CB_MESSAGE_QUEUE_SIZE          10u
#define   GR_CB_MESSAGE_ARRAY_SIZE          (2*GR_CB_MESSAGE_QUEUE_SIZE)
#define   GR_CB_MESSAGE_TASK_STACK_SIZE     768u
#define   GR_CB_MESSAGE_TASK_PRIO           (tskIDLE_PRIORITY + 2)


typedef enum {    
    GR_CB_MSG_ADV_START = 0,
    GR_CB_MSG_GAP_CONNECT,
    GR_CB_MSG_GAP_DISCONNECT,
    GR_CB_MSG_GAP_CONNECT_UPDATE,
    GR_CB_MSG_GATT_MTU_EXCHANGE,
    GR_CB_MSG_GATTS_WRITE,
    GR_CB_MSG_GATTS_READ,
    GR_CB_MSG_GATTS_PREP_WRITE,
    GR_CB_MSG_GATTS_NTF_IND,
    GR_CB_MSG_SM_ENC_REQ,
    GR_CB_MSG_SM_PAIR_INC,
    GR_CB_MSG_SM_BOND_INC,
    
    GR_CB_MSG_KILL_TASK,        //kill the message handler task
} GR_CALLBACK_MSG_E;


typedef struct{
    GR_CALLBACK_MSG_E           msg_type;
    void *                      msg;
} GR_CALLBACK_MSG_T;

typedef struct {
    bool                        is_used;
    GR_CALLBACK_MSG_T           cb_msg;    
} GR_CALLBACK_MSG_ARRAY_T;

typedef struct{
    uint8_t                     gr_index;
    uint8_t                     gr_status;
    uint8_t                     gr_reason;
} GR_CB_MSG_BASIC_T;

typedef struct{
    GR_CB_MSG_BASIC_T           msg_basic;
    gap_conn_update_cmp_t       update;
} GR_CB_MSG_CONN_UPDATE_T;

typedef struct{
    GR_CB_MSG_BASIC_T           msg_basic;
    uint16_t                    mtu;
} GR_CB_MSG_MTU_EXCHANGE_T;

typedef struct {
    uint8_t                     gr_index;   
    uint16_t                    handle;
    uint16_t                    offset;
    uint16_t                    length;
    uint8_t *                   value;
} GR_CB_MSG_GATTS_WRITE_T;

typedef struct {
    uint8_t                     gr_index;
    gatts_read_req_cb_t         read_req;
} GR_CB_MSG_GATTS_READ_T;

typedef struct {
    uint8_t                     gr_index;
    gatts_prep_write_req_cb_t   prep_write_req;
} GR_CB_MSG_GATTS_PREP_WRITE_T;

typedef struct {
    GR_CB_MSG_BASIC_T           msg_basic;
    ble_gatts_ntf_ind_t         ntf_ind;
} GR_CB_MSG_NTF_IND_T;

typedef struct {
    uint8_t                     gr_index;
    sec_enc_req_t               enc_req;
} GR_CB_MSG_ENC_REQ_T;

typedef struct {
    uint8_t                     gr_index;
    sec_enc_ind_t               enc_pair_ind;
    uint8_t                     auth;
} GR_CB_MSG_PAIR_INC_T;

typedef GR_CB_MSG_PAIR_INC_T GR_CB_MSG_BOND_INC_T;

void                    gr_ble_cb_msg_task_startup(void * ctxt);
void                    gr_ble_cb_msg_task_deinit(void);
GR_CALLBACK_MSG_T *     gr_ble_cb_msg_alloc_mem(void);
void                    gr_ble_cb_msg_free_mem(GR_CALLBACK_MSG_T * buf);
bool                    gr_ble_cb_msg_send(GR_CALLBACK_MSG_T * msg, BaseType_t xHigherPriorityTaskWoken);
    
#endif /*__GR_MESSAGE_H__*/

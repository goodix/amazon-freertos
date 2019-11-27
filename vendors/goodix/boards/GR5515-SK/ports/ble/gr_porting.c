#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "app_error.h"
#include "gr551xx.h"
#include "user_app.h"
#include "user_periph_setup.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "gr_porting.h"
#include "gr_message.h"
#include "gr_utils.h"
#include "patch.h"
#include "iot_ble_hal_internals.h"
#include "boards.h"
#if APP_DRIVER_USE_ENABLE
#include "app_uart.h"
#else
#include "gr55xx_hal.h"
#endif


/*
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */

extern  gap_cb_fun_t            app_gap_callbacks;
extern  gatt_common_cb_fun_t    app_gatt_common_callback;
extern  gattc_cb_fun_t          app_gattc_callback;
extern  l2cap_lecb_cb_fun_t     app_l2cap_callback;
extern  sec_cb_fun_t            app_sec_callback;

gr_ble_common_params_t          s_gr_ble_common_params_ins = {
    .is_ble_initialized     = false,
};

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
/**@brief Stack global variables for Bluetooth protocol stack. */
STACK_HEAP_INIT(heaps_table);

static void ble_init_complete_callback(void);

static app_callback_t s_app_ble_callback = 
{
    .app_ble_init_cmp_callback = ble_init_complete_callback,
    .app_gap_callbacks         = &app_gap_callbacks,
    .app_gatt_common_callback  = &app_gatt_common_callback,
    .app_gattc_callback        = &app_gattc_callback,
    .app_sec_callback          = &app_sec_callback,
};

static void ble_init_complete_callback(void){
    gap_bdaddr_t bd_addr;
    sdk_err_t    error_code;

    error_code = ble_gap_addr_get(&bd_addr);    
    GRH_LOG(INFO, (">>> code: %d  ", error_code));
    GRH_LOG(INFO, (">>> ble_init_complete_callback called, Local Board %02X:%02X:%02X:%02X:%02X:%02X. \r\n",
                                         bd_addr.gap_addr.addr[5],
                                         bd_addr.gap_addr.addr[4],
                                         bd_addr.gap_addr.addr[3],
                                         bd_addr.gap_addr.addr[2],
                                         bd_addr.gap_addr.addr[1],
                                         bd_addr.gap_addr.addr[0]));   
    
    s_gr_ble_common_params_ins.is_ble_initialized = true;
    memcpy(&s_gr_ble_common_params_ins.local_bd_addr, &bd_addr, sizeof(gap_bdaddr_t));
}


void gr_ble_stack_init(void){
    s_gr_ble_common_params_ins.is_ble_initialized = false;
    ble_stack_init(&s_app_ble_callback, &heaps_table);
}


/*
 * transfer porting layer handle to gatt handle in ble stack
 * JUST Be called when connected
 */
uint16_t gr_gatt_transto_ble_stack_handle(uint16_t porting_handle){
    uint16_t stack_handle;

    if((porting_handle < GR_BLE_GATT_PORTING_LAYER_START_HANDLE) || (porting_handle == GR_BLE_GATT_INVALID_HANDLE)){
        stack_handle = GR_BLE_GATT_INVALID_HANDLE;
    } else {
        stack_handle = porting_handle - GR_BLE_GATT_PORTING_LAYER_START_HANDLE + s_gattsp_instance.start_handle;
    }    
    
    return stack_handle;
}

/*
 * transfer gatt handle in ble stack to porting layer handle
 * JUST Be called when connected
 */
uint16_t gr_gatt_transto_porting_layer_handle(uint16_t stack_handle){
    uint16_t porting_handle = 0;    
    if(stack_handle < s_gattsp_instance.start_handle){
        porting_handle = GR_BLE_GATT_INVALID_HANDLE;
    } else {
        porting_handle = (stack_handle - s_gattsp_instance.start_handle) + GR_BLE_GATT_PORTING_LAYER_START_HANDLE;
    }    
    GRH_LOG(DEBUG, (">>> Stack Handle %d transferred to Porting Handle: %d ", stack_handle, porting_handle));
    return porting_handle;
}


/**
 * @brief Implementation of gr_mutex_init
 *
 */
void gr_mutex_init( gr_mutex_t * mutex )
{
    if( mutex->is_valid == false )
    {
        mutex->is_actived   = false;
        mutex->mutex        = xSemaphoreCreateMutex();

        if( mutex->mutex != NULL )
        {
            mutex->is_valid = true;
        }        
    }
}

/**
 * @brief Implementation of gr_mutex_free for thread-safety.
 *
 */
void gr_mutex_free( gr_mutex_t * mutex )
{
    if( mutex->is_valid == true )
    {
        vSemaphoreDelete( mutex->mutex );
        mutex->is_valid     = 0;
        mutex->is_actived   = 0;
    }
}

/**
 * @brief Implementation of gr_mutex_lock
 *
 * @return true if successful, false if timeout,
 *
 */
bool gr_mutex_lock( gr_mutex_t * mutex )
{
    bool ret = false;

    if( mutex->is_valid && mutex->is_actived )
    {
        if( xSemaphoreTake( mutex->mutex, portMAX_DELAY ) )
        {
            ret = true;
        }
        else
        {
            ret = false;
        }
    }

    return ret;
}

/**
 * @brief Implementation of gr_mutex_unlock
 *
 * @return true if successful, false if timeout,
 *
 */
bool gr_mutex_unlock( gr_mutex_t * mutex )
{
    bool ret = false;

    if( mutex->is_valid && mutex->is_actived )
    {
        if( xSemaphoreGive( mutex->mutex ) )
        {
            ret = true;
        }
        else
        {
            ret = false;
        }
    }

    return ret;
}


/**
 * @brief Implementation of gr_mutex_activate
 *
 * @return true if successful, false if timeout,
 *
 */
void gr_mutex_set_activate( gr_mutex_t * mutex , bool active)
{
    if( mutex->is_valid == true )
    {
        mutex->is_actived = active;
    } else {
        mutex->is_actived = false;
    }
}




#if APP_DRIVER_USE_ENABLE

#define UART_RX_BUFF_SIZE				1u
#define UART_TX_BUFF_SIZE				2048u

static uint8_t s_uart_rx_buffer[UART_RX_BUFF_SIZE];
static uint8_t s_uart_tx_buffer[UART_TX_BUFF_SIZE];

extern void app_log_transfer_rx_char(char data);

void gr_bsp_uart_send(uint8_t *p_data, uint16_t length)
{
#if APP_DRIVER_USE_ENABLE
    app_uart_transmit_async(APP_UART_ID, p_data, length);
#endif
}

void gr_bsp_uart_flush(void)
{
#if APP_DRIVER_USE_ENABLE
    app_uart_flush(APP_UART_ID);
#endif
}

__WEAK void app_log_transfer_rx_char(char c) 
{
}

/*
 * Handle Rx Of UART0
 * __WEAK version in bsp.c
 */
void app_uart_evt_handler(app_uart_evt_t *p_evt)
{
    if(p_evt->type == APP_UART_EVT_RX_DATA) {
				if(p_evt->data.size == UART_RX_BUFF_SIZE) {
						app_log_transfer_rx_char(s_uart_rx_buffer[0]);
				}
				memset(&s_uart_rx_buffer[0], 0 , UART_RX_BUFF_SIZE);
				app_uart_receive_async(APP_UART_ID, &s_uart_rx_buffer[0], UART_RX_BUFF_SIZE);
		}
}

#endif

void gr_bsp_uart_init(void)
{
#if APP_DRIVER_USE_ENABLE

    app_uart_tx_buf_t uart_buffer;
    app_uart_params_t uart_param;

    uart_buffer.tx_buf       = s_uart_tx_buffer;
    uart_buffer.tx_buf_size  = UART_TX_BUFF_SIZE;

    uart_param.id                   = APP_UART_ID;
    uart_param.init.baud_rate       = APP_UART_BAUDRATE;
    uart_param.init.data_bits       = UART_DATABITS_8;
    uart_param.init.stop_bits       = UART_STOPBITS_1;
    uart_param.init.parity          = UART_PARITY_NONE;
    uart_param.init.hw_flow_ctrl    = UART_HWCONTROL_NONE;
    uart_param.init.rx_timeout_mode = UART_RECEIVER_TIMEOUT_ENABLE;
    uart_param.pin_cfg.rx.type      = APP_UART_RX_IO_TYPE;
    uart_param.pin_cfg.rx.pin       = APP_UART_RX_PIN;
    uart_param.pin_cfg.rx.mux       = APP_UART_RX_PINMUX;
    uart_param.pin_cfg.tx.type      = APP_UART_TX_IO_TYPE;
    uart_param.pin_cfg.tx.pin       = APP_UART_TX_PIN;
    uart_param.pin_cfg.tx.mux       = APP_UART_TX_PINMUX;
    uart_param.use_mode.type        = APP_UART_TYPE_INTERRUPT;

    app_uart_init(&uart_param, app_uart_evt_handler, &uart_buffer);
		
		app_uart_receive_async(APP_UART_ID, s_uart_rx_buffer, UART_RX_BUFF_SIZE);
#endif
}





void HardFault_Handler_C(unsigned int * hardfault_args)
{
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;
    
    __BKPT(0);
    
    stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);

    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);
    GRH_LOG(INFO,  ("\n\n[Hard fault handler - all numbers in hex]\n"));
    GRH_LOG(INFO,  ("R0 = %x\n", stacked_r0));
    GRH_LOG(INFO,  ("R1 = %x\n", stacked_r1));
    GRH_LOG(INFO,  ("R2 = %x\n", stacked_r2));
    GRH_LOG(INFO,  ("R3 = %x\n", stacked_r3));
    GRH_LOG(INFO,  ("R12 = %x\n", stacked_r12));
    GRH_LOG(INFO,  ("LR [R14] = %x  subroutine call return address\n", stacked_lr));
    GRH_LOG(INFO,  ("PC [R15] = %x  program counter\n", stacked_pc));
    GRH_LOG(INFO,  ("PSR = %x\n", stacked_psr));
    GRH_LOG(INFO,  ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38)))));
    GRH_LOG(INFO,  ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28)))));
    GRH_LOG(INFO,  ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C)))));
    GRH_LOG(INFO,  ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30)))));
    GRH_LOG(INFO,  ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C)))));
    GRH_LOG(INFO,  ("SCB_SHCSR = %x\n", SCB->SHCSR));    
    while (1);
}

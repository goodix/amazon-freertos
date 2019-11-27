#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "gr551xx.h"
#include "gr_porting.h"
#include "gr_message.h"
#include "gr_utils.h"
#include "gr_debug.h"

#if GR_DEBUG_CODE > 0u

static GR_MM_T                  gr_mm_table[GR_MM_TABLE_LEN];
static int32_t                  malloc_times = 0;

bool                            is_bkpt_open = false;

void gr_mm_init(void) {
    malloc_times = 0;
    memset(&gr_mm_table[0], 0 , sizeof(GR_MM_T)*GR_MM_TABLE_LEN);
}

void * gr_mm_malloc(uint32_t size){
    void * ptr = pvPortMalloc(size);
    
    if(ptr != NULL){
        for(int i = 0; i < GR_MM_TABLE_LEN; i++){
            if(gr_mm_table[i].is_saved == false){
                gr_mm_table[i].addr     = (uint32_t) ptr;
                gr_mm_table[i].size     = size;
                gr_mm_table[i].is_saved = true;
                malloc_times ++;
                break;
            }
        }
    }
    
    return ptr;
}

void gr_mm_free(void * ptr){
    if(ptr != NULL){
        for(int i = 0; i < GR_MM_TABLE_LEN; i++){
            if(gr_mm_table[i].addr == (uint32_t) ptr){
                gr_mm_table[i].addr     = 0;
                gr_mm_table[i].size     = 0;
                gr_mm_table[i].is_saved = false;
                malloc_times --;
                break;
            }
        }
        
        vPortFree(ptr);
    }
}

void gr_mm_report(void) {
    uint32_t total = 0;
    
    vTaskDelay(2000);
    GRH_LOG(INFO, ("+++ Memory Leak Test Report +++"));
    GRH_LOG(INFO, ("+++++++++++++++++++++++++++++++"));
    for(int i = 0; i < GR_MM_TABLE_LEN; i++){
        if(gr_mm_table[i].is_saved){
            GRH_LOG(INFO, ("+++ Memory Leak - Address:0x%x, Size: %d ", gr_mm_table[i].addr, gr_mm_table[i].size));
            total += gr_mm_table[i].size;
        }
    }
    
    GRH_LOG(INFO, ("+++ Memory Leak Total: %d bytes, %d times ", total, malloc_times));
    
    GRH_LOG(INFO, ("+++++++++++++++++++++++++++++++"));
    
    vTaskDelay(2000);
}


void gr_debug_force_bkpt(void){
    if(is_bkpt_open){
        __BKPT(0);
    } else {
        //
    }
}

void gr_debug_print_irq_prio(void){
    
    uint8_t prio = 0, i = 0; 
    
    GRH_LOG(DEBUG, ("++++++++++++++++++++++++++++++++++++++++++++++"));
    for(i=0;i<MAX_NUMS_IRQn;i++){
        prio = NVIC_GetPriority((IRQn_Type)i);        
        GRH_LOG(DEBUG, ("irq(%d) prio: 0x%x / %d", i, prio, prio>>4));
    }
    GRH_LOG(DEBUG, ("++++++++++++++++++++++++++++++++++++++++++++++\r\n"));
}


#endif /* GR_DEBUG_CODE */


static void gr_ble_stack_task(void * pvParameter){
#if GR_CALLBACK_TRACE_ENABLE == 0u
    //if callback trace is on, disable log sync
    //app_log_activate_thread_safe();
#endif
    
#if GR_DEBUG_CODE > 0u    
    UBaseType_t task_num=0, count = 0;
    TaskStatus_t *TStatus = NULL;
    uint32_t  tun_time = 0, i=0;
    
    //vTaskDelay(500);    
    gr_debug_print_irq_prio();
#endif
    
    //vTaskDelay(500);
    
    while(true) {
#if GR_DEBUG_CODE > 0u        
        /*Check Task stack*/
        task_num = uxTaskGetNumberOfTasks();        
        TStatus = pvPortMalloc(task_num* sizeof(TaskStatus_t));
        
        if(TStatus != NULL) {
            count = uxTaskGetSystemState(TStatus, task_num, &tun_time);
            GRH_LOG(INFO, ("id   name           prio    state   stack-left"));
            GRH_LOG(INFO, ("======================================================="));
            for(i=0; i< count; i++){
                GRH_LOG(INFO, ("%-5d%-15s%-8d%-8d%-8d", TStatus[i].xTaskNumber, TStatus[i].pcTaskName, 
                                TStatus[i].uxCurrentPriority, TStatus[i].eCurrentState, TStatus[i].usStackHighWaterMark));
            }
            GRH_LOG(INFO, ("---------------------------------------------"));
            
            vPortFree(TStatus);
        }
        
        /*Check heap info*/
        {
            uint32_t total_heap = configTOTAL_HEAP_SIZE;
            uint32_t now_free   = xPortGetFreeHeapSize();
            uint32_t min_free   = xPortGetMinimumEverFreeHeapSize();            
            uint32_t now_usage  = ((total_heap - now_free)*100)/total_heap;
            uint32_t max_usage  = ((total_heap - min_free)*100)/total_heap;
            
            GRH_LOG(INFO, ("CPU RAM: %-4d KBytes, Heap RAM: %-4d KBytes", 256, total_heap/1024));
            GRH_LOG(INFO, ("Cur used: %-6d Bytes, Cur free:%-6d Bytes, Cur usage: %-3d%%", total_heap-now_free, now_free, now_usage));
            GRH_LOG(INFO, ("Max used: %-6d Bytes, Min free:%-6d Bytes, Max usage: %-3d%%", total_heap-min_free, min_free, max_usage));            
            GRH_LOG(INFO, ("=======================================================\r\n"));
        }        
                
        vTaskDelay(1000);
        
        //__BKPT(0);
        
        //gr_jump2iap();
        
        //__set_FAULTMASK(1);
        //NVIC_SystemReset();
#else
        vTaskSuspend(NULL);
#endif
    }
}


static TaskHandle_t             s_gr_ble_task_handle;

void gr_ble_task_startup(void * ctxt) {

    BaseType_t xReturned = xTaskCreate(gr_ble_stack_task,
                                       "GR-BLE",
                                       GR_BLE_TASK_STACK_SIZE,
                                       ctxt,
                                       GR_BLE_TASK_PRIO,
                                       &s_gr_ble_task_handle);
    if (xReturned != pdPASS)
    {
        GRH_LOG(ERROR, ("xxx ble task not created. "));
    }
}

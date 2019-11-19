#ifndef __GR_CONFIG_H__
#define __GR_CONFIG_H__

/*
 * put All Goodix Porting Layer Config DEFINEs in this files
 */

/************************************************************
 * enable | disable the debug code
 ************************************************************/
#define     GR_DEBUG_CODE                       (0)

/************************************************************
 * enable | disable the trace info in goodix porting callback
 ************************************************************/
#define     GR_CALLBACK_TRACE_ENABLE            (0)

/************************************************************
 * enable | disable the trace info in goodix ble hal
 ************************************************************/
#define     GR_BLE_HAL_TRACE_ENABLE             (0)

/************************************************************
 * enable | disable the trace info in goodix sdk
 ************************************************************/
#define     GR_BLE_SDK_TRACE_ENABLE             (0)

/************************************************************
 * config trace level of Goodix related trace
 *      optional value: (err > warn > info > debug in prio)
 *          APP_LOG_LVL_ERROR   : just trace err
 *          APP_LOG_LVL_WARNING : just trace err,warn,
 *          APP_LOG_LVL_INFO    : just trace err,warm, info
 *          APP_LOG_LVL_DEBUG   : trace err,warm, info, debug
 ************************************************************/
#define     GR_TRACE_LEVEL                      APP_LOG_LVL_DEBUG



/************************************************************
 * optional value for TYPE, other values are invalid:
 *          ERROR   : print error log
 *          WARNING : print warn log
 *          INFO    : print info log
 *          DEBUG   : print denug log
 ************************************************************/
#if GR_CALLBACK_TRACE_ENABLE > 0u
    #define GRC_LOG(TYPE, X)    APP_LOG_##TYPE X
#else
    #define GRC_LOG(TYPE, X)    
#endif

#if GR_BLE_HAL_TRACE_ENABLE > 0u
    #define GRH_LOG(TYPE, X)    APP_LOG_##TYPE X
#else
    #define GRH_LOG(TYPE, X)    
#endif


#define GR_BLE_TASK_STACK_SIZE              512u
#define GR_BLE_TASK_PRIO                    tskIDLE_PRIORITY

#endif /*__GR_CONFIG_H__*/

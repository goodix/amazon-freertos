/*
 * Amazon FreeRTOS V1.4.7
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




/*
 * INCLUDE FILES
 *****************************************************************************************
 */

#include "gr55xx_sys.h"
#include "scatter_common.h"
#include "flash_scatter_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
//#include "watcher.h"
#include "custom_config.h"
#include "patch.h"
#include "gr55xx_delay.h"
#include "gr55xx_hal.h"
#include "user_periph_setup.h"
#include "app_log.h"
#include "aws_logging_task.h"
#include "aws_system_init.h"

#include "aws_demo.h"
#include "iot_mqtt.h"
#include "iot_ble.h"
#include "iot_ble_numericComparison.h"

#include "gr_porting.h"
#include "gr_debug.h"


/* Logging Task Defines. */
#define mainLOGGING_MESSAGE_QUEUE_LENGTH    ( 15 )
#define mainLOGGING_TASK_STACK_SIZE         ( 512u)

/**********************************************************************************************
 **** Quick Search 
 *
 * static const IotBleAttributeEventCallback_t _callbacks[ IOT_BLE_DATA_TRANSFER_MAX_ATTRIBUTES ]
 * 
 * APP_VERSION_MINOR   CONFIG_OTA_UPDATE_DEMO_ENABLED
 * 
 * IOT_MQTT_ENABLE_SERIALIZER_OVERRIDES (MQTT over ble)
 *
 **********************************************************************************************/
 
static QueueHandle_t UARTqueue = NULL;


static void prvMiscInitialization( void )
{
    app_periph_init();                                              /*<init user periph .*/     
    gr_ble_stack_init();    
    UARTqueue = xQueueCreate( 1, sizeof( INPUTMessage_t ) );    
    configASSERT(UARTqueue != NULL);
    
}


void app_log_transfer_rx_char(char data){
    static volatile uint8_t ucRxByte = 0;
    INPUTMessage_t  xInputMessage;
    BaseType_t      xHigherPriorityTaskWoken = pdFALSE;
    
    ucRxByte = data;
    
    xInputMessage.pcData = (uint8_t *)&ucRxByte;
    xInputMessage.xDataSize = 1;

    xQueueSendFromISR(UARTqueue, (void * )&xInputMessage, &xHigherPriorityTaskWoken);
}

/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
    /* Start the demo tasks. */
    DEMO_RUNNER_RunDemos();
}

/**
 *****************************************************************************************
 * @brief main function
 *****************************************************************************************
 */
int main (void)
{
    /* Perform any hardware initialization that does not require the RTOS to be
     * running.  */
    prvMiscInitialization();    
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );  
    NumericComparisonInit();
    gr_ble_task_startup(NULL);
    vTaskStartScheduler();                                      /*< freertos run all tasks*/
    return 0;                                                   /*< Never perform here */
}


BaseType_t getUserMessage( INPUTMessage_t * pxINPUTmessage, TickType_t xAuthTimeout )
{
    BaseType_t xReturnMessage = pdFALSE;
    INPUTMessage_t xTmpINPUTmessage;

    pxINPUTmessage->pcData = (uint8_t *)pvPortMalloc(sizeof(uint8_t));
    pxINPUTmessage->xDataSize = 1;

#if 0   //return Y for test    
    GRH_LOG(DEBUG, (">>> Get User Input: Y .... "));    
    if(pxINPUTmessage->pcData != NULL)
    {
        pxINPUTmessage->pcData[0] = 'Y';
        pxINPUTmessage->xDataSize = 1;
        xReturnMessage = pdTRUE;
    }
#else    
    if(pxINPUTmessage->pcData != NULL)
    {
        if (xQueueReceive(UARTqueue, (void * )&xTmpINPUTmessage, (portTickType) xAuthTimeout ))
        {
            *pxINPUTmessage->pcData = *xTmpINPUTmessage.pcData;
            pxINPUTmessage->xDataSize = xTmpINPUTmessage.xDataSize;
            xReturnMessage = pdTRUE;

            GRH_LOG(DEBUG, (">>> Recv User Input: %c  ", (uint8_t)*xTmpINPUTmessage.pcData));
        }
    }
#endif
      return xReturnMessage;
}



/*-----------------------------------------------------------*/

/**
 * @brief User defined assertion call. This function is plugged into configASSERT.
 * See FreeRTOSConfig.h to define configASSERT to something different.
 */
void vAssertCalled(const char * pcFile,
	uint32_t ulLine)
{
	const uint32_t ulLongSleep = 1000UL;
	volatile uint32_t ulBlockVariable = 0UL;
	volatile char * pcFileName = (volatile char *)pcFile;
	volatile uint32_t ulLineNumber = ulLine;

	(void)pcFileName;
	(void)ulLineNumber;

	APP_LOG_ERROR("vAssertCalled %s, %ld\n", pcFile, (long)ulLine);	
    
    __BKPT(0);
    
	/* Setting ulBlockVariable to a non-zero value in the debugger will allow
	* this function to be exited. */
	taskDISABLE_INTERRUPTS();
	{
		while (ulBlockVariable == 0UL)
		{
			vTaskDelay( pdMS_TO_TICKS( ulLongSleep ) );
		}
	}
	taskENABLE_INTERRUPTS();
}


#if ( configUSE_IDLE_HOOK == 1 )
    /**
     * @brief User defined Idle task function.
     *
     * @note Do not make any blocking operations in this function.
     */
    void vApplicationIdleHook( void )
    {
        /* FIX ME. If necessary, update to application idle periodic actions. */
        
        static TickType_t xLastPrint = 0;
        TickType_t xTimeNow;
        const TickType_t xPrintFrequency = pdMS_TO_TICKS( 5000 );

        xTimeNow = xTaskGetTickCount();

        if( ( xTimeNow - xLastPrint ) > xPrintFrequency )
        {
            APP_LOG_DEBUG( "." );
            xLastPrint = xTimeNow;
        }
    }
#endif /* configUSE_IDLE_HOOK */

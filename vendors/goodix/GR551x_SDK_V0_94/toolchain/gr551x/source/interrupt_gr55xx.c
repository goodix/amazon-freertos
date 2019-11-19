/******************************************************************************/
/*                 GR55xx BLE Peripherals Interrupt Handlers                  */
/*  Add here the Interrupt Handler for the BLE peripheral(s), for the         */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_gr55xx.s).                                                  */
/*  Note: The Application project should not contain this file when sdk.lib   */
/*        be linked.                                                          */
/******************************************************************************/

#include "gr55xx_sys.h"
#include "custom_config.h"

/**
  * @brief  This function handles SVC Handler.
  * @param  None
  * @retval None
  */
__asm void SVC_Handler (void) {
                PRESERVE8
                IMPORT  SVC_handler_proc
                
                TST     LR,#4                   ; Called from Handler Mode?
                MRSNE   R12,PSP                 ; Yes, use PSP
                MOVEQ   R12,SP                  ; No, use MSP
                PUSH    {R0-R3,LR}
                MOV  R0, R12
                BL  SVC_handler_proc
                MOV  R12, R0
                POP {R0-R3}
                CMP R12,#0                      //make sure current point isn't null
                BLXNE     R12
                POP {LR}
                //BKPT    #12
                TST     LR,#4
                MRSNE   R12,PSP
                MOVEQ   R12,SP
                STM     R12,{R0-R3}           ; Function return values
                BX      LR                      ; RETI

SVC_Dead
                B       SVC_Dead                ; None Existing SVC


                ALIGN
} 




 /*
  * @brief  This function handles HardFault Handler.
  * @param  None
  * @retval None
  */
#if ENABLE_FAULT_TRACE
uint32_t R4_R11_REG[8];
__WEAK void app_log_port_flush(void){};
void print_exception_stack(uint32_t sp)
{
    printf("HARDFAULT CALLSTACK INFO:\r\n");
    printf("================================\r\n");
    printf("  r0: %08x     r1: %08x\r\n",   ((uint32_t *)sp)[0], ((uint32_t *)sp)[1]);
    printf("  r2: %08x     r3: %08x\r\n",   ((uint32_t *)sp)[2], ((uint32_t *)sp)[3]);
    printf("  r4: %08x     r5: %08x\r\n",   R4_R11_REG[0],  R4_R11_REG[1] );
    printf("  r6: %08x     r7: %08x\r\n",   R4_R11_REG[2],  R4_R11_REG[3] );
    printf("  r8: %08x     r9: %08x\r\n",   R4_R11_REG[4],  R4_R11_REG[5] );
    printf("  r10:%08x     r11:%08x\r\n",   R4_R11_REG[6],  R4_R11_REG[7] );
    printf("  r12:%08x     lr: %08x\r\n",   ((uint32_t *)sp)[4], ((uint32_t *)sp)[5]);
    printf("  pc: %08x     xpsr: %08x\r\n", ((uint32_t *)sp)[6], ((uint32_t *)sp)[7]);
    printf("================================\r\n");
    app_log_port_flush();
    while (1);
}

__asm void HardFault_Handler (void) 
{
    PRESERVE8
    IMPORT  print_exception_stack
    IMPORT  R4_R11_REG
    LDR R0,=R4_R11_REG
    STMIA R0!,{R4-R11}
    MOV R0,SP
    BL  print_exception_stack
    ALIGN
}
#else

void HardFault_Handler (void) 
{
   while (1);
}

#endif

/**
  * @brief  This function handles MemManage Handler.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    while (1);
}

/**
  * @brief  This function handles BusFault Handler.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    while (1);
}

/**
  * @brief  This function handles UsageFault Handler.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    while (1);
}





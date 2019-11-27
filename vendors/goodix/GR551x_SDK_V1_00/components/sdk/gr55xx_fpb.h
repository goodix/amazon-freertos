#ifndef _FPB_H_
#define _FPB_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

typedef enum
{
    FPB_MODE_PATCH_ONLY = 0,                /* FPB MODE ENABLE FOR PATCH ONLY*/
    FPB_MODE_DEBUG_ONLY,                    /* FPB MODE ENABLE FOR DEBUG ONLY*/
    FPB_MODE_PATCH_AND_DEBUG,               /* FPB MODE ENABLE FOR PATCH AND DEBUG*/
} fpb_mode_t ;

typedef struct
{
    volatile uint32_t CTRL;                 /* Offset: 0x000 (R/W)  Data */ 
    volatile uint32_t REMAP;                /* Offset: 0x004 (R/W)  Data */
    volatile uint32_t COMP[8];              /* Offset: 0x008  (R)   Data */
} FPB_REG_TypeDef;
#define FPB               ((FPB_REG_TypeDef *)  0xE0002000UL)


typedef void(*fun_t)(void);

 /*
 ****************************************************************************************
 * @brief  Enabling patch function
 * @param[in] index_start :  Start Index Number
 * @param[in] index_end   :  End Index Number
 * @retval :  void
 ****************************************************************************************
 */
void fpb_enable(uint8_t index_start ,uint8_t index_end);

 /*
 ****************************************************************************************
 * @brief  Replace old and new functions
 * @param[in] ori_func : primitive function address
 * @param[in] rep_func : replacement function address
 * @param[in] patch_table_num : group number
 * @retval :  void
 ****************************************************************************************
 */
int fun_replace_by_svc(uint32_t ori_func, uint32_t rep_func, uint8_t patch_table_num);

 /*
 ****************************************************************************************
 * @brief  SVC handler process function
 * @retval :  void
 ****************************************************************************************
 */
uint32_t SVC_handler_proc(uint32_t *svc_args);
    
 /*
 ****************************************************************************************
 * @brief  Register FPB patch enable function
 * @param[in] patch_enable_func : pointer of function
 * @retval :  void
 ****************************************************************************************
 */
void fpb_register_patch_init_func(fun_t patch_enable_func);

 /*
 ****************************************************************************************
 * @brief  FPB init function
 * @param[in] fpb_mode : the mode of FPB 
 * @retval :  void
 ****************************************************************************************
 */
void fpb_init(fpb_mode_t fpb_mode);


 /*
 ****************************************************************************************
 * @brief  svc sub-function register
 * @param[in] svc_num : the number of svc 
 * @param[in] func : sub-function callback
 * @retval :  void
 ****************************************************************************************
 */
void svc_func_register(uint8_t svc_num, uint32_t func);


 /*
 ****************************************************************************************
 * @brief  register sve table function
 * @param[in] p_svc_table : the pointer of sve table
 * @retval :  void
 ****************************************************************************************
 */
void svc_table_register(uint32_t *p_svc_table);

#endif

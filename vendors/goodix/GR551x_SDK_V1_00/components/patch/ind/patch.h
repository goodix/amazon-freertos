#ifndef __PATCH_H_
#define __PATCH_H_

/**
 ****************************************************************************************
 *
 * @file patch.h
 *
 * @brief offer the interface for the patch function based on the FPB of the cortex arm-m4;
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
 * All Rights Reserved
 *
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */
enum
{
    BIT_CO_LIST_PUSH_BACK,
    BIT_CO_LIST_POP_FRONT,
    BIT_GAPM_SEND_COMPLETE_EVT,
    BIT_SCH_ARB_INSERT,  // it's used to realize the ble idle time notify function, not MANDATORY
    BIT_ATTMDB_SVC_INIT,
    BIT_LLC_PREF_PARAM_COMPUTE, // it's used to implement the anchor point movement, not MANDATORY
    BIT_LLC_HCI_CON_UPD_INFO_SEND, // it's used to implement the anchor point movement accuracy, not MANDATORY
    BIT_SCH_ARB_PROG_TIMER, // MANDATORY
    BIT_GAPM_SMP_RESOLV_OP_CONT, // it's used for multi-link(over 3 links) pair process
};

/*
 * MACRO DECLARATIONS
 ****************************************************************************************
 */
#define PATCH_ENABLE_FLAG(BIT) (1<<BIT)
//please add the macro for the different application(Only Support 6 patches);
#define MANDATORY_PATCH         ( PATCH_ENABLE_FLAG(BIT_CO_LIST_PUSH_BACK)         \
                                | PATCH_ENABLE_FLAG(BIT_CO_LIST_POP_FRONT)         \
                                | PATCH_ENABLE_FLAG(BIT_GAPM_SEND_COMPLETE_EVT)    \
                                | PATCH_ENABLE_FLAG(BIT_GAPM_SMP_RESOLV_OP_CONT)   \
                                | PATCH_ENABLE_FLAG(BIT_ATTMDB_SVC_INIT)           \
                                | PATCH_ENABLE_FLAG(BIT_SCH_ARB_PROG_TIMER)        \
                                )

#define OPTIMIZING_PATCH        0
                               
#define THROUGHPUT_PATCH        MANDATORY_PATCH

#define MULTI_LINK_PATCH        ( PATCH_ENABLE_FLAG(BIT_CO_LIST_PUSH_BACK)         \
                                | PATCH_ENABLE_FLAG(BIT_CO_LIST_POP_FRONT)         \
                                | PATCH_ENABLE_FLAG(BIT_GAPM_SEND_COMPLETE_EVT)    \
                                | PATCH_ENABLE_FLAG(BIT_GAPM_SMP_RESOLV_OP_CONT)   \
                                | PATCH_ENABLE_FLAG(BIT_ATTMDB_SVC_INIT)           \
                                | PATCH_ENABLE_FLAG(BIT_LLC_PREF_PARAM_COMPUTE)    \
                                )
                                
#define IDLE_NOTIFY_PATCH       ( PATCH_ENABLE_FLAG(BIT_CO_LIST_PUSH_BACK)         \
                                | PATCH_ENABLE_FLAG(BIT_CO_LIST_POP_FRONT)         \
                                | PATCH_ENABLE_FLAG(BIT_GAPM_SEND_COMPLETE_EVT)    \
                                | PATCH_ENABLE_FLAG(BIT_SCH_ARB_INSERT)            \
                                | PATCH_ENABLE_FLAG(BIT_ATTMDB_SVC_INIT)           \
                                | PATCH_ENABLE_FLAG(BIT_LLC_PREF_PARAM_COMPUTE)    \
                                )

/*
 * FUNCTION DECLARATIONS
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief  The enable of the patch featurn based on the FPB of the Cortex ARM-m4.
 *
 * @param[in]  patch_flag: The flag used to control the function to be selected as the patch function,one bit for one function.
 *
 * @note This parameter can be a combiantion of the following values:
 *       @arg @ref (1<<BIT_LLD_LLCP_OPCODE_IS_INVALID ,    )
 *       @arg @ref (1<<BIT_LLD_TEST_ISR,                )
 *       please used the MACRO PATCH_ENABLE_FLAG(BIT) just like the MANDATORY_PATCH;
 *       and the different MACRO maybe defined for the different application;
 *****************************************************************************************
 */
extern void set_patch_flag(uint32_t patch_flag);

/**
 *****************************************************************************************
 * @brief  Patch Enabling Function 
 *         This function can not be used directly. It needs to be registered with FPB unit 
 *         and automatically enabled by the system.
 *****************************************************************************************
 */
void fpb_patch_enable(void);

#endif  // __PATCH_H_


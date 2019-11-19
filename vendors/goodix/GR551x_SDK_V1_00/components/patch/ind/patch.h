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
    BIT_LLD_CI_SCHED,
    BIT_HCI_LE_SET_CIG_PARAMS_CMD_HANDLER,
    BIT_GAPM_OP_SETUP_CONTINUE,
    BIT_SMPC_IS_SEC_MODE_REACHED,
    BIT_GAPM_OP_RESET_CONTINUE,
    BIT_GAPM_PER_SYNC_SEND_SYNC_ESTABLISHED_IND,
    BIT_HCI_SEND_2_HOST,
    BIT_HCI_SEND_2_CONTROLLER,
    BIT_GAPM_SEND_COMPLETE_EVT,
    BIT_GAPM_ADV_CHECK_PARAM,
    BIT_CO_LIST_PUSH_BACK,
    BIT_CO_LIST_POP_FRONT,
    BIT_SCH_PROG_PUSH,
    BIT_LLD_CON_EVT_TIME_UPDATE,
    BIT_LLD_CON_FRM_CBK,
};

/*
 * MACRO DECLARATIONS
 ****************************************************************************************
 */
#define PATCH_ENABLE_FLAG(BIT) (1<<BIT)
//please add the macro for the different application(Only Support 6 patches);
// PATCH_ENABLE_FLAG(BIT_SCH_PROG_PUSH)
#define MANDATORY_PATCH         ( PATCH_ENABLE_FLAG(BIT_CO_LIST_PUSH_BACK)         \
                                | PATCH_ENABLE_FLAG(BIT_CO_LIST_POP_FRONT)         \
                                | PATCH_ENABLE_FLAG(BIT_GAPM_SEND_COMPLETE_EVT)    \
                                | PATCH_ENABLE_FLAG(BIT_LLD_CON_EVT_TIME_UPDATE)   \
                                | PATCH_ENABLE_FLAG(BIT_LLD_CON_FRM_CBK)           \
                                | PATCH_ENABLE_FLAG(BIT_SMPC_IS_SEC_MODE_REACHED)  \
                                )

#define OPTIMIZING_PATCH        0

#define ISO_PATCH               ( PATCH_ENABLE_FLAG(BIT_GAPM_OP_RESET_CONTINUE)                   \
                                | PATCH_ENABLE_FLAG(BIT_HCI_SEND_2_HOST)                          \
                                | PATCH_ENABLE_FLAG(BIT_HCI_SEND_2_CONTROLLER)                    \
                                | PATCH_ENABLE_FLAG(BIT_GAPM_PER_SYNC_SEND_SYNC_ESTABLISHED_IND)  \
                                )
                                
#define THROUGHPUT_PATCH        MANDATORY_PATCH

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


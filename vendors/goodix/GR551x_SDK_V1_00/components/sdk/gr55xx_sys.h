/**
 *******************************************************************************
 *
 * @file gr55xx_sys.h
 *
 * @brief GR55XX System API
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix Technology Co., Ltd
 * All Rights Reserved
 *
 *******************************************************************************
 */

/**
 @addtogroup SYSTEM
 @{
 */
 
/**
 * @addtogroup SYS System SDK
 * @{
 * @brief Definitions and prototypes for the system SDK interface.
*/


 
#ifndef __GR55XX_SYS_H__
#define __GR55XX_SYS_H__

#include "gr55xx_sys_cfg.h"
#include "gr55xx_nvds.h"
#include "gr55xx_pwr.h"
#include "gr55xx_fpb.h"
#include "ble.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/** @addtogroup GR55XX_SYS_DEFINES Defines
 * @{ */
#define SYS_INVALID_TIMER_ID              0xFF              /**< Invalid system Timer ID. */
#define SYS_BD_ADDR_LEN                   GAP_ADDR_LEN      /**< Length of Bluetoth Device Address. */
#define SYS_CHIP_UID_LEN                  0x10              /**< Length of Bluetoth Chip UID. */
#define SYS_SET_BD_ADDR(BD_ADDR_ARRAY)    nvds_put(0xC001, SYS_BD_ADDR_LEN, BD_ADDR_ARRAY)
/** @} */

/**
 * @defgroup GR55XX_SYS_TYPEDEF Typedefs
 * @{
 */
/**@brief The function pointers to register event callback. */
typedef void (*callback_t)(int);

/** @brief Timer callback type. */
typedef void (*timer_callback_t)(uint8_t timer_id);

/**@brief Printf callback type. */
typedef int (*vprintf_callback_t) (const char *fmt, __va_list argp); 
/** @} */ 

/** @addtogroup GR55XX_SYS_ENUMERATIONS Enumerations
* @{*/
/**@brief Device SRAM Size Enumerations definition. */
typedef enum
{
    SYS_DEV_SRAM_64K           = 0x02,    /**< Supported 64K SRAM                    */
    SYS_DEV_SRAM_128K          = 0x01,    /**< Supported 128K SRAM                  */
    SYS_DEV_SRAM_256K          = 0x00,    /**< Supported 256K SRAM                  */
} sram_size_t;

/** @} */

/** @addtogroup GR55XX_SYS_STRUCTURES Structures
 * @{ */
/**@brief SDK version define. */
typedef struct
{   
    uint8_t  major;                         /**< Major version*/
    uint8_t  minor;                         /**< Minor version*/
    uint16_t build;                         /**< Build number*/
    uint32_t commit_id;                     /**< commit ID*/
}sdk_version_t;

/**@brief Link RX information define. */
typedef struct
{
    uint32_t rx_total_cnt;     /**< Counts of RX times*/
    uint32_t rx_sync_err_cnt;  /**< Counts of RX sync error times*/
    uint32_t rx_crc_err_cnt;   /**< Counts of RX crc error times*/
    uint32_t rx_other_err_cnt; /**< Counts of RX other error times*/
    uint32_t rx_sn_err_cnt;    /**< Counts of sn CRC error times*/
    uint32_t rx_mic_err_cnt;   /**< Counts of mic CRC error times*/
    uint32_t rx_normal_cnt;    /**< Counts of RX normal times*/
} link_rx_info_t;

/**@brief RF trim parameter information define. */
typedef struct
{
    uint8_t  rssi_cali;
    uint8_t  tx_power;
} rf_trim_info_t;

/**@brief ADC trim parameter information define. */
typedef struct
{
    uint16_t adc_temp;
    uint16_t adc_slope;
    uint16_t adc_offset;
} adc_trim_info_t;

/**@brief PMU trim parameter information define. */
typedef struct
{
    uint8_t  io_ldo_vout;
    uint8_t  dig_ldo_64m;
    uint8_t  dig_ldo_16m;
    uint8_t  dcdc_vout;
} pmu_trim_info_t;


/** @} */

/** @addtogroup GR55XX_SYS_FUNCTIONS Functions
 * @{ */
/**
 *****************************************************************************************
 * @brief Output debug logs.
 *
 * @param[in] format: Pointer to the log information.
 *****************************************************************************************
 */
void sys_app_printf(const char *format, ...);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] us:  Microsecond.
 *****************************************************************************************
 */
void sys_delay_us(uint32_t us);

/**
 *****************************************************************************************
 * @brief Delay the function execution.
 *
 * @param[in] ms:  Millisecond.
 *****************************************************************************************
 */
void sys_delay_ms(uint32_t ms);

/**
 *****************************************************************************************
 * @brief Memory allocation.
 *
 * @param[in] size:  Requested memory size.
 *
 * @return Valid memory location if successful, else null
 *****************************************************************************************
 */
void *sys_malloc(uint32_t size);

/**
 *****************************************************************************************
 * @brief Free allocated memory.
 *
 * @param[in] p_mem: Pointer to memory block.
 *****************************************************************************************
 */
void sys_free(void *p_mem);

/**
 *****************************************************************************************
 * @brief Register signal handler.
 *
 * @note This function is mainly used to register the upper-layer APP callback functions to the protocol layer, 
 *       which will be invoked when there are event responses in the protocol layer.
 *****************************************************************************************
 */
void sys_signal_handler_register(callback_t isr_handler);

/**
 *****************************************************************************************
 * @brief Get SDK version.
 *
 * @note This function is mainly used to get the version of SDK.
 *
 * @param[out] p_version: The pointer to struct of @ref sdk_version_t.
 *****************************************************************************************
 */
void sys_sdk_verison_get(sdk_version_t *p_version);

/**
 *****************************************************************************************
 * @brief Save system context.
 *
 * @note This function is used to save system context before the system goes to deep sleep.
 *       Boot codes will be used to restore system context in the wakeup procedure.
 *****************************************************************************************
 */
void sys_context_save(void);

/**
 *****************************************************************************************
 * @brief Save system registers.
 *
 * @note This function is used to save system register before the system goes to deep sleep.
 *
 * @param[in] p_address: The pointer to register address.
 * @param[in] value: The register value to be saved, it will be restored when system wakeup
 *****************************************************************************************
 */
void sys_regs_save(volatile uint32_t *p_address, uint32_t value);

/**
 *****************************************************************************************
 * @brief Generate checksum info for system context.
 *
 * @note This function is used to generate checksum for system context, it will be called
 *       before deep sleep in power management module.
 *****************************************************************************************
 */
void sys_context_checksum_gen(void);

/**
 *****************************************************************************************
 * @brief Function pointer for user context save before deep sleep .
 * 
 *****************************************************************************************
 */
typedef void (*sys_context_func_t)(void);

/**
 *****************************************************************************************
 * @brief Register user save function .
 *
 * @note This function is used to register user-level save function, which will be executed
 *       before deep sleep in power management module.
 *****************************************************************************************
 */
void sys_context_reg_save_func(sys_context_func_t func);

/**
 *****************************************************************************************
 * @brief Encrypt and decrypt data using Present.
 *
 * @note  This function is only used to encrypt and decrypt data that needs to be stored in Flash.
 * 
 * @param[in]  addr:   Operation address(Flash address minus Flash start address).
 * @param[in]  input:  Data before encryption and decryption.
 * @param[in]  size:   Data size.
 * @param[out] output: Data after encryption and decryption.
 *****************************************************************************************
 */
void sys_security_data_use_present(uint32_t addr, uint8_t *input, uint32_t size, uint8_t *output);

/**
 *****************************************************************************************
 * @brief Check the chip's security level.
 *
 * @return 0:  Security is not supported.
 *         1:  Security is supported.
 *****************************************************************************************
 */
uint32_t sys_security_enable_status_check(void);

/**
 *****************************************************************************************
 * @brief Get the RF trim information.
 *
 * @param[out] p_rf_trim: The pointer to struct of @ref rf_trim_info_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_rf_trim_get(rf_trim_info_t *p_rf_trim);

/**
 *****************************************************************************************
 * @brief Get the ADC trim information.
 *
 * @param[out] p_adc_trim: The pointer to struct of @ref adc_trim_info_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_adc_trim_get(adc_trim_info_t *p_adc_trim);

/**
 *****************************************************************************************
 * @brief Get the PMU trim information.
 *
 * @param[out] p_pmu_trim: The pointer to struct of @ref pmu_trim_info_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_pmu_trim_get(pmu_trim_info_t *p_pmu_trim);

/**
 *****************************************************************************************
 * @brief Get the crystal trim information.
 *
 * @param[out] p_crystal_trim: offset information for crystal.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_crystal_trim_get(uint16_t *p_crystal_trim);

/**
 *****************************************************************************************
 * @brief Get the device address information.
 *
 * @param[out] p_device_addr: Bluetooth address by default.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_addr_get(uint8_t *p_device_addr);

/**
 *****************************************************************************************
 * @brief Get the device address information.
 *
 * @param[out] p_device_uid: Device chip UID.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_uid_get(uint8_t *p_device_uid);

/**
 *****************************************************************************************
 * @brief Get the RAM size information.
 *
 * @param[out] p_sram_size: The pointer to enumeration of @ref sram_size_t.
 * @return 0:  Operation is OK.
 *         1:  the chip's parameter is incorrect.
 *****************************************************************************************
 */
uint16_t sys_device_sram_get(sram_size_t *p_sram_size);

/**
 *****************************************************************************************
 * @brief Set low power CLK frequency.
 *
 * @param[in] user_lpclk: CLK frequency.
 *****************************************************************************************
 */
void sys_lpclk_set(uint32_t user_lpclk);

/**
 ****************************************************************************************
 * @brief Converts a duration in us into a duration in lp cycles.
 *
 * The function converts a duration in us into a duration is lp cycles, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 *
 * @param[in] us:    Duration in us
 *
 * @return Duration in lpcycles
 ****************************************************************************************
 */
uint32_t sys_us_2_lpcycles(uint32_t us);

/**
 ****************************************************************************************
 * @brief Converts a duration in lp cycles into a duration in half us.
 *
 * The function converts a duration in lp cycles into a duration is half us, according to the
 * low power clock frequency (32768Hz or 32000Hz).
 * @param[in]     lpcycles:    Duration in lp cycles
 * @param[in/out] error_corr:  Insert and retrieve error created by truncating the LP Cycle Time to a half us (in half us)
 *
 * @return Duration in half us
 ****************************************************************************************
 */
uint32_t sys_lpcycles_2_hus(uint32_t lpcycles, uint32_t *error_corr);

/**
 ****************************************************************************************
 * @brief Set system maximum usage ratio of message heap.
 *
 * The function will used to set message ratio of message heap.
 * Valid ratio range is 50 - 100 percent in full message size.
 *
 * @param[in]     usage_ratio:  Usage ratio of message heap size.
 *
 ****************************************************************************************
 */
void sys_max_msg_usage_ratio_set(uint8_t usage_ratio);

/**
 ****************************************************************************************
 * @brief Get system message usage ratio.
 *
 * The function will used to get message ratio of message heap.
 * This ratio is heap used percent in full message size.
 *
 * @return current heap used percent.
 ****************************************************************************************
 */
uint8_t sys_msg_usage_ratio_get(void);


/**
 ****************************************************************************************
 * @brief Get link quality info
 *
 * @param[in]      conn_idx:  Connect index.
 * @param[in/out]  rx_info:   RX detailed information
 *
 * @return Current connect index link quality.
 ****************************************************************************************
 */
uint8_t get_link_quality(uint8_t conn_idx, link_rx_info_t* rx_info);

/**
 ****************************************************************************************
 * @brief Clear link quality info.
 *
 * @param[in]      conn_idx:  Connect index.
 ****************************************************************************************
 */
void clear_link_quality(uint8_t conn_idx);

/**
 *****************************************************************************************
 * @brief Function pointer for low power clock update function.
 * 
 *****************************************************************************************
 */
typedef void (*void_func_t)(void);
/**
 ****************************************************************************************
 * @brief register low power clock update function.
 *
 * @param[in]  func_update_lpclk: function pointer to update_lpclk
 ****************************************************************************************
 */
void register_lpclk_update(void_func_t func_update_lpclk);


/**
 ****************************************************************************************
 * @brief Get low power clk frequence.
 ****************************************************************************************
 */
uint32_t get_lpclk_freq(void);

/**
 ****************************************************************************************
 * @brief Set low power clk frequence.
 * @param[in] user_lpclk: user low power clk frequence.
 ****************************************************************************************
 */
void set_lpclk_freq(uint32_t user_lpclk);

/**
 ****************************************************************************************
 * @brief Get low power clk period.
 ****************************************************************************************
 */
uint32_t get_lpclk_period(void);

/**
 ****************************************************************************************
 * @brief Set low power clk period.
 * @param[in] user_lpclk_period: user low power clk period.
 ****************************************************************************************
 */
void set_lpclk_period(uint32_t user_lpclk_period);


/** @} */
#endif

/** @} */
/** @} */

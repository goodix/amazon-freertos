/**
 ******************************************************************************
 *
 * @file gr55xx_nvds.h
 *
 * @brief NVDS API
 *
 * Copyright(C) 2016-2018, Shenzhen Goodix  Technology Co., Ltd
 * All Rights Reserved
 *
 ******************************************************************************
 */

/**
 * @addtogroup SYSTEM
 * @{
 */
 /**
  @addtogroup NVDS Non-Volatile Data Storage
  @{
  @brief Definitions and prototypes for the NVDS interface.
 */

#ifndef __GR55XX_NVDS_H__
#define __GR55XX_NVDS_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup NVDS_DEFINES Defines
 * @{ */
#define NV_TAGCAT_APP       0x4000  /**< NVDS tag mask for user application. */

/**@brief Get NVDS tag for user application.
 * The values of Tag 0x0000 and 0xFFFF are invalid. idx should not be used
 * as the parameter of NVDS APIs directly. The range of idx is 0x0001~0x3FFF.
 */
#define NV_TAG_APP(idx)     (NV_TAGCAT_APP | ((idx) & 0x3FFF))
/** @} */

/**@addtogroup NVDS_ENUMERATIONS Enumerations
 * @{ */
/**@brief NVDS Returned Status. */
enum NVDS_STATUS
{
    NVDS_SUCCESS,               /**< NVDS succeeds. */
    NVDS_FAIL,                  /**< NVDS failed. */
    NVDS_TAG_NOT_EXISTED,       /**< NVDS tag does not exist. */
    NVDS_SPACE_NOT_ENOUGH,      /**< NVDS space is not enough. */
    NVDS_LENGTH_OUT_OF_RANGE,   /**< NVDS length out of range. */
    NVDS_INVALID_PARA,          /**< NVDS invalid params. */
    NVDS_INVALID_START_ADDR,    /**< NVDS invalid start address. */
    NVDS_INVALID_SECTORS,       /**< NVDS invalid sector. */
    NVDS_COMPACT_FAILED,        /**< NVDS failed to compact sectors. */
    NVDS_STORAGE_ACCESS_FAILED  /**< NVDS failed to access storage. */
};
/** @} */

/**@addtogroup NVDS_STRUCTURES Structures
 * @{ */
/**@brief NVDS Item tag. */
typedef uint16_t NvdsTag_t;
/** @} */


/** @addtogroup NVDS_FUNCTIONS Functions
 * @{ */
/**
 ****************************************************************************************
 * @brief Initialize the sectors for NVDS.
 *
 * @note NVDS locates in the last sector of Flash.
 *
 * @param[in] start_addr: Start address of NVDS area. If the value equals zero,
                          NVDS area will locate in the last sector(s) in flash
                          memory. If the value does not equal zero, it must be
                          sector-aligned.
 * @param[in] sectors:    The number of sectors.
 *
 * @return ::NVDS_SUCCESS if successful.
 * @return error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_init(uint32_t start_addr, uint8_t sectors);

/**
 ****************************************************************************************
 * @brief Read data from NVDS.
 *
 * @param[in]     tag:   Valid NVDS item tag.
 * @param[in,out] p_len: Pointer to the length of data.
 * @param[out]    p_buf: Data is read into the buffer.
 *
 * @return ::NVDS_SUCCESS if successful.
 * @return error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_get(NvdsTag_t tag, uint16_t *p_len, uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Write data to NVDS. If the tag does not exist, create one.
 *
 * @param[in] tag:   Valid NVDS item tag.
 * @param[in] len:   Length of data to be written.
 * @param[in] p_buf: Data to be written.
 *
 * @return ::NVDS_SUCCESS: if successful.
 * @return error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_put(NvdsTag_t tag, uint16_t len, const uint8_t *p_buf);

/**
 ****************************************************************************************
 * @brief Delete a tag in NVDS
 *
 * @param[in] tag: The tag to be deleted.
 *
 * @return NVDS_SUCCESS: If the deletion is successful.
 * @return Error code in ::NVDS_STATUS if not successful.
 ****************************************************************************************
 */
uint8_t nvds_del(NvdsTag_t tag);

/**
 ****************************************************************************************
 * @brief Get the length of a tag in NVDS
 *
 * @param[in] tag: The tag to get the length.
 *
 * @return length: if tag exists.
 * @return 0: if tag does not exist.
 ****************************************************************************************
 */
uint16_t nvds_tag_length(NvdsTag_t tag);
/** @} */

#endif

/** @} */
/** @} */


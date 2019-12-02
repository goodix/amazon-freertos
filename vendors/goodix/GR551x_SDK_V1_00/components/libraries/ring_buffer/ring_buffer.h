/**
 *****************************************************************************************
 *
 * @file ring_buffer.h
 *
 * @brief Header file - ring buffer APIs
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.

 *****************************************************************************************
 */
#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup RING_BUFFER_STRUCT Structures
 * @{
 */
typedef struct
{
    uint32_t             buffer_size;           /**< Size of ring buffer. */
    uint8_t             *p_buffer;              /**< Pointer to buffer saved data. */
    uint32_t             write_index;           /**< Index of write. */
    uint32_t             read_index;            /**< Index of read. */
} ring_buffer_t;
/** @} */

/**
 * @defgroup RING_BUFFER_FUNCTION Functions
 * @{
 */
/*
 * GLOBAL FUNCTION DECLARATION
 *****************************************************************************************
 */
/**
 *****************************************************************************************
 * @brief Initialize one ring buffer.
 *
 * @param[in] p_ring_buff: Pointer to ring buffer structure.
 * @param[in] p_buff:      Pointer to where save data.
 * @param[in] buff_size:   Size of buffer save data.
 *
 * @return Result of initializing ring buffer.
 *****************************************************************************************
 */
bool ring_buffer_init(ring_buffer_t *p_ring_buff, uint8_t *p_buff, uint32_t buff_size);

/**
 *****************************************************************************************
 * @brief Write data to one ring buffer.
 *
 * @param[in] p_ring_buff: Pointer to ring buffer.
 * @param[in] p_wr_data:   Pointer to data need be wrote.
 * @param[in] length:      Length of data need be wrote.
 *
 * @return Length of writen.
 *****************************************************************************************
 */
uint32_t ring_buffer_write(ring_buffer_t *p_ring_buff, uint8_t const *p_wr_data, uint32_t length);

/**
 *****************************************************************************************
 * @brief Read data from one ring buffer.
 *
 * @param[in] p_ring_buff: Pointer to ring buffer.
 * @param[in] p_rd_data:   Pointer to where save read data.
 * @param[in] length:      Length of data want to read.
 *
 * @return Length of availdble read data.
 *****************************************************************************************
 */
uint32_t ring_buffer_read(ring_buffer_t *p_ring_buff, uint8_t *p_rd_data, uint32_t length);

/**
 *****************************************************************************************
 * @brief Get surplus space of one ring buffer.
 *
 * @param[in] p_ring_buff: Pointer to ring buffer.
 *
 * @retval  Size of surplus space.
 *****************************************************************************************
 */
uint32_t ring_buffer_surplus_space_get(ring_buffer_t *p_ring_buff);

/**
 *****************************************************************************************
 * @brief Get availdble data from one ring buffer.
 *
 * @param[in] p_ring_buff: Pointer to ring buffer.
 *
 * @retval  Length of availdble data.
 *****************************************************************************************
 */
uint32_t ring_buffer_items_count_get(ring_buffer_t *p_ring_buff);

/**
 *****************************************************************************************
 * @brief Clean one ring buffer.
 *
 * @param[in] p_ring_buff: Pointer to ring buffer structure.
 *****************************************************************************************
 */
void ring_buffer_clean(ring_buffer_t *p_ring_buff);

/**
 *****************************************************************************************
 * @brief  Check if ring buffer is almost full.
 *
 * @param[in] p_ring_buff:    Pointer to ring buffer.
 * @param[in] left_threshold: Left room threshold.
 *
 * @return State of buffer almost full.
 *****************************************************************************************
 */
bool ring_buffer_is_reach_left_threshold(ring_buffer_t *p_ring_buff, uint32_t letf_threshold);
/** @} */

#endif

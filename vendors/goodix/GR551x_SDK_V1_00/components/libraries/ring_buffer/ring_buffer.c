/**
 *****************************************************************************************
 *
 * @file ring_buffer.c
 *
 * @brief Ring buffer function Implementation.
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "ring_buffer.h"
#include "gr55xx_hal.h"
#include "utility.h"
#include <string.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define RING_BUFFER_LOCK()    GLOBAL_EXCEPTION_DISABLE()
#define RING_BUFFER_UNLOCK()  GLOBAL_EXCEPTION_ENABLE()

/*
 * GLOBAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
bool ring_buffer_init(ring_buffer_t *p_ring_buff, uint8_t *p_buff, uint32_t buff_size)
{
    if (NULL == p_buff || NULL == p_ring_buff)
    {
        return false;
    }
    else
    {
        p_ring_buff->buffer_size   = buff_size;
        p_ring_buff->p_buffer      = p_buff;
        p_ring_buff->write_index   = 0;
        p_ring_buff->read_index    = 0;

        return true;
    }
}

uint32_t ring_buffer_write(ring_buffer_t *p_ring_buff, uint8_t const *p_wr_data, uint32_t length)
{
    uint32_t surplus_space  = 0;
    uint32_t over_flow      = 0;
    uint32_t wr_idx         = p_ring_buff->write_index;
    uint32_t rd_idx         = p_ring_buff->read_index;

    RING_BUFFER_LOCK();

    if (rd_idx > wr_idx)
    {
        surplus_space = rd_idx - wr_idx - 1;
        length        = (length > surplus_space ? surplus_space : length);
    }
    else
    {
        surplus_space = p_ring_buff->buffer_size - wr_idx + rd_idx - 1;
        length        = (length > surplus_space ? surplus_space : length);

        if (wr_idx + length >= p_ring_buff->buffer_size)
        {
            over_flow = wr_idx + length - p_ring_buff->buffer_size;
        }
    }

    memcpy(p_ring_buff->p_buffer + wr_idx, p_wr_data, length - over_flow);
    memcpy(p_ring_buff->p_buffer, p_wr_data + length - over_flow, over_flow);
    wr_idx += length;

    if (wr_idx >= p_ring_buff->buffer_size)
    {
        wr_idx -= p_ring_buff->buffer_size;
    }

    p_ring_buff->write_index = wr_idx;

    RING_BUFFER_UNLOCK();

    return length;
}

uint32_t ring_buffer_read(ring_buffer_t *p_ring_buff, uint8_t *p_rd_data, uint32_t length)
{
    uint32_t items_avail = 0;
    uint32_t over_flow   = 0;
    uint32_t wr_idx      = p_ring_buff->write_index;
    uint32_t rd_idx      = p_ring_buff->read_index;

    RING_BUFFER_LOCK();

    if (wr_idx >= rd_idx)
    {
        items_avail = wr_idx - rd_idx;
        length = (length > items_avail ? items_avail : length);
    }
    else
    {
        items_avail = p_ring_buff->buffer_size - rd_idx + wr_idx;
        length = (length > items_avail ? items_avail : length);

        if (rd_idx + length >= p_ring_buff->buffer_size)
        {
            over_flow = length + rd_idx - p_ring_buff->buffer_size;
        }
    }

    memcpy(p_rd_data, p_ring_buff->p_buffer + rd_idx, length - over_flow);
    memcpy(p_rd_data + length - over_flow, p_ring_buff->p_buffer, over_flow);
    rd_idx += length;

    if (rd_idx >= p_ring_buff->buffer_size && rd_idx > wr_idx)
    {
        rd_idx -= p_ring_buff->buffer_size;
    }

    p_ring_buff->read_index = rd_idx;

    RING_BUFFER_UNLOCK();

    return length;
}

uint32_t ring_buffer_items_count_get(ring_buffer_t *p_ring_buff)
{
    uint32_t wr_idx = p_ring_buff->write_index;
    uint32_t rd_idx = p_ring_buff->read_index;

    if (rd_idx <= wr_idx)
    {
        return wr_idx - rd_idx;
    }
    else
    {
        return p_ring_buff->buffer_size - rd_idx + wr_idx;
    }
}

uint32_t ring_buffer_surplus_space_get(ring_buffer_t *p_ring_buff)
{
    uint32_t surplus_space;
    uint32_t wr_idx = p_ring_buff->write_index;
    uint32_t rd_idx = p_ring_buff->read_index;

    RING_BUFFER_LOCK();

    if (rd_idx > wr_idx)
    {
        surplus_space = rd_idx - wr_idx - 1;
    }
    else
    {
        surplus_space = p_ring_buff->buffer_size - wr_idx + rd_idx - 1;
    }

    RING_BUFFER_UNLOCK();

    return surplus_space;
}

bool ring_buffer_is_reach_left_threshold(ring_buffer_t *p_ring_buff, uint32_t letf_threshold)
{
    uint32_t surplus_space;

    surplus_space = ring_buffer_surplus_space_get(p_ring_buff);

    if (letf_threshold >= surplus_space)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ring_buffer_clean(ring_buffer_t *p_ring_buff)
{
    RING_BUFFER_LOCK();

    p_ring_buff->write_index = 0;
    p_ring_buff->read_index  = 0;

    RING_BUFFER_UNLOCK();
}


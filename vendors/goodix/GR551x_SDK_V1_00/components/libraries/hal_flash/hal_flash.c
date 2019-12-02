/**
  ******************************************************************************
  * @file    hal_flash.c
  * @author  Engineering Team
  * @brief   This file contains the implementation of HAL flash.
  ******************************************************************************
  * @attention
  *
  * Copyright(C) 2016-2017, Shenzhen Huiding Technology Co., Ltd
  * All Rights Reserved
  *
  ******************************************************************************
  */

/*******************************************************************************
 * The file is the interface of Flash HAL. We should implement the interface
 * for each specified type of flash. We must enbale only one implementation
 * with the macro *FLASH_ENABLE.
 ******************************************************************************/

#include "gr55xx_hal.h"
#include "hal_flash.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef EXFLASH_ENABLE
#define EXFLASH_ENABLE                        /**<Use exflash. */
#endif

#ifndef VFLASH_ENABLE
//#define VFLASH_ENABLE                       /**<Use vflash for BLE Stack in Flash. */
#endif

extern uint32_t sys_security_enable_status_check(void);

#ifdef EXFLASH_ENABLE

#if defined(ROM_RUN_IN_FLASH) || defined(GR551xx_C0) || defined(GR551xx_C1)
const uint32_t baud_rate[6] = {XQSPI_BAUD_RATE_64M, XQSPI_BAUD_RATE_48M, XQSPI_BAUD_RATE_16M,
                               XQSPI_BAUD_RATE_24M, XQSPI_BAUD_RATE_16M, XQSPI_BAUD_RATE_32M};
xqspi_handle_t g_xqspi_handle = {0};
#endif

#if defined(GR551xx_C0) || defined(GR551xx_C1)
exflash_handle_t g_exflash_handle;
#else
extern exflash_handle_t g_exflash_handle;
#endif

bool hal_flash_init(void)
{
#if defined(ROM_RUN_IN_FLASH) || defined(GR551xx_C0) || defined(GR551xx_C1)
    mcu_clock_type_t clk_type = XO_S16M_CLK;
    SystemCoreGetClock(&clk_type);

    if (g_exflash_handle.p_xqspi == NULL)
    {
        g_exflash_handle.p_xqspi       = &g_xqspi_handle;
        g_xqspi_handle.p_instance      = XQSPI;
        g_xqspi_handle.init.work_mode  = XQSPI_WORK_MODE_QSPI;
        g_xqspi_handle.init.cache_mode = ENABLE;
        g_xqspi_handle.init.read_cmd   = XQSPI_READ_CMD_QUAD_IO_READ;
        /* The XQSPI clock speed should not be greater than system clock. */
        g_xqspi_handle.init.baud_rate  = baud_rate[clk_type];
        g_xqspi_handle.init.clock_mode = XQSPI_CLOCK_MODE_0;
    }
#endif
    g_exflash_handle.security = sys_security_enable_status_check() ? HAL_EXFLASH_ENCRYPTED : HAL_EXFLASH_UNENCRYPTED;
    return (HAL_OK == hal_exflash_init(&g_exflash_handle)) ? true : false;
}

uint32_t hal_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    return (HAL_OK == hal_exflash_read(&g_exflash_handle, addr, buf, size)) ? size : 0;
}

uint32_t hal_flash_write(const uint32_t addr, const uint8_t *buf, const uint32_t size)
{
    return (HAL_OK == hal_exflash_write(&g_exflash_handle, addr, (uint8_t*)buf, size)) ? size : 0;
}

uint32_t hal_flash_write_r(const uint32_t addr, const uint8_t *buf, const uint32_t size)
{
    hal_status_t status;

    status = hal_exflash_write(&g_exflash_handle, addr, (uint8_t*)buf, size);
    if (HAL_OK == status)
    {
        /* It's possible that the data is not written to flash memory.
         * So we must read the data from flash memory, and check it. */
        uint8_t  rd_buf[EXFLASH_SIZE_PAGE_BYTES];
        uint32_t offset     = 0;
        uint32_t unrd_bytes = size;
        uint32_t rd_bytes   = size > EXFLASH_SIZE_PAGE_BYTES ?
                              EXFLASH_SIZE_PAGE_BYTES : size;

        do
        {
            status = hal_exflash_read(&g_exflash_handle, addr + offset,
                                      rd_buf, rd_bytes);
            if ((HAL_OK == status) && (memcmp(buf + offset, rd_buf, rd_bytes) == 0))
            {
                unrd_bytes -= rd_bytes;
                if (0 == unrd_bytes)
                {
                    return size;
                }
                else
                {
                    offset += rd_bytes;
                    rd_bytes = unrd_bytes > EXFLASH_SIZE_PAGE_BYTES ?
                               EXFLASH_SIZE_PAGE_BYTES : unrd_bytes;
                    if ((offset >= size) || ((offset + rd_bytes) > size))
                    {
                        break;
                    }
                }
            }
            else
            {
                break;
            }
        } while(1);
    }

    return 0;
}

void hal_flash_set_security(bool enable)
{
    g_exflash_handle.security = (enable ? HAL_EXFLASH_ENCRYPTED : HAL_EXFLASH_UNENCRYPTED);
    return;
}

bool hal_flash_get_security(void)
{
    return (bool)g_exflash_handle.security;
}

bool hal_flash_erase(const uint32_t addr, const uint32_t size)
{
   return (HAL_OK == hal_exflash_erase(&g_exflash_handle, 0, addr, size)) ? true : false;
}

bool hal_flash_erase_chip(void)
{
   return (HAL_OK == hal_exflash_erase(&g_exflash_handle, 1, 0, 0)) ? true : false;
}

void hal_flash_get_info(uint32_t *id, uint32_t *size)
{
    if (NULL == id || NULL == size)
    {
        return;
    }

    *id   = g_exflash_handle.flash_id;
    *size = g_exflash_handle.flash_size;
}

uint32_t hal_flash_sector_size(void)
{
    return EXFLASH_SIZE_SECTOR_BYTES;
}

#endif // EXFLASH_ENABLE

/******************************************************************************/
/******************************************************************************/

#ifdef VFLASH_ENABLE

#include "vflash/vflash.h"

bool hal_flash_init(void)
{
    return vflash_init();
}

uint32_t hal_flash_read(const uint32_t addr, uint8_t *buf, const uint32_t size)
{
    return vflash_read(addr, buf, size);
}

uint32_t hal_flash_write(const uint32_t addr, const uint8_t *buf,
                         const uint32_t size)
{
    return vflash_write(addr, (uint8_t*)buf, size);
}

bool hal_flash_erase(const uint32_t addr, const uint32_t size)
{
   return vflash_erase(addr, size);
}

bool hal_flash_erase_chip(void)
{
   return false;
}

void hal_flash_get_info(uint32_t *id, uint32_t *size)
{
    if (NULL == id || NULL == size)
    {
        return;
    }

    *id   = 0;
    *size = 0;
}

uint32_t hal_flash_sector_size(void)
{
    return vflash_sector_size();
}

bool hal_flash_get_security(void)
{
    return false;
}
void hal_flash_set_security(bool enable)
{
    return;
}

#endif // VFLASH_ENABLE


/**
  ****************************************************************************************
  * @file    gr55xx_hal_exflash.c
  * @author  BLE Driver Team
  * @brief   EXFLASH HAL module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of GOODIX nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gr55xx_hal.h"

#ifdef HAL_EXFLASH_MODULE_ENABLED

#define FLASH_MANU_ID_INVALID0    0x00
#define FLASH_MANU_ID_INVALID1    0xFF
#define FLASH_MANU_GD_ID          0xC8
#define FLASH_MANU_PY_ID          0x85

#define FLASH_MEM_SIZE_512K       0x10
#define FLASH_MEM_SIZE_1M         0x11
#define FLASH_MEM_SIZE_2M         0x12
#define FLASH_MEM_SIZE_4M         0x13
#define FLASH_MEM_SIZE_8M         0x14
#define FLASH_MEM_SIZE_16M        0x15
#define FLASH_MEM_SIZE_32M        0x16
#define FLASH_MEM_SIZE_64M        0x17

#define FLASH_ALIGNED_4BYTES      0x03
#define FLASH_ALIGNED_16BYTES     0x0F

#define FLASH_STATUS_WIP_MASK     0x0001
#define FLASH_STATUS_WEL_MASK     0x0002
#define FLASH_STATUS_BP_MASK      0x007C
#define FLASH_STATUS_SRP_MASK     0x0180
#define FLASH_STATUS_QE_MASK      0x0200
#define FLASH_STATUS_SUS_MASK     0x8400
#define FLASH_STATUS_LB_MASK      0x3800
#define FLASH_STATUS_CMP_MASK     0x4000

#define GET_IRQ_RESET()             __get_BASEPRI()
#define DISABLE_INTERRUPT_LOW()                 \
    __set_BASEPRI(NVIC_GetPriority(BLE_IRQn) + (1 << (NVIC_GetPriorityGrouping() + 1)))

#define ENABLE_INTERRUPT_LOW()                  \
    __set_BASEPRI(__l_basepri_rest)

extern uint32_t sys_security_enable_status_check(void);
extern void sys_security_data_use_present(uint32_t addr, uint8_t *p_input, uint32_t size, uint8_t *p_output);

static hal_status_t exflash_write_status(exflash_handle_t *p_exflash, uint16_t data);
static hal_status_t exflash_check_id(exflash_handle_t *p_exflash);
static hal_status_t exflash_enable_quad(exflash_handle_t *p_exflash);
static hal_status_t exflash_erase_sector(exflash_handle_t *p_exflash, uint32_t addr);
static hal_status_t exflash_erase_chip(exflash_handle_t *p_exflash);
static hal_status_t exflash_reset(exflash_handle_t *p_exflash);
static hal_status_t exflash_deepsleep(exflash_handle_t *p_exflash);
static hal_status_t exflash_wakeup(exflash_handle_t *p_exflash);
static hal_status_t exflash_page_program(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t nbytes, uint32_t retry);
static hal_status_t exflash_suspend(exflash_handle_t *p_exflash);
static hal_status_t exflash_resume(exflash_handle_t *p_exflash);

#if defined(CFG_SECURT_BOOT)
typedef void (*security_present_t)(uint32_t addr, uint8_t *input, uint32_t size, uint8_t *output);
security_present_t security_present_func = (security_present_t)(0x00001200+1);
void sys_security_data_use_present(uint32_t addr, uint8_t *input, uint32_t size, uint8_t *output)
{
    security_present_func(addr, input, size, output);
}
#endif

__WEAK hal_status_t hal_exflash_init(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;

    /* Check the EXFLASH handle allocation */
    if (NULL == p_exflash)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_XQSPI_ALL_INSTANCE(p_exflash->p_xqspi));

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_RESET == p_exflash->state)
    {
        /* initialize flash id to 0x00 */
        p_exflash->flash_id = 0x0;

        /* initialize flash size to 0x00 */
        p_exflash->flash_size = 0x0;

        /* Allocate lock resource and initialize it */
        p_exflash->lock = HAL_UNLOCKED;

        /* init the low level hardware : GPIO, CLOCK, NVIC, DMA */
        hal_exflash_msp_init(p_exflash);

        /* Configure the default repeat times for the QSPI memory access */
        hal_exflash_set_retry(p_exflash, HAL_EXFLASH_RETRY_DEFAULT_VALUE);
    }

    if (!ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
    {
        __HAL_EXFLASH_POWER_ON();

        p_exflash->fw_mode = p_exflash->p_xqspi->init.work_mode;
        p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
        status = hal_xqspi_init(p_exflash->p_xqspi);

        if (HAL_OK == status)
        {
            /* Reset Flash & Release from DeepSleep mode */
            exflash_wakeup(p_exflash);

            do {
                /* Check Flash ID */
                status = exflash_check_id(p_exflash);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_ID;
                    break;
                }

                /* Enable Quad mode */
                status = exflash_enable_quad(p_exflash);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_QUAD;
                    break;
                }

                if (XQSPI_WORK_MODE_XIP == p_exflash->fw_mode)
                {
                    p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
                    status = hal_xqspi_init(p_exflash->p_xqspi);
                }
                else
                {
                    /* Enter the DeepSleep mode */
                    exflash_deepsleep(p_exflash);
                }

                /* Set EXFLASH error code to none */
                p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;
            } while(0);
        }
    }
    else
    {
        if((0 == p_exflash->flash_size) && (0 == p_exflash->flash_id))
        {
            GLOBAL_EXCEPTION_DISABLE();

            /* Configure XQSPI initial paraments */
            p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
            hal_xqspi_init(p_exflash->p_xqspi);

            /* Check Flash ID */
            status = exflash_check_id(p_exflash);
            if (HAL_OK != status)
            {
                p_exflash->error_code = HAL_EXFLASH_ERROR_ID;
            }

            /* Configure XQSPI initial paraments */
            p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
            hal_xqspi_init(p_exflash->p_xqspi);

            GLOBAL_EXCEPTION_ENABLE();
        }

        p_exflash->fw_mode = XQSPI_WORK_MODE_XIP;
    }

    /* Initialize the EXFLASH state */
    p_exflash->state = HAL_EXFLASH_STATE_READY;

    /* Release Lock */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_exflash_deinit(exflash_handle_t *p_exflash)
{
    /* Check the QSPI handle allocation */
    if (NULL == p_exflash)
    {
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_exflash);

    /* Disable the QSPI and XIP */
    hal_xqspi_deinit(p_exflash->p_xqspi);

    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    hal_exflash_msp_deinit(p_exflash);

    /* Set EXFLASH error code to none */
    p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

    /* Initialize the QSPI state */
    p_exflash->state = HAL_EXFLASH_STATE_RESET;

    /* Release Lock */
    __HAL_UNLOCK(p_exflash);

    return HAL_OK;
}

__WEAK void hal_exflash_msp_init(exflash_handle_t *p_exflash)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_exflash_msp_init can be implemented in the user file
     */
}

__WEAK void hal_exflash_msp_deinit(exflash_handle_t *p_exflash)
{
    /* Prevent unused argument(s) compilation warning */
    return;

    /* NOTE : This function should not be modified, when the callback is needed,
              the hal_exflash_msp_deinit can be implemented in the user file
     */
}

__WEAK hal_status_t hal_exflash_write(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t size)
{
    hal_status_t status = HAL_OK;
    uint8_t     *p_buf  = p_data;
    uint8_t      tmp_buffer[EXFLASH_SIZE_PAGE_BYTES];
    uint8_t      page_buffer[EXFLASH_SIZE_PAGE_BYTES];
    uint32_t     absolute_addr = addr - EXFLASH_START_ADDR;
    uint32_t     __l_basepri_rest = GET_IRQ_RESET();

    if ((addr < EXFLASH_START_ADDR) || ((addr + size) >= (EXFLASH_START_ADDR + p_exflash->flash_size)) || (p_data == NULL) || \
        (size == 0) || (HAL_EXFLASH_ENCRYPTED == p_exflash->security && (addr & FLASH_ALIGNED_4BYTES)) )
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {
                DISABLE_INTERRUPT_LOW();

                /* Disable global interrupt, aviod call function in flash */
                GLOBAL_EXCEPTION_DISABLE();

                /* Configure XQSPI initial paraments */
                p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
                hal_xqspi_init(p_exflash->p_xqspi);

                /* Update EXFALSH state */
                p_exflash->state = HAL_EXFLASH_STATE_BUSY;

                GLOBAL_EXCEPTION_ENABLE();
            }
            else
            {
                /* EXFLASH in QSPI mode */
                exflash_wakeup(p_exflash);
                status = exflash_check_id(p_exflash);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_ID;
                    break;
                }
                /* Update EXFALSH state */
                p_exflash->state = HAL_EXFLASH_STATE_BUSY;
            }

            uint32_t write_addr = absolute_addr;
            uint32_t write_size;

            while (write_addr < (absolute_addr + size))
            {
                write_size = EXFLASH_SIZE_PAGE_BYTES - (write_addr & (EXFLASH_SIZE_PAGE_BYTES - 1));

                /* Avoid writing the last page errors */
                if ((absolute_addr + size) < (write_addr + write_size))
                {
                    write_size = absolute_addr + size - write_addr;
                }

                /* Avoid buffer point from flash area */
                if ((EXFLASH_START_ADDR <= (uint32_t)p_data) && ((uint32_t)p_data < EXFLASH_END_ADDR))
                {
                    /* Disable global interrupt, aviod call function in flash */
                    GLOBAL_EXCEPTION_DISABLE();

                    p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
                    hal_xqspi_init(p_exflash->p_xqspi);

                    for (uint32_t i = 0; i < write_size; i++)
                        tmp_buffer[i] = *(p_data + i);
                    p_buf = tmp_buffer;

                    p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
                    hal_xqspi_init(p_exflash->p_xqspi);

                    GLOBAL_EXCEPTION_ENABLE();
                }
                else
                {
                    for (uint32_t i = 0; i < write_size; i++)
                        tmp_buffer[i] = *(p_data + i);
                    p_buf = tmp_buffer;
                }
                if ((HAL_EXFLASH_ENCRYPTED == p_exflash->security) && (sys_security_enable_status_check()))
                {
                    uint8_t offset = write_size & ~(FLASH_ALIGNED_16BYTES);

                    if (FLASH_ALIGNED_16BYTES < write_size)
                    {
                        sys_security_data_use_present(write_addr, tmp_buffer, write_size & ~(FLASH_ALIGNED_16BYTES), page_buffer);
                    }
                    if (write_size & FLASH_ALIGNED_16BYTES)
                    {
                        uint8_t tmp_buff[16] = {0};

                        for(uint32_t i = 0; i < (write_size & FLASH_ALIGNED_16BYTES); i++)
                        {
                            tmp_buff[i] = tmp_buffer[offset+i];
                        }
                        sys_security_data_use_present(write_addr+offset, tmp_buff, FLASH_ALIGNED_16BYTES+1, tmp_buff);
                        for(uint32_t i = 0; i < (write_size & FLASH_ALIGNED_16BYTES); i++)
                        {
                            page_buffer[offset+i] = tmp_buff[i];
                        }
                    }
                    p_buf = page_buffer;
                }

                status = exflash_page_program(p_exflash, write_addr, p_buf, write_size, 1000);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_TIMEOUT;
                    break;
                }
                write_addr += write_size;
                p_data += write_size;
            }
        } while(0);

        if (XQSPI_WORK_MODE_XIP == p_exflash->fw_mode)
        {
            /* Disable global interrupt, aviod call function in flash */
            GLOBAL_EXCEPTION_DISABLE();

            p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
            hal_xqspi_init(p_exflash->p_xqspi);

            p_exflash->state = HAL_EXFLASH_STATE_READY;

            GLOBAL_EXCEPTION_ENABLE();

            ENABLE_INTERRUPT_LOW();
        }
        else
        {
            exflash_deepsleep(p_exflash);
            p_exflash->state = HAL_EXFLASH_STATE_READY;
        }
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_exflash_read(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t size)
{
    hal_status_t status       = HAL_OK;

    /* Check the parameters */
    if ((addr < EXFLASH_START_ADDR) || ((addr + size) >= (EXFLASH_START_ADDR + p_exflash->flash_size)) || (p_data == NULL) || \
        (size == 0) || (HAL_EXFLASH_ENCRYPTED == p_exflash->security && (addr & FLASH_ALIGNED_4BYTES)) )
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;
        p_exflash->state = HAL_EXFLASH_STATE_BUSY_READ;

        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {  //xip mode
                if ((HAL_EXFLASH_UNENCRYPTED == p_exflash->security) && sys_security_enable_status_check())
                {
                    GLOBAL_EXCEPTION_DISABLE();

                    uint32_t __l_present_rest = ll_xqspi_get_present_bypass(p_exflash->p_xqspi->p_instance);
                    hal_xqspi_set_xip_present_status(p_exflash->p_xqspi, XQSPI_DISABLE_PRESENT);

                    addr += EXFLASH_ALIAS_OFFSET;
                    for (uint32_t i = 0; i < size; i++)
                        p_data[i] = ((volatile uint8_t *)addr)[i];

                    hal_xqspi_set_xip_present_status(p_exflash->p_xqspi, __l_present_rest);
                    GLOBAL_EXCEPTION_ENABLE();
                }
                else
                {
                    addr += EXFLASH_ALIAS_OFFSET;
                    for (uint32_t i = 0; i < size; i++)
                        p_data[i] = ((volatile uint8_t *)addr)[i];
                }
            }
            else
            {  //QSPI mode
                uint32_t __l_present_rest = ll_xqspi_get_present_bypass(p_exflash->p_xqspi->p_instance);
                if ((HAL_EXFLASH_ENCRYPTED == p_exflash->security) && sys_security_enable_status_check())
                    ll_xqspi_set_present_bypass(p_exflash->p_xqspi->p_instance, LL_XQSPI_ENABLE_PRESENT);
                else
                    ll_xqspi_set_present_bypass(p_exflash->p_xqspi->p_instance, LL_XQSPI_DISABLE_PRESENT);

                exflash_wakeup(p_exflash);
                status = exflash_check_id(p_exflash);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_ID;
                    break;
                }
                
                p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
                hal_xqspi_init(p_exflash->p_xqspi);

                addr += EXFLASH_ALIAS_OFFSET;
                for (uint32_t i = 0; i < size; i++)
                    p_data[i] = ((volatile uint8_t *)addr)[i];

                p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
                hal_xqspi_init(p_exflash->p_xqspi);
                exflash_deepsleep(p_exflash);
                
                ll_xqspi_set_present_bypass(p_exflash->p_xqspi->p_instance, __l_present_rest);
            }
        }while(0);

        p_exflash->state = HAL_EXFLASH_STATE_READY;
    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_exflash_erase(exflash_handle_t *p_exflash, uint32_t erase_type, uint32_t addr, uint32_t size)
{
    hal_status_t status       = HAL_OK;
    uint32_t     absolute_addr = addr - EXFLASH_START_ADDR;
    uint32_t     __l_basepri_rest = GET_IRQ_RESET();

    /* Check the parameters */
    gr_assert_param(IS_EXFLASH_ERASE_TYPE(erase_type));
    if ((erase_type == EXFLASH_ERASE_SECTOR) &&
        ((addr < EXFLASH_START_ADDR) || ((addr + size) >= (EXFLASH_START_ADDR + p_exflash->flash_size)) || (0 == size)))
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_INVALID_PARAM;
        return HAL_ERROR;
    }

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {
                DISABLE_INTERRUPT_LOW();

                /* Disable global interrupt, aviod call function in flash */
                GLOBAL_EXCEPTION_DISABLE();

                /* Configure XQSPI initial paraments */
                p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
                hal_xqspi_init(p_exflash->p_xqspi);

                p_exflash->state = HAL_EXFLASH_STATE_BUSY;

                GLOBAL_EXCEPTION_ENABLE();
            }
            else
            {
                /* EXFLASH in QSPI mode */
                exflash_wakeup(p_exflash);
                status = exflash_check_id(p_exflash);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_ID;
                    break;
                }
                p_exflash->state = HAL_EXFLASH_STATE_BUSY;
            }

            if (EXFLASH_ERASE_CHIP == erase_type)
            {
                status = exflash_erase_chip(p_exflash);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_TIMEOUT;
                    break;
                }
            }
            else
            {
                uint32_t erase_addr = absolute_addr & ~(EXFLASH_SIZE_SECTOR_BYTES - 1);

                while (erase_addr < (absolute_addr + size))
                {
                    status = exflash_erase_sector(p_exflash, erase_addr);
                    if (HAL_OK != status)
                    {
                        p_exflash->error_code = HAL_EXFLASH_ERROR_TIMEOUT;
                        break;
                    }
                    erase_addr += EXFLASH_SIZE_SECTOR_BYTES;
                }
            }
        } while(0);

        if (XQSPI_WORK_MODE_XIP == p_exflash->fw_mode)
        {
            GLOBAL_EXCEPTION_DISABLE();

            p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
            hal_xqspi_init(p_exflash->p_xqspi);

            p_exflash->state = HAL_EXFLASH_STATE_READY;

            GLOBAL_EXCEPTION_ENABLE();

            ENABLE_INTERRUPT_LOW();
        }
        else
        {
            exflash_deepsleep(p_exflash);
            p_exflash->state = HAL_EXFLASH_STATE_READY;
        }

    }
    else
    {
        status = HAL_BUSY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

hal_status_t hal_exflash_suspend(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;

    if (p_exflash->fw_mode != XQSPI_WORK_MODE_XIP)
        return HAL_ERROR;

    if ((p_exflash->state != HAL_EXFLASH_STATE_BUSY) && \
        (p_exflash->state != HAL_EXFLASH_STATE_BUSY_WRITE) && \
        (p_exflash->state != HAL_EXFLASH_STATE_BUSY_ERASE))
    {
        return HAL_ERROR;
    }

    if (p_exflash->state != HAL_EXFLASH_STATE_BUSY)
    {
        /* Suspend write/erase operation */
        status = exflash_suspend(p_exflash);
        if (status != HAL_OK)
        {
            return status;
        }
        p_exflash->state = (p_exflash->state == HAL_EXFLASH_STATE_BUSY_WRITE) ? \
                           HAL_EXFLASH_STATE_SUSPEND_WRITE : HAL_EXFLASH_STATE_SUSPEND_ERASE;
    }

    /* Enable XIP mode */
    p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_XIP;
    hal_xqspi_init(p_exflash->p_xqspi);

    return HAL_OK;
}

hal_status_t hal_exflash_resume(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;

    if (p_exflash->fw_mode != XQSPI_WORK_MODE_XIP)
        return HAL_ERROR;

    if ((p_exflash->state != HAL_EXFLASH_STATE_BUSY) && \
        (p_exflash->state != HAL_EXFLASH_STATE_SUSPEND_WRITE) && \
        (p_exflash->state != HAL_EXFLASH_STATE_SUSPEND_ERASE))
    {
        return HAL_ERROR;
    }

    /* Enable XIP mode */
    p_exflash->p_xqspi->init.work_mode = XQSPI_WORK_MODE_QSPI;
    hal_xqspi_init(p_exflash->p_xqspi);

    if (p_exflash->state != HAL_EXFLASH_STATE_BUSY)
    {
        /* Suspend write/erase operation */
        status = exflash_resume(p_exflash);
        if (status != HAL_OK)
        {
            return status;
        }
        p_exflash->state = (p_exflash->state == HAL_EXFLASH_STATE_SUSPEND_WRITE) ? \
                           HAL_EXFLASH_STATE_BUSY_WRITE : HAL_EXFLASH_STATE_BUSY_ERASE;
    }

    return HAL_OK;
}

__WEAK hal_status_t hal_exflash_lock(exflash_handle_t *p_exflash, uint32_t lock_type)
{
    hal_status_t status = HAL_OK;
    xqspi_command_t command;
    uint8_t         tmp;
    uint16_t        reg_status;

    /* Check the parameters */
    gr_assert_param(IS_EXFLASH_LOCK_AREA(lock_type));

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

        /* Update EXFALSH state */
        p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {
                status = HAL_ERROR;
            }
            else
            {
                command.inst           = 0x35;
                command.addr           = 0;
                command.inst_size      = XQSPI_INSTSIZE_08_BITS;
                command.addr_size      = XQSPI_ADDRSIZE_00_BITS;
                command.dummy_cycles   = 0;
                command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
                command.data_mode      = XQSPI_DATA_MODE_SPI;
                command.length         = 1;

                hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
                reg_status = (uint16_t)tmp << 8;

                command.inst = 0x05;
                hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
                reg_status |= tmp;

                if (lock_type & 0x20)       // CMP == 1
                {
                    reg_status |= 0x4000;
                }
                reg_status |= (lock_type & 0x1F) << 2;

                status = exflash_write_status(p_exflash, reg_status);
            }
        } while(0);

        p_exflash->state = HAL_EXFLASH_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_exflash_unlock(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;
    xqspi_command_t command;
    uint8_t         tmp;
    uint16_t        reg_status;

    /* Check the parameters */
    gr_assert_param(IS_EXFLASH_LOCK_AREA(lock_type));

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

        /* Update EXFALSH state */
        p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {
                status = HAL_ERROR;
            }
            else
            {
                command.inst           = 0x35;
                command.addr           = 0;
                command.inst_size      = XQSPI_INSTSIZE_08_BITS;
                command.addr_size      = XQSPI_ADDRSIZE_00_BITS;
                command.dummy_cycles   = 0;
                command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
                command.data_mode      = XQSPI_DATA_MODE_SPI;
                command.length         = 1;

                hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
                reg_status = (uint16_t)tmp << 8;

                command.inst = 0x05;
                hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
                reg_status |= tmp;

                if (0 != (reg_status & 0x407C))
                {
                    reg_status &= ~0x407C;
                    status = exflash_write_status(p_exflash, reg_status);
                }
            }
        } while(0);

        p_exflash->state = HAL_EXFLASH_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_exflash_deepsleep(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

        /* Update EXFALSH state */
        p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {
                status = HAL_ERROR;
            }
            else
            {
                status = exflash_deepsleep(p_exflash);
                if (HAL_OK != status)
                {
                    p_exflash->error_code = HAL_EXFLASH_ERROR_TIMEOUT;
                    break;
                }
            }
        } while(0);

        p_exflash->state = HAL_EXFLASH_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_exflash_wakeup(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

        /* Update EXFALSH state */
        p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {
                status = HAL_ERROR;
            }
            else
            {
                status = exflash_wakeup(p_exflash);
                if (HAL_OK == status)
                {
                    status = exflash_check_id(p_exflash);

                }
            }
        } while(0);

        p_exflash->state = HAL_EXFLASH_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_status_t hal_exflash_reset(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;

    /* Process locked */
    __HAL_LOCK(p_exflash);

    if (HAL_EXFLASH_STATE_READY == p_exflash->state)
    {
        p_exflash->error_code = HAL_EXFLASH_ERROR_NONE;

        /* Update EXFALSH state */
        p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        do {
            if (ll_xqspi_get_xip_flag(p_exflash->p_xqspi->p_instance))
            {
                status = HAL_ERROR;
            }
            else
            {
                status = exflash_reset(p_exflash);
            }
        } while(0);

        p_exflash->state = HAL_EXFLASH_STATE_READY;
    }

    /* Process unlocked */
    __HAL_UNLOCK(p_exflash);

    /* Return function status */
    return status;
}

__WEAK hal_exflash_state_t hal_exflash_get_state(exflash_handle_t *p_exflash)
{
    /* Return QSPI handle state */
    return p_exflash->state;
}

__WEAK uint32_t hal_exflash_get_error(exflash_handle_t *p_exflash)
{
    return p_exflash->error_code;
}

__WEAK void hal_exflash_set_retry(exflash_handle_t *p_exflash, uint32_t retry)
{
    p_exflash->retry = retry;
}

static hal_status_t exflash_check_id(exflash_handle_t *p_exflash)
{
    hal_status_t    status = HAL_OK;
    xqspi_command_t command;
    uint8_t id[3], retry = 100;

    command.inst           = 0x9F;
    command.addr           = 0;
    command.inst_size      = XQSPI_INSTSIZE_08_BITS;
    command.addr_size      = XQSPI_ADDRSIZE_00_BITS;
    command.dummy_cycles   = 0;
    command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
    command.data_mode      = XQSPI_DATA_MODE_SPI;
    command.length         = 3;

    while (retry)
    {
        status = hal_xqspi_command_receive(p_exflash->p_xqspi, &command, id, 1000);
        if (HAL_OK != status)
            return status;

        if ((FLASH_MANU_ID_INVALID0 != id[0]) && (FLASH_MANU_ID_INVALID1 != id[0]))
        {
            p_exflash->flash_id = id[2] + (id[1] << 8) + (id[0] << 16);
            p_exflash->flash_size = 0x10000 << (id[2] & 0x0F);
            break;
        }

        retry--;
    }

    return (retry == 0) ? HAL_ERROR : HAL_OK;
}

static hal_status_t exflash_enable_write(exflash_handle_t *p_exflash)
{
    uint8_t command[1] = {0x06};

    return hal_xqspi_transmit(p_exflash->p_xqspi, command, sizeof(command), 1000);
}

static hal_status_t exflash_wait_busy(exflash_handle_t *p_exflash, uint32_t retry)
{
    xqspi_command_t command;
    uint8_t         status;

    command.inst           = 0x05;
    command.addr           = 0;
    command.inst_size      = XQSPI_INSTSIZE_08_BITS;
    command.addr_size      = XQSPI_ADDRSIZE_00_BITS;
    command.dummy_cycles   = 0;
    command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
    command.data_mode      = XQSPI_DATA_MODE_SPI;
    command.length         = 1;

    do {
        GLOBAL_EXCEPTION_DISABLE();
        hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &status, 1000);
        if (0 == (status & FLASH_STATUS_WIP_MASK))
        {
            p_exflash->state = HAL_EXFLASH_STATE_BUSY;
        }
        GLOBAL_EXCEPTION_ENABLE();

        if (HAL_MAX_DELAY != retry)
        {
            if (retry-- == 0)
            {
                p_exflash->state       = HAL_EXFLASH_STATE_ERROR;
                p_exflash->error_code |= HAL_EXFLASH_ERROR_TIMEOUT;

                return HAL_ERROR;
            }
        }
    } while(status & FLASH_STATUS_WIP_MASK);

    return HAL_OK;
}

static hal_status_t exflash_write_status(exflash_handle_t *p_exflash, uint16_t data)
{
    hal_status_t status = HAL_OK;
    uint8_t      length, command[3];
    uint8_t      manu_id, mem_size;

    manu_id = (p_exflash->flash_id & 0xFF0000) >> 16;
    mem_size = (p_exflash->flash_id & 0xFF);
    //check flash size.
    length = ((manu_id == FLASH_MANU_PY_ID) && (mem_size >= FLASH_MEM_SIZE_32M)) ? (sizeof(command) - 1) : sizeof(command);

    command[0] = 0x01; //Write Status Register
    command[1] = data & 0xFF;
    command[2] = (data >> 8) & 0xFF;

    do {
        status = exflash_enable_write(p_exflash);
        if (HAL_OK != status)
            break;

        status = hal_xqspi_transmit(p_exflash->p_xqspi, command, length, 1000);
        if (HAL_OK != status)
            break;

        status = exflash_wait_busy(p_exflash, HAL_EXFLASH_RETRY_DEFAULT_VALUE);

    } while(0);

    if (length != sizeof(command))
    {
        command[0] = 0x31; //Write Status Register-1
        command[1] = (data >> 8) & 0xFF;
        do {
            status = exflash_enable_write(p_exflash);
            if (HAL_OK != status)
                break;

            status = hal_xqspi_transmit(p_exflash->p_xqspi, command, length, 1000);
            if (HAL_OK != status)
                break;

            status = exflash_wait_busy(p_exflash, HAL_EXFLASH_RETRY_DEFAULT_VALUE);

        } while(0);
    }

    return status;
}



static hal_status_t exflash_enable_quad(exflash_handle_t *p_exflash)
{
    hal_status_t    status = HAL_OK;
    xqspi_command_t command;
    uint8_t         tmp;
    uint16_t        reg_status;

    command.inst           = 0x35;
    command.addr           = 0;
    command.inst_size      = XQSPI_INSTSIZE_08_BITS;
    command.addr_size      = XQSPI_ADDRSIZE_00_BITS;
    command.dummy_cycles   = 0;
    command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
    command.data_mode      = XQSPI_DATA_MODE_SPI;
    command.length         = 1;

    hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
    if ((tmp & 0x2) != 0x2)
    {
        reg_status = (uint16_t)(tmp | 0x2) << 8;
        command.inst = 0x05;
        hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &tmp, 1000);
        reg_status |= tmp;

        status = exflash_write_status(p_exflash, reg_status);
    }
    return status;
}

static hal_status_t exflash_erase_sector(exflash_handle_t *p_exflash, uint32_t addr)
{
    hal_status_t status = HAL_OK;
    uint8_t      command[4];

    command[0] = 0x20;
    command[1] = (addr >> 16) & 0xFF;
    command[2] = (addr >> 8) & 0xFF;
    command[3] =  addr & 0xFF;

    do {
        GLOBAL_EXCEPTION_DISABLE();
        /* Update EXFLASH state */
        if (HAL_EXFLASH_STATE_BUSY == p_exflash->state)
            p_exflash->state = HAL_EXFLASH_STATE_BUSY_ERASE;

        status = exflash_enable_write(p_exflash);
        if (HAL_OK == status)
        {
            status = hal_xqspi_transmit(p_exflash->p_xqspi, command, sizeof(command), 1000);
        }

        GLOBAL_EXCEPTION_ENABLE();
        if (HAL_OK != status)
            break;

        status = exflash_wait_busy(p_exflash, HAL_EXFLASH_RETRY_DEFAULT_VALUE);
    } while(0);

    return status;
}

static hal_status_t exflash_erase_chip(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;
    uint8_t      command[1] = {0x60};

    do {
        GLOBAL_EXCEPTION_DISABLE();
        /* Update EXFLASH state */
        if (HAL_EXFLASH_STATE_BUSY == p_exflash->state)
            p_exflash->state = HAL_EXFLASH_STATE_BUSY_ERASE;

        status = exflash_enable_write(p_exflash);
        if (HAL_OK == status)
        {
            status = hal_xqspi_transmit(p_exflash->p_xqspi, command, sizeof(command), 1000);
        }

        GLOBAL_EXCEPTION_ENABLE();
        if (HAL_OK != status)
            break;

        status = exflash_wait_busy(p_exflash, HAL_EXFLASH_RETRY_DEFAULT_VALUE);
    } while(0);

    return status;
}

static hal_status_t exflash_reset(exflash_handle_t *p_exflash)
{
    hal_status_t status = HAL_OK;
    uint8_t      command[1] = {0x66};

    do {
        status = hal_xqspi_transmit(p_exflash->p_xqspi, command, sizeof(command), 1000);
        if (HAL_OK != status)
            break;

        command[0] = 0x99;
        status = hal_xqspi_transmit(p_exflash->p_xqspi, command, sizeof(command), 1000);
    } while(0);

    return status;
}

static hal_status_t exflash_deepsleep(exflash_handle_t *p_exflash)
{
    uint8_t command[1] = {0xB9};
    return hal_xqspi_transmit(p_exflash->p_xqspi, command, sizeof(command), 1000);
}

static hal_status_t exflash_wakeup(exflash_handle_t *p_exflash)
{
    uint8_t command[1] = {0xAB};

    return hal_xqspi_transmit(p_exflash->p_xqspi, command, sizeof(command), 1000);
}

static hal_status_t exflash_page_program(exflash_handle_t *p_exflash, uint32_t addr, uint8_t *p_data, uint32_t nbytes, uint32_t retry)
{
    hal_status_t status = HAL_OK;
    uint8_t      buffer[260];
    uint16_t     length = EXFLASH_SIZE_PAGE_BYTES - (addr & 0xFF);

    buffer[0] = 0x02;
    buffer[1] = (addr >> 16) & 0xFF;
    buffer[2] = (addr >> 8) & 0xFF;
    buffer[3] =  addr & 0xFF;
    for (uint32_t i = 0; i < nbytes; i++)
        buffer[4 + i] = p_data[i];

    do {
        GLOBAL_EXCEPTION_DISABLE();

        /* Update EXFLASH state */
        if (HAL_EXFLASH_STATE_BUSY == p_exflash->state)
            p_exflash->state = HAL_EXFLASH_STATE_BUSY_WRITE;

        status = exflash_enable_write(p_exflash);
        if (HAL_OK == status)
        {
            length = length > nbytes ? nbytes : length;
            status = hal_xqspi_transmit(p_exflash->p_xqspi, buffer, length + 4, retry);
        }
        GLOBAL_EXCEPTION_ENABLE();
        if (HAL_OK != status)
            break;

        status = exflash_wait_busy(p_exflash, HAL_EXFLASH_RETRY_DEFAULT_VALUE);
    } while(0);

    return status;
}

static hal_status_t exflash_suspend(exflash_handle_t *p_exflash)
{
    hal_status_t    status = HAL_OK;
    xqspi_command_t command;
    uint8_t         cmd_suspend[1] = {0x75};
    uint8_t         reg_status;

    command.inst           = 0x05;
    command.addr           = 0;
    command.inst_size      = XQSPI_INSTSIZE_08_BITS;
    command.addr_size      = XQSPI_ADDRSIZE_00_BITS;
    command.dummy_cycles   = 0;
    command.inst_addr_mode = XQSPI_INST_ADDR_ALL_IN_SPI;
    command.data_mode      = XQSPI_DATA_MODE_SPI;
    command.length         = 1;

    status = hal_xqspi_transmit(p_exflash->p_xqspi, cmd_suspend, sizeof(cmd_suspend), 1000);
    if (status != HAL_OK)
    {
        return status;
    }

    /* Delay maxmum 30us for PY, or 20us for GD */
    while(1)
    {
        hal_xqspi_command_receive(p_exflash->p_xqspi, &command, &reg_status, 1000);
        if (!(reg_status & FLASH_STATUS_WIP_MASK))
            break;
    }

    return HAL_OK;
}

static hal_status_t exflash_resume(exflash_handle_t *p_exflash)
{
    uint8_t cmd_resume[1] = {0x7A};

    return hal_xqspi_transmit(p_exflash->p_xqspi, cmd_resume, sizeof(cmd_resume), 1000);
}

#endif /* HAL_EXFLASH_MODULE_ENABLED */


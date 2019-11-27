/**
  ****************************************************************************************
  * @file    gr55xx_hal_i2c.c
  * @author  BLE Driver Team
  * @brief   I2C HAL module driver.
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

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_I2C_MODULE_ENABLED

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/** @defgroup I2C_Private_Define I2C Private Define
  * @{
  */
#define I2C_TIMEOUT_ADDR    (10000U)       /*!< 10 s  */
#define I2C_TIMEOUT_BUSY    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_RXNE    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_STOP    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_TFNF    (25U)          /*!< 25 ms */
#define I2C_TIMEOUT_FLAG    (25U)          /*!< 25 ms */

/* Private define for @ref PreviousState usage */
#define I2C_STATE_MSK             ((uint32_t)((HAL_I2C_STATE_BUSY_TX | HAL_I2C_STATE_BUSY_RX) & (~((uint32_t)HAL_I2C_STATE_READY)))) /*!< Mask State define, keep only RX and TX bits            */
#define I2C_STATE_NONE            ((uint32_t)(HAL_I2C_MODE_NONE))                                                        /*!< Default Value                                          */
#define I2C_STATE_MASTER_BUSY_TX  ((uint32_t)((HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | HAL_I2C_MODE_MASTER))            /*!< Master Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MASTER_BUSY_RX  ((uint32_t)((HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | HAL_I2C_MODE_MASTER))            /*!< Master Busy RX, combinaison of State LSB and Mode enum */
#define I2C_STATE_SLAVE_BUSY_TX   ((uint32_t)((HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | HAL_I2C_MODE_SLAVE))             /*!< Slave Busy TX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_SLAVE_BUSY_RX   ((uint32_t)((HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | HAL_I2C_MODE_SLAVE))             /*!< Slave Busy RX, combinaison of State LSB and Mode enum  */
#define I2C_STATE_MEM_BUSY_TX     ((uint32_t)((HAL_I2C_STATE_BUSY_TX & I2C_STATE_MSK) | HAL_I2C_MODE_MEM))               /*!< Memory Busy TX, combinaison of State LSB and Mode enum */
#define I2C_STATE_MEM_BUSY_RX     ((uint32_t)((HAL_I2C_STATE_BUSY_RX & I2C_STATE_MSK) | HAL_I2C_MODE_MEM))               /*!< Memory Busy RX, combinaison of State LSB and Mode enum */

/* Private define to centralize the enable/disable of Interrupts */
#define I2C_MST_XFER_TX_IT      (LL_I2C_INTR_MASK_TX_ABRT  | \
                                 LL_I2C_INTR_MASK_TX_EMPTY | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_MST_XFER_RX_IT      (LL_I2C_INTR_MASK_TX_ABRT  | \
                                 LL_I2C_INTR_MASK_TX_EMPTY | \
                                 LL_I2C_INTR_MASK_RX_FULL  | \
                                 LL_I2C_INTR_MASK_RX_OVER  | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_SLV_XFER_TX_IT      (LL_I2C_INTR_MASK_TX_ABRT  | \
                                 LL_I2C_INTR_MASK_TX_EMPTY | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_SLV_XFER_RX_IT      (LL_I2C_INTR_MASK_TX_ABRT  | \
                                 LL_I2C_INTR_MASK_RX_FULL  | \
                                 LL_I2C_INTR_MASK_RX_OVER  | \
                                 LL_I2C_INTR_MASK_STOP_DET)

#define I2C_XFER_LISTEN_IT      (LL_I2C_INTR_MASK_TX_ABRT  | \
                                 LL_I2C_INTR_MASK_STOP_DET | \
                                 LL_I2C_INTR_MASK_RD_REQ)

#define I2C_XFER_ERROR_IT       (LL_I2C_INTR_MASK_TX_ABRT)

#define I2C_XFER_CPLT_IT        (LL_I2C_INTR_MASK_STOP_DET)

/* Private define to Abort Source */
#define I2C_TX_ABRT_NOACK       (LL_I2C_ABRT_GCALL_NOACK   | \
                                 LL_I2C_ABRT_TXDATA_NOACK  | \
                                 LL_I2C_ABRT_10ADDR2_NOACK | \
                                 LL_I2C_ABRT_10ADDR1_NOACK | \
                                 LL_I2C_ABRT_7B_ADDR_NOACK)

/* Private define Sequential Transfer Options default/reset value */
#define I2C_NO_OPTION_FRAME     (0xFFFF0000U)
/** @} */

/* Private macro -------------------------------------------------------------*/
#define I2C_ABS(a, b)   ((a) > (b) ? ((a) - (b)) : ((b) - (a)))

#define I2C_GET_TX_DMA_COUNT(__HANDLE__) \
     I2C_ABS(((uint32_t)(__HANDLE__)->p_buffer), ll_dma_get_source_address(DMA, (__HANDLE__)->p_dmatx->instance))

#define I2C_GET_RX_DMA_COUNT(__HANDLE__) \
     I2C_ABS(((uint32_t)(__HANDLE__)->p_buffer), ll_dma_get_destination_address(DMA, (__HANDLE__)->p_dmarx->instance))

#define I2C_GET_TX_DMA_REMAIN_DATA(__HANDLE__) ((__HANDLE__)->xfer_size - I2C_GET_TX_DMA_COUNT(__HANDLE__))
#define I2C_GET_RX_DMA_REMAIN_DATA(__HANDLE__) ((__HANDLE__)->xfer_size - I2C_GET_RX_DMA_COUNT(__HANDLE__))

#define I2C_GET_DMA_REMAIN_DATA(__HANDLE__) \
    ((((__HANDLE__)->state) == HAL_I2C_STATE_BUSY_TX) ? \
    I2C_GET_TX_DMA_REMAIN_DATA(__HANDLE__) : I2C_GET_RX_DMA_REMAIN_DATA(__HANDLE__))

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/** @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */

/* Private function to config master or slave mode  */
__STATIC_INLINE hal_status_t i2c_master_transfer_config(i2c_handle_t *p_i2c, uint16_t dev_address);
__STATIC_INLINE hal_status_t i2c_slave_transfer_config(i2c_handle_t *p_i2c);

/* Private function to check error flags  */
static hal_status_t i2c_master_check_error(i2c_handle_t *p_i2c);
static hal_status_t i2c_slave_check_error(i2c_handle_t *p_i2c);

/* Private functions to handle flags during polling transfer */
static hal_status_t i2c_wait_on_flag_until_timeout(i2c_handle_t *p_i2c, __IM uint32_t *regs, uint32_t mask, \
        uint32_t status, uint32_t timeout, uint32_t tick_start);
static hal_status_t i2c_wait_on_raw_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout, uint32_t tick_start);
static hal_status_t i2c_wait_on_sta_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout, uint32_t tick_start);

/* Private functions to start master transfer */
static hal_status_t i2c_master_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start);
static hal_status_t i2c_master_start_receive(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start);
static hal_status_t i2c_master_start_transmit_it(i2c_handle_t *p_i2c);
static hal_status_t i2c_master_start_receive_it(i2c_handle_t *p_i2c);
static hal_status_t i2c_master_start_transmit_dma(i2c_handle_t *p_i2c);
static hal_status_t i2c_master_start_receive_dma(i2c_handle_t *p_i2c);

/* Private functions to start slave transfer */
static hal_status_t i2c_slave_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start);
static hal_status_t i2c_slave_start_receive(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start);

/* Private functions to handle IT transfer */
static hal_status_t i2c_req_mem_read_write(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, \
        uint16_t mem_addr_size);

/* Private functions for I2C transfer IRQ handler */
static hal_status_t i2c_master_isr_it(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);
static hal_status_t i2c_slave_isr_it(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);
static hal_status_t i2c_master_isr_dma(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);
static hal_status_t i2c_slave_isr_dma(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources);

/* Private functions to handle DMA transfer */
static void i2c_dma_master_transmit_cplt(dma_handle_t *p_dma);
static void i2c_dma_master_receive_cplt(dma_handle_t *p_dma);
static void i2c_dma_slave_transmit_cplt(dma_handle_t *p_dma);
static void i2c_dma_slave_receive_cplt(dma_handle_t *p_dma);
static void i2c_dma_error(dma_handle_t *p_dma);
static void i2c_dma_abort(dma_handle_t *p_dma);

/* Private functions to handle IT transfer */
//static void i2c_it_master_sequential_cplt(i2c_handle_t *p_i2c);
static void i2c_it_slave_sequential_cplt(i2c_handle_t *p_i2c);
static void i2c_it_master_cplt(i2c_handle_t *p_i2c);
static void i2c_it_slave_cplt(i2c_handle_t *p_i2c);
static void i2c_it_listen_cplt(i2c_handle_t *p_i2c);
static void i2c_it_error(i2c_handle_t *p_i2c, uint32_t error_code);

/** @} */

/* Exported functions --------------------------------------------------------*/

/** @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */

/** @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
  * @{
  */
__WEAK hal_status_t hal_i2c_init(i2c_handle_t *p_i2c)
{
    /* Check the I2C handle allocation */
    if (NULL == p_i2c)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));
    gr_assert_param(IS_I2C_SPEED(p_i2c->init.speed));
    gr_assert_param(IS_I2C_OWN_ADDRESS(p_i2c->init.own_address));
    gr_assert_param(IS_I2C_ADDRESSING_MODE(p_i2c->init.addressing_mode));
    gr_assert_param(IS_I2C_GENERAL_CALL(p_i2c->init.general_call_mode));

    if (HAL_I2C_STATE_RESET == p_i2c->state)
    {
        /* Allocate lock resource and initialize it */
        p_i2c->lock = HAL_UNLOCKED;

        /* Enable Clock for Serial blocks and Automatic turn off Serial blocks clock during WFI. */
        ll_cgc_disable_force_off_serial_hclk();
        ll_cgc_disable_wfi_off_serial_hclk();

        /* Enable I2Cx Clock */
        if(p_i2c->p_instance == I2C0)
        {
            ll_cgc_disable_force_off_i2c0_hclk();
        }
        else if(p_i2c->p_instance == I2C1)
        {
            ll_cgc_disable_force_off_i2c1_hclk();
        }

        /* init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        hal_i2c_msp_init(p_i2c);
    }

    p_i2c->state = HAL_I2C_STATE_BUSY;

    /* Disable the selected I2C peripheral */
    ll_i2c_disable(p_i2c->p_instance);

    /* Configure I2Cx: Frequency range */
    uint32_t hcnt, lcnt;
    ll_i2c_set_speed_mode(p_i2c->p_instance, __LL_I2C_CONVERT_SPEED_MODE(p_i2c->init.speed));
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    lcnt = SystemCoreClock / 2 / p_i2c->init.speed - 1;
    if (p_i2c->init.speed < I2C_SPEED_2000K)
    {
        hcnt = SystemCoreClock / 2 / p_i2c->init.speed - 7 - ll_i2c_get_spike_len_fs(p_i2c->p_instance);
        if (p_i2c->init.speed < I2C_SPEED_400K)
        {
            ll_i2c_set_clock_high_period_ss(p_i2c->p_instance, hcnt);
            ll_i2c_set_clock_low_period_ss(p_i2c->p_instance, lcnt);
        }
        else
        {
            ll_i2c_set_clock_high_period_fs(p_i2c->p_instance, hcnt);
            ll_i2c_set_clock_low_period_fs(p_i2c->p_instance, lcnt);
        }
    }
    else
    {
        hcnt = SystemCoreClock / 2 / p_i2c->init.speed - 7 - ll_i2c_get_spike_len_hs(p_i2c->p_instance);
        ll_i2c_set_clock_high_period_hs(p_i2c->p_instance, hcnt);
        ll_i2c_set_clock_low_period_hs(p_i2c->p_instance, lcnt);
    }

    /* Configure I2Cx: Own Address, ack own address mode and Addressing Master mode */
    if (I2C_ADDRESSINGMODE_7BIT == p_i2c->init.addressing_mode)
    {
        ll_i2c_set_own_address(p_i2c->p_instance, p_i2c->init.own_address, LL_I2C_OWNADDRESS_7BIT);
        ll_i2c_set_master_addressing_mode(p_i2c->p_instance, LL_I2C_ADDRESSING_MODE_7BIT);
    }
    else /* I2C_ADDRESSINGMODE_10BIT */
    {
        ll_i2c_set_own_address(p_i2c->p_instance, p_i2c->init.own_address, LL_I2C_OWNADDRESS_10BIT);
        ll_i2c_set_master_addressing_mode(p_i2c->p_instance, LL_I2C_ADDRESSING_MODE_10BIT);
    }

    /* Configure I2Cx: Generalcall mode */
    if (I2C_GENERALCALL_ENABLE == p_i2c->init.general_call_mode)
    {
        ll_i2c_enable_general_call(p_i2c->p_instance);
    }
    else
    {
        ll_i2c_disable_general_call(p_i2c->p_instance);
    }

    /* CLear all interrupt */
    ll_i2c_clear_flag_intr(p_i2c->p_instance);
    /* Disable all interrupt */
    ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

    /* Enable the selected I2C peripheral */
    ll_i2c_enable(p_i2c->p_instance);

    p_i2c->error_code        = HAL_I2C_ERROR_NONE;
    p_i2c->state             = HAL_I2C_STATE_READY;
    p_i2c->previous_state    = I2C_STATE_NONE;
    p_i2c->mode              = HAL_I2C_MODE_NONE;

    return HAL_OK;
}

__WEAK hal_status_t hal_i2c_deinit(i2c_handle_t *p_i2c)
{
    /* Check the I2C handle allocation */
    if (NULL == p_i2c)
    {
        return HAL_ERROR;
    }

    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));

    if (p_i2c->state != HAL_I2C_STATE_RESET)
    {
        p_i2c->state = HAL_I2C_STATE_BUSY;

        /* Disable the I2C Peripheral Clock */
        ll_i2c_deinit(p_i2c->p_instance);

        /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
        hal_i2c_msp_deinit(p_i2c);

        /* Disable I2Cx Clock */
        if(p_i2c->p_instance == I2C0)
        {
            ll_cgc_enable_force_off_i2c0_hclk();
        }
        else if(p_i2c->p_instance == I2C1)
        {
            ll_cgc_enable_force_off_i2c1_hclk();
        }

        if((LL_CGC_FRC_I2S_S_HCLK & ll_cgc_get_force_off_hclk_0()) && 
          ((LL_CGC_FRC_SERIALS_HCLK2 & ll_cgc_get_force_off_hclk_2()) == LL_CGC_FRC_SERIALS_HCLK2))
        {
            /* Disable Clock for Serial blocks  */
            ll_cgc_enable_force_off_serial_hclk();
        }

        p_i2c->error_code        = HAL_I2C_ERROR_NONE;
        p_i2c->state             = HAL_I2C_STATE_RESET;
        p_i2c->previous_state    = I2C_STATE_NONE;
        p_i2c->mode              = HAL_I2C_MODE_NONE;
    }

    /* Release Lock */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

__WEAK void hal_i2c_msp_init(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_msp_deinit(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

/** @} */

/** @defgroup I2C_Exported_Functions_Group2 Input and Output operation functions
  * @brief   Data transfers functions
  * @{
  */

__WEAK hal_status_t hal_i2c_master_transmit(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        /* init tickstart for timeout management*/
        tickstart = hal_get_tick();

        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_ACTIVITY, I2C_STATUS_ACTIVITY, I2C_TIMEOUT_BUSY, tickstart))
        {
            return HAL_TIMEOUT;
        }

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_MASTER;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        return i2c_master_start_transmit(p_i2c, timeout, tickstart);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_master_receive(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        /* init tickstart for timeout management*/
        tickstart = hal_get_tick();

        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_ACTIVITY, I2C_STATUS_ACTIVITY, I2C_TIMEOUT_BUSY, tickstart))
        {
            return HAL_TIMEOUT;
        }

        p_i2c->state             = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode              = HAL_I2C_MODE_MASTER;
        p_i2c->error_code        = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_count        = size;
        p_i2c->xfer_isr          = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        return i2c_master_start_receive(p_i2c, timeout, tickstart);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_slave_transmit(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        /* init tickstart for timeout management*/
        tickstart = hal_get_tick();

        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_ACTIVITY, I2C_STATUS_ACTIVITY, I2C_TIMEOUT_BUSY, tickstart))
        {
            return HAL_TIMEOUT;
        }

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        return i2c_slave_start_transmit(p_i2c, timeout, tickstart);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_slave_receive(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        /* init tickstart for timeout management*/
        tickstart = hal_get_tick();

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        return i2c_slave_start_receive(p_i2c, timeout, tickstart);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_master_transmit_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_MASTER;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->xfer_count        = size;
        p_i2c->xfer_options      = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr          = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        return i2c_master_start_transmit_it(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_master_receive_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_MASTER;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->xfer_count        = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_options      = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr          = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        return i2c_master_start_receive_it(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_slave_transmit_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_it;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Set FIFO threshold */
        ll_i2c_set_tx_fifo_threshold(p_i2c->p_instance, LL_I2C_TX_FIFO_TH_CHAR_3);

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */

        /* Enable RD_REQ, STOP_DET interrupt */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_slave_receive_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
    uint32_t rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_1;

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_it;

        /* Increase RX FIFO threshold when data size >= 5 */
        if (5U <= p_i2c->xfer_size)
        {
            rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_5;
        }

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);
        /* Set FIFO threshold */
        ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, rxfifothreshold);

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */

        /* Enable TX_ABRT, RX_FULL, RX_OVER, STOP_DET interrupt */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_SLV_XFER_RX_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_master_transmit_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_MASTER;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        return i2c_master_start_transmit_dma(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_master_receive_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_MASTER;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        return i2c_master_start_receive_dma(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_slave_transmit_dma(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_dma;

        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmatx->xfer_tfr_callback = i2c_dma_slave_transmit_cplt;
        /* Set the DMA error callback */
        p_i2c->p_dmatx->xfer_error_callback = i2c_dma_error;
        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmatx->xfer_abort_callback = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);

        /* Set DMA transfer data level */
        ll_i2c_set_dma_tx_data_level(p_i2c->p_instance, 4U);
        ll_dma_set_destination_burst_length(DMA, p_i2c->p_dmatx->instance, LL_DMA_DST_BURST_LENGTH_4);

        /* Enable the DMA channel */
        hal_dma_start_it(p_i2c->p_dmatx, (uint32_t)p_data, ll_i2c_dma_get_register_address(p_i2c->p_instance), p_i2c->xfer_size);

        /* Update XferCount value */
        p_i2c->xfer_count = 0;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                    to avoid the risk of I2C interrupt handle execution before current
                    process unlock */

        /* Enable RD_REQ interrupts */
        /* DMA Request need to be Enabled when RD_REQ interrupt occurred */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_slave_receive_dma(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_SLAVE;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_slave_isr_dma;

        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmarx->xfer_tfr_callback = i2c_dma_slave_receive_cplt;

        /* Set the DMA error callback */
        p_i2c->p_dmarx->xfer_error_callback = i2c_dma_error;

        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmarx->xfer_abort_callback = NULL;

        /* Enable Slave Mode */
        i2c_slave_transfer_config(p_i2c);

        /* Set DMA transfer data level */
        ll_i2c_set_dma_rx_data_level(p_i2c->p_instance, 3U);
        ll_dma_set_source_burst_length(DMA, p_i2c->p_dmarx->instance, LL_DMA_SRC_BURST_LENGTH_4);

        /* Enable the DMA channel */
        hal_dma_start_it(p_i2c->p_dmarx, ll_i2c_dma_get_register_address(p_i2c->p_instance), (uint32_t)p_data, p_i2c->xfer_size);

        /* Update XferCount value */
        p_i2c->xfer_count = 0;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                    to avoid the risk of I2C interrupt handle execution before current
                    process unlock */

        /* Enable TX_ABORT interrupts */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_ERROR_IT | I2C_XFER_CPLT_IT);

        /* Enable DMA Request */
        ll_i2c_enable_dma_req_rx(p_i2c->p_instance);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_mem_write(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        /* init tickstart for timeout management*/
        tickstart = hal_get_tick();

        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_ACTIVITY, I2C_STATUS_ACTIVITY, I2C_TIMEOUT_BUSY, tickstart))
        {
            return HAL_TIMEOUT;
        }

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_MEM;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_isr      = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* After master re-configuration, TX FIFO should be empty */
        if (RESET == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);
            return HAL_ERROR;
        }

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_transmit(p_i2c, timeout, tickstart);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_mem_read(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout)
{
    uint32_t tickstart = 0U;

    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        /* init tickstart for timeout management*/
        tickstart = hal_get_tick();

        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_ACTIVITY, I2C_STATUS_ACTIVITY, I2C_TIMEOUT_BUSY, tickstart))
        {
            return HAL_TIMEOUT;
        }

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_MEM;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_count        = size;
        p_i2c->xfer_isr          = NULL;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);
        /* Disable all interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* After master re-configuration, TX FIFO should be empty */
        if (RESET == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);
            return HAL_ERROR;
        }

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_receive(p_i2c, timeout, tickstart);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_mem_write_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_MEM;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer          = p_data;
        p_i2c->xfer_size         = size;
        p_i2c->xfer_count        = size;
        p_i2c->master_ack_count  = size;
        p_i2c->xfer_options      = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr          = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (RESET == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);
            return HAL_ERROR;
        }

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_transmit_it(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_mem_read_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_MEM;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_it;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (RESET == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);
            return HAL_ERROR;
        }

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_receive_it(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_mem_write_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_TX;
        p_i2c->mode          = HAL_I2C_MODE_MEM;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (RESET == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);
            return HAL_ERROR;
        }

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_transmit_dma(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_mem_read_dma(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size)
{
    /* Check the parameters */
    gr_assert_param(IS_I2C_MEMADD_SIZE(mem_addr_size));

    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        if ((NULL == p_data) || (0U == size))
        {
            return  HAL_ERROR;
        }
        else if (SET == ll_i2c_is_active_flag_status_activity(p_i2c->p_instance))
        {
            return HAL_BUSY;
        }

        /* Process Locked */
        __HAL_LOCK(p_i2c);

        p_i2c->state         = HAL_I2C_STATE_BUSY_RX;
        p_i2c->mode          = HAL_I2C_MODE_MEM;
        p_i2c->error_code    = HAL_I2C_ERROR_NONE;

        /* Prepare transfer parameters */
        p_i2c->p_buffer      = p_data;
        p_i2c->xfer_size     = size;
        p_i2c->xfer_count    = size;
        p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
        p_i2c->xfer_isr      = i2c_master_isr_dma;

        /* Enable Master Mode and Set Slave Address */
        i2c_master_transfer_config(p_i2c, dev_address);

        /* After master re-configuration, TX FIFO should be empty */
        if (RESET == ll_i2c_is_active_flag_status_tfe(p_i2c->p_instance))
        {
            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);
            return HAL_ERROR;
        }

        /* Send Slave Address and Memory Address */
        i2c_req_mem_read_write(p_i2c, dev_address, mem_address, mem_addr_size);

        return i2c_master_start_receive_dma(p_i2c);
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_master_sequential_transmit_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t xfer_options)
{
    //TODO
    return HAL_ERROR;
}

__WEAK hal_status_t hal_i2c_master_sequential_receive_it(i2c_handle_t *p_i2c, uint16_t dev_address, uint8_t *p_data, uint16_t size, uint32_t xfer_options)
{
    //TODO
    return HAL_ERROR;
}

__WEAK hal_status_t hal_i2c_slave_sequential_transmit_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t xfer_options)
{
    //TODO
    return HAL_ERROR;
}

__WEAK hal_status_t hal_i2c_slave_sequential_receive_it(i2c_handle_t *p_i2c, uint8_t *p_data, uint16_t size, uint32_t xfer_options)
{
    //TODO
    return HAL_ERROR;
}

__WEAK hal_status_t hal_i2c_enable_listen_it(i2c_handle_t *p_i2c)
{
    if (HAL_I2C_STATE_READY == p_i2c->state)
    {
        p_i2c->state     = HAL_I2C_STATE_LISTEN;
        p_i2c->xfer_isr  = i2c_slave_isr_it;

        /* Enable the RD_REQ(master Read Request) interrupt */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_disable_listen_it(i2c_handle_t *p_i2c)
{
    /* Declaration of tmp to prevent undefined behavior of volatile usage */
    uint32_t tmp;

    /* Disable Address listen mode only if a transfer is not ongoing */
    if (HAL_I2C_STATE_LISTEN == p_i2c->state)
    {
        tmp = (uint32_t)(p_i2c->state) & I2C_STATE_MSK;
        p_i2c->previous_state = tmp | (uint32_t)(p_i2c->mode);
        p_i2c->state = HAL_I2C_STATE_READY;
        p_i2c->mode = HAL_I2C_MODE_NONE;
        p_i2c->xfer_isr = NULL;

        /* Disable the (master Read Request) interrupt */
        ll_i2c_disable_it(p_i2c->p_instance, I2C_XFER_LISTEN_IT);

        return HAL_OK;
    }
    else
    {
        return HAL_BUSY;
    }
}

__WEAK hal_status_t hal_i2c_master_abort_it(i2c_handle_t *p_i2c)
{

    if (HAL_I2C_MODE_MASTER == p_i2c->mode)
    {
        /* Process Locked */
        __HAL_LOCK(p_i2c);

        /* Disable Interrupts */
        ll_i2c_disable_it(p_i2c->p_instance, I2C_MST_XFER_RX_IT | I2C_MST_XFER_TX_IT);

        /* Set previous state */
        if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
        {
            p_i2c->previous_state = I2C_STATE_MASTER_BUSY_TX;
        }
        else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
        {
            p_i2c->previous_state = I2C_STATE_MASTER_BUSY_RX;
        }

        /* Set State at HAL_I2C_STATE_ABORT */
        p_i2c->state = HAL_I2C_STATE_ABORT;

        /* Abort DMA RX transfer if any */
        if ((SET == ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmarx))
        {
            p_i2c->p_dmarx->xfer_abort_callback = NULL;

            /* Abort DMA TX and Rx */
            if ((HAL_I2C_MODE_MASTER == p_i2c->mode) && (NULL != p_i2c->p_dmatx))
            {
                /* Master receive need to abort Tx too. */
                p_i2c->p_dmatx->xfer_abort_callback = NULL;
                hal_dma_abort_it(p_i2c->p_dmatx);
                ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
            }

            hal_dma_abort_it(p_i2c->p_dmarx);
            ll_i2c_disable_dma_req_rx(p_i2c->p_instance);
            p_i2c->xfer_count = I2C_GET_RX_DMA_REMAIN_DATA(p_i2c);
        }
        /* Abort DMA TX transfer if any */
        else if ((SET == ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmarx))
        {
            p_i2c->p_dmatx->xfer_abort_callback = NULL;

            /* Abort DMA TX */
            hal_dma_abort_it(p_i2c->p_dmatx);
            ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
            p_i2c->xfer_count = I2C_GET_TX_DMA_REMAIN_DATA(p_i2c);
        }

        /* issues a STOP and flushes the Tx FIFO after completing the current transfer */
        ll_i2c_enable_transfer_abort(p_i2c->p_instance);

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                    to avoid the risk of I2C interrupt handle execution before current
                    process unlock */
        ll_i2c_enable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_ABRT | LL_I2C_INTR_MASK_STOP_DET);

        return HAL_OK;
    }
    else
    {
        /* Wrong usage of abort function */
        /* This function should be used only in case of abort monitored by master device */
        return HAL_ERROR;
    }
}

/** @} */

/** @defgroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */

__WEAK void hal_i2c_irq_handler(i2c_handle_t *p_i2c)
{
    /* Get current IT sources value */
    uint32_t itsources    = READ_REG(p_i2c->p_instance->INTR_STAT);
    uint32_t abortsources = 0U;

    if (itsources & LL_I2C_INTR_STAT_TX_ABRT)
    {
        abortsources = ll_i2c_get_abort_source(p_i2c->p_instance);
        /* Clear TX ABORT Flag */
        ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);

        if ((abortsources & LL_I2C_ABRT_ARB_LOST) || (abortsources & LL_I2C_ABRT_SLV_ARBLOST))
        {
            p_i2c->error_code |= HAL_I2C_ERROR_ARB_LOST;
        }

        if ((HAL_I2C_STATE_BUSY_TX == p_i2c->state) ||
                ((I2C_STATE_MASTER_BUSY_TX == p_i2c->previous_state) && (HAL_I2C_STATE_ABORT == p_i2c->state)))
        {
            /* Flushed data were send failed */
            p_i2c->xfer_count += ll_i2c_get_tx_flush_count(p_i2c->p_instance);
        }
    }

    if (itsources & LL_I2C_INTR_STAT_RX_OVER)
    {
        ll_i2c_clear_flag_rx_over(p_i2c->p_instance);
        p_i2c->error_code |= HAL_I2C_ERROR_OVER;
    }

    if (HAL_I2C_ERROR_NONE != p_i2c->error_code)
    {
        i2c_it_error(p_i2c, p_i2c->error_code);
    }
    else if (NULL != p_i2c->xfer_isr)
    {
        p_i2c->xfer_isr(p_i2c, itsources, abortsources);
    }
}

__WEAK void hal_i2c_master_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_master_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_slave_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_slave_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_listen_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_mem_tx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_mem_rx_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_error_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

__WEAK void hal_i2c_abort_cplt_callback(i2c_handle_t *p_i2c)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_i2c);
}

/** @} */

/** @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
 *  @brief   Peripheral State, Mode and Error functions
  * @{
  */

__WEAK hal_i2c_state_t hal_i2c_get_state(i2c_handle_t *p_i2c)
{
    /* Return I2C handle state */
    return p_i2c->state;
}

__WEAK hal_i2c_mode_t hal_i2c_get_mode(i2c_handle_t *p_i2c)
{
    return p_i2c->mode;
}

__WEAK uint32_t hal_i2c_get_error(i2c_handle_t *p_i2c)
{
    return p_i2c->error_code;
}

/** @} */

/** @} */

/** @addtogroup I2C_Private_Functions
  * @{
  */

/**
  * @brief  Handles I2Cx communication when starting a master transfer.
  * @param  p_i2c I2C handle.
  * @param  DevAddress Specifies the slave address to be programmed.
  *   This parameter must be a value between 0 and 0x3FF.
  * @retval None
  */
__STATIC_INLINE hal_status_t i2c_master_transfer_config(i2c_handle_t *p_i2c, uint16_t dev_address)
{
    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));
    gr_assert_param(IS_I2C_SLV_ADDRESS(dev_address));

    /* Enable Master Mode and Set Slave Address */
    ll_i2c_disable(p_i2c->p_instance);
    ll_i2c_enable_master_mode(p_i2c->p_instance);
    ll_i2c_set_slave_address(p_i2c->p_instance, dev_address);
    ll_i2c_enable(p_i2c->p_instance);

    /* CLear all interrupt */
    ll_i2c_clear_flag_intr(p_i2c->p_instance);

    return HAL_OK;
}

/**
  * @brief  Handles I2Cx communication when starting a slave transfer.
  * @param  p_i2c I2C handle.
  * @retval None
  */
__STATIC_INLINE hal_status_t i2c_slave_transfer_config(i2c_handle_t *p_i2c)
{
    /* Check the parameters */
    gr_assert_param(IS_I2C_ALL_INSTANCE(p_i2c->p_instance));

    /* Enable Slave Mode and Enable STOP_DET only if addressed */
    ll_i2c_disable(p_i2c->p_instance);
    ll_i2c_disable_master_mode(p_i2c->p_instance);
    ll_i2c_enable_stop_det_if_addressed(p_i2c->p_instance);
    ll_i2c_enable(p_i2c->p_instance);

    /* CLear all interrupt */
    ll_i2c_clear_flag_intr(p_i2c->p_instance);

    return HAL_OK;
}

static hal_status_t i2c_master_check_error(i2c_handle_t *p_i2c)
{
    hal_status_t status = HAL_OK;
    uint32_t     abortsrc;

    if (ll_i2c_is_active_flag_raw_tx_abort(p_i2c->p_instance))
    {
        abortsrc = ll_i2c_get_abort_source(p_i2c->p_instance);
        if (abortsrc & LL_I2C_ABRT_ARB_LOST)
        {
            p_i2c->error_code = HAL_I2C_ERROR_ARB_LOST;
        }
        else if ((abortsrc & I2C_TX_ABRT_NOACK))
        {
            p_i2c->error_code = HAL_I2C_ERROR_NOACK;
        }

        ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);

        if (RESET == (abortsrc & LL_I2C_ABRT_USER_ABRT))
        {
            status = HAL_ERROR;
        }
    }
    else
    {
        if (ll_i2c_is_active_flag_raw_rx_over(p_i2c->p_instance))
        {
            ll_i2c_clear_flag_rx_over(p_i2c->p_instance);
            p_i2c->error_code = HAL_I2C_ERROR_OVER;
            status = HAL_ERROR;
        }
    }

    return status;
}

static hal_status_t i2c_slave_check_error(i2c_handle_t *p_i2c)
{
    hal_status_t status = HAL_OK;
    uint32_t     abortsrc;

    if (ll_i2c_is_active_flag_raw_tx_abort(p_i2c->p_instance))
    {
        if (abortsrc & LL_I2C_ABRT_SLV_ARBLOST)
        {
            p_i2c->error_code = HAL_I2C_ERROR_ARB_LOST;
        }
        else if ((abortsrc & I2C_TX_ABRT_NOACK))
        {
            p_i2c->error_code = HAL_I2C_ERROR_NOACK;
        }

        ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);
        status = HAL_ERROR;
    }
    else
    {
        if (ll_i2c_is_active_flag_raw_rx_over(p_i2c->p_instance))
        {
            ll_i2c_clear_flag_rx_over(p_i2c->p_instance);
            p_i2c->error_code = HAL_I2C_ERROR_OVER;
            status = HAL_ERROR;
        }
    }

    return status;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of flag.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static hal_status_t i2c_wait_on_flag_until_timeout(i2c_handle_t *p_i2c, __IM uint32_t *regs, uint32_t mask, \
        uint32_t status, uint32_t timeout, uint32_t tick_start)
{
    while ((*regs & mask) == status)
    {
        /* Check for the Timeout */
        if (HAL_MAX_DELAY != timeout)
        {
            if ((0U == timeout) || (timeout < (hal_get_tick() - tick_start)))
            {
                p_i2c->error_code |= HAL_I2C_ERROR_TIMEOUT;
                p_i2c->state = HAL_I2C_STATE_READY;
                p_i2c->mode = HAL_I2C_MODE_NONE;

                /* Process Unlocked */
                __HAL_UNLOCK(p_i2c);
                return HAL_TIMEOUT;
            }
        }
    }
    return HAL_OK;
}

static hal_status_t i2c_wait_on_raw_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout, uint32_t tick_start)
{
    return i2c_wait_on_flag_until_timeout(p_i2c, &p_i2c->p_instance->RAW_INTR_STAT, flag, status, timeout, tick_start);
}

static hal_status_t i2c_wait_on_sta_flag_until_timeout(i2c_handle_t *p_i2c, uint32_t flag, uint32_t status, \
        uint32_t timeout, uint32_t tick_start)
{
    return i2c_wait_on_flag_until_timeout(p_i2c, &p_i2c->p_instance->STATUS, flag, status, timeout, tick_start);
}

static hal_status_t i2c_master_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start)
{
    uint32_t cmd = LL_I2C_CMD_MST_WRITE;

    while (0U < p_i2c->xfer_count)
    {
        /* Wait until TFNF flag is set */
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_TFNF, 0, timeout, tick_start))
        {
            /* Check what kind of error */
            i2c_master_check_error(p_i2c);
            if (p_i2c->error_code & (HAL_I2C_ERROR_ARB_LOST | HAL_I2C_ERROR_NOACK))
                return HAL_ERROR;
            return HAL_TIMEOUT;
        }

        /* Generate STOP condition after transmit the last byte */
        if (1U == p_i2c->xfer_count)
        {
            cmd = LL_I2C_CMD_MST_WRITE | LL_I2C_CMD_MST_GEN_STOP;
        }

        ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, cmd);

        p_i2c->xfer_count--;
    }

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_INTR_STOP_DET, 0, timeout, tick_start))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    p_i2c->state = HAL_I2C_STATE_READY;
    p_i2c->mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

static hal_status_t i2c_master_start_receive(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start)
{
    uint32_t cmd = LL_I2C_CMD_MST_READ;

    while (0U < p_i2c->xfer_count)
    {
        /* Write the READ command into TX FIFO to generate ACK */
        while ((0 < p_i2c->master_ack_count) && (p_i2c->xfer_count - p_i2c->master_ack_count < 8))
        {
            /* Generate STOP condition after receive the last byte */
            if (1 == p_i2c->master_ack_count)
            {
                cmd = LL_I2C_CMD_MST_READ | LL_I2C_CMD_MST_GEN_STOP;
            }

            if (SET == ll_i2c_is_active_flag_status_tfnf(p_i2c->p_instance))
            {
                ll_i2c_transmit_data8(p_i2c->p_instance, 0, cmd);
                p_i2c->master_ack_count--;
            }
            else
            {
                break;
            }
        }
        /* Wait until RFNE flag is set */
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_RFNE, 0, timeout, tick_start))
        {
            /* Check what kind of error */
            i2c_master_check_error(p_i2c);
            if (p_i2c->error_code & (HAL_I2C_ERROR_ARB_LOST | HAL_I2C_ERROR_OVER))
                return HAL_ERROR;
            return HAL_TIMEOUT;
        }

        *p_i2c->p_buffer++ = ll_i2c_receive_data8(p_i2c->p_instance);
        p_i2c->xfer_count--;
    }

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_INTR_STOP_DET, 0, timeout, tick_start))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    p_i2c->state = HAL_I2C_STATE_READY;
    p_i2c->mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

static hal_status_t i2c_master_start_transmit_it(i2c_handle_t *p_i2c)
{
    /* Set FIFO threshold */
    ll_i2c_set_tx_fifo_threshold(p_i2c->p_instance, LL_I2C_TX_FIFO_TH_CHAR_3);

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
            to avoid the risk of I2C interrupt handle execution before current
            process unlock */

    /* Enable TX_ABRT, TX_EMPTY, STOP_DET interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_MST_XFER_TX_IT);

    return HAL_OK;
}

static hal_status_t i2c_master_start_receive_it(i2c_handle_t *p_i2c)
{
    uint32_t rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_1;

    /* Increase RX FIFO threshold when data size >= 5 */
    if (5U <= p_i2c->xfer_size)
    {
        rxfifothreshold = LL_I2C_RX_FIFO_TH_CHAR_5;
    }

    /* Set FIFO threshold */
    ll_i2c_set_tx_fifo_threshold(p_i2c->p_instance, LL_I2C_TX_FIFO_TH_CHAR_3);
    ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, rxfifothreshold);

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    /* Note : The I2C interrupts must be enabled after unlocking current process
            to avoid the risk of I2C interrupt handle execution before current
            process unlock */

    /* Enable TX_ABRT, TX_EMPTY, RX_FULL, RX_OVER, STOP_DET interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_MST_XFER_RX_IT);

    return HAL_OK;
}

static hal_status_t i2c_master_start_transmit_dma(i2c_handle_t *p_i2c)
{
    if (1 < p_i2c->xfer_size)
    {
        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmatx->xfer_tfr_callback     = i2c_dma_master_transmit_cplt;
        /* Set the DMA error callback */
        p_i2c->p_dmatx->xfer_error_callback   = i2c_dma_error;
        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmatx->xfer_abort_callback   = NULL;

        /* Set DMA transfer data level */
        ll_i2c_set_dma_tx_data_level(p_i2c->p_instance, 4U);
        ll_dma_set_destination_burst_length(DMA, p_i2c->p_dmatx->instance, LL_DMA_DST_BURST_LENGTH_4);

        /* Re-config increase mode and transfer width in case that HAL_I2C_Master_Receive_DMA has been called */
        ll_dma_set_source_increment_mode(DMA, p_i2c->p_dmatx->instance, p_i2c->p_dmatx->init.src_increment);
        ll_dma_set_source_width(DMA, p_i2c->p_dmatx->instance, p_i2c->p_dmatx->init.src_data_alignment);
        ll_dma_set_destination_width(DMA, p_i2c->p_dmatx->instance, p_i2c->p_dmatx->init.dst_data_alignment);

        /* Enable the DMA channel */
        hal_dma_start_it(p_i2c->p_dmatx, (uint32_t)(p_i2c->p_buffer), ll_i2c_dma_get_register_address(p_i2c->p_instance), p_i2c->xfer_size - 1);

        /* Update Buffer & XferCount value */
        p_i2c->p_buffer = &p_i2c->p_buffer[p_i2c->xfer_size - 1];
        p_i2c->xfer_count = 1;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
        /* Enable TX_ABORT interrupts */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_ERROR_IT);

        /* Enable DMA Request */
        ll_i2c_enable_dma_req_tx(p_i2c->p_instance);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        i2c_dma_master_transmit_cplt(p_i2c->p_dmatx);
    }

    return HAL_OK;
}

static hal_status_t i2c_master_start_receive_dma(i2c_handle_t *p_i2c)
{
    if (1 < p_i2c->xfer_size)
    {
        static uint32_t rxcmd = LL_I2C_CMD_MST_READ;

        /* Set the I2C DMA transfer complete callback */
        p_i2c->p_dmarx->xfer_tfr_callback     = i2c_dma_master_receive_cplt;
        /* Set the DMA error callback */
        p_i2c->p_dmarx->xfer_error_callback   = i2c_dma_error;
        /* Set the unused DMA callbacks to NULL */
        p_i2c->p_dmarx->xfer_abort_callback   = NULL;
        p_i2c->p_dmatx->xfer_tfr_callback     = NULL;
        p_i2c->p_dmatx->xfer_error_callback   = NULL;
        p_i2c->p_dmatx->xfer_abort_callback   = NULL;

        /* Set DMA transfer data level */
        ll_i2c_set_dma_rx_data_level(p_i2c->p_instance, 3U);
        ll_dma_set_source_burst_length(DMA, p_i2c->p_dmarx->instance, LL_DMA_SRC_BURST_LENGTH_4);

        /* Configure Tx channel to generate ACK during master receiving */
        /* During master receiving, Tx channel will be used to generate ACK. */
        ll_i2c_set_dma_tx_data_level(p_i2c->p_instance, 4U);
        ll_dma_set_destination_burst_length(DMA, p_i2c->p_dmatx->instance, LL_DMA_DST_BURST_LENGTH_4);
        /* Config the increase mode and  to NO_CHANGE to write the MST_READ command into TX FIFO */
        ll_dma_set_source_increment_mode(DMA, p_i2c->p_dmatx->instance, LL_DMA_SRC_NO_CHANGE);
        ll_dma_set_source_width(DMA, p_i2c->p_dmatx->instance, LL_DMA_SDATAALIGN_HALFWORD);
        ll_dma_set_destination_width(DMA, p_i2c->p_dmatx->instance, LL_DMA_DDATAALIGN_HALFWORD);

        /* Enable the DMA channel */
        hal_dma_start_it(p_i2c->p_dmatx, (uint32_t)&rxcmd, ll_i2c_dma_get_register_address(p_i2c->p_instance), p_i2c->xfer_size - 1);
        hal_dma_start_it(p_i2c->p_dmarx, ll_i2c_dma_get_register_address(p_i2c->p_instance), (uint32_t)(p_i2c->p_buffer), p_i2c->xfer_size - 1);

        /* Update Buffer & XferCount value */
        p_i2c->p_buffer = &p_i2c->p_buffer[p_i2c->xfer_size - 1];
        p_i2c->xfer_count = 1;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Note : The I2C interrupts must be enabled after unlocking current process
                    to avoid the risk of I2C interrupt handle execution before current
                    process unlock */
        /* Enable TX_ABORT interrupts */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_ERROR_IT);

        /* Enable DMA Request */
        ll_i2c_enable_dma_req_tx(p_i2c->p_instance);
        ll_i2c_enable_dma_req_rx(p_i2c->p_instance);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        i2c_dma_master_receive_cplt(p_i2c->p_dmarx);
    }

    return HAL_OK;
}

static hal_status_t i2c_slave_start_transmit(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start)
{
    while (0U < p_i2c->xfer_count)
    {
        /* Wait master read request */
        if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_INTR_RD_REQ, 0, timeout, tick_start))
        {
            /* Check what kind of error */
            i2c_slave_check_error(p_i2c);
            if (p_i2c->error_code & (HAL_I2C_ERROR_ARB_LOST | HAL_I2C_ERROR_NOACK))
                return HAL_ERROR;
            return HAL_TIMEOUT;
        }

        /* Clear the read req flag */
        ll_i2c_clear_flag_read_req(p_i2c->p_instance);

        ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, LL_I2C_CMD_SLV_NONE);
        p_i2c->xfer_count--;
    }

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_INTR_STOP_DET, 0, timeout, tick_start))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    p_i2c->state = HAL_I2C_STATE_READY;
    p_i2c->mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

static hal_status_t i2c_slave_start_receive(i2c_handle_t *p_i2c, uint32_t timeout, uint32_t tick_start)
{
    while (0U < p_i2c->xfer_count)
    {
        /* Wait until RFNE flag is set */
        if (HAL_OK != i2c_wait_on_sta_flag_until_timeout(p_i2c, I2C_STATUS_RFNE, 0, timeout, tick_start))
        {
            /* Check what kind of error */
            i2c_slave_check_error(p_i2c);
            if (p_i2c->error_code & (HAL_I2C_ERROR_ARB_LOST | HAL_I2C_ERROR_OVER))
                return HAL_ERROR;
            return HAL_TIMEOUT;
        }

        *p_i2c->p_buffer++ = ll_i2c_receive_data8(p_i2c->p_instance);
        p_i2c->xfer_count--;
    }

    /* Wait until STOP_DET flag is set */
    if (HAL_OK != i2c_wait_on_raw_flag_until_timeout(p_i2c, I2C_INTR_STOP_DET, 0, timeout, tick_start))
    {
        return HAL_TIMEOUT;
    }

    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    p_i2c->state = HAL_I2C_STATE_READY;
    p_i2c->mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

/**
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @retval HAL status
  */
static hal_status_t i2c_req_mem_read_write(i2c_handle_t *p_i2c, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size)
{
    uint32_t cmd = LL_I2C_CMD_MST_WRITE;

    /* If Memory address size is 16Bit, send MSB of Memory Address first */
    if (I2C_MEMADD_SIZE_16BIT == mem_addr_size)
    {
        ll_i2c_transmit_data8(p_i2c->p_instance, I2C_MEM_ADD_MSB(mem_address), cmd);
    }

    /* Send LSB of Memory Address */
    ll_i2c_transmit_data8(p_i2c->p_instance, I2C_MEM_ADD_LSB(mem_address), cmd);

    return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITSources Interrupt sources triggered.
  * @param  AbortSources Sources of TX_ABORT interrupt.
  * @retval HAL status
  */
static hal_status_t i2c_master_isr_it(i2c_handle_t *p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    uint32_t curxfercnt = 0U;
    uint32_t cmd = LL_I2C_CMD_MST_WRITE;

    /* Process Locked */
    __HAL_LOCK(p_i2c);

    if (RESET != (it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if (RESET != (abort_sources & I2C_TX_ABRT_NOACK))
        {
            /* Set corresponding Error Code */
            p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
        }
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_RX_FULL))
    {
        /* Get data count in RX FIFO */
        curxfercnt = ll_i2c_get_rx_fifo_level(p_i2c->p_instance);

        /* Read data from RX FIFO */
        while ((curxfercnt--) && (0U != p_i2c->xfer_count))
        {
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }

        if (p_i2c->xfer_count < (ll_i2c_get_rx_fifo_threshold(p_i2c->p_instance) + 1))
        {
            ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, LL_I2C_RX_FIFO_TH_CHAR_1);
        }

        if (0U == p_i2c->xfer_count)
        {
            ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RX_FULL);
        }
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_TX_EMPTY))
    {
        /* Get free data count in TX FIFO */
        curxfercnt = I2C_TXFIFO_SIZE - ll_i2c_get_tx_fifo_level(p_i2c->p_instance);

        /* Master transmit process */
        if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
        {
            /* Write data into TX FIFO */
            while ((curxfercnt--) && (0U != p_i2c->xfer_count))
            {
                if (1U == p_i2c->xfer_count)
                {
                    cmd |= LL_I2C_CMD_MST_GEN_STOP;
                }

                ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, cmd);
                p_i2c->xfer_count--;
            }

            if (0U == p_i2c->xfer_count)
            {
                ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);
            }
        }
        /* Master receive process */
        else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
        {
            cmd = LL_I2C_CMD_MST_READ;

            /* Write LL_I2C_CMD_MST_READ into TX FIFO to generate ACK */
            while ((curxfercnt--) && (0U != p_i2c->master_ack_count))
            {
                if (1U == p_i2c->master_ack_count)
                {
                    cmd |= LL_I2C_CMD_MST_GEN_STOP;
                }

                ll_i2c_transmit_data8(p_i2c->p_instance, 0U, cmd);
                p_i2c->master_ack_count--;
            }

            if (0U == p_i2c->master_ack_count)
            {
                ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);
            }
        }
    }

    if (RESET != (it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Master complete process */
        i2c_it_master_cplt(p_i2c);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  p_i2c    Pointer to a i2c_handle_t structure that contains
  *                 the configuration information for the specified I2C.
  * @param  it_source       Interrupt sources to handle.
  * @param  abort_sources   Abort sources.
  * @retval HAL status
  */
static hal_status_t i2c_slave_isr_it(i2c_handle_t *p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    uint32_t curxfercnt = 0U;

    /* Process locked */
    __HAL_LOCK(p_i2c);

    if (RESET != (it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if (RESET != (abort_sources & I2C_TX_ABRT_NOACK))
        {
            //  /* Check that I2C transfer finished */
            // /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
            // if (0U == p_i2c->XferCount)
            // {
            //     if (((I2C_FIRST_AND_LAST_FRAME == p_i2c->XferOptions) || (I2C_LAST_FRAME == p_i2c->XferOptions)) && \
            //         (HAL_I2C_STATE_LISTEN == p_i2c->State))
            //     {
            //         /* Call I2C Listen complete process */
            //         I2C_ITListenCplt(p_i2c);
            //     }
            //     else if ((I2C_NO_OPTION_FRAME != p_i2c->XferOptions) && (HAL_I2C_STATE_BUSY_TX_LISTEN == p_i2c->State))
            //     {
            //         /* Last Byte is Transmitted */
            //         /* Call I2C Slave Sequential complete process */
            //         I2C_ITSlaveSequentialCplt(p_i2c);
            //     }
            // }
            // else
            // {
            //     /* Set ErrorCode corresponding to a Non-Acknowledge */
            //     p_i2c->ErrorCode |= HAL_I2C_ERROR_NOACK;
            // }
        }
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_RD_REQ))
    {
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RD_REQ);
        /* Start transmit */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_SLV_XFER_TX_IT);
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_RX_FULL))
    {
        /* Get data count in RX FIFO */
        curxfercnt = ll_i2c_get_rx_fifo_level(p_i2c->p_instance);

        /* Read data from RX FIFO */
        while ((curxfercnt--) && (0U != p_i2c->xfer_count))
        {
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }

        if (p_i2c->xfer_count < (ll_i2c_get_rx_fifo_threshold(p_i2c->p_instance) + 1))
        {
            ll_i2c_set_rx_fifo_threshold(p_i2c->p_instance, LL_I2C_RX_FIFO_TH_CHAR_1);
        }

        if (0U == p_i2c->xfer_count)
        {
            ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RX_FULL);

            if (I2C_NO_OPTION_FRAME != p_i2c->xfer_options)
            {
                /* Call I2C Slave Sequential complete process */
                i2c_it_slave_sequential_cplt(p_i2c);
            }
        }
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_TX_EMPTY))
    {
        /* Get free data count in TX FIFO */
        curxfercnt = I2C_TXFIFO_SIZE - ll_i2c_get_tx_fifo_level(p_i2c->p_instance);

        /* Write data into TX FIFO */
        while ((curxfercnt--) && (0U != p_i2c->xfer_count))
        {
            ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer++, LL_I2C_CMD_SLV_NONE);
            p_i2c->xfer_count--;
        }

        if (0U == p_i2c->xfer_count)
        {
            ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_TX_EMPTY);

            if ((I2C_NEXT_FRAME == p_i2c->xfer_options) || (I2C_FIRST_FRAME == p_i2c->xfer_options))
            {
                /* Last Byte is Transmitted */
                /* Call I2C Slave Sequential complete process */
                i2c_it_slave_sequential_cplt(p_i2c);
            }
        }
    }

    /* Check if STOP_DET is set */
    if (RESET != (it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Slave complete process */
        i2c_it_slave_cplt(p_i2c);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with DMA.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  it_source       Interrupt sources to handle.
  * @param  abort_sources   Abort sources.
  * @retval HAL status
  */
static hal_status_t i2c_master_isr_dma(i2c_handle_t*p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    /* Process Locked */
    __HAL_LOCK(p_i2c);

    if (RESET != (it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if (RESET != (abort_sources & I2C_TX_ABRT_NOACK))
        {
            /* Set corresponding Error Code */
            p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
        }

        if (RESET == (abort_sources & LL_I2C_ABRT_USER_ABRT))
        {
            /* USER_ABRT will generate STOP automatically */
            ll_i2c_transmit_data8(p_i2c->p_instance, 0, LL_I2C_CMD_MST_READ | LL_I2C_CMD_MST_GEN_STOP);
        }
        /* Enable STOP interrupt, to treat it */
        /* Error callback will be send during stop flag treatment */
        ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_CPLT_IT);
    }

    if (RESET != (it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Master complete process */
        i2c_it_master_cplt(p_i2c);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

/**
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with DMA.
  * @param  p_i2c Pointer to a i2c_handle_t structure that contains
  *                the configuration information for the specified I2C.
  * @param  it_source       Interrupt sources to handle.
  * @param  abort_sources   Abort sources.
  * @retval HAL status
  */
static hal_status_t i2c_slave_isr_dma(i2c_handle_t* p_i2c, uint32_t it_source, uint32_t abort_sources)
{
    /* Process locked */
    __HAL_LOCK(p_i2c);

    if (RESET != (it_source & LL_I2C_INTR_STAT_TX_ABRT))
    {
        if ((RESET != (abort_sources & I2C_TX_ABRT_NOACK)) && (0 != I2C_GET_DMA_REMAIN_DATA(p_i2c)) )
        {
            /* Set corresponding Error Code */
            p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
        }
    }
    else if (RESET != (it_source & LL_I2C_INTR_STAT_RD_REQ))
    {
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_RD_REQ);
        /* Start transmit */
        ll_i2c_enable_dma_req_tx(p_i2c->p_instance);
    }

    /* Check if STOP_DET is set */
    if (RESET != (it_source & LL_I2C_INTR_STAT_STOP_DET))
    {
        /* Call I2C Slave complete process */
        i2c_it_slave_cplt(p_i2c);
    }

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    return HAL_OK;
}

/**
  * @brief  I2C Master sequential complete process.
  * @param  p_i2c I2C handle.
  * @retval None
  */
#if 0
static void i2c_it_master_sequential_cplt(i2c_handle_t *p_i2c)
{

    /* Reset I2C handle mode */
    p_i2c->mode = HAL_I2C_MODE_NONE;

    /* No Generate Stop, to permit restart mode */
    /* The stop will be done at the end of transfer, when I2C_AUTOEND_MODE enable */
    if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
    {
        p_i2c->state             = HAL_I2C_STATE_READY;
        p_i2c->previous_state    = I2C_STATE_MASTER_BUSY_TX;
        p_i2c->xfer_isr          = NULL;

        /* Disable Interrupts */
        I2C_Disable_IRQ(p_i2c, I2C_XFER_TX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_master_tx_cplt_callback(p_i2c);
    }
    /* p_i2c->State == HAL_I2C_STATE_BUSY_RX */
    else
    {
        p_i2c->state             = HAL_I2C_STATE_READY;
        p_i2c->previous_state    = I2C_STATE_MASTER_BUSY_RX;
        p_i2c->xfer_isr          = NULL;

        /* Disable Interrupts */
        I2C_Disable_IRQ(p_i2c, I2C_XFER_RX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_master_rx_cplt_callback(p_i2c);
    }
}
#endif

/**
  * @brief  I2C Slave sequential complete process.
  * @param  p_i2c I2C handle.
  * @retval None
  */
static void i2c_it_slave_sequential_cplt(i2c_handle_t *p_i2c)
{
#if 0
    /* Reset I2C handle mode */
    p_i2c->mode = HAL_I2C_MODE_NONE;

    if (HAL_I2C_STATE_BUSY_TX_LISTEN == p_i2c->state)
    {
        /* Remove HAL_I2C_STATE_SLAVE_BUSY_TX, keep only HAL_I2C_STATE_LISTEN */
        p_i2c->state          = HAL_I2C_STATE_LISTEN;
        p_i2c->previous_state = I2C_STATE_SLAVE_BUSY_TX;

        /* Disable Interrupts */
        I2C_Disable_IRQ(p_i2c, I2C_XFER_TX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the Tx complete callback to inform upper layer of the end of transmit process */
        hal_i2c_slave_tx_cplt_callback(p_i2c);
    }

    else if (HAL_I2C_STATE_BUSY_RX_LISTEN == p_i2c->state)
    {
        /* Remove HAL_I2C_STATE_SLAVE_BUSY_RX, keep only HAL_I2C_STATE_LISTEN */
        p_i2c->state          = HAL_I2C_STATE_LISTEN;
        p_i2c->previous_state = I2C_STATE_SLAVE_BUSY_RX;

        /* Disable Interrupts */
        I2C_Disable_IRQ(p_i2c, I2C_XFER_RX_IT);

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the Rx complete callback to inform upper layer of the end of receive process */
        hal_i2c_slave_rx_cplt_callback(p_i2c);
    }
#endif
}

/**
  * @brief  I2C Master complete process.
  * @param  p_i2c I2C handle.
  * @retval None
  */
static void i2c_it_master_cplt(i2c_handle_t *p_i2c)
{
    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    /* Reset handle parameters */
    p_i2c->xfer_isr       = NULL;
    p_i2c->xfer_options   = I2C_NO_OPTION_FRAME;

    p_i2c->previous_state = I2C_STATE_NONE;

    if ((SET == ll_i2c_is_active_flag_raw_tx_abort(p_i2c->p_instance)) &&
            (RESET != (ll_i2c_get_abort_source(p_i2c->p_instance) & I2C_TX_ABRT_NOACK)))
    {
        /* Clear TX_ABORT Flag */
        ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);

        /* Set acknowledge error code */
        p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
    }

    /* Disable Interrupts */
    ll_i2c_disable_it(p_i2c->p_instance, I2C_MST_XFER_TX_IT | I2C_MST_XFER_RX_IT);

    if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
    {
        /* Data remained in TX FIFO were sent failed, FIFO will be flushed after I2C was disabled. */
        p_i2c->xfer_count += ll_i2c_get_tx_fifo_level(p_i2c->p_instance);
    }
    else
    {
        /* Store Last receive data if any */
        while (ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance) && (0U != p_i2c->xfer_count))
        {
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }
    }

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    if ((HAL_I2C_ERROR_NONE != p_i2c->error_code) || (HAL_I2C_STATE_ABORT == p_i2c->state))
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        i2c_it_error(p_i2c, p_i2c->error_code);
    }
    /* p_i2c->State == HAL_I2C_STATE_BUSY_TX */
    else if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
    {
        p_i2c->state = HAL_I2C_STATE_READY;

        if (HAL_I2C_MODE_MEM == p_i2c->mode)
        {
            p_i2c->mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);

            /* Call the corresponding callback to inform upper layer of End of Transfer */
            hal_i2c_mem_tx_cplt_callback(p_i2c);
        }
        else
        {
            p_i2c->mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);

            /* Call the corresponding callback to inform upper layer of End of Transfer */
            hal_i2c_master_tx_cplt_callback(p_i2c);
        }
    }
    /* p_i2c->State == HAL_I2C_STATE_BUSY_RX */
    else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
    {
        p_i2c->state = HAL_I2C_STATE_READY;

        if (HAL_I2C_MODE_MEM == p_i2c->mode)
        {
            p_i2c->mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);

            hal_i2c_mem_rx_cplt_callback(p_i2c);
        }
        else
        {
            p_i2c->mode = HAL_I2C_MODE_NONE;

            /* Process Unlocked */
            __HAL_UNLOCK(p_i2c);

            hal_i2c_master_rx_cplt_callback(p_i2c);
        }
    }
}

/**
  * @brief  I2C Slave complete process.
  * @param  p_i2c I2C handle.
  * @retval None
  */
static void i2c_it_slave_cplt(i2c_handle_t *p_i2c)
{
    /* Clear STOP_DET Flag */
    ll_i2c_clear_flag_stop_det(p_i2c->p_instance);

    /* Disable all interrupts */
    ll_i2c_disable_it(p_i2c->p_instance, I2C_SLV_XFER_TX_IT | I2C_SLV_XFER_RX_IT | I2C_XFER_LISTEN_IT);

    /* If a DMA is ongoing, Update handle size context */
    if ((SET == ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance)) ||
            (SET == ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance)))
    {
        p_i2c->xfer_count = I2C_GET_DMA_REMAIN_DATA(p_i2c);
    }

    if (HAL_I2C_STATE_BUSY_TX == p_i2c->state)
    {
        /* Data remained in TX FIFO were sent failed, FIFO will be flushed after I2C was disabled. */
        p_i2c->xfer_count += ll_i2c_get_tx_fifo_level(p_i2c->p_instance);
    }
    else
    {
        /* Store Last receive data if any */
        while (ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance) && (0U != p_i2c->xfer_count))
        {
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }
    }

    /* All data are not transferred, so set error code accordingly */
    if (0U != p_i2c->xfer_count)
    {
        /* Set ErrorCode corresponding to a Non-Acknowledge */
        p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
    }

    p_i2c->previous_state    = I2C_STATE_NONE;
    p_i2c->mode              = HAL_I2C_MODE_NONE;
    p_i2c->xfer_isr          = NULL;

    if (HAL_I2C_ERROR_NONE != p_i2c->error_code)
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        i2c_it_error(p_i2c, p_i2c->error_code);

        /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
        if (HAL_I2C_STATE_LISTEN == p_i2c->state)
        {
            /* Call I2C Listen complete process */
            i2c_it_listen_cplt(p_i2c);
        }
    }
    else if (I2C_NO_OPTION_FRAME != p_i2c->xfer_options)
    {
        p_i2c->xfer_options = I2C_NO_OPTION_FRAME;
        p_i2c->state = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
        hal_i2c_listen_cplt_callback(p_i2c);
    }
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    else if (HAL_I2C_STATE_BUSY_RX == p_i2c->state)
    {
        p_i2c->state = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the Slave Rx Complete callback */
        hal_i2c_slave_rx_cplt_callback(p_i2c);
    }
    else
    {
        p_i2c->state = HAL_I2C_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the Slave Tx Complete callback */
        hal_i2c_slave_tx_cplt_callback(p_i2c);
    }
}

/**
  * @brief  I2C Listen complete process.
  * @param  p_i2c I2C handle.
  * @retval None
  */
static void i2c_it_listen_cplt(i2c_handle_t *p_i2c)
{
    /* Reset handle parameters */
    p_i2c->xfer_options      = I2C_NO_OPTION_FRAME;
    p_i2c->previous_state    = I2C_STATE_NONE;
    p_i2c->state             = HAL_I2C_STATE_READY;
    p_i2c->mode              = HAL_I2C_MODE_NONE;
    p_i2c->xfer_isr          = NULL;

    /* Store Last receive data if any */
    while (ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance) && (0U != p_i2c->xfer_count))
    {
        (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
        p_i2c->xfer_count--;
    }

    /* All data are not transferred, so set error code accordingly */
    if (0U != p_i2c->xfer_count)
    {
        /* Set ErrorCode corresponding to a Non-Acknowledge */
        p_i2c->error_code |= HAL_I2C_ERROR_NOACK;
    }

    /* Disable all slave Interrupts*/
    ll_i2c_disable_it(p_i2c->p_instance, I2C_XFER_LISTEN_IT | I2C_SLV_XFER_TX_IT | I2C_SLV_XFER_RX_IT);

    /* Clear TX_ABORT Flag */
    ll_i2c_clear_flag_tx_abort(p_i2c->p_instance);

    /* Process Unlocked */
    __HAL_UNLOCK(p_i2c);

    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
    hal_i2c_listen_cplt_callback(p_i2c);
}

/**
  * @brief  I2C interrupts error process.
  * @param  p_i2c I2C handle.
  * @param  error_code Error code to handle.
  * @retval None
  */
static void i2c_it_error(i2c_handle_t *p_i2c, uint32_t error_code)
{
    /* Reset handle parameters */
    p_i2c->mode          = HAL_I2C_MODE_NONE;
    p_i2c->xfer_options  = I2C_NO_OPTION_FRAME;
    // p_i2c->XferCount     = 0U;

    /* Set new error code */
    p_i2c->error_code |= error_code;

    /* Disable Interrupts */
    if ((HAL_I2C_STATE_LISTEN == p_i2c->state)         ||
        (HAL_I2C_STATE_BUSY_TX_LISTEN == p_i2c->state) ||
        (HAL_I2C_STATE_BUSY_RX_LISTEN == p_i2c->state))
    {
        /* Disable all interrupts, except interrupts related to LISTEN state */
        ll_i2c_disable_it(p_i2c->p_instance, I2C_SLV_XFER_RX_IT | I2C_SLV_XFER_TX_IT);

        /* keep HAL_I2C_STATE_LISTEN if set */
        p_i2c->state = HAL_I2C_STATE_LISTEN;
        p_i2c->previous_state = I2C_STATE_NONE;
        p_i2c->xfer_isr = i2c_slave_isr_it;
    }
    else
    {
        /* Disable all interrupts */
        ll_i2c_disable_it(p_i2c->p_instance, LL_I2C_INTR_MASK_ALL);

        /* If state is an abort treatment on goind, don't change state */
        /* This change will be do later */
        if (HAL_I2C_STATE_ABORT != p_i2c->state)
        {
            /* Set HAL_I2C_STATE_READY */
            p_i2c->state = HAL_I2C_STATE_READY;
            p_i2c->previous_state = I2C_STATE_NONE;
        }
        p_i2c->xfer_isr = NULL;
    }

    /* Abort DMA RX transfer if any */
    if ((SET == ll_i2c_is_enabled_dma_req_rx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmarx))
    {
        /* Set the I2C DMA Abort callback will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        p_i2c->p_dmarx->xfer_abort_callback = i2c_dma_abort;
        /* Master receive need to abort Tx too. */

        /* Abort DMA TX and Rx */
        if ((HAL_I2C_MODE_MASTER == p_i2c->mode) && (NULL != p_i2c->p_dmatx))
        {
            p_i2c->p_dmatx->xfer_abort_callback = NULL;
            hal_dma_abort_it(p_i2c->p_dmatx);
            ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
        }

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        if (HAL_OK != hal_dma_abort_it(p_i2c->p_dmarx))
        {
            ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

            /* Call Directly p_i2c->p_dmarx->XferAbortCallback function in case of error */
            p_i2c->p_dmarx->xfer_abort_callback(p_i2c->p_dmarx);
        }
    }
    /* Abort DMA TX transfer if any */
    else if ((SET == ll_i2c_is_enabled_dma_req_tx(p_i2c->p_instance)) && (NULL != p_i2c->p_dmarx))
    {
        /* Set the I2C DMA Abort callback :
        will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        p_i2c->p_dmatx->xfer_abort_callback = i2c_dma_abort;

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Abort DMA TX */
        if (HAL_OK != hal_dma_abort_it(p_i2c->p_dmatx))
        {
            ll_i2c_disable_dma_req_tx(p_i2c->p_instance);

            /* Call Directly XferAbortCallback function in case of error */
            p_i2c->p_dmatx->xfer_abort_callback(p_i2c->p_dmatx);
        }
    }
    else if (HAL_I2C_STATE_ABORT == p_i2c->state)
    {
        p_i2c->previous_state = I2C_STATE_NONE;
        p_i2c->state = HAL_I2C_STATE_READY;

        /* Store Last receive data if any */
        while (ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance) && (0U != p_i2c->xfer_count))
        {
            (*p_i2c->p_buffer++) = ll_i2c_receive_data8(p_i2c->p_instance);
            p_i2c->xfer_count--;
        }

        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_abort_cplt_callback(p_i2c);
    }
    else
    {
        /* Process Unlocked */
        __HAL_UNLOCK(p_i2c);

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_error_callback(p_i2c);
    }
}

/**
  * @brief  DMA I2C master transmit process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_master_transmit_cplt(dma_handle_t *p_dma)
{
    i2c_handle_t *p_i2c = (i2c_handle_t *)((dma_handle_t *)p_dma)->p_parent;

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);

    /* Transmit the last data with STOP signal */
    if (1 == p_i2c->xfer_count)
    {
        while(RESET == ll_i2c_is_active_flag_status_tfnf(p_i2c->p_instance));
        ll_i2c_transmit_data8(p_i2c->p_instance, *p_i2c->p_buffer, LL_I2C_CMD_MST_WRITE | LL_I2C_CMD_MST_GEN_STOP);
        p_i2c->xfer_count--;
    }

    /* If last transfer, enable STOP interrupt */
    /* Enable STOP interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_CPLT_IT);
}

/**
  * @brief  DMA I2C slave transmit process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_slave_transmit_cplt(dma_handle_t *p_dma)
{
    i2c_handle_t *p_i2c = (i2c_handle_t *)((dma_handle_t *)p_dma)->p_parent;

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);

    /* No specific action, Master fully manage the generation of STOP condition */
    /* Mean that this generation can arrive at any time, at the end or during DMA process */
    /* So STOP condition should be manage through Interrupt treatment */
}

/**
  * @brief DMA I2C master receive process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_master_receive_cplt(dma_handle_t *p_dma)
{
    i2c_handle_t *p_i2c = (i2c_handle_t *)((dma_handle_t *)p_dma)->p_parent;

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
    ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

    /* Current p_i2c->xfer_count should be equal to 1 */
    /* Receive the last data with STOP signal */
    if (1 == p_i2c->xfer_count)
    {
        while(RESET == ll_i2c_is_active_flag_status_tfnf(p_i2c->p_instance));
        ll_i2c_transmit_data8(p_i2c->p_instance, 0, LL_I2C_CMD_MST_READ | LL_I2C_CMD_MST_GEN_STOP);
        while(RESET == ll_i2c_is_active_flag_status_rfne(p_i2c->p_instance));
        *p_i2c->p_buffer++ = ll_i2c_receive_data8(p_i2c->p_instance);
        p_i2c->xfer_count--;
    }

    /* If last transfer, enable STOP interrupt */
    /* Enable STOP interrupt */
    ll_i2c_enable_it(p_i2c->p_instance, I2C_XFER_CPLT_IT);
}

/**
  * @brief  DMA I2C slave receive process complete callback.
  * @param  p_dma DMA handle
  * @retval None
  */
static void i2c_dma_slave_receive_cplt(dma_handle_t *p_dma)
{
    i2c_handle_t *p_i2c = (i2c_handle_t *)((dma_handle_t *)p_dma)->p_parent;

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

    /* No specific action, Master fully manage the generation of STOP condition */
    /* Mean that this generation can arrive at any time, at the end or during DMA process */
    /* So STOP condition should be manage through Interrupt treatment */
}

/**
  * @brief  DMA I2C communication error callback.
  * @param p_dma DMA handle
  * @retval None
  */
static void i2c_dma_error(dma_handle_t *p_dma)
{
    i2c_handle_t *p_i2c = (i2c_handle_t *)((dma_handle_t *)p_dma)->p_parent;

    /* Call the corresponding callback to inform upper layer of End of Transfer */
    i2c_it_error(p_i2c, HAL_I2C_ERROR_DMA);
}

/**
  * @brief DMA I2C communication abort callback
  *        (To be called at end of DMA Abort procedure).
  * @param p_dma DMA handle.
  * @retval None
  */
static void i2c_dma_abort(dma_handle_t *p_dma)
{
    i2c_handle_t *p_i2c = (i2c_handle_t *)((dma_handle_t *)p_dma)->p_parent;

    /* Disable DMA Request */
    ll_i2c_disable_dma_req_tx(p_i2c->p_instance);
    ll_i2c_disable_dma_req_rx(p_i2c->p_instance);

    if (p_i2c->p_dmatx->instance == p_dma->instance)
    {
        p_i2c->xfer_count = I2C_GET_TX_DMA_REMAIN_DATA(p_i2c);
    }
    else
    {
        p_i2c->xfer_count = I2C_GET_RX_DMA_REMAIN_DATA(p_i2c);
    }

    /* Reset AbortCpltCallback */
    p_i2c->p_dmatx->xfer_abort_callback = NULL;
    p_i2c->p_dmarx->xfer_abort_callback = NULL;

    /* Check if come from abort from user */
    if (HAL_I2C_STATE_ABORT == p_i2c->state)
    {
        p_i2c->state = HAL_I2C_STATE_READY;

        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_abort_cplt_callback(p_i2c);
    }
    else
    {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        hal_i2c_error_callback(p_i2c);
    }
}


/** @} */

#endif /* HAL_I2C_MODULE_ENABLED */

/** @} */

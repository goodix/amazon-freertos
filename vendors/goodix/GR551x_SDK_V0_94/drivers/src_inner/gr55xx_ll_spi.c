/**
  ****************************************************************************************
  * @file    gr55xx_ll_spi.c
  * @author  BLE Driver Team
  * @brief   SPI LL module driver.
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
#include "gr55xx_ll_spi.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (SPIM) || defined (SPIS) || defined (QSPI0) || defined (QSPI1)

/**
  * @brief  Set spim registers to their reset values.
  * @param  SPIx SSI instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: invalid spi instance
  */
__WEAK error_status_t ll_spim_deinit(ssi_regs_t *SPIx)
{
    error_status_t status = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    WRITE_REG(SPIx->SSI_EN, 0);
    WRITE_REG(SPIx->CTRL0, LL_SSI_DATASIZE_32BIT);
    WRITE_REG(SPIx->CTRL1, 0);
    WRITE_REG(SPIx->MWC, 0);
    WRITE_REG(SPIx->SE, 0);
    WRITE_REG(SPIx->BAUD, 0);
    WRITE_REG(SPIx->INTMASK, 0);
    WRITE_REG(SPIx->DMAC, 0);

    return status;
}

/**
  * @brief  Set the fields of the spim unit configuration data structure
  *         to their default values.
  * @param  p_spi_init pointer to a @ref ll_spim_init_t structure (spim unit configuration data structure)
  * @retval None
  */
__WEAK void ll_spim_struct_init(ll_spim_init_t *p_spi_init)
{
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    /* Set the default configuration */
    p_spi_init->transfer_direction  = LL_SSI_FULL_DUPLEX;
    p_spi_init->data_size           = LL_SSI_DATASIZE_8BIT;
    p_spi_init->clock_polarity      = LL_SSI_SCPOL_LOW;
    p_spi_init->clock_phase         = LL_SSI_SCPHA_1EDGE;
    p_spi_init->slave_select        = LL_SSI_SLAVE0;
    p_spi_init->baud_rate           = SystemCoreClock / 2000000;
}

/**
  * @brief  Configure the spim unit.
  * @param  SPIx SSI instance
  * @param  p_spi_init pointer to a @ref ll_spim_init_t structure (spim unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_spim_init(ssi_regs_t *SPIx, ll_spim_init_t *p_spi_init)
{
    error_status_t status = ERROR;

    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    if (!ll_spi_is_enabled(SPIx))
    {
        ll_spi_set_baud_rate_prescaler(SPIx, p_spi_init->baud_rate);
        ll_spi_disable_ss_toggle(SPIx);
        ll_spi_enable_ss(SPIx, p_spi_init->slave_select);
        ll_spi_set_data_size(SPIx, p_spi_init->data_size);
        ll_spi_set_clock_polarity(SPIx, p_spi_init->clock_polarity);
        ll_spi_set_clock_phase(SPIx, p_spi_init->clock_phase);
        ll_spi_set_transfer_direction(SPIx, p_spi_init->transfer_direction);
        ll_spi_disable_it(SPIx, LL_SSI_IM_MST | LL_SSI_IM_RXF | LL_SSI_IM_RXO | LL_SSI_IM_RXU | LL_SSI_IM_TXO | LL_SSI_IM_TXE);

        status = SUCCESS;
    }

    return status;
}

/**
  * @brief  Set spis registers to their reset values.
  * @param  SPIx SSI instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: invalid spi instance
  */
__WEAK error_status_t ll_spis_deinit(ssi_regs_t *SPIx)
{
    error_status_t status = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    WRITE_REG(SPIx->SSI_EN, 0);
    WRITE_REG(SPIx->CTRL0, LL_SSI_DATASIZE_32BIT);
    WRITE_REG(SPIx->MWC, 0);
    WRITE_REG(SPIx->INTMASK, 0);
    WRITE_REG(SPIx->DMAC, 0);

    return status;
}

/**
  * @brief  Set the fields of the spis unit configuration data structure
  *         to their default values.
  * @param  p_spi_init pointer to a @ref ll_spis_init_t structure (spis unit configuration data structure)
  * @retval None
  */
__WEAK void ll_spis_struct_init(ll_spis_init_t *p_spi_init)
{
    /* Set the default configuration */
    p_spi_init->data_size           = LL_SSI_DATASIZE_8BIT;
    p_spi_init->clock_polarity      = LL_SSI_SCPOL_LOW;
    p_spi_init->clock_phase         = LL_SSI_SCPHA_1EDGE;
}

/**
  * @brief  Configure the spis unit.
  * @param  SPIx SSI instance
  * @param  p_spi_init pointer to a @ref ll_spis_init_t structure (spis unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_spis_init(ssi_regs_t *SPIx, ll_spis_init_t *p_spi_init)
{
    error_status_t status = ERROR;

    /* Check the parameters */
    gr_assert_param(IS_SPI_ALL_INSTANCE(SPIx));

    if (!ll_spi_is_enabled(SPIx))
    {
        ll_spi_disable_ss_toggle(SPIx);
        ll_spi_enable_slave_out(SPIx);
        ll_spi_set_data_size(SPIx, p_spi_init->data_size);
        ll_spi_set_clock_polarity(SPIx, p_spi_init->clock_polarity);
        ll_spi_set_clock_phase(SPIx, p_spi_init->clock_phase);
        ll_spi_disable_it(SPIx, LL_SSI_IM_RXF | LL_SSI_IM_RXO | LL_SSI_IM_RXU | LL_SSI_IM_TXO | LL_SSI_IM_TXE);

        status = SUCCESS;
    }

    return status;
}

/**
  * @brief  Set qspi registers to their reset values.
  * @param  SPIx SSI instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: invalid spi instance
  */
__WEAK error_status_t ll_qspi_deinit(ssi_regs_t *SPIx)
{
    error_status_t status = SUCCESS;

    /* Check the parameters */
    gr_assert_param(IS_QSPI_ALL_INSTANCE(SPIx));

    WRITE_REG(SPIx->SSI_EN, 0);
    WRITE_REG(SPIx->CTRL0, LL_SSI_DATASIZE_32BIT);
    WRITE_REG(SPIx->CTRL1, 0);
    WRITE_REG(SPIx->MWC, 0);
    WRITE_REG(SPIx->SE, 0);
    WRITE_REG(SPIx->BAUD, 0);
    WRITE_REG(SPIx->INTMASK, 0);
    WRITE_REG(SPIx->DMAC, 0);
    WRITE_REG(SPIx->SPI_CTRL0, 0);

    return status;
}

/**
  * @brief  Set the fields of the qspi unit configuration data structure
  *         to their default values.
  * @param  p_spi_init pointer to a @ref ll_qspi_init_t structure (qspi unit configuration data structure)
  * @retval None
  */
__WEAK void ll_qspi_struct_init(ll_qspi_init_t *p_spi_init)
{
    /* Update SystemCoreClock */
    SystemCoreUpdateClock();
    /* Set the default configuration */
    p_spi_init->transfer_direction      = LL_SSI_FULL_DUPLEX;
    p_spi_init->instruction_size        = LL_SSI_INSTSIZE_8BIT;
    p_spi_init->address_size            = LL_SSI_ADDRSIZE_24BIT;
    p_spi_init->inst_addr_transfer_format = LL_SSI_INST_ADDR_ALL_IN_SPI;
    p_spi_init->wait_cycles             = 0;
    p_spi_init->data_size               = LL_SSI_DATASIZE_8BIT;
    p_spi_init->clock_polarity          = LL_SSI_SCPOL_LOW;
    p_spi_init->clock_phase             = LL_SSI_SCPHA_1EDGE;
    p_spi_init->baud_rate               = SystemCoreClock / 2000000;
}

/**
  * @brief  Configure the qspi unit.
  * @param  SPIx SSI instance
  * @param  p_spi_init pointer to a @ref ll_qspi_init_t structure (qspi unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: spi registers are de-initialized
  *          - ERROR: not applicable
  */
__WEAK error_status_t ll_qspi_init(ssi_regs_t *SPIx, ll_qspi_init_t *p_spi_init)
{
    error_status_t status = ERROR;

    /* Check the parameters */
    gr_assert_param(IS_QSPI_ALL_INSTANCE(SPIx));

    if (!ll_spi_is_enabled(SPIx))
    {
        ll_spi_set_baud_rate_prescaler(SPIx, p_spi_init->baud_rate);
        ll_spi_disable_ss_toggle(SPIx);
        ll_spi_enable_ss(SPIx, LL_SSI_SLAVE0);
        ll_spi_set_address_size(SPIx, p_spi_init->address_size);
        ll_spi_set_instruction_size(SPIx, p_spi_init->instruction_size);
        ll_spi_set_add_inst_transfer_format(SPIx, p_spi_init->inst_addr_transfer_format);
        ll_spi_set_wait_cycles(SPIx, p_spi_init->wait_cycles);
        ll_spi_set_data_size(SPIx, p_spi_init->data_size);
        ll_spi_set_clock_polarity(SPIx, p_spi_init->clock_polarity);
        ll_spi_set_clock_phase(SPIx, p_spi_init->clock_phase);
        ll_spi_set_transfer_direction(SPIx, p_spi_init->transfer_direction);
        ll_spi_disable_it(SPIx, LL_SSI_IM_MST | LL_SSI_IM_RXF | LL_SSI_IM_RXO | LL_SSI_IM_RXU | LL_SSI_IM_TXO | LL_SSI_IM_TXE);

        status = SUCCESS;
    }

    return status;
}

#endif /* SPIM || SPIS || QSPI0 || QSPI1 */


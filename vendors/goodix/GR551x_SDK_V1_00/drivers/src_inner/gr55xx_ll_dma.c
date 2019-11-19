/**
  ****************************************************************************************
  * @file    gr55xx_ll_dma.c
  * @author  BLE Driver Team
  * @brief   DMA LL module driver.
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
#include "gr55xx_ll_dma.h"

#ifdef  USE_FULL_ASSERT
#include "gr_assert.h"
#else
#define gr_assert_param(expr) ((void)0U)
#endif

#if defined (DMA)

#define IS_LL_DMA_DIRECTION(__VALUE__)          (((__VALUE__) == LL_DMA_DIRECTION_MEMORY_TO_MEMORY) || \
                                                 ((__VALUE__) == LL_DMA_DIRECTION_MEMORY_TO_PERIPH) || \
                                                 ((__VALUE__) == LL_DMA_DIRECTION_PERIPH_TO_MEMORY) || \
                                                 ((__VALUE__) == LL_DMA_DIRECTION_PERIPH_TO_PERIPH))

#define IS_LL_DMA_MODE(__VALUE__)               (((__VALUE__) == LL_DMA_MODE_SINGLE_BLOCK) || \
                                                 ((__VALUE__) == LL_DMA_MODE_MULTI_BLOCK_SRC_RELOAD) || \
                                                 ((__VALUE__) == LL_DMA_MODE_MULTI_BLOCK_DST_RELOAD) || \
                                                 ((__VALUE__) == LL_DMA_MODE_MULTI_BLOCK_ALL_RELOAD))

#define IS_LL_DMA_SRCINCMODE(__VALUE__)         (((__VALUE__) == LL_DMA_SRC_INCREMENT) || \
                                                 ((__VALUE__) == LL_DMA_SRC_DECREMENT) || \
                                                 ((__VALUE__) == LL_DMA_SRC_NO_CHANGE))

#define IS_LL_DMA_DSTINCMODE(__VALUE__)         (((__VALUE__) == LL_DMA_DST_INCREMENT) || \
                                                 ((__VALUE__) == LL_DMA_DST_DECREMENT) || \
                                                 ((__VALUE__) == LL_DMA_DST_NO_CHANGE)))

#define IS_LL_DMA_SRCDATAWITCH(__VALUE__)       (((__VALUE__) == LL_DMA_SDATAALIGN_BYTE)      || \
                                                 ((__VALUE__) == LL_DMA_SDATAALIGN_HALFWORD)  || \
                                                 ((__VALUE__) == LL_DMA_SDATAALIGN_WORD))

#define IS_LL_DMA_DSTDATAWITCH(__VALUE__)       (((__VALUE__) == LL_DMA_DDATAALIGN_BYTE)      || \
                                                 ((__VALUE__) == LL_DMA_DDATAALIGN_HALFWORD)  || \
                                                 ((__VALUE__) == LL_DMA_DDATAALIGN_WORD))

#define IS_LL_DMA_BLKSIZE(__VALUE__)             ((__VALUE__)  <= (uint32_t)0x000001FFU)

#define IS_LL_DMA_PERIPHTYPE(__VALUE__)         (((__VALUE__) >= LL_DMA_PERIPH_SPIM_TX)  && \
                                                 ((__VALUE__) <= LL_DMA_PERIPH_MEM))

#define IS_LL_DMA_PRIORITY(__VALUE__)           (((__VALUE__) >= LL_DMA_PRIORITY_0)    && \
                                                 ((__VALUE__) <= LL_DMA_PRIORITY_7))

#define IS_LL_DMA_ALL_CHANNEL_instance(instance, CHANNEL)  ((((instance) == DMA) && \
                                                            (((CHANNEL) == LL_DMA_CHANNEL_0)|| \
                                                            ((CHANNEL) == LL_DMA_CHANNEL_1) || \
                                                            ((CHANNEL) == LL_DMA_CHANNEL_2) || \
                                                            ((CHANNEL) == LL_DMA_CHANNEL_3) || \
                                                            ((CHANNEL) == LL_DMA_CHANNEL_4) || \
                                                            ((CHANNEL) == LL_DMA_CHANNEL_5) || \
                                                            ((CHANNEL) == LL_DMA_CHANNEL_6) || \
                                                            ((CHANNEL) == LL_DMA_CHANNEL_7))))

__WEAK error_status_t ll_dma_deinit(dma_regs_t *DMAx, uint32_t channel)
{
    error_status_t status = SUCCESS;

    /* Check the DMA instance DMAx and Channel parameters*/
    //gr_assert_param(IS_LL_DMA_ALL_CHANNEL_instance(DMAx, Channel) || (Channel == LL_DMA_CHANNEL_ALL));

    if (LL_DMA_CHANNEL_ALL == channel)
    {
        ll_dma_disable(DMAx);
        while(ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_0) || \
              ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_1) || \
              ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_2) || \
              ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_3) || \
              ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_4) || \
              ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_5) || \
              ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_6) || \
              ll_dma_is_enabled_channel(DMAx, LL_DMA_CHANNEL_7) || \
              ll_dma_is_enable(DMAx));
    }
    else
    {
        if (ll_dma_is_enabled_channel(DMAx, channel))
        {
            ll_dma_suspend_channel(DMAx, channel);
            while(!ll_dma_is_empty_fifo(DMAx, channel));
            ll_dma_disable_channel(DMAx, channel);
            while(ll_dma_is_enabled_channel(DMAx, channel));
            /* Resume the channel */
            ll_dma_resume_channel(DMAx, channel);
        }
        /* Mask interrupt bits for DMAx_Channely */
        ll_dma_disable_it_tfr(DMAx, channel);
        ll_dma_disable_it_blk(DMAx, channel);
        ll_dma_disable_it_srct(DMAx, channel);
        ll_dma_disable_it_dstt(DMAx, channel);
        ll_dma_disable_it_err(DMAx, channel);

        /* Reset interrupt pending bits for DMAx_Channely */
        ll_dma_clear_flag_tfr(DMAx, channel);
        ll_dma_clear_flag_blk(DMAx, channel);
        ll_dma_clear_flag_srct(DMAx, channel);
        ll_dma_clear_flag_dstt(DMAx, channel);
        ll_dma_clear_flag_err(DMAx, channel);

        /* Reset DMAx_Channely CFGL register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_LO, 0x00000e00 + (channel * 0x20));

        /* Reset DMAx_Channely CFGH register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_HI, 0x00000004U);

        /* Reset DMAx_Channely CTLL register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_LO, 0U);

        /* Reset DMAx_Channely CTLH register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_HI, 0U);

        /* Reset DMAx_Channely Source address register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], SAR, 0U);

        /* Reset DMAx_Channely Destination address register */
        LL_DMA_WriteReg(DMAx->CHANNEL[channel], DAR, 0U);

        if(!LL_DMA_ReadReg(DMAx->MISCELLANEOU, CH_EN))
        {
            ll_dma_disable(DMAx);
        }
    }
    return status;
}

/**
  * @brief  Initialize the DMA registers according to the specified parameters in DMA_InitStruct.
  * @note   To convert DMAx_Channely instance to DMAx instance and Channely, use helper macros :
  *         @arg @ref __LL_DMA_GET_instance
  *         @arg @ref __LL_DMA_GET_CHANNEL
  * @param  DMAx DMAx instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_DMA_CHANNEL_1
  *         @arg @ref LL_DMA_CHANNEL_2
  *         @arg @ref LL_DMA_CHANNEL_3
  *         @arg @ref LL_DMA_CHANNEL_4
  *         @arg @ref LL_DMA_CHANNEL_5
  *         @arg @ref LL_DMA_CHANNEL_6 (*)
  *         @arg @ref LL_DMA_CHANNEL_7 (*)
  *
  *         (*) value not defined in all devices
  * @param  p_dma_init pointer to a @ref ll_dma_init_t structure.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: DMA registers are initialized
  *          - ERROR: Not applicable
  */
__WEAK error_status_t ll_dma_init(dma_regs_t *DMAx, uint32_t channel, ll_dma_init_t *p_dma_init)
{
    /* Check the DMA instance DMAx and Channel parameters*/
    gr_assert_param(IS_LL_DMA_ALL_CHANNEL_instance(DMAx, channel));

    /* Check the DMA parameters from DMA_InitStruct */
    gr_assert_param(IS_LL_DMA_DIRECTION(p_dma_init->direction));
    gr_assert_param(IS_LL_DMA_MODE(p_dma_init->mode));
    gr_assert_param(IS_LL_DMA_SRCINCMODE(p_dma_init->src_increment_mode));
    gr_assert_param(IS_LL_DMA_DSTINCMODE(p_dma_init->dst_increment_mode));
    gr_assert_param(IS_LL_DMA_SRCDATAWITCH(p_dma_init->src_data_width));
    gr_assert_param(IS_LL_DMA_DSTDATAWITCH(p_dma_init->dst_data_width));
    gr_assert_param(IS_LL_DMA_BLKSIZE(p_dma_init->block_size));
    gr_assert_param(IS_LL_DMA_PERIPHTYPE(p_dma_init->src_peripheral));
    gr_assert_param(IS_LL_DMA_PERIPHTYPE(p_dma_init->dst_peripheral));
    gr_assert_param(IS_LL_DMA_PRIORITY(p_dma_init->priority));

    if (!ll_dma_is_enable(DMAx))
    {
        ll_dma_enable(DMAx);
    }
    else
    {
        /* Disable DMA channel that has been enabled.*/
        if (ll_dma_is_enabled_channel(DMAx, channel))
        {
            ll_dma_suspend_channel(DMAx, channel);
            while(!ll_dma_is_empty_fifo(DMAx, channel));
            ll_dma_disable_channel(DMAx, channel);
            while(ll_dma_is_enabled_channel(DMAx, channel));
            /* Resume the channel */
            ll_dma_resume_channel(DMAx, channel);
        }
    }

    /* Reset DMAx_Channely CFGL register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_LO, 0x00000e00 + (channel * 0x20));

    /* Reset DMAx_Channely CFGH register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CFG_HI, 0x00000004U);

    /* Reset DMAx_Channely CTLL register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_LO, 0U);

    /* Reset DMAx_Channely CTLH register */
    LL_DMA_WriteReg(DMAx->CHANNEL[channel], CTL_HI, 0U);

    /*---------------------------- DMAx CTLL Configuration ------------------------
    * Configure DMAx_Channely: data transfer mode, peripheral and memory increment mode,
    *                          data size alignment and burst length:
    * - Direction:      CTLL_TT_FC bit
    * - SrcIncMode   :  CTLL_SINC bit
    * - DstIncMode   :  CTLL_DINC bit
    * - SrcDataWitch :  CTLL_SRC_TR_WIDTH bits
    * - dst_data_width :  CTLL_DST_TR_WIDTH bits
    */
    ll_dma_config_transfer(DMAx, channel, p_dma_init->direction | \
                           p_dma_init->src_increment_mode       | \
                           p_dma_init->dst_increment_mode       | \
                           p_dma_init->src_data_width           | \
                           p_dma_init->dst_data_width           | \
                           LL_DMA_SRC_BURST_LENGTH_1            | \
                           LL_DMA_DST_BURST_LENGTH_1);

    /*-------------------------- DMAx CFGL Configuration -------------------------
     * Configure Channel handshaking interface with parameters :
     * - src_handshaking: HS_SEL_SRC bits
     * - dst_handshaking: HS_SEL_DST bits
     */
    ll_dma_select_handshaking(DMAx, channel, LL_DMA_SHANDSHAKING_HW, LL_DMA_DHANDSHAKING_HW);

    /*-------------------------- DMAx CFGL Configuration -------------------------
     * Configure Channel operation mode with parameters :
     * - Mode: RELOAD_DST&RELOAD_SRC bits
     */
    ll_dma_set_mode(DMAx, channel, p_dma_init->mode);

    /*-------------------------- DMAx SAR Configuration -------------------------
     * Configure the source base address with parameter :
     * - SrcAddress: SAR[31:0] bits
     */
    ll_dma_set_source_address(DMAx, channel, p_dma_init->src_address);

    /*-------------------------- DMAx DAR Configuration -------------------------
     * Configure the destination base address with parameter :
     * - DstAddress: DAR[31:0] bits
     */
    ll_dma_set_destination_address(DMAx, channel, p_dma_init->dst_address);

    /*--------------------------- DMAx CTLH Configuration -----------------------
     * Configure the number of data to transfer with parameter :
     * - block_size: BLOCK_TS bits
     */
    ll_dma_set_block_size(DMAx, channel, p_dma_init->block_size);

    /*--------------------------- DMAx CFGH Configuration -----------------------
     * Configure the source peripheral with parameter :
     * - src_peripheral: SRC_PER bits
     */
    ll_dma_set_source_peripheral(DMAx, channel, p_dma_init->src_peripheral);

    /*--------------------------- DMAx CFGH Configuration -----------------------
     * Configure the destination peripheral with parameter :
     * - dst_peripheral: DEST_PER bits
     */
    ll_dma_set_destination_peripheral(DMAx, channel, p_dma_init->dst_peripheral);

    return SUCCESS;
}

/**
  * @brief  Set each @ref ll_dma_init_t field to default value.
  * @param  p_dma_init Pointer to a @ref ll_dma_init_t structure.
  * @retval None
  */
__WEAK void ll_dma_struct_init(ll_dma_init_t *p_dma_init)
{
    /* Set DMA_InitStruct fields to default values */
    p_dma_init->src_address        = (uint32_t)0x00000000U;
    p_dma_init->dst_address        = (uint32_t)0x00000000U;
    p_dma_init->direction          = LL_DMA_DIRECTION_MEMORY_TO_MEMORY;
    p_dma_init->mode               = LL_DMA_MODE_SINGLE_BLOCK;
    p_dma_init->src_increment_mode = LL_DMA_SRC_NO_CHANGE;
    p_dma_init->dst_increment_mode = LL_DMA_DST_NO_CHANGE;
    p_dma_init->src_data_width     = LL_DMA_SDATAALIGN_BYTE;
    p_dma_init->dst_data_width     = LL_DMA_DDATAALIGN_BYTE;
    p_dma_init->block_size         = (uint32_t)0x00000000U;
    p_dma_init->src_peripheral     = LL_DMA_PERIPH_MEM;
    p_dma_init->dst_peripheral     = LL_DMA_PERIPH_MEM;
    p_dma_init->priority           = LL_DMA_PRIORITY_0;
    return;
}

#endif


/**
 ****************************************************************************************
 *
 * @file    app_dma.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of DMA app library.
 *
 ****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 ****************************************************************************************
 */

/**
 @addtogroup PERIPHERAL APP DRIVER
 @{
*/

/**
  @addtogroup PERIPHERAL_API_HAL_APP_DMA_DRIVER HAL APP DMA Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_DMA_H_
#define _APP_DMA_H_

#include "gr55xx_hal.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_DMA_MODULE_ENABLED

/** @addtogroup HAL_APP_DMA_STRUCTURES Structures
  * @{
  */

/**
  * @brief DMA id definition
  */
typedef int16_t dma_id_t;

/**
  * @brief DMA event Enumerations definition
  */
typedef enum
{
    APP_DMA_EVT_ERROR,                     /**< The event of error interrupt. */
    APP_DMA_EVT_TFR,                       /**< The event of transfer complete interrupt. */
} app_dma_evt_type_t;

/**
  * @brief DMA parameters structure definition
  */
typedef struct 
{
    dma_channel_t       channel_number;    /**< Specifies the channel of DMA. */
    dma_init_t          init;              /**< DMA communication parameters. */
} app_dma_params_t;

/**
  * @brief DMA event callback definition
  */
typedef void (*app_dma_evt_handler_t)(app_dma_evt_type_t type);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_DMA_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP DMA DRIVER according to the specified parameters
 *         in the app_dma_params_t and app_dma_evt_handler_t.
 *
 * @param[in]  p_params: Pointer to app_dma_params_t parameter which contains the
 *                       configuration information for the specified DMA module.
 * @param[in]  evt_handler: DMA user callback function.
 *
 * @return DMA ID
 ****************************************************************************************
 */
dma_id_t app_dma_init(app_dma_params_t *p_params, app_dma_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP ADC DRIVER peripheral.
 *
 * @param[in]  ins_id: Deinitialize DMA channel for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_dma_deinit(dma_id_t ins_id);

/**
 ****************************************************************************************
 * @brief  Start the DMA Transfer.
 *
 * @param[in]  id: DMA channel id.
 * @param[in]  src_address: The source memory Buffer address
 * @param[in]  dst_address: The destination memory Buffer address
 * @param[in]  data_length: The length of data to be transferred from source to destination, ranging between 0 and 4095.
 ****************************************************************************************
 */
void app_dma_start(dma_id_t id, uint32_t src_address, uint32_t dst_address, uint32_t data_length);

/**
 ****************************************************************************************
 * @brief  Return the DMA hande.
 *
 * @param[in]  id: DMA Channel ID.
 *
 * @return Pointer to the specified ID's DMA handle.
 ****************************************************************************************
 */
dma_handle_t *app_dma_get_handle(dma_id_t id);

/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

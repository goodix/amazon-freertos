/**
 ****************************************************************************************
 *
 * @file    app_i2s.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2S app library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_I2S_DRIVER HAL APP I2S Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_I2S_H_
#define _APP_I2S_H_

#include "gr55xx_hal.h"
#include "app_io.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_I2S_MODULE_ENABLED

/** @addtogroup HAL_APP_I2S_STRUCTURES Structures
  * @{
  */

/**
  * @brief I2S module Enumerations definition
  */
typedef enum
{
    APP_I2S_ID_SLAVE,                /**< I2S slave module.  */
    APP_I2S_ID_MASTER,               /**< I2S master module. */
    APP_I2S_ID_MAX                   /**< Only for check parameter, not used as input parameters. */
} app_i2s_id_t;

/**
  * @brief I2S operating mode Enumerations definition
  */
typedef enum
{
    APP_I2S_TYPE_INTERRUPT,          /**< Interrupt operation mode */
    APP_I2S_TYPE_POLLING,            /**< Polling operation mode   */
    APP_I2S_TYPE_DMA,                /**< DMA operation mode       */
    APP_I2S_TYPE_MAX                 /**< Only for check parameter, not used as input parameters. */
} app_i2s_type_t;

/**
  * @brief I2S IO configuration Structures
  */
typedef struct
{
   app_io_type_t        type;        /**< Specifies the type of I2S IO.                                  */
   app_io_mux_t         mux;         /**< Specifies the Peripheral to be connected to the selected pins. */
   uint32_t             pin;         /**< Specifies the IO pins to be configured.
                                          This parameter can be any value of @ref GR551x_pins.           */
} app_i2s_pin_t;

typedef struct
{
    app_i2s_pin_t       ws;          /**< Set the configuration of I2S WS pin.    */
    app_i2s_pin_t       sdo;         /**< Set the configuration of I2S SDO pin.   */
    app_i2s_pin_t       sdi;         /**< Set the configuration of I2S SDI pin.   */
    app_i2s_pin_t       sclk;        /**< Set the configuration of I2S SCLK pin.  */
} app_i2s_pin_cfg_t;

/**
  * @brief I2S operate mode Enumerations definition
  */
typedef struct
{
    app_i2s_type_t      type;           /**< Specifies the operation mode of I2S. */
    dma_channel_t       tx_dma_channel; /**< Specifies the dma channel of I2S TX. */
    dma_channel_t       rx_dma_channel; /**< Specifies the dma channel of I2S RX. */
} app_i2s_mode_t;

/**
  * @brief I2S parameters structure definition
  */
typedef struct
{
    app_i2s_id_t        id;             /**< specified I2S module ID.                                        */
    app_i2s_pin_cfg_t   pin_cfg;        /**< the pin configuration information for the specified I2S module. */
    app_i2s_mode_t      use_mode;       /**< I2S operate mode.                                               */
    i2s_init_t          init;           /**< I2S communication parameters.                                   */
} app_i2s_params_t;

/** @} */

/** @addtogroup HAL_APP_I2S_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief I2S event Enumerations definition
  */
typedef enum
{
    APP_I2S_EVT_ERROR,                  /**< Error reported by UART peripheral.  */
    APP_I2S_EVT_TX_CPLT,                /**< Requested TX transfer completed.    */
    APP_I2S_EVT_RX_DATA,                /**< Requested RX transfer completed.    */
    APP_I2S_EVT_TX_RX,                  /**< Requested TX/RX transfer completed. */
} app_i2s_evt_type_t;

/**
  * @brief I2S event structure definition
  */
typedef struct
{
    app_i2s_evt_type_t  type; /**< Type of event. */
    union
    {
        uint32_t error_code;            /**< I2S Error code . */    
        uint16_t size;                  /**< I2S transmitted/received counter. */
    } data;
} app_i2s_evt_t;

/**
  * @brief I2S event callback definition
  */
typedef void (*app_i2s_evt_handler_t)(app_i2s_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_I2S_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP I2S DRIVER according to the specified parameters
 *         in the app_i2s_params_t and app_i2s_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_i2s_params_t parameter which contains the
 *                       configuration information for the specified I2S module.
 * @param[in]  evt_handler: I2S user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_i2s_init(app_i2s_params_t *p_params, app_i2s_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP I2S DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_i2s_deinit(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2S module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_receive_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in non-blocking mode with Interrupt
 *
 * @param[in]  id: which I2S module want to receive.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_receive_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2S module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_transmit_sync(app_i2s_id_t id, uint16_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in non-blocking mode with Interrupt
 *
 * @param[in]  id: which I2S module want to transmit.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_transmit_async(app_i2s_id_t id, uint16_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Enable the master I2S clock.
 *
 * @param[in]  id: The I2S master module id.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_enable_clock(app_i2s_id_t id); 

/**
 ****************************************************************************************
 * @brief  Disable the master I2S clock.
 *
 * @param[in]  id: The I2S master module id.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_disable_clock(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Flush the I2S transmitter FIFO.
 *
 * @param[in]  id: which I2S module want to flush.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_flush_tx_fifo(app_i2s_id_t id);

/**
 ****************************************************************************************
 * @brief  Flush the I2S receiver FIFO.
 *
 * @param[in]  id: which I2S module want to flush.
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2s_flush_rx_fifo(app_i2s_id_t id);
/** @} */

#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */

/** @} */


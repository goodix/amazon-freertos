/**
 ****************************************************************************************
 *
 * @file    app_i2c.h
 * @author  BLE Driver Team
 * @brief   Header file containing functions prototypes of I2C app library.
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
  @addtogroup PERIPHERAL_API_HAL_APP_I2C_DRIVER HAL APP I2C Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER Interface.
 */

#ifndef _APP_I2C_H_
#define _APP_I2C_H_

#include "gr55xx_hal.h"
#include "app_io.h"
#include "app_drv_error.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAL_I2C_MODULE_ENABLED

/** @addtogroup HAL_APP_I2C_STRUCTURES Structures
  * @{
  */

/**
  * @brief I2C module Enumerations definition
  */
typedef enum
{
    APP_I2C_ID_0,                   /**< I2C module 0. */
    APP_I2C_ID_1,                   /**< I2C module 1. */
    APP_I2C_ID_MAX                  /**< Only for check parameter, not used as input parameters. */
} app_i2c_id_t;

/**
  * @brief I2C role Enumerations definition
  */
typedef enum
{
    APP_I2C_ROLE_MASTER,           /**< I2C master device. */
    APP_I2C_ROLE_SLAVE,            /**< I2C slave device.  */
    APP_I2C_ROLE_MAX,              /**< Only for check parameter, not used as input parameters. */
} app_i2c_role_t;

/**
  * @brief I2C operating mode Enumerations definition
  */
typedef enum
{
    APP_I2C_TYPE_INTERRUPT,        /**< Interrupt operation mode */
    APP_I2C_TYPE_POLLING,          /**< Polling operation mode   */
    APP_I2C_TYPE_DMA,              /**< DMA operation mode       */
    APP_I2C_TYPE_MAX,              /**< Only for check parameter, not used as input parameters. */
} app_i2c_type_t;

/**
  * @brief I2C IO configuration Structures
  */
typedef struct
{
   app_io_type_t        type;      /**< Specifies the type of SPI IO.                                  */
   app_io_mux_t         mux;       /**< Specifies the Peripheral to be connected to the selected pins. */
   uint32_t             pin;       /**< Specifies the IO pins to be configured.
                                        This parameter can be any value of @ref GR551x_pins.           */
} app_i2c_pin_t;

typedef struct
{
    app_i2c_pin_t       scl;       /**< Set the configuration of I2C SCL pin.   */
    app_i2c_pin_t       sda;       /**< Set the configuration of I2C SDA pin.   */
} app_i2c_pin_cfg_t;

/**
  * @brief I2C operate mode Enumerations definition
  */
typedef struct
{
    app_i2c_type_t      type;            /**< Specifies the operation mode of I2C. */
    dma_channel_t       tx_dma_channel;  /**< Specifies the dma channel of I2C TX. */
    dma_channel_t       rx_dma_channel;  /**< Specifies the dma channel of I2C RX. */
} app_i2c_mode_t;

/**
  * @brief I2C parameters structure definition
  */
typedef struct
{
    app_i2c_id_t        id;              /**< specified I2C module ID.                                        */
    app_i2c_role_t      role;            /**< specified the role of I2C.                                      */
    app_i2c_pin_cfg_t   pin_cfg;         /**< the pin configuration information for the specified I2C module. */
    app_i2c_mode_t      use_mode;        /**< I2C operate mode.                                               */
    i2c_init_t          init;            /**< I2C communication parameters.                                   */
} app_i2c_params_t;

/** @} */

/** @addtogroup HAL_APP_I2C_STRUCTURES Event Structures
  * @{
  */

/**
  * @brief I2C event Enumerations definition
  */
typedef enum
{
    APP_I2C_EVT_ERROR,                   /**< Error reported by UART peripheral.  */
    APP_I2C_EVT_TX_CPLT,                 /**< Requested TX transfer completed.    */
    APP_I2C_EVT_RX_DATA,                 /**< Requested RX transfer completed.    */
} app_i2c_evt_type_t;

/**
  * @brief I2C event structure definition
  */
typedef struct
{
    app_i2c_evt_type_t type; /**< Type of event. */
    union
    {
        uint32_t error_code;             /**< I2C Error code . */      
        uint16_t size;                   /**< I2C transmitted/received counter. */
    } data;
} app_i2c_evt_t;

/**
  * @brief I2C event callback definition
  */
typedef void (*app_i2c_evt_handler_t)(app_i2c_evt_t *p_evt);

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_APP_I2C_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the APP I2C DRIVER according to the specified parameters
 *         in the app_i2c_params_t and app_i2c_evt_handler_t.
 * @note   If interrupt mode is set, you can use blocking mode. Conversely, if blocking mode
 *         is set, you can't use interrupt mode.
 *
 * @param[in]  p_params: Pointer to app_i2c_params_t parameter which contains the
 *                       configuration information for the specified I2C module.
 * @param[in]  evt_handler: I2C user callback function.
 *
 * @return Result of initialization.
 ****************************************************************************************
 */
uint16_t app_i2c_init(app_i2c_params_t *p_params, app_i2c_evt_handler_t evt_handler);

/**
 ****************************************************************************************
 * @brief  De-initialize the APP I2C DRIVER peripheral.
 *
 * @param[in]  id: De-initialize for a specific ID.
 *
 * @return Result of De-initialization.
 ****************************************************************************************
 */
uint16_t app_i2c_deinit(app_i2c_id_t id);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2C module want to receive.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet 
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_receive_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Receive in master or slave mode an amount of data in non-blocking mode with Interrupt/DMA.
 *
 * @param[in]  id: which I2C module want to receive.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet 
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_receive_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in blocking mode.
 *
 * @param[in]  id: which I2C module want to transmit.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet 
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_transmit_sync(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Transmits in master or slave mode an amount of data in non-blocking mode with Interrupt/DMA.
 *
 * @param[in]  id: which I2C module want to transmit.
 * @param[in]  target_address: Target device address: The device 7 bits address value in datasheet 
               must be shifted at right before call interface.
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_transmit_async(app_i2c_id_t id, uint16_t target_address, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Read an amount of data in blocking mode from a specific memory address
 *
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_mem_read_sync(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Read an amount of data in non-blocking mode with Interrupt/DMA from a specific memory address
 *
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_mem_read_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in blocking mode to a specific memory address
 *
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 * @param[in]  timeout: Timeout duration
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_mem_write_sync(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size, uint32_t timeout);

/**
 ****************************************************************************************
 * @brief  Write an amount of data in non-blocking mode with Interrupt/DMA to a specific memory address
 *
 * @param[in]  p_i2c: Pointer to an I2C handle which contains the configuration information for the specified I2C.
 * @param[in]  dev_address: Target device address: The device 7 bits address value in datasheet must be shifted at right before call interface
 * @param[in]  mem_address: Internal memory address
 * @param[in]  mem_addr_size: Size of internal memory address
 * @param[in]  p_data: Pointer to data buffer
 * @param[in]  size: Amount of data to be sent
 *
 * @return Result of operation.
 ****************************************************************************************
 */
uint16_t app_i2c_mem_write_async(app_i2c_id_t id, uint16_t dev_address, uint16_t mem_address, uint16_t mem_addr_size, uint8_t *p_data, uint16_t size);


/** @} */


#endif

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */

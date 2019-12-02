/**
 *****************************************************************************************
 *
 * @file bsp.h
 *
 * @brief Board Support Package API
 *
 *****************************************************************************************
 * @attention
  #####Copyright (c) 2019 GOODIX
  All rights reserved.
 *****************************************************************************************
 */

#ifndef __BSP_H__
#define __BSP_H__

#include "custom_config.h"
#include "boards.h"
#if APP_DRIVER_USE_ENABLE
#include "app_key.h"
#include "app_uart.h"
#else
#include "gr55xx_hal.h"
#endif

/**
 * @defgroup BSP_MAROC Defines
 * @{
 */
#if APP_DRIVER_USE_ENABLE
    #define BSP_KEY_UP_ID      0x00      /**< ID for UP KEY. */
    #define BSP_KEY_DOWN_ID    0x01      /**< ID for DOWN KEY. */
    #define BSP_KEY_LEFT_ID    0x02      /**< ID for LEFT KEY. */
    #define BSP_KEY_RIGHT_ID   0x03      /**< ID for RIGHT KEY. */
    #define BSP_KEY_OK_ID      0x04      /**< ID for OK KEY. */

    #define UART_TX_BUFF_SIZE  0x2000    /**< Size of app uart tx buffer. */
#endif
/** @} */

/**
 * @defgroup BSP_ENUM Enumerations
 * @{
 */
typedef enum
{
    BSP_LED_NUM_0,
    BSP_LED_NUM_1,
} bsp_led_num_t;
/** @} */

/**
 * @defgroup BSP_FUNCTION Functions
 * @{
 */
#if APP_DRIVER_USE_ENABLE
/**
 *****************************************************************************************
 * @brief Initialize boards key.
 *****************************************************************************************
 */
void bsp_key_init(void);

/**
 *****************************************************************************************
 * @brief App key event handler
 *****************************************************************************************
 */
void app_key_evt_handler(uint8_t key_id, app_key_click_type_t key_click_type);
#endif

/**
 *****************************************************************************************
 * @brief Initialize app uart.
 *****************************************************************************************
 */
void bsp_uart_init(void);

/**
 *****************************************************************************************
 * @brief Uart data send.
 *****************************************************************************************
 */
void bsp_uart_send(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief Uart data flush.
 *****************************************************************************************
 */
void bsp_uart_flush(void);

#if APP_DRIVER_USE_ENABLE
/**
 *****************************************************************************************
 * @brief App uart event handler.
 *****************************************************************************************
 */
void app_uart_evt_handler(app_uart_evt_t *p_evt);
#endif

/**
 *****************************************************************************************
 * @brief Initialize boards led.
 *****************************************************************************************
 */
void bsp_led_init(void);

/**
 *****************************************************************************************
 * @brief Open boards led.
 *
 * @param[in] led_num: Number of led needed open.
 *****************************************************************************************
 */
void bsp_led_open(bsp_led_num_t led_num);

/**
 *****************************************************************************************
 * @brief Close boards led.
 *
 * @param[in] led_num: Number of led needed close.
 *****************************************************************************************
 */
void bsp_led_close(bsp_led_num_t led_num);
#endif


#ifndef __ST7735X_CONFIG_H__
#define __ST7735X_CONFIG_H__
#include "gr55xx_hal.h"
#include "boards.h"

#define CS_HIGH	   ll_gpio_set_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_SPIM_CS0_PIN)       /**< set cs pin to high. */
#define CS_LOW	   ll_gpio_reset_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_SPIM_CS0_PIN)      /**< set cs pin to low. */
#define SEND_CMD   ll_gpio_reset_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_CMD_AND_DATA_PIN)  /**< set cmd pin to high. */
#define SEND_DATA  ll_gpio_set_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_CMD_AND_DATA_PIN)    /**< set cmd pin to low. */

#ifdef DISPLAY_DRIVER_TYPE_SW_IO

#define SCK_HIGH   ll_gpio_set_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_SPIM_CLK_PIN)        /**< set sck pin to high. */
#define SCK_LOW    ll_gpio_reset_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_SPIM_CLK_PIN)      /**< set sck pin to low. */
#define SDA_HIGH   ll_gpio_set_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_SPIM_MOSI_PIN)       /**< set sda pin to high. */
#define SDA_LOW    ll_gpio_reset_output_pin(DISPLAY_SPIM_GPIO_PORT, DISPLAY_SPIM_MOSI_PIN)     /**< set sda pin to low. */

#endif

/**
 * @defgroup st7735_CONFIG_FUNCTION Functions
 * @{
 */
/**
 *****************************************************************************************
 * @brief st7735 init(config gpio spi).
 *****************************************************************************************
 */
void st7735_init(void);

/**
 *****************************************************************************************
 * @brief Write cmd to st7735.
 *
 * @param[in] cmd:  Cmd to write.
 *****************************************************************************************
 */
void st7735_write_cmd(uint8_t cmd);

/**
 *****************************************************************************************
 * @brief Write one data to st7735.
 *
 * @param[in] data:  Data to write.
 *****************************************************************************************
 */
void st7735_write_data(uint8_t data);

/**
 *****************************************************************************************
 * @brief Write data buffer to st7735.
 *
 * @param[in] p_data: The pointer of the data.
 * @param[in] length: The length of write data.
 *****************************************************************************************
 */
void st7735_write_buffer(uint8_t *p_data, uint16_t length);

/**
 *****************************************************************************************
 * @brief st7735 delay function. 
 *
 * @param[in] time:  Delay time.
 *****************************************************************************************
 */
void st7735_delay(uint16_t time);



#endif


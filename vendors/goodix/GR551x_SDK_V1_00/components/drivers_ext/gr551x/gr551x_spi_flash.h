/**
  ****************************************************************************************
  * @file    gr551x_spi_flash.h
  * @author  BLE Driver Team
  * @brief   Header file containing functions prototypes of spi flash library.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.
  ****************************************************************************************
  */

#ifndef __GR551X_SPI_FLASH_H__
#define __GR551X_SPI_FLASH_H__

#include <stdbool.h>
#include "gr55xx_hal.h"
#include "app_io.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Flash operation instruction macro definition
  * @{
  */

#define SPI_FLASH_CMD_WRSR              0x01
#define SPI_FLASH_CMD_WRSR1             0x31
#define SPI_FLASH_CMD_RDSR              0x05

#define SPI_FLASH_CMD_WREN              0x06
#define SPI_FLASH_CMD_WRDI              0x04

#define SPI_FLASH_CMD_READ              0x03
#define SPI_FLASH_CMD_FREAD             0x0B
#define SPI_FLASH_CMD_DOFR              0x3B
#define SPI_FLASH_CMD_DIOFR             0xBB
#define SPI_FLASH_CMD_QOFR              0x6B
#define SPI_FLASH_CMD_QIOFR             0xEB
#define SPI_FLASH_CMD_READ_RESET        0xFF

#define SPI_FLASH_CMD_PP                0x02
#define SPI_FLASH_CMD_SE                0x20
#define SPI_FLASH_CMD_BE_32             0x52
#define SPI_FLASH_CMD_BE_64             0xD8
#define SPI_FLASH_CMD_CE                0xC7
#define SPI_FLASH_CMD_PES               0x75
#define SPI_FLASH_CMD_PER               0x7A

#define SPI_FLASH_CMD_RDI               0xAB
#define SPI_FLASH_CMD_REMS              0x90
#define SPI_FLASH_CMD_RDID              0x9F

#define SPI_FLASH_CMD_RSTEN             0x66
#define SPI_FLASH_CMD_RST               0x99
#define SPI_FLASH_CMD_DP                0xB9
#define SPI_FLASH_CMD_RDP               0xAB

#define SPI_FLASH_CMD_SFUD              0x5A

#define DUMMY_BYTE                      0xFF

#define SPI_FLASH_PAGE_SIZE             0x000100
#define SPI_FLASH_SECTOR_SIZE           0x001000
#define SPI_FLASH_BLOCK_SIZE            0x010000
#define SPI_FLASH_ADDRESS_MAX           0x0FFFFF

#define SPI_FLASH_TYE_GD25              0xC8
#define SPI_FLASH_TYE_PY25              0x85
#define SPI_FLASH_TYE_MX25              0xC2
#define SPI_FLASH_TYE_SST26             0xBF

/** @} */

/** @addtogroup Flash hardware interface default parameters
  * @{
  */

#define DEFAULT_SPIM_GROUP0   {{APP_IO_TYPE_NORMAL, APP_IO_PIN_6, APP_IO_MUX_7}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_3, APP_IO_MUX_2},\
                               {APP_IO_TYPE_NORMAL, APP_IO_PIN_4, APP_IO_MUX_2}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_5, APP_IO_MUX_2}}
#define DEFAULT_SPIM_GROUP1   {{APP_IO_TYPE_NORMAL, APP_IO_PIN_4, APP_IO_MUX_7}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_7, APP_IO_MUX_4},\
                               {APP_IO_TYPE_NORMAL, APP_IO_PIN_6, APP_IO_MUX_4}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_5, APP_IO_MUX_4}}
#define DEFAULT_SPIM_GROUP2   {{APP_IO_TYPE_NORMAL, APP_IO_PIN_15, APP_IO_MUX_7}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_12, APP_IO_MUX_1},\
                               {APP_IO_TYPE_NORMAL, APP_IO_PIN_13, APP_IO_MUX_1}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_14, APP_IO_MUX_1}}
#define DEFAULT_SPIM_GROUP3   {{APP_IO_TYPE_NORMAL, APP_IO_PIN_17, APP_IO_MUX_7}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_24, APP_IO_MUX_0},\
                               {APP_IO_TYPE_NORMAL, APP_IO_PIN_25, APP_IO_MUX_0}, {APP_IO_TYPE_NORMAL, APP_IO_PIN_16, APP_IO_MUX_0}}

/** @} */

/**
  * @addtogroup Spi Flash IO configuration Structures
  * @{
  */
typedef struct _spi_io
{
    app_io_type_t gpio;
    uint32_t      pin;
    app_io_mux_t  mux;
} spi_io_t;

typedef struct _spi_flash_io
{
    spi_io_t spi_cs;
    spi_io_t spi_clk;
    spi_io_t spi_mosi;
    spi_io_t spi_miso;
} spi_flash_io_t;

/** @} */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup HAL_SPI_FLASH_DRIVER_FUNCTIONS Functions
  * @{
  */
/**
 ****************************************************************************************
 * @brief  Initialize the SPI FLASH DRIVER according to the specified parameters
 *         in the spi_flash_io_t.
 *
 * @param[in]  p_params: Pointer to spi_flash_io_t parameter.
 *
 ****************************************************************************************
 */
void spi_flash_init(spi_flash_io_t *p_spi_flash);

/**
 *******************************************************************************
 * @brief Read flash Memory.
 *
 * @param[in]       address: start address in flash to read data.
 * @param[in,out]   buffer: buffer to read data to.
 * @param[in]       nbytes: number of bytes to read.
 *
 * @return          number of bytes read
 *******************************************************************************
 */
uint32_t spi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes);

/**
 *******************************************************************************
 * @brief Write flash Memory.
 *
 * @param[in]       address: start address in flash to write data to.
 * @param[in,out]   buffer: buffer of data to write.
 * @param[in]       nbytes: number of bytes to write.
 *
 * @return          number of bytes written
 *******************************************************************************
 */
uint32_t spi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes);

/**
 *******************************************************************************
 * @brief Erase flash region.
 *
 * @note All sectors that have address in range of [addr, addr+len]
 *       will be erased. If addr is not sector aligned, preceding data
 *       on the sector that addr belongs to will also be erased.
 *       If (addr + size) is not sector aligned, the whole sector
 *       will also be erased.
 *
 * @param[in] address: start address in flash to write data to.
 * @param[in] size: number of bytes to write.
 *
 * @retval true: If successful.
 * @retval false: If failure.
 *******************************************************************************
 */
bool spi_flash_sector_erase(uint32_t address, uint32_t size);

/**
 *******************************************************************************
 * @brief Erase flash chip.
 *
 * @retval true: If successful.
 * @retval false: If failure.
 *******************************************************************************
 */
bool spi_flash_chip_erase(void);

/**
 *******************************************************************************
 * @brief Reset flash chip.
 *
 *******************************************************************************
 */
void spi_flash_chip_reset(void);

/**
 *******************************************************************************
 * @brief Get flash chip id.
 *
 * @retval Flash chip id.
 *******************************************************************************
 */
uint32_t spi_flash_device_id(void);

/**
 *******************************************************************************
 * @brief Get Flash information.
 *
 * @param[in,out] id: Pointer to flash id.
 * @param[in,out] size: Pointer to flash size.
 *
 *******************************************************************************
 */
void spi_flash_device_info(uint32_t *id, uint32_t *size);
/** @} */

#ifdef __cplusplus
}
#endif

#endif // __GR551X_SPI_FLASH_H__

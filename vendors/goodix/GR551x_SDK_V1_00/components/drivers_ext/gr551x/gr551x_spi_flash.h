#ifndef __GR551X_SPI_FLASH_H__
#define __GR551X_SPI_FLASH_H__

#include <stdbool.h>
#include "gr55xx_hal.h"

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

#define DEFAULT_SPIM_GROUP0   {{GPIO0, GPIO_PIN_6, GPIO_MUX_7}, {GPIO0, GPIO_PIN_3, GPIO_MUX_2}, {GPIO0, GPIO_PIN_4, GPIO_MUX_2}, {GPIO0, GPIO_PIN_5, GPIO_MUX_2}}
#define DEFAULT_SPIM_GROUP1   {{GPIO0, GPIO_PIN_4, GPIO_MUX_7}, {GPIO0, GPIO_PIN_7, GPIO_MUX_4}, {GPIO0, GPIO_PIN_6, GPIO_MUX_4}, {GPIO0, GPIO_PIN_5, GPIO_MUX_4}}
#define DEFAULT_SPIM_GROUP2   {{GPIO0, GPIO_PIN_15, GPIO_MUX_7}, {GPIO0, GPIO_PIN_12, GPIO_MUX_1}, {GPIO0, GPIO_PIN_13, GPIO_MUX_1}, {GPIO0, GPIO_PIN_14, GPIO_MUX_1}}
#define DEFAULT_SPIM_GROUP3   {{GPIO1, GPIO_PIN_1, GPIO_MUX_7}, {GPIO1, GPIO_PIN_8, GPIO_MUX_0}, {GPIO1, GPIO_PIN_9, GPIO_MUX_0}, {GPIO1, GPIO_PIN_0, GPIO_MUX_0}}

typedef struct _spi_io
{
    gpio_regs_t *gpio;
    uint32_t pin;
    uint32_t mux;
} spi_io_t;

typedef struct _spi_flash_io
{
    spi_io_t spi_cs;
    spi_io_t spi_clk;
    spi_io_t spi_mosi;
    spi_io_t spi_miso;
} spi_flash_io_t;

void spi_flash_init(spi_flash_io_t *p_spi_flash);
uint32_t spi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes);
uint32_t spi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes);
bool spi_flash_sector_erase(uint32_t address, uint32_t size);
bool spi_flash_chip_erase(void);
void spi_flash_chip_reset(void);
uint32_t spi_flash_device_id(void);
void spi_flash_device_info(uint32_t *id, uint32_t *size);

#endif // __GR551X_SPI_FLASH_H__

#include <string.h>
#include "gr55xx_hal.h"
#include "gr551x_spi_flash.h"

#define SPIM_SPEED_1M                     (1000000)
#define SPIM_FLASH_CS_LOW()               hal_gpio_write_pin(g_spim_flash_io.spi_cs.gpio, g_spim_flash_io.spi_cs.pin, GPIO_PIN_RESET)
#define SPIM_FLASH_CS_HIGH()              hal_gpio_write_pin(g_spim_flash_io.spi_cs.gpio, g_spim_flash_io.spi_cs.pin, GPIO_PIN_SET)

spi_handle_t   g_spim_handle;
spi_flash_io_t g_spim_flash_io;

void spi_flash_init(spi_flash_io_t *p_spi_flash)
{
    hal_status_t status = HAL_OK;
    gpio_init_t gpio_config = GPIO_DEFAULT_CONFIG;

    g_spim_handle.p_instance              = SPIM;
    g_spim_handle.init.data_size          = SPI_DATASIZE_8BIT;
    g_spim_handle.init.clock_polarity     = SPI_POLARITY_LOW;
    g_spim_handle.init.clock_phase        = SPI_PHASE_1EDGE;
    g_spim_handle.init.baudrate_prescaler = SystemCoreClock / SPIM_SPEED_1M;
    g_spim_handle.init.ti_mode            = SPI_TIMODE_DISABLE;
    g_spim_handle.init.slave_select       = SPI_SLAVE_SELECT_0;
    hal_spi_deinit(&g_spim_handle);
    
    memcpy(&g_spim_flash_io, p_spi_flash, sizeof(spi_flash_io_t));
    
    /* Configurate the cs pin of spi */
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pin  = g_spim_flash_io.spi_cs.pin;
    gpio_config.mux  = g_spim_flash_io.spi_cs.mux;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(g_spim_flash_io.spi_cs.gpio, &gpio_config);
    SPIM_FLASH_CS_HIGH();
    
    /* Configurate the clk pin of spi */
    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = g_spim_flash_io.spi_clk.pin;
    gpio_config.mux  = g_spim_flash_io.spi_clk.mux;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(g_spim_flash_io.spi_clk.gpio, &gpio_config);
    
    /* Configurate the mosi pin of spi */
    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = g_spim_flash_io.spi_mosi.pin;
    gpio_config.mux  = g_spim_flash_io.spi_mosi.mux;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(g_spim_flash_io.spi_mosi.gpio, &gpio_config);
    
    /* Configurate the miso pin of spi */
    gpio_config.mode = GPIO_MODE_MUX;
    gpio_config.pin  = g_spim_flash_io.spi_miso.pin;
    gpio_config.mux  = g_spim_flash_io.spi_miso.mux;
    gpio_config.pull = GPIO_PULLUP;
    hal_gpio_init(g_spim_flash_io.spi_miso.gpio, &gpio_config);

    
    status = hal_spi_init(&g_spim_handle);
    if (status != HAL_OK)
    {
        return;
    }
    return;
}

static void spi_flash_write_enable(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_WREN};
 
    SPIM_FLASH_CS_LOW();
    hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    SPIM_FLASH_CS_HIGH();
    return;
}

static uint32_t spi_flash_read_status(void)
{
    uint32_t ret = 0;
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RDSR};

    SPIM_FLASH_CS_LOW();
    hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    hal_spi_receive(&g_spim_handle, (uint8_t*)&ret, 1, 5000);
    SPIM_FLASH_CS_HIGH();
    
    return ret;
}

uint32_t spi_flash_write(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    uint32_t page_ofs, write_size, write_cont = nbytes;
    uint8_t control_frame[4] = {0};
    control_frame[0]         = SPI_FLASH_CMD_PP;
    hal_status_t status      = HAL_OK;
    
    while (write_cont)
    {
        page_ofs = address & 0xFF;
        write_size = EXFLASH_SIZE_PAGE_BYTES - page_ofs;

        if (write_cont < write_size)
        {
            write_size = write_cont;
            write_cont = 0;
        }
        else
        {
            write_cont -= write_size;
        }

        control_frame[1] = (address >> 16) & 0xFF;
        control_frame[2] = (address >> 8) & 0xFF;
        control_frame[3] = address & 0xFF;

        spi_flash_write_enable();

        SPIM_FLASH_CS_LOW();
        status = hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
        if (status != HAL_OK)
        {
            SPIM_FLASH_CS_HIGH();
            break;
        }
        status = hal_spi_transmit(&g_spim_handle, buffer, write_size, 5000);
        if (status != HAL_OK)
        {
            SPIM_FLASH_CS_HIGH();
            break;
        }
        SPIM_FLASH_CS_HIGH();

        while(spi_flash_read_status() & 0x1);
        
        address += write_size;
        buffer += write_size;
    }

    return ((status == HAL_OK) ? nbytes : 0);
}

uint32_t spi_flash_read(uint32_t address, uint8_t *buffer, uint32_t nbytes)
{
    hal_status_t status = HAL_OK;
    uint8_t control_frame[4] = {0};

    control_frame[0] = SPI_FLASH_CMD_READ;
    control_frame[1] = (address >> 16) & 0xFF;
    control_frame[2] = (address >> 8) & 0xFF;
    control_frame[3] = address & 0xFF;

    SPIM_FLASH_CS_LOW();
    status = hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    if (status != HAL_OK)
    {
        SPIM_FLASH_CS_HIGH();
        return 0;
    }
    
    while(nbytes > SPI_RX_FIFO_LEVEL_MAX)
    {
        status = hal_spi_receive(&g_spim_handle, buffer, SPI_RX_FIFO_LEVEL_MAX, 5000);
        if (status != HAL_OK)
        {
            SPIM_FLASH_CS_HIGH();
            return 0;
        }
        nbytes -= SPI_RX_FIFO_LEVEL_MAX;
        buffer += SPI_RX_FIFO_LEVEL_MAX;
    }
    status = hal_spi_receive(&g_spim_handle, buffer, nbytes, 5000);
    if (status != HAL_OK)
    {
        SPIM_FLASH_CS_HIGH();
        return 0;
    }
    SPIM_FLASH_CS_HIGH();

    return nbytes;
}

bool spi_flash_sector_erase(uint32_t address, uint32_t size)
{
    uint8_t control_frame[4];
    hal_status_t status = HAL_OK;
    control_frame[0] = SPI_FLASH_CMD_SE;

    uint32_t erase_addr = address;
    uint32_t sector_ofs, erase_size, erase_cont = size;

    while (erase_cont)
    {
        sector_ofs = erase_addr & 0xFFF;
        erase_size = EXFLASH_SIZE_SECTOR_BYTES - sector_ofs;
    
        if (erase_cont < erase_size)
        {
            erase_size = erase_cont;
            erase_cont = 0;
        }
        else
        {
            erase_cont -= erase_size;
        }

        control_frame[1] = (erase_addr >> 16) & 0xFF;
        control_frame[2] = (erase_addr >> 8) & 0xFF;
        control_frame[3] = erase_addr & 0xFF;

        spi_flash_write_enable();
    
        SPIM_FLASH_CS_LOW();
        status = hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
        SPIM_FLASH_CS_HIGH();
        if (status != HAL_OK)
        {
            break;
        }

        while(spi_flash_read_status() & 0x1);
        
        erase_addr += erase_size;
    }

    return ((status == HAL_OK) ? true : false);
}

bool spi_flash_chip_erase(void)
{
    hal_status_t status = HAL_OK;
    uint8_t control_frame[1] = {SPI_FLASH_CMD_CE};

    spi_flash_write_enable();
    
    SPIM_FLASH_CS_LOW();
    status = hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    SPIM_FLASH_CS_HIGH();

    while(spi_flash_read_status() & 0x1);

    return ((status == HAL_OK) ? true : false);
}

void spi_flash_chip_reset(void)
{
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RSTEN};

    SPIM_FLASH_CS_LOW();
    hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    SPIM_FLASH_CS_HIGH();

    control_frame[0] = SPI_FLASH_CMD_RST;
    SPIM_FLASH_CS_LOW();
    hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    SPIM_FLASH_CS_HIGH();
}

uint32_t spi_flash_device_id(void)
{
    uint8_t data[3] = {0};
    uint8_t control_frame[1] = {SPI_FLASH_CMD_RDID};
    
    SPIM_FLASH_CS_LOW();
    hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    hal_spi_receive(&g_spim_handle, data, sizeof(data), 5000);
    SPIM_FLASH_CS_HIGH();

    return (((uint32_t)data[0] << 16) + ((uint32_t)data[1] << 8) + data[2]);
}

static uint32_t spi_flash_device_size(void)
{
    uint32_t flash_size = 0;
    uint8_t control_frame[5] = {0};
    uint8_t data[4] = {0};

    control_frame[0] = SPI_FLASH_CMD_SFUD;
    control_frame[3] = 0x34;
    control_frame[4] = DUMMY_BYTE;

    SPIM_FLASH_CS_LOW();
    hal_spi_transmit(&g_spim_handle, control_frame, sizeof(control_frame), 5000);
    hal_spi_receive(&g_spim_handle, data, sizeof(data), 5000);
    SPIM_FLASH_CS_HIGH();

    if (data[0] != 0 && data[3] < 0xFF)
    {
        flash_size = ((data[3] << 24) + (data[2] << 16) + (data[1] << 8) + (data[0] << 0) + 1) / 8;
    }

    return flash_size;
}

void spi_flash_device_info(uint32_t *id, uint32_t *size)
{
    if (NULL == id || NULL == size)
    {
        return;
    }

    *id   = spi_flash_device_id();
    *size = spi_flash_device_size();
}

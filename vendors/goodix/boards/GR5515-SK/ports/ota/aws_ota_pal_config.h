#ifndef __AWS_OTA_PAL_CONFIG_H__
#define __AWS_OTA_PAL_CONFIG_H__

#include "custom_config.h"

/**************************************************************
 *  Flash Layout For OTA
 *
 *  Flash ROM Size:             1MB [0x0100 0000, 0x0110 0000)
 *  Flash Start Addr:           0x0100 0000
 *
 *  IMAGE Boot  Area1:          [0x0100 0000, 0x0100 2000) 
 *  IMAGE Bank1 Start Addr:     0x0100 2000
 *  IMAGE Bank1 End   Addr:     0x0107 FFFF
 *  IMAGE Bank1 End   Size:     504KB
 *
 *  IMAGE Boot  Area2:          [0x0108 0000, 0x0108 2000)
 *  IMAGE Bank2 Start Addr:     0x0108 2000
 *  IMAGE Bank2 End   Addr:     0x010F BFFF
 *  IMAGE Bank2 End   Size:     488KB
 *
 *  NVDS  Start Addr:           0x010F C000
 *  NVDS  Area:                 [0x010F C000, 0x0110 0000)
 *
 *  (Because of NVDS Area, Image Bank2's Size is less than Image Bank1)
 *
 *  Always use IMAGE_FLASH_BOOT_1_START as boot area
 *  and use IMAGE_FLASH_BOOT_2_START as OTA status record
 **************************************************************/

#define IMAGE_FLASH_BOOT_1_START            0x01000000
#define IMAGE_FLASH_BANK_1_START            0x01002000
#define IMAGE_FLASH_BANK_1_END              0x01080000
#define IMAGE_FLASH_BANK_1_SIZE             (IMAGE_FLASH_BANK_1_END - IMAGE_FLASH_BANK_1_START)

#define IMAGE_FLASH_BOOT_2_START            0x01080000
#define IMAGE_FLASH_BANK_2_START            0x01082000
#define IMAGE_FLASH_BANK_2_END              0x010FC000
#define IMAGE_FLASH_BANK_2_SIZE             (IMAGE_FLASH_BANK_2_END - IMAGE_FLASH_BANK_2_START)

#define IMAGE_FLASH_BLOCK_SIZE              0x1000

#define GR_XIP_READ_MODE                    0xEB
#define GR_IMAGE_PATTERN_GD                 0x4744      //magic number
#define GR_IMAGE_VERSION                    VERSION

/*****************************************************************************************
 * GR_IMAGE_BANK_1 is used to run app
 * GR_IMAGE_BANK_2 is used to save ota image
 * GR_IMAGE_BANK_INVALID means some error params exists, cannot run ota
 *****************************************************************************************/
typedef enum {
    GR_IMAGE_BANK_INVALID = 0,      //invalid, some error params exists
    GR_IMAGE_BANK_1,
    GR_IMAGE_BANK_2,
} gr_bank_order_e;

typedef enum
{
    otapalIMAGE_FLAG_NEW = 0xFF,            /* If the application image is running for the first time and never executed before. */
    otapalIMAGE_FLAG_COMMIT_PENDING = 0xFE, /* The application image is marked to execute for test boot. */
    otapalIMAGE_FLAG_VALID = 0xFC,          /*The application image is marked valid and committed. */
    otapalIMAGE_FLAG_INVALID = 0xF8         /*The application image is marked invalid. */
} gr_image_flag_e;

typedef struct{
    uint32_t        bin_size;
    uint32_t        check_sum;
    uint32_t        load_addr;
    uint32_t        run_addr ;
    uint32_t        xqspi_xip_cmd;
    union {
        uint32_t    i_val;
        struct {
            uint32_t    xqspi_speed:4;           /*!< bit: 0..3  clock speed */
            uint32_t    code_copy_mode:1;        /*!< bit: 4 code copy mode */
            uint32_t    system_clk:3;            /*!< bit: 5..7 system clock */
            uint32_t    check_image:1;           /*!< bit: 8 check image */
            uint32_t    boot_delay:1;            /*!< bit: 9 boot delay time */
            uint32_t    reserved:22;             /*!< bit: 22 reserved */
        } b_val;
    } boot_config;
} gr_boot_info_t;


typedef struct {
    uint16_t        img_pattern_gd;
    uint16_t        img_version;
    gr_boot_info_t  img_boot_info;
    uint8_t         img_comments[12];
    uint8_t         img_flag;
    uint8_t         img_ota_status;
    uint8_t         img_reserved[6];
} gr_image_info_t;

typedef enum {
    GR_OTA_STATUS_UNSET = 0,
    GR_OTA_STATUS_READY,
    GR_OTA_STATUS_SET,
} gr_ota_status_e;

void gr_ota_startup_check_and_update(void);

void gr_ota_test(void);

void gr_jump2iap(void);

#endif /*__AWS_OTA_PAL_CONFIG_H__*/

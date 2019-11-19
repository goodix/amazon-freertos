#include "gr55xx_sys.h"
#include "gr55xx_hal_xqspi.h"
#include "gr55xx_hal_exflash.h"
#include "aws_ota_pal_config.h"
#include "gr55xx_dfu.h"
#include "gr55xx_hal_pwr.h"

/**************************************************************************************************************
 * This file's code will be loaded in ram, when reset the ota image, it will over-write the low-half area flash
 * by using the ota image where saved in high-half area.
 **************************************************************************************************************/

extern exflash_handle_t     g_exflash_handle;
static uint8_t              s_block_data[IMAGE_FLASH_BLOCK_SIZE];

static void gr_ota_update_boot_info(gr_image_info_t * image_info){
    boot_info_t   boot_info ;
    uint8_t * ptr = NULL;
    uint32_t i = 0;
    
    ptr = (uint8_t*)&boot_info;
    for(i=0; i < sizeof(boot_info_t); i++){
        *ptr++ = *((uint8_t*) (IMAGE_FLASH_BOOT_1_START + i));
    }
    
    boot_info.bin_size       = image_info->img_boot_info.bin_size;
    boot_info.check_sum      = image_info->img_boot_info.check_sum;
    boot_info.load_addr      = IMAGE_FLASH_BANK_1_START;//image_info->img_boot_info.load_addr;
    boot_info.run_addr       = IMAGE_FLASH_BANK_1_START;//image_info->img_boot_info.run_addr;
    boot_info.xqspi_xip_cmd  = GR_XIP_READ_MODE;     
        
    do
    {
        for(i=0;i<IMAGE_FLASH_BLOCK_SIZE;i++){
            s_block_data[i] = 0;
        }
        
        hal_exflash_read(&g_exflash_handle, IMAGE_FLASH_BOOT_1_START, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);
        //update start loader
        ptr = (uint8_t*)&boot_info;
        for(i=0; i < sizeof(boot_info_t); i++){
            s_block_data[i] = *ptr++;
        }
        
        hal_exflash_erase(&g_exflash_handle, EXFLASH_ERASE_SECTOR, IMAGE_FLASH_BOOT_1_START, IMAGE_FLASH_BLOCK_SIZE);        
        hal_exflash_write(&g_exflash_handle, IMAGE_FLASH_BOOT_1_START, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);
    } while(0);
    
    return;
}

static void gr_ota_update_image_status(void){
    gr_image_info_t image_info;
    uint32_t i = 0;
    uint8_t * ptr = NULL;
    
    ptr = (uint8_t*)&image_info;
    
    for(i = 0; i < sizeof(gr_image_info_t); i++){
        *ptr++ = *((uint8_t*) (IMAGE_FLASH_BOOT_2_START + i));
    }
    
    image_info.img_ota_status = GR_OTA_STATUS_SET;

    do
    {
        for(i=0;i<IMAGE_FLASH_BLOCK_SIZE;i++) {
            s_block_data[i] = 0;
        }
        
        hal_exflash_read(&g_exflash_handle, IMAGE_FLASH_BOOT_2_START, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);
        //update start loader
        ptr = (uint8_t*)&image_info;
        for(i=0; i < sizeof(gr_image_info_t); i++){
            s_block_data[i] = *ptr++;
        }
        
        hal_exflash_erase(&g_exflash_handle, EXFLASH_ERASE_SECTOR, IMAGE_FLASH_BOOT_2_START, IMAGE_FLASH_BLOCK_SIZE);        
        hal_exflash_write(&g_exflash_handle, IMAGE_FLASH_BOOT_2_START, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);
    } while(0);
    
    return;
}

static void gr_ota_move_image(void){
    gr_image_info_t * p_image_info = NULL;
    uint32_t    app_size = 0, 
                image_size = 0,  
                image_desc_size=0;          //app_size == image_size + image_desc_size
    
    p_image_info = (gr_image_info_t *) IMAGE_FLASH_BOOT_2_START;    
#if 0 //fake for test
    p_image_info->img_boot_info.bin_size       = 0x00021720;//image_info->img_boot_info.bin_size;
    p_image_info->img_boot_info.check_sum      = 0x00ce6c1b;//image_info->img_boot_info.check_sum;
    p_image_info->img_boot_info.load_addr      = IMAGE_FLASH_BANK_1_START;//image_info->img_boot_info.load_addr;
    p_image_info->img_boot_info.run_addr       = IMAGE_FLASH_BANK_1_START;//image_info->img_boot_info.run_addr;
#endif    
    
    image_size = p_image_info->img_boot_info.bin_size;
    image_desc_size = sizeof(gr_image_info_t);
    app_size = image_size + image_desc_size;
    
    /******************************/
    uint32_t sector_num = 0;
    uint32_t bytes_left = 0;
    uint32_t i = 0;
    hal_status_t status = HAL_OK; 
    
    sector_num = app_size / IMAGE_FLASH_BLOCK_SIZE;
    bytes_left = app_size % IMAGE_FLASH_BLOCK_SIZE;
    
    for(i = 0; i < sector_num; i++){
        status |= hal_exflash_read(&g_exflash_handle, IMAGE_FLASH_BANK_2_START + i*IMAGE_FLASH_BLOCK_SIZE, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);        
        status |= hal_exflash_erase(&g_exflash_handle, EXFLASH_ERASE_SECTOR, IMAGE_FLASH_BANK_1_START + i*IMAGE_FLASH_BLOCK_SIZE , IMAGE_FLASH_BLOCK_SIZE);        
        status |= hal_exflash_write(&g_exflash_handle, IMAGE_FLASH_BANK_1_START + i*IMAGE_FLASH_BLOCK_SIZE, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);
    }
    
    if(bytes_left > 0){
        status |= hal_exflash_read(&g_exflash_handle, IMAGE_FLASH_BANK_2_START + sector_num*IMAGE_FLASH_BLOCK_SIZE, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);        
        status |= hal_exflash_erase(&g_exflash_handle, EXFLASH_ERASE_SECTOR, IMAGE_FLASH_BANK_1_START + sector_num*IMAGE_FLASH_BLOCK_SIZE , IMAGE_FLASH_BLOCK_SIZE);        
        status |= hal_exflash_write(&g_exflash_handle, IMAGE_FLASH_BANK_1_START + sector_num*IMAGE_FLASH_BLOCK_SIZE, &s_block_data[0], IMAGE_FLASH_BLOCK_SIZE);
    }
    status = status;
        
    gr_ota_update_boot_info(p_image_info);
    gr_ota_update_image_status();
    return;
}

static void gr_iap_main(void ){
    gr_ota_move_image();
    
    //wakeup all ram mem
    ll_pwr_set_mem_wakeup_power(LL_PWR_MEM_ALL, LL_PWR_MEM_POWER_FULL);
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());
    ll_pwr_req_excute_psc_command(LL_PWR_CMD_LD_MEM_WKUP_CFG);
    while(SET == ll_pwr_is_active_flag_psc_cmd_busy());

    // set cold boot and reset
    *(uint32_t *)0xa000c578 = 0;    
    __set_FAULTMASK(1);
    NVIC_SystemReset();
}

#pragma push
#pragma O0
void gr_jump2iap(void){

    //disable all interrupts
    __asm("CPSID   I");
    __asm("CPSID   F");

    //reset the sp
    //__set_MSP(0x30040000);
    gr_iap_main();
}
#pragma pop

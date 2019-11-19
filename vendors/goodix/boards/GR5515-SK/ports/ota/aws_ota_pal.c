/*
 * Amazon FreeRTOS OTA PAL V1.0.0
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */

/* C Runtime includes. */
#include <stdio.h>
#include <stdlib.h>

/* Amazon FreeRTOS include. */
#include "FreeRTOS.h"
#include "event_groups.h"
#include "aws_ota_pal.h"
#include "aws_ota_pal_config.h"
#include "aws_ota_agent_internal.h"
#include "aws_crypto.h"
#include "aws_ota_codesigner_certificate.h"

/* mbedTLS includes. */
//#include "mbedtls/platform.h"
//#include "mbedtls/net.h"
//#include "mbedtls/ctr_drbg.h"
//#include "mbedtls/entropy.h"
//#include "mbedtls/sha256.h"
//#include "mbedtls/pk.h"
//#include "mbedtls/base64.h"
//#include "mbedtls/pk_internal.h"
//#include "mbedtls/debug.h"
//#include "tinycrypt/sha256.h"
//#include "tinycrypt/constants.h"
#include "string.h"
#include "app_log.h"
#include "hal_flash.h"
#include "gr55xx_dfu.h"
#include "gr55xx_hal_hmac.h"
#include "gr55xx_hal_cortex.h"
#include "gr_debug.h"

typedef struct
{
    bool                        is_valid_image;         //record current image is valid or not
    uint32_t                    cur_file_handle;
    gr_bank_order_e             cur_used_bank;
    const OTA_FileContext_t *   cur_ota_file; /* Current OTA file to be processed. */    
} gr_ota_record_t;

static gr_ota_record_t g_ota_record = {
    .is_valid_image         = false,
    .cur_file_handle        = 1,
    .cur_used_bank          = GR_IMAGE_BANK_1,      //BANK_1 used to run, BANK_2 used to save ota image
    .cur_ota_file           = NULL
};


/* Specify the OTA signature algorithm we support on this platform. */
const char cOTA_JSON_FileSignatureKey[ OTA_FILE_SIG_KEY_STR_MAX_LENGTH ] = "sig-sha256-ecdsa";   /* FIX ME. */

/* The static functions below (prvPAL_CheckFileSignature and prvPAL_ReadAndAssumeCertificate) 
 * are optionally implemented. If these functions are implemented then please set the following macros in 
 * aws_test_ota_config.h to 1:
 * otatestpalCHECK_FILE_SIGNATURE_SUPPORTED
 * otatestpalREAD_AND_ASSUME_CERTIFICATE_SUPPORTED
 */

/**
 * @brief Verify the signature of the specified file.
 * 
 * This function should be implemented if signature verification is not offloaded
 * to non-volatile memory io functions.
 * 
 * This function is called from prvPAL_Close(). 
 * 
 * @param[in] C OTA file context information.
 * 
 * @return Below are the valid return values for this function.
 * kOTA_Err_None if the signature verification passes.
 * kOTA_Err_SignatureCheckFailed if the signature verification fails.
 * kOTA_Err_BadSignerCert if the if the signature verification certificate cannot be read.
 * 
 */
static OTA_Err_t prvPAL_CheckFileSignature( OTA_FileContext_t * const C );

/**
 * @brief Read the specified signer certificate from the filesystem into a local buffer.
 * 
 * The allocated memory returned becomes the property of the caller who is responsible for freeing it.
 * 
 * This function is called from prvPAL_CheckFileSignature(). It should be implemented if signature
 * verification is not offloaded to non-volatile memory io function.
 * 
 * @param[in] pucCertName The file path of the certificate file.
 * @param[out] ulSignerCertSize The size of the certificate file read.
 * 
 * @return A pointer to the signer certificate in the file system. NULL if the certificate cannot be read.
 * This returned pointer is the responsibility of the caller; if the memory is allocated the caller must free it.
 */
static uint8_t * prvPAL_ReadAndAssumeCertificate( const uint8_t * const pucCertName,
                                                  uint32_t * const ulSignerCertSize );



static gr_bank_order_e gr_ota_get_current_used_bank(void) {
    gr_boot_info_t  boot_info;
    memset(&boot_info, 0, sizeof(gr_boot_info_t));
    memcpy(&boot_info, (void*) IMAGE_FLASH_BOOT_1_START, sizeof(gr_boot_info_t));
    
    //just support xip mode
    if((boot_info.load_addr != boot_info.run_addr) && (boot_info.xqspi_xip_cmd != GR_XIP_READ_MODE)){
        return GR_IMAGE_BANK_INVALID;
    }
    
    if(boot_info.load_addr == IMAGE_FLASH_BANK_1_START){
        return GR_IMAGE_BANK_1;
    } else if(boot_info.load_addr == IMAGE_FLASH_BANK_2_START){
        return GR_IMAGE_BANK_2;
    }
    
    return GR_IMAGE_BANK_INVALID;
}

static bool gr_ota_erase_flash_pages(void){
    bool ret = false;
    if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_1){
        ret = hal_flash_erase(IMAGE_FLASH_BANK_2_START, IMAGE_FLASH_BANK_2_SIZE);
    } else if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_2) {
        ret = hal_flash_erase(IMAGE_FLASH_BANK_1_START, IMAGE_FLASH_BANK_1_SIZE);
    }
    
    return ret;
}

static uint32_t gr_ota_calculate_image_checksum(const uint8_t * p_data, uint32_t size) {
    uint32_t cksum = 0;

    for (int i=0; i < size; i++){
        cksum += 0x000000ff & (p_data[i]);
    }

    return cksum;
}
/*********************************************************************************************
 * Goodix image contains : 
 *          the pure image + 48bytes image_info after the image
 *          in test mode: the test data not contains the 48 bytes image_info data.
 *          for meet with the logic check, add the extra 48bytes image info in test mode
 *********************************************************************************************/
static uint32_t gr_ota_get_image_size(uint32_t fileSize){
    uint32_t tsize = 0;
#if defined(AMAZON_FREERTOS_ENABLE_UNIT_TESTS)
    tsize = fileSize + sizeof(gr_image_info_t);
#else
    tsize = fileSize;
#endif
    
    return tsize;
}

static gr_ota_status_e gr_ota_get_ota_status(void){
    gr_image_info_t * image_info = (gr_image_info_t *) IMAGE_FLASH_BOOT_2_START;
    
    switch(image_info->img_ota_status){
        case GR_OTA_STATUS_UNSET:
        case GR_OTA_STATUS_READY:
        case GR_OTA_STATUS_SET:
            return image_info->img_ota_status;
                
        default:
            break;
    }
    
    return GR_OTA_STATUS_UNSET;
}

static bool gr_ota_check_file_valid(uint32_t raw_file_size){
    uint32_t image_size         = gr_ota_get_image_size(raw_file_size);
    uint32_t image_info_size    = sizeof(gr_image_info_t);
    uint32_t app_size           = image_size - image_info_size;

    uint32_t        image_start_addr    = 0;
    uint32_t        check_sum           = 0;
    gr_image_info_t image_info;
    uint8_t *       addr = NULL;
    
    if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_1){
        image_start_addr = IMAGE_FLASH_BANK_2_START;
    } else if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_2){
        image_start_addr = IMAGE_FLASH_BANK_1_START;
    } else {
        return false;
    }
    
    memset(&image_info, 0, image_info_size);
    memcpy(&image_info, (void*) (image_start_addr + app_size), image_info_size);
    
    addr = (uint8_t *) image_start_addr;
    
    check_sum = gr_ota_calculate_image_checksum(addr, app_size);
    if((image_info.img_pattern_gd != GR_IMAGE_PATTERN_GD) ||
       (image_info.img_version != GR_IMAGE_VERSION) ||
       (image_info.img_boot_info.bin_size != app_size) ||
       (image_info.img_boot_info.check_sum != check_sum) ||
       (image_info.img_boot_info.xqspi_xip_cmd != GR_XIP_READ_MODE)
    ){
        return false;
    }
    
    return true;
}


static bool gr_ota_update_rx_image_info(uint32_t raw_file_size){
    uint32_t image_size         = gr_ota_get_image_size(raw_file_size);
    uint32_t image_info_size    = sizeof(gr_image_info_t);
    uint32_t app_size           = image_size - image_info_size;   
    uint32_t image_start_addr   = 0;    
    gr_image_info_t image_info;
    
    memset(&image_info, 0, image_info_size);
    
    if(true == gr_ota_check_file_valid(raw_file_size)){
        if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_1){
            image_start_addr = IMAGE_FLASH_BANK_2_START;
        } else if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_2){
            image_start_addr = IMAGE_FLASH_BANK_1_START;
        } else {
            return false;
        }
        
        memcpy(&image_info, (void*) (image_start_addr + app_size), image_info_size);
        image_info.img_ota_status = GR_OTA_STATUS_READY;
        
        image_info.img_boot_info.load_addr = IMAGE_FLASH_BANK_1_START;
        image_info.img_boot_info.run_addr  = IMAGE_FLASH_BANK_1_START;
        
        if(true == hal_flash_erase(IMAGE_FLASH_BOOT_2_START, IMAGE_FLASH_BLOCK_SIZE)){
            if(0 != hal_flash_write(IMAGE_FLASH_BOOT_2_START, (const uint8_t *)&image_info, sizeof(gr_image_info_t))){
                return true;
            }
        }        
    } 
    
    return false;
}

void gr_ota_dump_hex(uint8_t * buff, uint32_t length){
    static uint8_t tbuff[512];
    uint32_t len = 0, i, m,n;
    
    
    memset(&tbuff[0], 0 , 512);
    
    len = 0;
    
    if(true)
    {
        m = length / 10;
        n = length % 10;
        
        for(i=0; i < m; i++){
            len += sprintf((char*)&tbuff[len], "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x \r\n", buff[i*10 + 0], buff[i*10 + 1], buff[i*10 + 2], buff[i*10 + 3], 
                                                                                                  buff[i*10 + 4], buff[i*10 + 5], buff[i*10 + 6], buff[i*10 + 7], 
                                                                                                  buff[i*10 + 8], buff[i*10 +9]);
        }
        
        if(n > 0){
            for(i=0; i<n ;i++){
                len += sprintf((char*)&tbuff[len], "%02x ", buff[m*10 + i]);
            }
            len += sprintf((char*)&tbuff[len], "\r\n");
        }
    }
    
    APP_LOG_INFO(">>>>> 0x%08x: \r\n %s \r\n", (int)buff, &tbuff[0]);    
}

static void gr_ota_reset_image_status(gr_ota_status_e gr_status, OTA_ImageState_t aws_status){
    gr_image_info_t image_info;
    
    memset(&image_info, 0, sizeof(gr_image_info_t));
    /*Read IMAGE_FLASH_BOOT_2_START*/    
    memcpy(&image_info, (void*) IMAGE_FLASH_BOOT_2_START, sizeof(gr_image_info_t));
    
    image_info.img_flag                 = aws_status;
    image_info.img_ota_status           = gr_status;
    
    hal_flash_erase(IMAGE_FLASH_BOOT_2_START, IMAGE_FLASH_BLOCK_SIZE);        
    hal_flash_write(IMAGE_FLASH_BOOT_2_START, (const uint8_t *)&image_info, sizeof(gr_image_info_t));
}

void gr_ota_startup_check_and_update(void){
    gr_boot_info_t  boot_info;
    gr_image_info_t image_info;
    
    memset(&boot_info,  0, sizeof(gr_boot_info_t));
    memset(&image_info, 0, sizeof(gr_image_info_t));
    
    memcpy(&boot_info, (void*) IMAGE_FLASH_BOOT_1_START, sizeof(gr_boot_info_t));
#if GR_DEBUG_CODE > 0u    
    APP_LOG_INFO(">>>>> boot image app size  : %d ",         boot_info.bin_size);
    APP_LOG_INFO(">>>>> boot image checksum  : 0x%08x ",     boot_info.check_sum);
    APP_LOG_INFO(">>>>> boot image load addr : 0x%08x ",     boot_info.load_addr);
    APP_LOG_INFO(">>>>> boot image run  addr : 0x%08x ",     boot_info.run_addr);
    APP_LOG_INFO(">>>>> boot image xip cmd   : 0x%02x ",     boot_info.xqspi_xip_cmd);
    APP_LOG_INFO("\r\n");
#endif
    /*Read IMAGE_FLASH_BOOT_2_START*/    
    memcpy(&image_info, (void*) IMAGE_FLASH_BOOT_2_START, sizeof(gr_image_info_t));
    
    //check inited, if never inited, init it 
    if((image_info.img_pattern_gd != GR_IMAGE_PATTERN_GD) ||
       (image_info.img_version != GR_IMAGE_VERSION) ||       
       (image_info.img_boot_info.xqspi_xip_cmd != GR_XIP_READ_MODE)
    ) {
        
        memset(&image_info, 0, sizeof(gr_image_info_t));
        memcpy(&image_info.img_boot_info, &boot_info, sizeof(gr_boot_info_t));
        image_info.img_boot_info.load_addr  = IMAGE_FLASH_BANK_1_START;
        image_info.img_boot_info.run_addr   = IMAGE_FLASH_BANK_1_START;
        image_info.img_version              = GR_IMAGE_VERSION;
        image_info.img_pattern_gd           = GR_IMAGE_PATTERN_GD;
        image_info.img_flag                 = eOTA_ImageState_Unknown;
        image_info.img_ota_status           = GR_OTA_STATUS_UNSET;
        
        hal_flash_erase(IMAGE_FLASH_BOOT_2_START, IMAGE_FLASH_BLOCK_SIZE);        
        hal_flash_write(IMAGE_FLASH_BOOT_2_START, (const uint8_t *)&image_info, sizeof(gr_image_info_t));
    }
    
    {
        char * flag = NULL;
        
        switch(image_info.img_flag){
            case eOTA_ImageState_Unknown:
                flag = "UnOTA";
                break;
            
            case eOTA_ImageState_Testing:
                flag = "Testing";
                break;
            
            case eOTA_ImageState_Accepted:
                flag = "Accepted";
                break;
            
            case eOTA_ImageState_Rejected:
                flag = "Rejected";
                break;
            
            case eOTA_ImageState_Aborted:
                flag = "Aborted";
                break;
            
            default:
                flag = "Error";
                break;
        }
#if GR_DEBUG_CODE > 0u            
        memcpy(&image_info, (void*) IMAGE_FLASH_BOOT_2_START, sizeof(gr_image_info_t));
        APP_LOG_INFO(">>>>> ota image app size  : %d ",         image_info.img_boot_info.bin_size);
        APP_LOG_INFO(">>>>> ota image checksum  : 0x%08x ",     image_info.img_boot_info.check_sum);
        APP_LOG_INFO(">>>>> ota image load addr : 0x%08x ",     image_info.img_boot_info.load_addr);
        APP_LOG_INFO(">>>>> ota image run  addr : 0x%08x ",     image_info.img_boot_info.run_addr);
        APP_LOG_INFO(">>>>> ota image xip cmd   : 0x%02x ",     image_info.img_boot_info.xqspi_xip_cmd);
        APP_LOG_INFO(">>>>> ota image valid flag: %d ",         image_info.img_ota_status);
        APP_LOG_INFO(">>>>> ota image ota flag  : %s ",         flag);
        APP_LOG_INFO("\r\n");
#endif
    }        
}

/*
static bool gr_ota_compute_sha256_hash( void * raw_message,
                                        uint32_t raw_message_len,
                                        void * digest_hash)
{    
    int ret = TC_CRYPTO_SUCCESS;
    struct tc_sha256_state_struct sha256_t;
    
    ret = tc_sha256_init(&sha256_t);
    
    if(ret != TC_CRYPTO_SUCCESS){
        //OTA_LOG_L1( "Compute sha256 hash failed, init ret: %d \r\n", ret);
        return false;
    }
    
    ret = tc_sha256_update(&sha256_t, raw_message, raw_message_len);        
    
    if(ret != TC_CRYPTO_SUCCESS){
        OTA_LOG_L1( "Compute sha256 hash failed, sha256 update: %d \r\n", ret);
        return false;
    }
    
    ret = tc_sha256_final(digest_hash, &sha256_t);
    
    if(ret != TC_CRYPTO_SUCCESS){
        OTA_LOG_L1( "Compute sha256 hash failed, sha256 final: %d \r\n", ret);
        return false;
    }

    return true;    
}
*/

#if defined(AMAZON_FREERTOS_ENABLE_UNIT_TESTS)
static void gr_ota_auto_add_image_info_padding(uint32_t image_size){
    uint32_t start_addr = 0;
    uint32_t bytes_written = 0;
    gr_image_info_t image_info;
    
    //data aligned in upper layer API
    
    if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_1){
        start_addr = IMAGE_FLASH_BANK_2_START;
    } else if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_2){
        start_addr = IMAGE_FLASH_BANK_1_START;
    } else {
        return ;
    }
    
    memset(&image_info, 0 , sizeof(gr_image_info_t));
    image_info.img_version      = GR_IMAGE_VERSION;
    image_info.img_pattern_gd   = GR_IMAGE_PATTERN_GD;
    image_info.img_boot_info.bin_size       = image_size ;//+ sizeof(gr_image_info_t);
    image_info.img_boot_info.xqspi_xip_cmd  = GR_XIP_READ_MODE;
    image_info.img_boot_info.load_addr      = IMAGE_FLASH_BANK_1_START;
    image_info.img_boot_info.run_addr       = IMAGE_FLASH_BANK_1_START;
    image_info.img_boot_info.check_sum      = gr_ota_calculate_image_checksum((const uint8_t *) start_addr, image_size);

    portENTER_CRITICAL();
    bytes_written = hal_flash_write(start_addr + image_size, (const uint8_t *)&image_info, sizeof(gr_image_info_t));    
    portEXIT_CRITICAL();    
}
#endif


static inline BaseType_t prvContextValidate( OTA_FileContext_t * C )
{
    return( ( C != NULL ) &&
            ( C->pucFile != NULL ) &&
            (g_ota_record.cur_ota_file != NULL) &&
            (g_ota_record.cur_ota_file->pucFile == C->pucFile)
          ); 
}

static OTA_Err_t prvPAL_CheckFileSignature( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_CheckFileSignature" );
    OTA_Err_t eResult = kOTA_Err_None;
       
    uint32_t ulBytesRead;
    uint32_t ulSignerCertSize;
    uint8_t * pucSignerCert;
    void * pvSigVerifyContext;
    
    if( prvContextValidate( C ) == pdTRUE )
    {
        CRYPTO_ConfigureHeap();
        /* Verify an ECDSA-SHA256 signature. */
        if( pdFALSE == CRYPTO_SignatureVerificationStart( &pvSigVerifyContext, cryptoASYMMETRIC_ALGORITHM_ECDSA, cryptoHASH_ALGORITHM_SHA256 ) )
        {
            OTA_LOG_L1( "[%s] Started signature verification Failed \r\n", OTA_METHOD_NAME);
            eResult = kOTA_Err_SignatureCheckFailed;
        }
        else
        {
            //OTA_LOG_L1( "[%s] Started %s signature verification, file: %s\r\n", OTA_METHOD_NAME, cOTA_JSON_FileSignatureKey, ( const char * ) C->pucCertFilepath );
            pucSignerCert = prvPAL_ReadAndAssumeCertificate( ( const uint8_t * const ) C->pucCertFilepath, &ulSignerCertSize );

            if( pucSignerCert != NULL )
            {
                uint32_t    start_addr = 0;  
                if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_1){
                    start_addr = IMAGE_FLASH_BANK_2_START;
                } else if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_2){
                    start_addr = IMAGE_FLASH_BANK_1_START;
                } else {
                    return kOTA_Err_SignatureCheckFailed;
                }  
                

                {                    
                    CRYPTO_SignatureVerificationUpdate( pvSigVerifyContext, (void*)start_addr, C->ulFileSize );
                    
                    if( pdFALSE == CRYPTO_SignatureVerificationFinal( pvSigVerifyContext,
                                                                      ( char * ) pucSignerCert,
                                                                      ( size_t ) ulSignerCertSize,
                                                                      C->pxSignature->ucData,
                                                                      C->pxSignature->usSize ) ) /*lint !e732 !e9034 Allow comparison in this context. */
                    {
                        eResult = kOTA_Err_SignatureCheckFailed;
                    }
                    pvSigVerifyContext = NULL;	/* The context has been freed by CRYPTO_SignatureVerificationFinal(). */                    
                }
            }
            else
            {
                eResult = kOTA_Err_BadSignerCert;
            }
        }
    }
    else
    {
        /* FIXME: Invalid error code for a NULL file context. */
        OTA_LOG_L1( "[%s] ERROR - Invalid OTA file context.\r\n", OTA_METHOD_NAME );
        /* Invalid OTA context or file pointer. */
        eResult = kOTA_Err_NullFilePtr;
    }
    
    return eResult;
}

/*-----------------------------------------------------------*/

static uint8_t * prvPAL_ReadAndAssumeCertificate( const uint8_t * const pucCertName,
                                                  uint32_t * const ulSignerCertSize )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_ReadAndAssumeCertificate" );
    uint8_t * pucSignerCert = NULL;
    
    uint8_t * pucCertData;
    uint32_t  ulCertSize;
    ulCertSize = sizeof( signingcredentialSIGNING_CERTIFICATE_PEM );
    pucSignerCert = pvPortMalloc( ulCertSize );                           /*lint !e9029 !e9079 !e838 malloc proto requires void*. */
    pucCertData = ( uint8_t * ) signingcredentialSIGNING_CERTIFICATE_PEM; /*lint !e9005 we don't modify the cert but it could be set by PKCS11 so it's not const. */

    if( pucSignerCert != NULL )
    {
        memcpy( pucSignerCert, pucCertData, ulCertSize );
        *ulSignerCertSize = ulCertSize;
    }    

    return pucSignerCert;
}


/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_CreateFileForRx( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_CreateFileForRx" );
    
    
    bool      ret;
    OTA_Err_t xStatus           = kOTA_Err_None;
    C->lFileHandle              = g_ota_record.cur_file_handle;
    g_ota_record.cur_ota_file   = C;
    g_ota_record.cur_used_bank  = GR_IMAGE_BANK_1; //gr_ota_get_current_used_bank();
    
    //check bank order
    if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_INVALID){
        xStatus = kOTA_Err_RxFileCreateFailed;
    }
    
    //check file size
    if (xStatus == kOTA_Err_None){
        if(((g_ota_record.cur_used_bank == GR_IMAGE_BANK_1) && (gr_ota_get_image_size(C->ulFileSize) > IMAGE_FLASH_BANK_2_SIZE)) || 
           ((g_ota_record.cur_used_bank == GR_IMAGE_BANK_2) && (gr_ota_get_image_size(C->ulFileSize) > IMAGE_FLASH_BANK_1_SIZE))
          ) {
            xStatus = kOTA_Err_OutOfMemory;
        }
    }

    if (xStatus == kOTA_Err_None){
        /* Erase the required memory */
        //OTA_LOG_L1( "[%s] Erasing the flash memory \r\n", OTA_METHOD_NAME );
        
        ret = gr_ota_erase_flash_pages();        
        if (ret != true) 
        {
            xStatus = kOTA_Err_RxFileCreateFailed ;
            OTA_LOG_L1( "[%s] Erasing the flash memory failed with error_code %d\r\n", OTA_METHOD_NAME, xStatus );
        }
    }
    
    if (xStatus == kOTA_Err_None){
        gr_ota_reset_image_status(GR_OTA_STATUS_UNSET, eOTA_ImageState_Unknown);
    }

    g_ota_record.is_valid_image    = false;

    return xStatus;
}
/*-----------------------------------------------------------*/
//free resources
OTA_Err_t prvPAL_Abort( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_Abort" );

    C->lFileHandle                  = ( int32_t ) NULL;
    
    //OTA_LOG_L1( "[%s] prvPAL_Abort called ...\n",  OTA_METHOD_NAME);

    return kOTA_Err_None;
}
/*-----------------------------------------------------------*/

/* Write a block of data to the specified file. */
int16_t prvPAL_WriteBlock( OTA_FileContext_t * const C,
                           uint32_t ulOffset,
                           uint8_t * const pacData,
                           uint32_t ulBlockSize )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_WriteBlock" );
    
    uint32_t start_addr = 0;
    uint32_t bytes_written = 0;
    
    //data aligned in upper layer API
    
    if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_1){
        start_addr = IMAGE_FLASH_BANK_2_START;
    } else if(g_ota_record.cur_used_bank == GR_IMAGE_BANK_2){
        start_addr = IMAGE_FLASH_BANK_1_START;
    } else {
        return -1;
    }

    portENTER_CRITICAL();
    bytes_written = hal_flash_write(start_addr + ulOffset, pacData, ulBlockSize);    
    portEXIT_CRITICAL();
    
    return bytes_written;
}
/*-----------------------------------------------------------*/


OTA_Err_t prvPAL_CloseFile( OTA_FileContext_t * const C )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_CloseFile" );
    
    OTA_Err_t xErr = kOTA_Err_None;
    bool ret ;
    uint32_t    file_size = C->ulFileSize;
    
    //OTA_LOG_L1( "[%s] Close File, Size : %d ...\r\n",  OTA_METHOD_NAME, file_size);
        
    if((C == NULL) || (C->pucFile != g_ota_record.cur_ota_file->pucFile)){
        xErr = kOTA_Err_FileClose;
    }
    
#if defined(AMAZON_FREERTOS_ENABLE_UNIT_TESTS)
    if(xErr == kOTA_Err_None){
        //__BKPT(0);
        if(true != gr_ota_check_file_valid(C->ulFileSize)){
            gr_ota_auto_add_image_info_padding(C->ulFileSize);
        }
    }        
#endif
    
    if(xErr == kOTA_Err_None) {
        xErr = prvPAL_CheckFileSignature( C );
        //OTA_LOG_L1( "[%s] prvPAL_CheckFileSignature ret : %d ...\r\n",  OTA_METHOD_NAME, xErr);
        if(xErr == kOTA_Err_None) {
            if(file_size > 0){
                ret = gr_ota_update_rx_image_info(file_size);
                
                if(ret){
                    g_ota_record.cur_file_handle += 1;
                } else {
                    xErr = kOTA_Err_SignatureCheckFailed;
                }
            } else {
                xErr = kOTA_Err_SignatureCheckFailed;
            }
        }
    }
    
    if(xErr == kOTA_Err_None){
        g_ota_record.is_valid_image     = true;
    } else {
        g_ota_record.is_valid_image     = false;
        //OTA_LOG_L1( "[%s] prvPAL_CloseFile, err: %d ...\r\n", OTA_METHOD_NAME, xErr);
    }
    
    g_ota_record.cur_ota_file = NULL;
    
    return xErr;
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_ResetDevice( void )
{
    DEFINE_OTA_METHOD_NAME("prvPAL_ResetDevice");
    boot_info_t         boot_info ;
    gr_image_info_t *   image_info = (gr_image_info_t *) IMAGE_FLASH_BOOT_2_START;
    
    //OTA_LOG_L1( "[%s] prvPAL_ResetDevice, image flag: %d ...\r\n", OTA_METHOD_NAME, image_info->img_flag);
    
    //meet with any one of conditions, can enter the dfu
    if((image_info->img_ota_status == GR_OTA_STATUS_READY) ||
       (image_info->img_flag == eOTA_ImageState_Testing) || 
       (image_info->img_flag == eOTA_ImageState_Accepted)){

        OTA_LOG_L1( "[%s] prvPAL_ResetDevice, Goto IAP Mode, please wait...\r\n", OTA_METHOD_NAME);
        vTaskDelay(1000);
        gr_jump2iap();

        //never reach here!
        return kOTA_Err_None;            
    } else {
        OTA_LOG_L1( "[%s] prvPAL_ResetDevice directly...\r\n", OTA_METHOD_NAME);
        vTaskDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }

    return kOTA_Err_ResetNotSupported;
}
/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_ActivateNewImage( void )
{
    DEFINE_OTA_METHOD_NAME("prvPAL_ActivateNewImage");

    prvPAL_ResetDevice();
    
    return kOTA_Err_None;
}
/*-----------------------------------------------------------*/

OTA_Err_t prvPAL_SetPlatformImageState( OTA_ImageState_t eState )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_SetPlatformImageState" );    
    gr_image_info_t image_info;
    
    OTA_Err_t rErr = kOTA_Err_None;
    
    if((eState <= eOTA_ImageState_Unknown)|| (eState > eOTA_LastImageState)){
        rErr = kOTA_Err_BadImageState;
    }
    
    if(rErr == kOTA_Err_None){
        
        if(eState == eOTA_ImageState_Accepted){
            if((!g_ota_record.is_valid_image && (g_ota_record.cur_ota_file != NULL)) || 
               (g_ota_record.is_valid_image && (GR_OTA_STATUS_UNSET == gr_ota_get_ota_status()))
              ) {
                rErr = kOTA_Err_CommitFailed;
            } 
        }
    }    
    
    if(rErr == kOTA_Err_None) {
        memset(&image_info, 0, sizeof(gr_image_info_t));    
        /*Read IMAGE_FLASH_BOOT_2_START*/    
        memcpy(&image_info, (void*) IMAGE_FLASH_BOOT_2_START, sizeof(gr_image_info_t));
        
        image_info.img_flag = eState;
        
        if(true == hal_flash_erase(IMAGE_FLASH_BOOT_2_START, IMAGE_FLASH_BLOCK_SIZE)){
            if(0 != hal_flash_write(IMAGE_FLASH_BOOT_2_START, (const uint8_t *)&image_info, sizeof(gr_image_info_t))){
                rErr = kOTA_Err_None;
            } else {
                rErr = kOTA_Err_CommitFailed;
            }
        } else {
            rErr = kOTA_Err_CommitFailed;
        }
    }
    
    //OTA_LOG_L1( "[%s] prvPAL_SetPlatformImageState, set state: %d ,ret :%d...\n", OTA_METHOD_NAME, eState, rErr);
    
    return rErr;
}
/*-----------------------------------------------------------*/

OTA_PAL_ImageState_t prvPAL_GetPlatformImageState( void )
{
    DEFINE_OTA_METHOD_NAME( "prvPAL_GetPlatformImageState" );
    
    gr_image_info_t * image_info = (gr_image_info_t *) IMAGE_FLASH_BOOT_2_START;
    
    OTA_PAL_ImageState_t state = eOTA_PAL_ImageState_Unknown;
    
    switch (image_info->img_flag){
        case eOTA_ImageState_Unknown:
        {
            if(g_ota_record.is_valid_image){
                state = eOTA_PAL_ImageState_PendingCommit;
            } else {
                gr_ota_status_e status = gr_ota_get_ota_status();
                //OTA_LOG_L1( "[%s] prvPAL_GetPlatformImageState, gr_ota_get_ota_status: %d ...\n", OTA_METHOD_NAME, status);
                if(status == GR_OTA_STATUS_UNSET){
                    state = eOTA_PAL_ImageState_Invalid;
                } else if(status == GR_OTA_STATUS_READY){
                    state = eOTA_PAL_ImageState_PendingCommit;
                } else if(status == GR_OTA_STATUS_SET){
                    state = eOTA_PAL_ImageState_Valid;
                }
            }
        }
        break;
            
        case eOTA_ImageState_Accepted:
            state = eOTA_PAL_ImageState_Valid;
            break;
        
        case eOTA_ImageState_Rejected:
        case eOTA_ImageState_Aborted:
            state = eOTA_PAL_ImageState_Invalid;
            break;
        
        case eOTA_ImageState_Testing:
            state = eOTA_PAL_ImageState_PendingCommit;
            break;
        
        default:
            break;        
    }
    //OTA_LOG_L1( "[%s] prvPAL_GetPlatformImageState, flag: %d, state: %d ...\n", OTA_METHOD_NAME, image_info->img_flag, state);
    
    return state;
}

#if GR_DEBUG_CODE > 0u

void gr_ota_test(void){
    uint8_t img_data[48] = {0x44,0x47,0x01,0x00,0x10,0x99,0x03,0x00,0x16,0xE1,0x57,0x01,0x00,0x20,0x00,0x01, \
                            0x00,0x20,0x00,0x01,0xEB,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x67,0x6F,0x6F,0x64, \
                            0x69,0x78,0x5F,0x61,0x77,0x73,0x5F,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    
    uint32_t boot_len   = sizeof(gr_boot_info_t);
    uint32_t image_len  = sizeof(gr_image_info_t);

    gr_image_info_t  img_info;    
    memcpy(&img_info, img_data, 48);

    APP_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    APP_LOG_INFO(">>>>> sizeof(gr_boot_info_t) = %d",   boot_len );
    APP_LOG_INFO(">>>>> sizeof(gr_image_info_t) = %d",  image_len);

    APP_LOG_INFO(">>>>> image pattern: 0x%04x ",        img_info.img_pattern_gd);
    APP_LOG_INFO(">>>>> image version: 0x%04x ",        img_info.img_version);
    APP_LOG_INFO(">>>>> image size: 0x%08x ",           img_info.img_boot_info.bin_size);
    APP_LOG_INFO(">>>>> image checksum: 0x%08x ",       img_info.img_boot_info.check_sum);
    APP_LOG_INFO(">>>>> image load addr: 0x%08x ",      img_info.img_boot_info.load_addr);
    APP_LOG_INFO(">>>>> image run  addr: 0x%08x ",      img_info.img_boot_info.run_addr);
    APP_LOG_INFO(">>>>> image xqspi_xip_cmd: 0x%08x ",  img_info.img_boot_info.xqspi_xip_cmd);
    APP_LOG_INFO(">>>>> image config: 0x%08x ",         img_info.img_boot_info.boot_config.i_val);
    APP_LOG_INFO(">>>>> image comment: %s ",            img_info.img_comments);
    APP_LOG_INFO("------------------------------------------");
        
    gr_boot_info_t  boot_info;
    memset(&boot_info, 0, sizeof(gr_boot_info_t));
    memcpy(&boot_info, (void*) IMAGE_FLASH_BOOT_1_START, sizeof(gr_boot_info_t));
    
    APP_LOG_INFO("++++++++++ Boot Info At ADDR: 0x%08x ", IMAGE_FLASH_BOOT_1_START);
    APP_LOG_INFO(">>>>> image size: 0x%08x ",           boot_info.bin_size);
    APP_LOG_INFO(">>>>> image checksum: 0x%08x ",       boot_info.check_sum);
    APP_LOG_INFO(">>>>> image load addr: 0x%08x ",      boot_info.load_addr);
    APP_LOG_INFO(">>>>> image run  addr: 0x%08x ",      boot_info.run_addr);
    APP_LOG_INFO(">>>>> image xqspi_xip_cmd: 0x%08x ",  boot_info.xqspi_xip_cmd);
    APP_LOG_INFO(">>>>> image config: 0x%08x ",         boot_info.boot_config.i_val);    
    APP_LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    
    uint32_t tail_addr = IMAGE_FLASH_BANK_1_START + boot_info.bin_size;
    
    memset(&img_info, 0, 48);
    memcpy(&img_info, (void*) (tail_addr), 48);
    
    APP_LOG_INFO(">>>>>>>>>> Image Info At ADDR: 0x%08x (%d) ", tail_addr, tail_addr);
    
    APP_LOG_INFO(">>>>> image pattern: 0x%04x ",        img_info.img_pattern_gd);
    APP_LOG_INFO(">>>>> image version: 0x%04x ",        img_info.img_version);
    APP_LOG_INFO(">>>>> image size: 0x%08x ",           img_info.img_boot_info.bin_size);
    APP_LOG_INFO(">>>>> image checksum: 0x%08x ",       img_info.img_boot_info.check_sum);
    APP_LOG_INFO(">>>>> image load addr: 0x%08x ",      img_info.img_boot_info.load_addr);
    APP_LOG_INFO(">>>>> image run  addr: 0x%08x ",      img_info.img_boot_info.run_addr);
    APP_LOG_INFO(">>>>> image xqspi_xip_cmd: 0x%08x ",  img_info.img_boot_info.xqspi_xip_cmd);
    APP_LOG_INFO(">>>>> image config: 0x%08x ",         img_info.img_boot_info.boot_config.i_val);
    APP_LOG_INFO(">>>>> image comment: %s ",            img_info.img_comments);
    APP_LOG_INFO("------------------------------------------\r\n\r\n");
}
#endif

/*-----------------------------------------------------------*/

/* Provide access to private members for testing. */
#ifdef AMAZON_FREERTOS_ENABLE_UNIT_TESTS
    #include "aws_ota_pal_test_access_define.h"
#endif

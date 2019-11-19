/*
 * Amazon FreeRTOS PKCS #11 PAL V1.0.0
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

/**
 * @file iot_pkcs11_pal.c
 * @brief Device specific helpers for PKCS11 Interface.
 */

/* Amazon FreeRTOS Includes. */
#include "aws_pkcs11.h"
#include "FreeRTOS.h"
#include "task.h"

/* C runtime includes. */
#include <stdio.h>
#include <string.h>

/* rng */
#include "gr55xx_nvds.h"
#include "gr55xx_hal.h"
#include "gr55xx_hal_rng.h"

#include "gr_config.h"

/******************************************************
 * STATIC APIs for porting 
 *******************************************************/

#define pkcs11palFILE_NAME_CLIENT_CERTIFICATE    "P11_Cert"
#define pkcs11palFILE_NAME_KEY                   "P11_Key"
#define pkcs11palFILE_CODE_SIGN_PUBLIC_KEY       "P11_CSK"

enum eObjectHandles
{
    eInvalidHandle = 0, /* According to PKCS #11 spec, 0 is never a valid object handle. */
    eAwsDevicePrivateKey = 1,
    eAwsDevicePublicKey,
    eAwsDeviceCertificate,
    eAwsCodeSigningKey,
};

enum PKCS11_NVDS_TAG {    
    NVDS_TAG_PRI_PUB_KEY_SIZE = 1,  //private & public key share the save file
    NVDS_TAG_PRI_PUB_KEY,
    NVDS_TAG_CERTIFICATE_KEY_SIZE,
    NVDS_TAG_CERTIFICATE_KEY,
    NVDS_TAG_SIGNING_KEY_SIZE,
    NVDS_TAG_SIGNING_KEY,
};



/* Converts a label to its respective filename and handle. */
static void prvLabelToFilenameHandle( uint8_t * pcLabel,
                               char ** pcFileName,
                               CK_OBJECT_HANDLE_PTR pHandle )
{
    if (pcLabel != NULL) {
        /* Translate from the PKCS#11 label to local storage file name. */
        if (0 == memcmp(pcLabel,
                        &pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,
                        sizeof(pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS))) {
            *pcFileName = pkcs11palFILE_NAME_CLIENT_CERTIFICATE;
            *pHandle = eAwsDeviceCertificate;
        } else if (0 == memcmp(pcLabel,
                               &pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,
                               sizeof(pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS))) {
            *pcFileName = pkcs11palFILE_NAME_KEY;
            *pHandle = eAwsDevicePrivateKey;
        } else if (0 == memcmp(pcLabel,
                               &pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS,
                               sizeof( pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS))) {
            *pcFileName = pkcs11palFILE_NAME_KEY;
            *pHandle = eAwsDevicePublicKey;
        } else if (0 == memcmp(pcLabel,
                               &pkcs11configLABEL_CODE_VERIFICATION_KEY,
                               sizeof( pkcs11configLABEL_CODE_VERIFICATION_KEY))) {
            *pcFileName = pkcs11palFILE_CODE_SIGN_PUBLIC_KEY;
            *pHandle = eAwsCodeSigningKey;
        } else {
            *pcFileName = NULL;
            *pHandle = eInvalidHandle;
        }
    }
}

/* Gen random number by RNG*/

static uint16_t g_random_seed[8] = {0x1234, 0x5678, 0x90AB, 0xCDEF, 0x1468, 0x2345, 0x5329, 0x2411};

static uint32_t getRandomNumber(void)
{
    uint32_t data = 0, i=0;
    rng_handle_t g_rng_handle;
    
    TickType_t xTicks = xTaskGetTickCount(); 
    
    for(i=0;i<8;i++){
        g_random_seed[i] += (uint16_t)xTicks;
    }

    g_rng_handle.p_instance = RNG;
    g_rng_handle.init.seed_mode  = RNG_SEED_USER;
    g_rng_handle.init.lfsr_mode  = RNG_LFSR_MODE_59BIT;
    g_rng_handle.init.out_mode   = RNG_OUTPUT_LFSR;
    g_rng_handle.init.post_mode  = RNG_POST_PRO_NOT;

    hal_rng_deinit(&g_rng_handle);
    hal_rng_init(&g_rng_handle);
    
    hal_rng_generate_random_number(&g_rng_handle, g_random_seed, &data);

    return data;
}



/**
* @brief Writes a file to local storage.
*
* Port-specific file write for crytographic information.
*
* @param[in] pxLabel       Label of the object to be saved.
* @param[in] pucData       Data buffer to be written to file
* @param[in] ulDataSize    Size (in bytes) of data to be saved.
*
* @return The file handle of the object that was stored.
*/
CK_OBJECT_HANDLE PKCS11_PAL_SaveObject( CK_ATTRIBUTE_PTR pxLabel,
    uint8_t * pucData,
    uint32_t ulDataSize )
{    
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;
    char * pcFileName = NULL;
    NvdsTag_t n_tag;
    uint8_t result; 

    /* Translate from the PKCS#11 label to local storage file name. */
    prvLabelToFilenameHandle( pxLabel->pValue, &pcFileName, &xHandle );
    if( xHandle != eInvalidHandle )
    {
        if( strcmp( pcFileName, pkcs11palFILE_NAME_KEY ) == 0 ){
            
            // save the data size first
            n_tag = NV_TAG_APP(NVDS_TAG_PRI_PUB_KEY_SIZE);            
            result = nvds_put(n_tag, sizeof(uint32_t), (const uint8_t *) &ulDataSize);            
            if(result != NVDS_SUCCESS){            
                GRH_LOG(ERROR, (">>> nvds_put failed for pri/pub key length: %d \n", result ));
            }
            
            //save the data buffer
            n_tag = NV_TAG_APP(NVDS_TAG_PRI_PUB_KEY);
            result = nvds_put(n_tag, ulDataSize, (const uint8_t *) pucData);
            if ( result != NVDS_SUCCESS)
            {
                GRH_LOG(ERROR, (">>> nvds_put failed for pri/pub key data: %d \n", result ));
            }
        
        } else if( strcmp( pcFileName, pkcs11palFILE_NAME_CLIENT_CERTIFICATE ) == 0 ){
            
            // save the data size first
            n_tag = NV_TAG_APP(NVDS_TAG_CERTIFICATE_KEY_SIZE);            
            result = nvds_put(n_tag, sizeof(uint32_t), (const uint8_t *) &ulDataSize);            
            if(result != NVDS_SUCCESS){            
                GRH_LOG(ERROR, (">>> nvds_put failed for cert length: %d \n", result ));
            }
            
            //save the data buffer
            n_tag = NV_TAG_APP(NVDS_TAG_CERTIFICATE_KEY);
            result = nvds_put(n_tag, ulDataSize, (const uint8_t *) pucData);
            if ( result != NVDS_SUCCESS)
            {
                GRH_LOG(ERROR, (">>> nvds_put failed for cert data: %d \n", result ));
            }
        } else if( strcmp( pcFileName, pkcs11palFILE_CODE_SIGN_PUBLIC_KEY ) == 0 ){
            // save the data size first
            n_tag = NV_TAG_APP(NVDS_TAG_SIGNING_KEY_SIZE);            
            result = nvds_put(n_tag, sizeof(uint32_t), (const uint8_t *) &ulDataSize);            
            if(result != NVDS_SUCCESS){            
                GRH_LOG(ERROR, (">>> nvds_put failed for sign length: %d \n", result ));
            }
            
            //save the data buffer
            n_tag = NV_TAG_APP(NVDS_TAG_SIGNING_KEY);
            result = nvds_put(n_tag, ulDataSize, (const uint8_t *) pucData);
            if ( result != NVDS_SUCCESS)
            {
                GRH_LOG(ERROR, (">>> nvds_put failed for sign length %d \n", result ));
            }
        }
    }
    
    
    return xHandle;
}

/**
* @brief Translates a PKCS #11 label into an object handle.
*
* Port-specific object handle retrieval.
*
*
* @param[in] pLabel         Pointer to the label of the object
*                           who's handle should be found.
* @param[in] usLength       The length of the label, in bytes.
*
* @return The object handle if operation was successful.
* Returns eInvalidHandle if unsuccessful.
*/
CK_OBJECT_HANDLE PKCS11_PAL_FindObject( uint8_t * pLabel,
    uint8_t usLength )
{
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;
    char * pcFileName = NULL;

    /* Translate from the PKCS#11 label to local storage file name. */
    prvLabelToFilenameHandle( pLabel, &pcFileName, &xHandle );

    return xHandle;
}

/**
* @brief Gets the value of an object in storage, by handle.
*
* Port-specific file access for cryptographic information.
*
* This call dynamically allocates the buffer which object value
* data is copied into.  PKCS11_PAL_GetObjectValueCleanup()
* should be called after each use to free the dynamically allocated
* buffer.
*
* @sa PKCS11_PAL_GetObjectValueCleanup
*
* @param[in] pcFileName    The name of the file to be read.
* @param[out] ppucData     Pointer to buffer for file data.
* @param[out] pulDataSize  Size (in bytes) of data located in file.
* @param[out] pIsPrivate   Boolean indicating if value is private (CK_TRUE)
*                          or exportable (CK_FALSE)
*
* @return CKR_OK if operation was successful.  CKR_KEY_HANDLE_INVALID if
* no such object handle was found, CKR_DEVICE_MEMORY if memory for
* buffer could not be allocated, CKR_FUNCTION_FAILED for device driver
* error.
*/
CK_RV PKCS11_PAL_GetObjectValue( CK_OBJECT_HANDLE xHandle,
    uint8_t ** ppucData,
    uint32_t * pulDataSize,
    CK_BBOOL * pIsPrivate )
{    
    uint8_t     result;
    uint16_t    len = 0;
    uint32_t    data_len = 0;
    char *      pcFileName = NULL;
    NvdsTag_t   n_tag;
    uint8_t *   tdata = NULL;

    if( xHandle == eAwsDeviceCertificate )
    {
        pcFileName = pkcs11palFILE_NAME_CLIENT_CERTIFICATE;
        *pIsPrivate = CK_FALSE;
    }
    else if( xHandle == eAwsDevicePrivateKey )
    {
        pcFileName = pkcs11palFILE_NAME_KEY;
        *pIsPrivate = CK_TRUE;
    }
    else if( xHandle == eAwsDevicePublicKey )
    {
        /* Public and private key are stored together in same file. */
        pcFileName = pkcs11palFILE_NAME_KEY;
        *pIsPrivate = CK_FALSE;
    }
    else if( xHandle == eAwsCodeSigningKey )
    {
        pcFileName = pkcs11palFILE_CODE_SIGN_PUBLIC_KEY;
        *pIsPrivate = CK_FALSE;
    }
    else
    {
        GRH_LOG(ERROR, (">>> PKCS11_PAL_GetObjectValue, invalid handle :%d... ", xHandle));
        return CKR_KEY_HANDLE_INVALID;
    }

    if( strcmp( pcFileName, pkcs11palFILE_NAME_KEY ) == 0 )
    {
        n_tag = NV_TAG_APP(NVDS_TAG_PRI_PUB_KEY_SIZE);
        len = sizeof(uint32_t);        
        result = nvds_get(n_tag, &len, (uint8_t *)&data_len);

        if(result != NVDS_SUCCESS){
            GRH_LOG(ERROR, (">>> nvds_get failed for key length: %d  ", result));
            return CKR_FUNCTION_FAILED;
        }
        
        n_tag = NV_TAG_APP(NVDS_TAG_PRI_PUB_KEY);        
        tdata = pvPortMalloc(data_len + 1);
        if (tdata == NULL) {
            GRH_LOG(ERROR, (">>> nvds_get failed for key by malloc fail  "));
            return CKR_HOST_MEMORY;
        }
        *ppucData = tdata;
        memset(tdata, 0 ,data_len + 1);
        
        result = nvds_get(n_tag, (uint16_t*)&data_len, (uint8_t *)tdata);
        if(result != NVDS_SUCCESS){
            GRH_LOG(ERROR, (">>> nvds_get failed for key data: %d  ", result));
            vPortFree(tdata);
            return CKR_FUNCTION_FAILED;
        }
        
        *pulDataSize = data_len;
    }
    else if( strcmp( pcFileName, pkcs11palFILE_NAME_CLIENT_CERTIFICATE ) == 0 )
    {
        n_tag = NV_TAG_APP(NVDS_TAG_CERTIFICATE_KEY_SIZE);
        len = sizeof(uint32_t);        
        result = nvds_get(n_tag, &len, (uint8_t *)&data_len);

        if(result != NVDS_SUCCESS){
            GRH_LOG(ERROR, (">>> nvds_get failed for cert length: %d  ", result));
            return CKR_FUNCTION_FAILED;
        }
        
        n_tag = NV_TAG_APP(NVDS_TAG_CERTIFICATE_KEY);        
        tdata = pvPortMalloc(data_len + 1);
        if (tdata == NULL) {
            GRH_LOG(ERROR, (">>> nvds_get failed for cert by malloc fail  "));
            return CKR_HOST_MEMORY;
        }
        *ppucData = tdata;
        memset(tdata, 0 ,data_len + 1);
        
        result = nvds_get(n_tag, (uint16_t*)&data_len, (uint8_t *)tdata);
        if(result != NVDS_SUCCESS){
            GRH_LOG(ERROR, (">>> nvds_get failed for cert data: %d  ", result));
            vPortFree(tdata);
            return CKR_FUNCTION_FAILED;
        }
        
        *pulDataSize = data_len;
    }
    else if( strcmp( pcFileName, pkcs11palFILE_CODE_SIGN_PUBLIC_KEY ) == 0 )
    {
        n_tag = NV_TAG_APP(NVDS_TAG_SIGNING_KEY_SIZE);
        len = sizeof(uint32_t);        
        result = nvds_get(n_tag, &len, (uint8_t *)&data_len);

        if(result != NVDS_SUCCESS){
            GRH_LOG(ERROR, (">>> nvds_get failed for sign length: %d  ", result));
            return CKR_FUNCTION_FAILED;
        }
        
        n_tag = NV_TAG_APP(NVDS_TAG_SIGNING_KEY);        
        tdata = pvPortMalloc(data_len + 1);
        if (tdata == NULL) {
            GRH_LOG(ERROR, (">>> nvds_get failed for sign by malloc fail  "));
            return CKR_HOST_MEMORY;
        }
        *ppucData = tdata;
        memset(tdata, 0 ,data_len + 1);
        
        result = nvds_get(n_tag, (uint16_t*)&data_len, (uint8_t *)tdata);
        if(result != NVDS_SUCCESS){
            GRH_LOG(ERROR, (">>> nvds_get failed for sign data: %d  ", result));
            vPortFree(tdata);
            return CKR_FUNCTION_FAILED;
        }
        
        *pulDataSize = data_len;
    }
            
    return CKR_OK;    
}


/**
* @brief Cleanup after PKCS11_GetObjectValue().
*
* @param[in] pucData       The buffer to free.
*                          (*ppucData from PKCS11_PAL_GetObjectValue())
* @param[in] ulDataSize    The length of the buffer to free.
*                          (*pulDataSize from PKCS11_PAL_GetObjectValue())
*/
void PKCS11_PAL_GetObjectValueCleanup( uint8_t * pucData,
    uint32_t ulDataSize )
{
    (void) ulDataSize;
    
    if(pucData != NULL){
        vPortFree(pucData);
    }
}

/*-----------------------------------------------------------*/

int mbedtls_hardware_poll( void * data,
                           unsigned char * output,
                           size_t len,
                           size_t * olen )
{    
    union
    {
        uint32_t v32;
        uint8_t v4[4];
    }
    suint_32;
    
    ((void)data);

    uint32_t n4Chunks = len / 4;
    uint32_t nLeft = len - n4Chunks * 4;

    while(n4Chunks--)
    {
        suint_32.v32 = getRandomNumber();
        memcpy( output, suint_32.v4, sizeof( suint_32.v4 ) );
        output += sizeof( suint_32.v4 );
    }

    if(nLeft > 0)
    {
        suint_32.v32 = getRandomNumber();
        memcpy(output, suint_32.v4, nLeft);
    }

    *olen = len;

    return 0;
}

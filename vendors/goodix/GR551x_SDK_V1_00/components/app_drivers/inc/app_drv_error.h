/**
 ****************************************************************************************
 *
 * @file    app_drv_error.h
 * @author  BLE Driver Team
 * @brief   Header file of app driver error code.
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
  @addtogroup PERIPHERAL_API_HAL_APP_DRIVER_ERROR HAL APP ERROR CODE Interface
  @{
  @brief Definitions and prototypes for HAL APP DRIVER ERROR CODE.
 */

#ifndef _APP_DRV_ERROR_H_
#define _APP_DRV_ERROR_H_

#include "gr55xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@addtogroup APP_DRV_ERR Defines 
 * @{ 
 */
/**@addtogroup APP_DRV_ERR_CODE App Driver error codes 
 * @{ 
 */
#define APP_DRV_SUCCESS                               0x0000   /**< Successful. */
#define APP_DRV_ERR_HAL                               0x0001   /**< Hal internal error. */
#define APP_DRV_ERR_BUSY                              0x0002   /**< Driver is busy. */
#define APP_DRV_ERR_TIMEOUT                           0x0003   /**< Timeout occurred. */
#define APP_DRV_ERR_INVALID_PARAM                     0x0004   /**< Invalid parameter supplied. */
#define APP_DRV_ERR_POINTER_NULL                      0x0005   /**< Invalid pointer supplied. */
#define APP_DRV_ERR_INVALID_TYPE                      0x0006   /**< Invalid type suplied. */
#define APP_DRV_ERR_INVALID_MODE                      0x0007   /**< Invalid mode suplied. */
#define APP_DRV_ERR_INVALID_ID                        0x0008   /**< Invalid ID suplied. */
/** @} */

#define APP_DRV_ERR_CODE_CHECK(err_code)            \
    do                                              \
    {                                               \
        if (APP_DRV_SUCCESS != err_code)            \
        {                                           \
            return err_code;                        \
        }                                           \
    } while(0)

#define HAL_ERR_CODE_CHECK(err_code)                \
    do                                              \
    {                                               \
        if (HAL_OK != err_code)                     \
        {                                           \
            return (uint16_t)err_code;              \
        }                                           \
    } while(0)

/** @} */
/**
 * @defgroup APP_DRV_ERROR_TYPEDEF Typedefs
 * @{
 */
/**@brief APP driver error type. */
typedef uint16_t   app_drv_err_t;
/**@} */

#ifdef __cplusplus
}
#endif

#endif

/** @} */
/** @} */


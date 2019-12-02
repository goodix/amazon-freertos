/**
  ****************************************************************************************
  * @file    app_pwr_mgmt.c
  * @author  BLE Driver Team
  * @brief   HAL APP module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.
  ****************************************************************************************
  */
/*
 * INCLUDE FILES
 *****************************************************************************************
 */
#include "app_pwr_mgmt.h"
#include "gr55xx_hal.h"
#include "gr55xx_pwr.h"

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static app_sleep_callbacks_t *pwr_sleep_cb[APP_SLEEP_CB_MAX];
static bool is_pwr_callback_reg;

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
pwr_id_t pwr_register_sleep_cb(const app_sleep_callbacks_t *p_cb)
{
    pwr_id_t id = -1;
    uint8_t  i  = 0;

    if (!is_pwr_callback_reg)
    {
        pwr_mgmt_dev_init(pwr_wake_up_ind);
        pwr_mgmt_set_callback(pwr_enter_sleep_check, NULL);
        is_pwr_callback_reg = true;
    }

    if(p_cb != NULL)
    {
        while ((i < APP_SLEEP_CB_MAX) && (pwr_sleep_cb[i] != NULL)) 
        {
            i++;
        }
        if (i < APP_SLEEP_CB_MAX) 
        {
            pwr_sleep_cb[i] = (app_sleep_callbacks_t *)p_cb;
            id = i;
        }

    }

    return id;
}

void pwr_unregister_sleep_cb(pwr_id_t id)
{
    if(id < APP_SLEEP_CB_MAX)// Is id valid?
    {
        pwr_sleep_cb[id] = NULL;
    }
}

void pwr_wake_up_ind(void)
{
    uint8_t i;
    app_sleep_callbacks_t *p_cb;

    for (i = 0; i < APP_SLEEP_CB_MAX; i++) 
    {
        p_cb = pwr_sleep_cb[i];
        if ((p_cb  != NULL) && (p_cb ->app_wake_up_ind != NULL)) 
        {
           p_cb ->app_wake_up_ind();
        }
    }
}

bool pwr_enter_sleep_check(void)
{
    int16_t i;
    bool allow_entering_sleep = true;
    app_sleep_callbacks_t *p_cb;

    // 1. Inquiry Adapters
    for (i = APP_SLEEP_CB_MAX - 1; i >= 0; i--) 
    {
        p_cb = pwr_sleep_cb[i];
        if ((p_cb != NULL) && (p_cb->app_prepare_for_sleep != NULL)) 
        {
            if (!p_cb->app_prepare_for_sleep()) 
            {
                allow_entering_sleep = false;
                break;
            }
        }
    }

    // 2. If an Adapter rejected sleep, resume any Adapters that have already accepted it.
    if(allow_entering_sleep == false)
    {
        i++;
        while (i < APP_SLEEP_CB_MAX) 
        {
            p_cb = pwr_sleep_cb[i];
            if ((p_cb != NULL) && (p_cb->app_sleep_canceled != NULL)) 
            {
                    p_cb->app_sleep_canceled();
            }
            i++;
        }
    }

    return allow_entering_sleep;
}


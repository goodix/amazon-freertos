/**
  ****************************************************************************************
  * @file    app_gpiote.c
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
#include "app_gpiote.h"
#include "app_pwr_mgmt.h"
#include <string.h>

/*
 * DEFINES
 *****************************************************************************************
 */
#define GPIOTE_USE_PATTERN     0x47

/*
 * STRUCT DEFINE
 *****************************************************************************************
 */
struct gpiote_env_t
{
    uint8_t                   total_used_io;
    const app_gpiote_param_t *p_params;
};

/*
 * LOCAL FUNCTION DECLARATION
 *****************************************************************************************
 */
static bool gpiote_prepare_for_sleep(void);
static void gpiote_sleep_canceled(void);
static void gpiote_wake_up_ind(void);

/*
 * LOCAL VARIABLE DEFINITIONS
 *****************************************************************************************
 */
static struct gpiote_env_t s_gpiote_env;

static bool     s_sleep_cb_registered_flag = false;
static pwr_id_t s_gpiote_pwr_id;

static const app_sleep_callbacks_t gpiote_sleep_cb =
{
    .app_prepare_for_sleep = gpiote_prepare_for_sleep,
    .app_sleep_canceled    = gpiote_sleep_canceled,
    .app_wake_up_ind       = gpiote_wake_up_ind
};

/*
 * LOCAL FUNCTION DEFINITIONS
 *****************************************************************************************
 */
static bool gpiote_prepare_for_sleep(void)
{
    return true;
}

static void gpiote_sleep_canceled(void)
{
}

static void gpiote_wake_up_ind(void)
{
    app_io_init_t io_init;

    bool is_ext0_need_enable = false;
    bool is_ext1_need_enable = false;
    bool is_ext2_need_enable = false;

    for (int idx=0; idx<s_gpiote_env.total_used_io; idx ++ )
    { 
        io_init.pin  = s_gpiote_env.p_params[idx].pin; 
        io_init.mode = s_gpiote_env.p_params[idx].mode;
        io_init.pull = s_gpiote_env.p_params[idx].pull; 
        io_init.mux  = APP_IO_MUX_7;

        app_io_init(s_gpiote_env.p_params[idx].type, &io_init);

        if (s_gpiote_env.p_params[idx].type == APP_IO_TYPE_NORMAL)
        {
            if(APP_IO_PINS_0_15 & s_gpiote_env.p_params[idx].pin)
            {
                is_ext0_need_enable = true;
            }
            if(APP_IO_PINS_16_31 & s_gpiote_env.p_params[idx].pin)
            {
                is_ext1_need_enable = true;
            }
        }
        else if (s_gpiote_env.p_params[idx].type == APP_IO_TYPE_AON)
        {
                is_ext2_need_enable = true;
        }
    }

    if (is_ext0_need_enable)
    {
        hal_nvic_clear_pending_irq(EXT0_IRQn);
        hal_nvic_enable_irq(EXT0_IRQn);
    }

    if (is_ext1_need_enable)
    {
        hal_nvic_clear_pending_irq(EXT1_IRQn);
        hal_nvic_enable_irq(EXT1_IRQn);
    }

    if (is_ext2_need_enable)
    {
        hal_nvic_clear_pending_irq(EXT2_IRQn);
        hal_nvic_enable_irq(EXT2_IRQn);
    }
}

/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
uint16_t app_gpiote_init(const app_gpiote_param_t *p_params, uint8_t table_cnt)
{
    app_drv_err_t err_code;

    if (NULL == p_params)
    {
        return APP_DRV_ERR_POINTER_NULL;
    }
    
    s_gpiote_env.p_params      = p_params;
    s_gpiote_env.total_used_io = table_cnt;
    
    app_io_init_t io_init;

    for (int idx=0; idx<table_cnt; idx ++ )
    { 
        io_init.pin  = p_params[idx].pin; 
        io_init.mode = p_params[idx].mode;
        io_init.pull = p_params[idx].pull; 
        io_init.mux  = APP_IO_MUX_7;
        err_code = app_io_init(p_params[idx].type, &io_init);
        APP_DRV_ERR_CODE_CHECK(err_code);

        if ((s_gpiote_env.p_params[idx].handle_mode == APP_IO_ENABLE_WAKEUP) && \
            (s_gpiote_env.p_params[idx].type == APP_IO_TYPE_AON))
        {
           switch (s_gpiote_env.p_params[idx].mode)
           {
             case APP_IO_MODE_IT_RISING:
                  hal_pwr_config_ext_wakeup(s_gpiote_env.p_params[idx].pin, PWR_EXTWKUP_TYPE_RISING);
                  break;

             case APP_IO_MODE_IT_FALLING:
                  hal_pwr_config_ext_wakeup(s_gpiote_env.p_params[idx].pin, PWR_EXTWKUP_TYPE_FALLING);
                  break;

             case APP_IO_MODE_IT_HIGH:
                  hal_pwr_config_ext_wakeup(s_gpiote_env.p_params[idx].pin, PWR_EXTWKUP_TYPE_HIGH);
                  break;

             case APP_IO_MODE_IT_LOW:
                  hal_pwr_config_ext_wakeup(s_gpiote_env.p_params[idx].pin, PWR_EXTWKUP_TYPE_LOW);
                  break;

             default:
                 break;
           }
           pwr_mgmt_wakeup_source_setup(PWR_WKUP_COND_EXT);
        }
        
        if (p_params[idx].type == APP_IO_TYPE_NORMAL)
        {
            hal_nvic_clear_pending_irq(EXT0_IRQn);
            hal_nvic_enable_irq(EXT0_IRQn);
            hal_nvic_clear_pending_irq(EXT1_IRQn);
            hal_nvic_enable_irq(EXT1_IRQn);
        }
        else if (p_params[idx].type == APP_IO_TYPE_AON)
        {
            hal_nvic_clear_pending_irq(EXT2_IRQn);
            hal_nvic_enable_irq(EXT2_IRQn);
        }
    }

    if (!s_sleep_cb_registered_flag ) // register sleep callback
    {
        s_gpiote_pwr_id = pwr_register_sleep_cb(&gpiote_sleep_cb);
        if (s_gpiote_pwr_id < 0)
        {
            return APP_DRV_ERR_INVALID_PARAM;
        }
        s_sleep_cb_registered_flag = true;
    }
 
    return APP_DRV_SUCCESS;
}

void app_gpiote_deinit(void)
{
    for (int idx=0; idx<s_gpiote_env.total_used_io; idx ++ )
    { 
        app_io_deinit(s_gpiote_env.p_params[idx].type, s_gpiote_env.p_params[idx].pin);
    }
    hal_nvic_disable_irq(EXT0_IRQn);
    hal_nvic_disable_irq(EXT1_IRQn);
    hal_nvic_disable_irq(EXT2_IRQn);
    pwr_unregister_sleep_cb(s_gpiote_pwr_id);
}

void hal_gpio_exti_callback(gpio_regs_t *GPIOx, uint16_t gpio_pin)
{
    uint32_t io_pin = gpio_pin;
    app_gpiote_evt_t gpiote_evt;

    if (GPIO1 == GPIOx)
    {
        io_pin = (uint32_t)(gpio_pin << 16);
    }

    gpiote_evt.type = APP_IO_TYPE_NORMAL;
    gpiote_evt.pin = io_pin;
    gpiote_evt.ctx_type = APP_IO_CTX_INT;

    for (uint8_t idx=0; idx<s_gpiote_env.total_used_io; idx ++)
    {
        if ((s_gpiote_env.p_params[idx].type == APP_IO_TYPE_NORMAL) && (io_pin == s_gpiote_env.p_params[idx].pin))
        {
            if (s_gpiote_env.p_params[idx].io_evt_cb)
                s_gpiote_env.p_params[idx].io_evt_cb(&gpiote_evt);
        }
    }
}

void hal_aon_gpio_callback(uint16_t aon_gpio_pin)
{
    app_gpiote_evt_t gpiote_evt;

    gpiote_evt.type = APP_IO_TYPE_AON;
    gpiote_evt.pin = aon_gpio_pin;
    
    if (pwr_mgmt_get_wakeup_flag() == WARM_BOOT)
    {
        gpiote_evt.ctx_type = APP_IO_CTX_WAKEUP;
    }
    else
    {
        gpiote_evt.ctx_type = APP_IO_CTX_INT;
    }
   
    for (uint8_t idx=0; idx<s_gpiote_env.total_used_io; idx ++)
    {
        if ((s_gpiote_env.p_params[idx].type == APP_IO_TYPE_AON) && \
            (aon_gpio_pin & s_gpiote_env.p_params[idx].pin) && \
            (s_gpiote_env.p_params[idx].io_evt_cb))
        {
            if (gpiote_evt.ctx_type == APP_IO_CTX_WAKEUP)
            {
                if (s_gpiote_env.p_params[idx].handle_mode == APP_IO_ENABLE_WAKEUP)  
                {
                    s_gpiote_env.p_params[idx].io_evt_cb(&gpiote_evt);
                }    
            }
            else
            {
                s_gpiote_env.p_params[idx].io_evt_cb(&gpiote_evt);
            }
        }
    }
}

void EXT0_IRQHandler(void)
{
    hal_gpio_exti_irq_handler(GPIO0);
}

void EXT1_IRQHandler(void)
{
    hal_gpio_exti_irq_handler(GPIO1);
}

void EXT2_IRQHandler(void)
{
    hal_aon_gpio_irq_handler();
}


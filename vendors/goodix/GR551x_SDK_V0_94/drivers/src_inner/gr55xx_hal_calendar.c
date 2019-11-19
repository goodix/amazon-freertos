/**
  ****************************************************************************************
  * @file    gr55xx_hal_calendar.c
  * @author  BLE Driver Team
  * @brief   CALENDAR HAL module driver.
  ****************************************************************************************
  * @attention
  #####Copyright (c) 2019 GOODIX
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of GOODIX nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
  ****************************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "gr55xx_hal.h"

/** @addtogroup HAL_DRIVER
  * @{
  */

#ifdef HAL_CALENDAR_MODULE_ENABLED
/** @addtogroup CALENDAR CALENDAR
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define CALENDAR_FIRST_YEAR             (2000UL)
#define CALENDAR_FIRST_MONTH            (1UL)
#define CALENDAR_FIRST_DATE             (1UL)
#define CALENDAR_FIRST_WEEK             (6UL)
#define CALENDAR_SECONDS_PER_HOUR       (3600UL)
#define CALENDAR_SECONDS_PER_DAY        (24UL * CALENDAR_SECONDS_PER_HOUR)
#define CALENDAR_SECONDS_PER_WEEK       (7UL * CALENDAR_SECONDS_PER_DAY)
#define CALENDAR_SECONDS_PER_YEAR       (365UL * CALENDAR_SECONDS_PER_DAY)
#define CALENDAR_DAYS_PER_MONTH         {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}

#define CALENDAR_TICKS_PER_SECOND       (32768UL)
#define CALENDAR_TICKS_PER_DAY          ((CALENDAR_SECONDS_PER_DAY) * (CALENDAR_TICKS_PER_SECOND))
#define CALENDAR_TICKS_PER_WARP         (0x100000000UL)
#define CALENDAR_SECONDS_PER_WARP       ((CALENDAR_TICKS_PER_WARP) / (CALENDAR_TICKS_PER_SECOND))
#define CALENDAR_REMAIN_PER_WARP        ((CALENDAR_TICKS_PER_WARP) % (CALENDAR_TICKS_PER_SECOND))
#define CALENDAR_WARPCNT_MAX            (0xFFUL)
#define CALENDAR_SECONDS_WARPCNT_OVER   ((CALENDAR_WARPCNT_MAX + 1) * (CALENDAR_SECONDS_PER_WARP))
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static hal_status_t calendar_get_timer_value(uint32_t *p_value);
static void calendar_cover_time2days(calendar_time_t *p_time, uint32_t *p_days);
static void calendar_cover_time2seconds(calendar_time_t *p_time, uint32_t *p_seconds);
static void calendar_cover_seconds2time(calendar_time_t *p_time, uint32_t seconds);
/* Exported functions --------------------------------------------------------*/

/** @defgroup CALENDAR_Exported_Functions CALENDAR Exported Functions
  * @{
  */

/** @defgroup CALENDAR_Exported_Functions_Group1 Initialization and Configuration functions
 *  @brief    Initialization and Configuration functions.
  * @{
  */

__WEAK hal_status_t hal_calendar_init(calendar_handle_t *p_calendar)
{
    hal_status_t status     = HAL_OK;
    uint32_t     wait_count = 1000;

    /* Check the CALENDAR handle allocation */
    if (NULL == p_calendar)
    {
        return HAL_ERROR;
    }

    /* Allocate lock resource and initialize it */
    p_calendar->lock = HAL_UNLOCKED;

    do {
        /* Disable CALENDAR */
        __HAL_CALENDAR_DISABLE();

        /* Select clock div */
        ll_calendar_set_clock_div(LL_CALENDAR_DIV_NONE);

        /* Load reload counter value into CALENDAR */
        ll_calendar_reload_counter(0);
        wait_count = 1000;
        do {
            if (ll_calendar_get_counter() == 0)
                break;
        } while(--wait_count);

        if (0 == wait_count)
        {
            status = HAL_ERROR;
            break;
        }

        /* Initialization time, 01.01.2000 00:00:00 */
        p_calendar->time_init.sec  = 0;
        p_calendar->time_init.min  = 0;
        p_calendar->time_init.hour = 0;
        p_calendar->time_init.date = CALENDAR_FIRST_DATE;
        p_calendar->time_init.mon  = CALENDAR_FIRST_MONTH;
        p_calendar->time_init.year = 0;
        p_calendar->time_init.week = CALENDAR_FIRST_WEEK;
        p_calendar->time_init.ms   = 0;

        /* Clear warp interrupt flag */
        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_WARP);
        /* Enable warp interrupt */
        __HAL_CALENDAR_ENABLE_IT(CALENDAR_IT_WARP);
        __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_ALARM);

        /* Clear pending IRQ and eable NVIC interrupt */
        NVIC_ClearPendingIRQ(CALENDAR_IRQn);
        NVIC_EnableIRQ(CALENDAR_IRQn);

        /* Enable CALENDAR */
        __HAL_CALENDAR_ENABLE();
    } while(0);

    return status;
}

__WEAK hal_status_t hal_calendar_deinit(calendar_handle_t *p_calendar)
{
    /* Check the CALENDAR handle allocation */
    if (NULL == p_calendar)
    {
        return HAL_ERROR;
    }

    /* Disable CALENDAR */
    __HAL_CALENDAR_DISABLE();

    /* Diseable NVIC interrupt and Clear pending IRQ */
    NVIC_DisableIRQ(CALENDAR_IRQn);
    NVIC_ClearPendingIRQ(CALENDAR_IRQn);

    /* Disable warp and alarm interrupt */
    __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_WARP | CALENDAR_IT_ALARM);

    /* Clear clock div */
    ll_calendar_set_clock_div(LL_CALENDAR_NO_CLOCK);

    /* Process Unlock */
    __HAL_UNLOCK(p_calendar);

    return HAL_OK;
}

/** @} */

/** @defgroup CALENDAR_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions
  * @{
  */

__WEAK hal_status_t hal_calendar_init_time(calendar_handle_t *p_calendar, calendar_time_t *p_time)
{
    hal_status_t status     = HAL_OK;
    uint32_t     wait_count = 1000;

    if (99 < p_time->year)
        return HAL_ERROR;

    /* Process Locked */
    __HAL_LOCK(p_calendar);

    do {
        /* Load reload counter value into CALENDAR */
        ll_calendar_reload_counter(0);
        do {
            if (0 == ll_calendar_get_counter())
                break;
        } while(--wait_count);

        if (0 == wait_count)
        {
            status = HAL_ERROR;
            break;
        }

        memcpy(&p_calendar->time_init, p_time, sizeof(calendar_time_t));
        /* Update weekday */
        uint32_t day;
        calendar_cover_time2days(&p_calendar->time_init, &day);
        p_calendar->time_init.week = (CALENDAR_FIRST_WEEK + day) % 7;
    } while(0);

    /* Process Unlock */
    __HAL_UNLOCK(p_calendar);

    return status;
}

__WEAK hal_status_t hal_calendar_get_time(calendar_handle_t *p_calendar, calendar_time_t *p_time)
{
    hal_status_t status = HAL_OK;
    uint32_t     timer_value, warp_cnt;
    uint32_t     dif_sec, millisec;

    /* Process Locked */
    __HAL_LOCK(p_calendar);

    do {
        /* Read counter value */
        status = calendar_get_timer_value(&timer_value);
        if (HAL_OK != status)
            break;

        /* Read warp counter value */
        warp_cnt = ll_calendar_get_warpcnt();
        /* Caculate seconds from 01.01.2000 00:00 to time_init */
        calendar_cover_time2seconds(&p_calendar->time_init, &dif_sec);
        /* Caculate seconds from time_init */
        dif_sec += warp_cnt * CALENDAR_SECONDS_PER_WARP;
        #if CALENDAR_REMAIN_PER_WARP != 0
        dif_sec += ((uint64_t)warp_cnt * CALENDAR_REMAIN_PER_WARP + (uint64_t)timer_value) / CALENDAR_TICKS_PER_SECOND;
        millisec = ((uint64_t)warp_cnt * CALENDAR_REMAIN_PER_WARP + (uint64_t)timer_value) % CALENDAR_TICKS_PER_SECOND * 1000 / CALENDAR_TICKS_PER_SECOND;
        #else
        dif_sec += timer_value / CALENDAR_TICKS_PER_SECOND;
        millisec = timer_value % CALENDAR_TICKS_PER_SECOND * 1000 / CALENDAR_TICKS_PER_SECOND;
        #endif
        calendar_cover_seconds2time(p_time, dif_sec);
        p_time->ms = millisec;
    } while(0);

    /* Process Unlock */
    __HAL_UNLOCK(p_calendar);

    return status;
}

__WEAK hal_status_t hal_calendar_set_alarm(calendar_handle_t *p_calendar, calendar_alarm_t *p_alarm)
{
    hal_status_t status = HAL_OK;
    calendar_time_t curr_time;
    uint32_t curr_sec, alarm_sec;
    uint32_t curr_value, alarm_value;

    gr_assert_param(IS_CALENDAR_ALARM_TYPE(p_alarm->alarm_sel));
    if (CALENDAR_ALARM_SEL_WEEKDAY == p_alarm->alarm_sel)
        gr_assert_param(((p_alarm->alarm_date_week_mask & 0x7F) != 0));
    else
        gr_assert_param(IS_CALENDAR_DATE(p_alarm->alarm_date_week_mask));

    hal_calendar_get_time(p_calendar, &curr_time);
    calendar_cover_time2seconds(&curr_time, &curr_sec);

    /* Process Locked */
    __HAL_LOCK(p_calendar);

    do {
        curr_time.hour = p_alarm->hour;
        curr_time.min  = p_alarm->min;
        curr_time.sec  = 0;
        calendar_cover_time2seconds(&curr_time, &alarm_sec);

        /* If alarm time is befor the present time, or within 3s after. */
        if (!(alarm_sec > curr_sec && (alarm_sec - curr_sec) > 3))
        {
            alarm_sec += CALENDAR_SECONDS_PER_DAY;
        }

        memcpy(&p_calendar->alarm, p_alarm, sizeof(calendar_alarm_t));

        /* Caculate alarm value */
        calendar_get_timer_value(&curr_value);
        alarm_value = curr_value + (alarm_sec - curr_sec) * CALENDAR_TICKS_PER_SECOND;
        /* Load alarm value into CALENDAR */
        ll_calendar_reload_alarm(alarm_value);

        /* Clear alarm interrupt flag */
        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_ALARM);
        /* Enable alarm interrupt */
        __HAL_CALENDAR_ENABLE_IT(CALENDAR_IT_ALARM);
    } while(0);

    /* Process Unlock */
    __HAL_UNLOCK(p_calendar);

    return status;
}

__WEAK hal_status_t hal_calendar_disable_alarm(calendar_handle_t *p_calendar)
{
    __HAL_CALENDAR_DISABLE_IT(CALENDAR_IT_ALARM);

    return HAL_OK;
}

__WEAK void calendar_irq_handler(calendar_handle_t *p_calendar)
{
    if (__HAL_CALENDAR_GET_IT_SOURCE(CALENDAR_FLAG_ALARM))
    {
        calendar_time_t curr_time;

        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_ALARM);

        hal_calendar_get_time(p_calendar, &curr_time);
        if (CALENDAR_ALARM_SEL_WEEKDAY == p_calendar->alarm.alarm_sel)
        {
            if (p_calendar->alarm.alarm_date_week_mask & (1UL << curr_time.week))
            {
                /* Alarm callback  */
                hal_calendar_alarm_callback(p_calendar);
            }
        }
        else
        {
            if (p_calendar->alarm.alarm_date_week_mask == curr_time.date)
            {
                /* Alarm callback  */
                hal_calendar_alarm_callback(p_calendar);
            }
        }
        /* Set alarm after 24h */
        uint32_t alarm_value = ll_calendar_get_alarm();
        alarm_value += CALENDAR_TICKS_PER_DAY;
        /* Load alarm value into CALENDAR */
        ll_calendar_reload_alarm(alarm_value);
    }

    if (__HAL_CALENDAR_GET_IT_SOURCE(CALENDAR_FLAG_WARP))
    {
        static uint8_t last_warp = 0;
        uint8_t        curr_warp = ll_calendar_get_warpcnt();

        __HAL_CALENDAR_CLEAR_FLAG(CALENDAR_FLAG_WARP);

        if ((0 == curr_warp) && (CALENDAR_WARPCNT_MAX == last_warp))
        {
            uint32_t dif_sec;
            calendar_cover_time2seconds(&p_calendar->time_init, &dif_sec);
            /* Caculate seconds from time_init */
            dif_sec += CALENDAR_SECONDS_WARPCNT_OVER;
            calendar_cover_seconds2time(&p_calendar->time_init, dif_sec);
        }
        last_warp = curr_warp;
    }
}

__WEAK void hal_calendar_alarm_callback(calendar_handle_t *p_calendar)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(p_calendar);

    /* NOTE: This function should not be modified, when the callback is needed,
             the hal_calendar_alarm_callback could be implemented in the user file
    */
}

/** @} */

/** @} */

static hal_status_t calendar_get_timer_value(uint32_t *p_value)
{
    uint32_t last_value, curr_value;
    uint32_t wait_count = 1000;

    /* Read counter value */
    last_value = ll_calendar_get_counter();
    do {
        curr_value = ll_calendar_get_counter();
        if (curr_value == last_value)
            break;
        last_value = curr_value;
    } while(--wait_count);

    if (0 == wait_count)
    {
        *p_value = 0;
        return HAL_ERROR;
    }
    else
    {
        *p_value = curr_value;
        return HAL_OK;
    }
}

/* Caculate days from 01.01.2000 00:00 to the time */
static void calendar_cover_time2days(calendar_time_t *p_time, uint32_t *p_days)
{
    uint32_t leap_year    = (p_time->year > 0) ? (p_time->year - 1) : 0;
    uint32_t mon_days[12] = CALENDAR_DAYS_PER_MONTH;

    if (0 < p_time->year)
        leap_year = leap_year / 4 - leap_year / 100 + leap_year / 400 + 1;

    *p_days = p_time->year * 365 + leap_year;

    mon_days[1] = IS_CALENDAR_LEAP_YEAR((uint32_t)p_time->year + CALENDAR_FIRST_YEAR) ? 29 : 28;
    for (uint32_t i = 0; i < p_time->mon - 1; i++)
    {
        *p_days += mon_days[i];
    }

    *p_days += p_time->date - 1;
}

/* Caculate seconds from 01.01.2000 00:00 to the time */
static void calendar_cover_time2seconds(calendar_time_t *p_time, uint32_t *p_seconds)
{
    uint32_t leap_year    = (p_time->year > 0) ? (p_time->year - 1) : 0;
    uint32_t mon_days[12] = CALENDAR_DAYS_PER_MONTH;
    uint32_t i;

    if (0 < p_time->year)
        leap_year = leap_year / 4 - leap_year / 100 + leap_year / 400 + 1;

    *p_seconds = p_time->year * CALENDAR_SECONDS_PER_YEAR + leap_year * CALENDAR_SECONDS_PER_DAY;

    mon_days[1] = IS_CALENDAR_LEAP_YEAR((uint32_t)p_time->year + CALENDAR_FIRST_YEAR) ? 29 : 28;
    for (i = 0; i < p_time->mon - 1; i++)
    {
        *p_seconds += mon_days[i] * CALENDAR_SECONDS_PER_DAY;
    }

    *p_seconds += (uint32_t)(p_time->date - 1) * CALENDAR_SECONDS_PER_DAY;

    *p_seconds += (uint32_t)p_time->hour * CALENDAR_SECONDS_PER_HOUR;

    *p_seconds += (uint32_t)p_time->min * 60 + (uint32_t)p_time->sec;
}

static void calendar_cover_seconds2time(calendar_time_t *p_time, uint32_t seconds)
{
    uint32_t mon_sec;
    uint32_t mon_days[12] = CALENDAR_DAYS_PER_MONTH;

    p_time->sec  = 0;
    p_time->min  = 0;
    p_time->hour = 0;
    p_time->date = CALENDAR_FIRST_DATE;
    p_time->mon  = CALENDAR_FIRST_MONTH;
    p_time->year = 0;
    p_time->week = CALENDAR_FIRST_WEEK;

    while (CALENDAR_SECONDS_PER_YEAR <= seconds)
    {
        if (IS_CALENDAR_LEAP_YEAR((uint32_t)p_time->year + CALENDAR_FIRST_YEAR))
        {
            if (seconds >= (CALENDAR_SECONDS_PER_YEAR + CALENDAR_SECONDS_PER_DAY))
            {
                seconds -= (CALENDAR_SECONDS_PER_YEAR + CALENDAR_SECONDS_PER_DAY);
                p_time->year++;
                p_time->week = (p_time->week + 2) % 7;
            }
            else
            {
                break;
            }
        }
        else
        {
            seconds -= CALENDAR_SECONDS_PER_YEAR;
            p_time->year++;
            p_time->week = (p_time->week + 1) % 7;
        }
    }

    mon_days[1] = IS_CALENDAR_LEAP_YEAR((uint32_t)p_time->year + CALENDAR_FIRST_YEAR) ? 29 : 28;
    for (uint32_t i = 0; i < 12; i++)
    {
        mon_sec = mon_days[i] * CALENDAR_SECONDS_PER_DAY;
        if (seconds >= mon_sec)
        {
            seconds -= mon_sec;
            p_time->mon++;
            p_time->week = (p_time->week + mon_days[i]) % 7;
        }
        else
        {
            break;
        }
    }

    p_time->date += seconds / CALENDAR_SECONDS_PER_DAY;
    p_time->week  = (p_time->week + p_time->date - 1) % 7;
    seconds -= (uint32_t)(p_time->date - 1) * CALENDAR_SECONDS_PER_DAY;
    p_time->hour += seconds / CALENDAR_SECONDS_PER_HOUR;
    seconds -= (uint32_t)p_time->hour * CALENDAR_SECONDS_PER_HOUR;
    p_time->min += seconds / 60;
    seconds -= (uint32_t)p_time->min * 60;
    p_time->sec += seconds;
}

#endif /* HAL_CALENDAR_MODULE_ENABLED */
/** @} */

/** @} */

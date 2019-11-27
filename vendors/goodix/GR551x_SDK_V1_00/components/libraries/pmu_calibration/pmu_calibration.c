
#include "platform_sdk.h" 
#include "app_timer.h"
#include <stdio.h>

static app_timer_id_t s_pmu_calibration_timer_id = 0;

void system_pmu_calibration_start(uint32_t interval)
{
    if(interval)
    {
        app_timer_delete(&s_pmu_calibration_timer_id);
        app_timer_create(&s_pmu_calibration_timer_id, ATIMER_REPEAT, pmu_calibration_handler);
        app_timer_start(s_pmu_calibration_timer_id, interval, NULL);
        pmu_calibration_handler(NULL);
    }
    return;
}

void system_pmu_calibration_stop(void)
{
    app_timer_delete(&s_pmu_calibration_timer_id);
    return;
}

#ifndef _PMU_CALIB_H
#define _PMU_CALIB_H

#include <stdint.h>

void system_pmu_calibration_stop(void);

void system_pmu_calibration_start(uint32_t interval);

#endif

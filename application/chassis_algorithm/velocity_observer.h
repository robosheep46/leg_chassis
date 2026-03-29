#ifndef VELOCITY_OBSERVER_H
#define VELOCITY_OBSERVER_H

#include "chassis.h"
#include "kalman_filter.h"
#include "bmi088.h"

void SpeedObserver(LegParam *lp, LegParam *rp, ChassisParam *cp, imu_data_t *imu, float delta_t);

#endif
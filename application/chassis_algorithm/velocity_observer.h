#ifndef VELOCITY_OBSERVER_H
#define VELOCITY_OBSERVER_H

#include "chassis.h"
#include "kalman_filter.h"
#include "ins_task.h"

void SpeedObserver(LegParam *lp, LegParam *rp, ChassisParam *cp, attitude_t *imu, float delta_t);

#endif
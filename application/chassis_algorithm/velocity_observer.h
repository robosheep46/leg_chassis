#ifndef VELOCITY_OBSERVER_H
#define VELOCITY_OBSERVER_H

#include "chassis.h"
#include "kalman_filter.h"
#include "ins_task.h"

void observe_speed(LegParam *lp, LegParam *rp, ChassisParam *cp, attitude_t *imu, float delta_t);
void observe_speed_init(ChassisParam *cp);

#endif
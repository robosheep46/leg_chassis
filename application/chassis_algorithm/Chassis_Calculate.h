#ifndef Chassis_Calculate_h
#define Chassis_Calculate_h
#include "chassis.h"
#include "ins_task.h"

void phi_transform_theta(LegParam * leg,ChassisParam *chassis);
void calculate_leg_theta_w(LegParam * leg,ChassisParam *chassis);
void set_left_leg_six_states(LegParam *leg, ChassisParam *chassis);
void set_right_leg_six_states(LegParam *leg, ChassisParam *chassis);


void calculate_leg_torgue(LegParam * leg);
void calculate_wheel_torgue(LegParam *l_leg,LegParam *r_leg, ChassisParam *chassis);


void calculate_support_force(LegParam *leg, attitude_t *imu);

#endif
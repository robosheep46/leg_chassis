#ifndef Chassis_Calculate_h
#define Chassis_Calculate_h
#include "chassis.h"
#include "bmi088.h"

void TranLeg(LegParam * leg,ChassisParam *chassis);

void VMC(LegParam * leg);


void SupportForceSolve(LegParam *leg, imu_data_t *imu);
void CalcThetaw(LegParam * leg,ChassisParam *chassis);
void CalLeftT_Hip(LegParam *leg, ChassisParam *chassis);
void CalLeftT_Wheel(LegParam *leg, ChassisParam *chassis);

void CalRightT_Hip(LegParam *leg, ChassisParam *chassis);
void CalRightT_Wheel(LegParam *leg, ChassisParam *chassis);

#endif
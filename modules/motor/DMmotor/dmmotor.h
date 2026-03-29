#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
#define DM_KP_MIN 0
#define DM_KP_MAX 500.0f
#define DM_KD_MIN 0
#define DM_KD_MAX 5.0f
#define ECD_ANGLE_COEF_DM 360/24 // (360/8192),将编码器值转化为角度制

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float last_torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
    float total_angle;

    float real_total_angle;
    int32_t real_total_round;
    float real_angle_single_round;

    float angle_single_round;
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;
typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;    // 电机控制器
    uint8_t mit_flag;
    uint8_t set_stop_mode_flag;
    uint8_t set_run_mode_flag;
    Motor_Type_e motor_type;        // 电机类型

    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
    uint8_t sender_group;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetPosition(DMMotorInstance *motor, float position);
void DMMotorSetSpeed(DMMotorInstance *motor, float Speed);
void DMMotorSetFFTorque(DMMotorInstance *motor, float Torque);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);
void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
#endif // !DMMOTOR
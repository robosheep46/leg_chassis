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
#define ECD_ANGLE_COEF_DM 360/24

typedef struct {
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
} DM_Motor_Measure_s;

typedef enum {
    DM_CMD_MOTOR_MODE   = 0xfc,
    DM_CMD_RESET_MODE   = 0xfd,
    DM_CMD_ZERO_POSITION = 0xfe,
    DM_CMD_CLEAR_ERROR  = 0xfb,
} dm_motor_mit_mode_e;

typedef enum {
    MIT_MODE      = 1,
    POSITION_MODE = 2
} dm_motor_control_mode_e;

// 模式切换状态机
typedef enum {
    SWITCH_IDLE = 0,
    SWITCH_SEND_CMD,
    SWITCH_WAIT_SEND,
    SWITCH_READ_REG,
    SWITCH_WAIT_CONFIRM
} MotorSwitchState_t;

// 初始化状态机
typedef enum {
    INIT_IDLE = 0,
    INIT_SEND_CMD,
    INIT_WAIT
} MotorInitState_t;

// 校准状态机
typedef enum {
    CALI_IDLE = 0,
    CALI_ENABLE_1,
    CALI_WAIT_1,
    CALI_SEND_CMD,
    CALI_WAIT_2,
    CALI_ENABLE_2
} MotorCaliState_t;

typedef struct {
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;
    Motor_Type_e motor_type;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance *motor_daemon;
    uint8_t sender_group;
    uint8_t init_flag;
    uint8_t other_error_flag;
    uint8_t position_mode_flag;
    uint8_t mit_mode_flag;
    uint8_t read_control_mode;
    dm_motor_control_mode_e control_mode;

    // 模式切换状态机
    MotorSwitchState_t switch_state;
    uint32_t switch_timestamp;
    uint8_t send_count;          // 已发送切换指令的次数 (0~2)
    uint8_t switch_retry_cnt;    // 超时重试计数
    uint8_t switch_target_mode;

    // 初始化状态机
    MotorInitState_t init_state;
    uint32_t init_timestamp;

    // 校准状态机
    MotorCaliState_t cali_state;
    uint32_t cali_timestamp;
} DMMotorInstance;

// 公共接口
DMMotorInstance *dmmotor_init(Motor_Init_Config_s *config);
void dmmotor_set_position(DMMotorInstance *motor, float position, float speed);
void dmmotor_set_torque(DMMotorInstance *motor, float Torque);
void dmmotor_enable(DMMotorInstance *motor);
void dmmotor_stop(DMMotorInstance *motor);
void dmmotor_cali_encoder(DMMotorInstance *motor);
void dmmotor_task_init(void);
void dmmotor_set_control_mode(DMMotorInstance *motor, dm_motor_control_mode_e mode);

#endif // DMMOTOR_H
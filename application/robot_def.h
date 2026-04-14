#pragma once // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#include "robot_def.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

// #include "master_process.h"
#include "stdint.h"
#include "cmsis_os.h"
#include "queue.h"
#include "ins_task.h"
/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define CHASSIS_BOARD_CONTROL_CHASSIS // 只控底盘
// #define CHASSIS_BOARD // 
// #define GIMBAL_BOARD // 
#define TESTCODE
// #define MATCH_CODE
/* 陀螺仪类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define BOARD_IMU
//#define DM_IMU
//#define ZF_IMU


/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_ALIGN_ANGLE 0  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改

// 机器人底盘修改的参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 电机减速比,因为编码器量测的是转子的速度而不是输出轴的速度故需进行转换

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define REDUCTION_RATIO_LOADER 36.0f





#define Rside_FInitAngle 111
#define Rside_BInitAngle 69

#define PITCH_MIN_ANGLE -20
#define PITCH_MAX_ANGLE 20








#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */


typedef struct {
    QueueHandle_t control_chassis_queue;
    QueueHandle_t control_gimbal_queue;
    QueueHandle_t gimbal_fetch_queue;
    QueueHandle_t chassis_fetch_queue;
    QueueHandle_t control_shoot_queue;
    QueueHandle_t shoot_fetch_queue;
} RobotCtrlQueues_t;

typedef struct 
{
    QueueHandle_t chassis_feedback_queue;
    QueueHandle_t chassis_recv_queue;
} ChassisQueues_t;

typedef struct 
{
    QueueHandle_t gimbal_feedback_queue;
    QueueHandle_t gimbal_recv_queue;
} GimbalQueues_t;

typedef struct 
{
    QueueHandle_t shoot_feedback_queue;
    QueueHandle_t shoot_recv_queue;
} ShootQueues_t;




// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,    // 电流零输入
    GIMBAL_GYRO_MODE ,
} gimbal_mode_e;

// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_STAND_UP,
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_CONTROL_WHEEL,     // 轮式控制模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式
} chassis_mode_e;


// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

typedef enum
{
    FRICTION_OFF = 0,
    FRICTION_ON,
} friction_mode_e;

typedef enum
{
    LOADER_STOP = 0,  // 停止发射
    LOADER_REVERSE,   // 反转
    LOADER_1_BULLET,  // 单发
    LOADER_BURSTFIRE, // 连发
} loader_mode_e;












/* ----------------用于记录时间或标志位的结构体---------------- */
typedef struct
{
    float T_Vision; 
    uint8_t aim_flag;
    uint8_t cmd_error_flag;
    uint8_t fire_flag;
    uint8_t reverse_flag;
}DataLebel_t;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;    
    float l_target_len;
    float r_target_len;
    float fly_flag;

    float change_length;
    float turn_position;
    float motor_rf;
    float motor_rb;
    float motor_lf;
    float motor_lb;
} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{
    float yaw;
    float pitch;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{
    friction_mode_e friction_mode;
    loader_mode_e loader_mode;

    shoot_mode_e shoot_mode;
    uint8_t shoot_rate;

} Shoot_Ctrl_Cmd_s;
/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
typedef enum
{
    STAGE_TORQUE_ZERO = 0,      // 力矩清零阶段
    STAGE_POSITION_CTRL = 1,    // 位置控制阶段
    STAGE_CHANGE_LENGTH = 2,    // 停止保持阶段
    STAGE_NORMAL_BALANCE = 3    // 正常平衡阶段
} stand_up_stage_e;




/* ----------------麦轮解算/电机中间量等数据----------------*/
typedef struct
{
    float yaw_angle;
    float yaw_motor_single_round_angle;
} Gimbal_Upload_Data_s;




typedef struct
{
    stand_up_stage_e stand_up_stage;
} Chassis_Upload_Data_s;

typedef struct
{
    uint8_t init_flag;
    uint8_t init_flag_last;
    uint8_t init_flag_now;
    uint8_t keep_flag;
}Arm_Flag_s;


typedef enum
{
    REMOTE_CONTROL = 0,
    CUSTOM_CONTROL,
    AUTO_CONTROL,
}Control_arm_mode_e;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float yaw_total_angle;
    float gyro[3];
    float dgyro[3];
    float MotionAccel_b[3];
}imu_data_t;

#pragma pack() // 开启字节对齐,结束前面的#pragma pack(1)

#endif // !ROBOT_DEF_H 
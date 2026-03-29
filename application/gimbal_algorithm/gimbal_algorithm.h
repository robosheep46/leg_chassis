#include <stdbool.h>
#include "bmi088.h"
#include "robot_def.h"
#include "dji_motor.h"

// 云台算法状态结构体
typedef struct 
{
    // 状态变量
    float yaw_cmd;                         // yaw指令
    float yaw_last_cmd;                    // 上一次yaw指令
    float yaw_cmd_delta;                   // 指令变化量
    float yaw_filtered_cmd;                // 滤波后的yaw指令
    float current_yaw;                     // 现在的yaw
    float yaw_error;                       // 现在与指令差值
    float yaw_error_rate;
    float yaw_static_error_accumulator;    // 静态误差累积
    float yaw_cmd_rate;                    // 指令变化率
    float current_time;
    
    // 方向检测相关
    bool  direction_changed;               // 方向变化标志
    float last_cmd_delta;                  // 上一次指令变化量
    float last_direction_change_time;      // 上次方向变化时间
    
    // 控制参数
    float filter_factor;                   // 当前滤波系数
    float feedforward;                     // 当前前馈值
} GimbalAlgorithm_t;

float Cal_FollowControl_Set(BMI088Instance gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv); 
float Cal_FollowControl_Feedforward(BMI088Instance gimbal_IMU_data, Gimbal_Ctrl_Cmd_s gimbal_cmd_recv) ;

// 滤波系数 
#define DEFAULT_FILTER_FACTOR 0.85f  // 默认
#define STEP_FILTER_FACTOR 0.4f      // 阶跃
#define CORNER_FILTER_FACTOR 0.5f    // 三角形拐点
#define FAST_FILTER_FACTOR 0.7f     // 快速连续变化
#define SLOW_FILTER_FACTOR 0.95f      // 静止

// 前馈系数
#define DEFAULT_FEEDFORWARD_FACTOR 0.0f  // 默认，超小变化 0
#define STEP_FEEDFORWARD_FACTOR 1.5f     // 阶跃（识别到目标，与目标距离较远）,差多少，补大一点离目标的差值，小于45
#define CORNER_FEEDFORWARD_FACTOR 0.5f   // 三角形拐点处（突然间变向），适当减小前馈
#define FAST_FEEDFORWARD_FACTOR 150.0f   // 快速连续变化（车在走，头在追）,这个是对于cmd_delta，45-625
#define SLOW_FEEDFORWARD_FACTOR 1.0f     // 小变化但有变化，差多少补多少，0.2-0.3
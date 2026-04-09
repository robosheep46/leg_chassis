#ifndef CHASSIS_H
#define CHASSIS_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "kalman_filter.h"
#include "robot_def.h"
// 电机枚举
#define JOINT_CNT 4u
#define LF 0u
#define LB 1u
#define RF 2u
#define RB 3u

#define DRIVEN_CNT 2u
#define LD 0u
#define RD 1u

#define CENTER_IMU_W 0.0f
#define CENTER_IMU_L 0.0f
#define CENTER_IMU_H 0.0f
// 物理参数
#define CALF_LEN 0.26f              // 小腿长度 (m)
#define THIGH_LEN 0.2f             // 大腿长度 (m) 
#define WHEEL_RADIUS 0.0775f         // 轮子半径 (m)
#define WHEEL_MASS 2.0f            // 驱动轮质量 (kg)
#define BODY_MASS 16.0f              // 机体质量 (kg)
#define GRAVITY 9.8f                // 重力加速度 (m/s²)

// 滤波器参数
#define VEL_PROCESS_NOISE 25.0f     // 速度过程噪声
#define VEL_MEASURE_NOISE 800.0f   // 速度测量噪声
#define ACC_PROCESS_NOISE 2000.0f   // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01f     // 加速度测量噪声

// 控制参数
#define MAX_WHEEL_TORQUE 5.0f       // 最大轮力矩 (N·m)
#define MAX_JOINT_TORQUE 8.0f       // 最大关节力矩 (N·m)

// 数学常量
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f


// 五连杆腿部参数
typedef struct {
    //五（四）连杆
    // BD长度的平方
    // 计算phi2用的中间量
    float phi1, phi2, phi3, phi4, phi0;
    float last_phi0;
    float phi1_angle, phi4_angle;

    float phi1_w, phi2_w, phi3_w, phi4_w, phi0_w;

    // 腿参数
    float theta_pred;
    float theta, theta_w; // 杆和垂直方向的夹角,为控制状态之一
    float leg_len, legd;
    float height, height_v;
    float F_leg, T_hip;
    float target_len;

    // 雅可比矩阵
    // float jacobian[2][2];
    
    //lqr
    float T_lqr_theta_wheel;
    float T_lqr_theta_hip;

    float T_lqr_dist_wheel;
    float T_lqr_dist_hip;

    float T_lqr_pitch_wheel;
    float T_lqr_pitch_hip;

    float T_lqr_wheel;
    float T_lqr_hip;

    // 转向
    float T_motion_wheel;
    float T_motion_hip;

    // 力和力矩
    float T_wheel;
    // float 
    float real_T_wheel;
    // 运动学参数
    float wheel_w, wheel_dist, body_v, zw_ddot, support_force;
    uint8_t fly_flag;
    float jacobian[2][2];

    float T_back, T_front;
    float real_T_back, real_T_front;
    float w_ecd;
    float pitch ,pitch_w;
    float wheel_state[6];
    float leg_state[6];

    uint8_t first_flag;
} LegParam;

// 腿部状态向量（LQR控制用）
typedef struct {
    float theta, theta_dot;     // 腿部角度和角速度
    float x, x_dot;             // 机体位移和速度  
    float phi, phi_dot;         // 机体俯仰角和角速度
} LegState_t;


// 主底盘参数结构体
typedef struct 
{
    
    // 当前状态
    float pitch, pitch_w, wz,roll,roll_w;
    float target_yaw, yaw;
    float target_yaw_w,yaw_w;
    float target_dist,dist;
    // 速度
    float vel, target_v;        // 底盘速度
    float vel_m;                // 底盘速度测量值
    float vel_cov;              // 速度方差
    
    LegState_t leg_state[2];


    KalmanFilter_t v_kf;  // 观测车体速度
} ChassisParam;


// 核心函数
void ChassisInit(ChassisQueues_t *chassis_queue);
void ChassisTask(void *argument);


#endif // CHASSIS_H
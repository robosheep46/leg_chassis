#include "Chassis_Calculate.h"
#include "arm_math.h"
#include "chassis.h"
#include "ins_task.h"
#include "robot_def.h"

#include <math.h>


// 1. 计算腿长、腿与地面角度
void phi_transform_theta(LegParam * leg,ChassisParam *chassis)
{
    float YD, XD, XB, YB;
    float lBD_2, A0, B0, C0;
    float XC, YC;
    float discriminant;
    
    // 计算B、D坐标
    YD = THIGH_LEN * arm_sin_f32(leg->phi4);    
    XD = THIGH_LEN * arm_cos_f32(leg->phi4);    //distance = 0
    XB = THIGH_LEN * arm_cos_f32(leg->phi1);    
    YB = THIGH_LEN * arm_sin_f32(leg->phi1);    
    
    // 使用atan2方法计算phi2
    // BD的平方
    lBD_2 = (XD - XB) * (XD - XB) 
                    + (YD - YB) * (YD - YB);
    A0 = 2 * CALF_LEN * (XD - XB);
    B0 = 2 * CALF_LEN * (YD - YB);
    C0 = lBD_2  ;
    // 判别式
    discriminant = A0 * A0 + B0 * B0 - C0 * C0;
    leg->phi2 = 2 * atan2((B0 + sqrt(discriminant)), A0 + C0);

    // 计算末端C点坐标
    // （THIGH_LEN * cos(leg->phi1) + CALF_LEN * cos(phi2)）
    XC = XB + CALF_LEN * cos(leg->phi2);
    // （THIGH_LEN * sin(leg->phi1) + CALF_LEN * sin(phi2)）
    YC = YB + CALF_LEN * sin(leg->phi2);
    leg->phi3 = atan2(YC - YD , XC - XD);

    //求腿长、phi0
    leg->leg_len = sqrt((XC ) * (XC) + YC * YC);
    leg->phi0 = atan2(YC, XC);
    leg->theta = M_PI_2 - leg->phi0 - leg->pitch;
    leg->height = leg->leg_len * cos(leg->theta);
}

// leg->jacobian[0][0] = (THIGH_LEN * sin(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / sin(leg->phi3 - leg->phi2);
// leg->jacobian[0][1] = (THIGH_LEN * sin(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / sin(leg->phi3 - leg->phi2);
// leg->jacobian[1][0] = (THIGH_LEN * cos(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
// leg->jacobian[1][1] = (THIGH_LEN * cos(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
void calculate_leg_theta_w(LegParam * leg,ChassisParam *chassis)
{
    leg->jacobian[0][0] = (THIGH_LEN * sin(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / sin(leg->phi3 - leg->phi2);
    leg->jacobian[0][1] = (THIGH_LEN * sin(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / sin(leg->phi3 - leg->phi2);
    leg->jacobian[1][0] = (THIGH_LEN * cos(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
    leg->jacobian[1][1] = (THIGH_LEN * cos(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
    leg->phi0_w = leg->jacobian[1][0] * leg->phi1_w + leg->jacobian[1][1] * leg->phi4_w;
    leg->theta_w = - leg->phi0_w - leg->pitch_w;
}

void set_left_leg_six_states(LegParam *leg, ChassisParam *chassis)
{
        leg->wheel_state[0] =  -chassis->dist - 0               ;        // 机体位移偏差
        leg->wheel_state[1] =  -chassis->vel + chassis->target_v;   // 机体速度偏差
        leg->wheel_state[2]   =  chassis->yaw                     ;        // 机体俯仰角偏差
        leg->wheel_state[3]   =  chassis->yaw_w                   ;        // 机体俯仰角速度偏差
        leg->wheel_state[4] =  leg->theta - (0)                   ;        // 腿部角度偏差
        leg->wheel_state[5] =  leg->theta_w - 0                 ;        // 腿部角速度偏差
        leg->wheel_state[6] =  leg->pitch                       ;        // 机体俯仰角偏差
        leg->wheel_state[7] =  leg->pitch_w                     ;        // 机体俯仰角速度偏差

        leg->leg_state[0]   =  - chassis->dist - 0                ;        // 机体位移偏差
        leg->leg_state[1]   =  - chassis->vel + chassis->target_v ;        // 机体速度偏差
        leg->leg_state[2]   =  chassis->yaw                     ;        // 机体俯仰角偏差
        leg->leg_state[3]   =  chassis->yaw_w                   ;        // 机体俯仰角速度偏差
        leg->leg_state[4]   =  leg->theta - (0)                   ;        // 腿部角度偏差
        leg->leg_state[5]   =  leg->theta_w - 0                 ;        // 腿部角速度偏差
        leg->leg_state[6]   =  leg->pitch                       ;        // 机体俯仰角偏差
        leg->leg_state[7]   =  leg->pitch_w                     ;        // 机体俯仰角速度偏差

}

void set_right_leg_six_states(LegParam *leg, ChassisParam *chassis)
{

    leg->wheel_state[0] =  chassis->dist - 0                ;        // 机体位移偏差
    leg->wheel_state[1] =  chassis->vel - chassis->target_v ;        // 机体速度偏差
    leg->wheel_state[2] =  -chassis->yaw                     ;        // 机体俯仰角偏差
    leg->wheel_state[3] =  -chassis->yaw_w                   ;        // 机体俯仰角速度偏差
    leg->wheel_state[4] =  leg->theta + 0                  ;        // 腿部角度偏差
    leg->wheel_state[5] =  leg->theta_w - 0                 ;        // 腿部角速度偏差
    leg->wheel_state[6] =  leg->pitch                       ;        // 机体俯仰角偏差
    leg->wheel_state[7] =  leg->pitch_w                     ;        // 机体俯仰角速度偏差

    leg->leg_state[0]   =  chassis->dist - 0                ;        // 机体位移偏差
    leg->leg_state[1]   =  chassis->vel - chassis->target_v ;        // 机体速度偏差
    leg->leg_state[2]   =  - chassis->yaw                     ;        // 机体俯仰角偏差
    leg->leg_state[3]   =  - chassis->yaw_w                   ;        // 机体俯仰角速度偏差
    leg->leg_state[4]   =  leg->theta + 0                   ;        // 腿部角度偏差
    leg->leg_state[5]   =  leg->theta_w - 0                 ;        // 腿部角速度偏差
    leg->wheel_state[6] =  leg->pitch                       ;        // 机体俯仰角偏差
    leg->wheel_state[7] =  leg->pitch_w                     ;        // 机体俯仰角速度偏差

}

void calculate_wheel_torgue(LegParam *l_leg,LegParam *r_leg, ChassisParam *chassis)
{
        static float k[2][10]=
        {
            -1.2195,  -5.9618,  -133.77,  -17.545,  -51.446,  -2.7782,  -18.6,  -1.072,  22.369,  0.10979
            -1.1216,  -5.4856,  157.75,  20.728,  -25.08,  -1.3408,  -40.385,  -2.3439,  19.564,  0.027888
            -0.37342,  -1.8308,  -20.889,  -3.3057,  19.142,  0.63857,  -21.444,  -1.6335,  -65.065,  -3.2945
            -0.26402,  -1.291,  20.828,  3.3317,  -18.581,  -1.3211,  22.308,  1.012,  -66.607,  -3.3623
        };

        l_leg->T_lqr_dist_wheel  = k[0][0] * l_leg->wheel_state[0] + k[0][1] * l_leg->wheel_state[1];
        l_leg->T_lqr_yaw_wheel   = k[0][2] * l_leg->wheel_state[2] + k[0][3] * l_leg->wheel_state[3];
        l_leg->T_lqr_theta_wheel = k[0][4] * l_leg->wheel_state[4] + k[0][5] * l_leg->wheel_state[5];
        l_leg->T_lqr_pitch_wheel = k[0][8] * l_leg->wheel_state[6] + k[0][9] * l_leg->wheel_state[7];
    
        l_leg->T_lqr_dist_hip  = k[1][0] * l_leg->leg_state[0] + k[1][1] * l_leg->leg_state[1];
        l_leg->T_lqr_yaw_hip   = k[1][2] * l_leg->leg_state[2] + k[1][3] * l_leg->leg_state[3];
        l_leg->T_lqr_theta_hip = k[1][4] * l_leg->leg_state[4] + k[1][5] * l_leg->leg_state[5];
        l_leg->T_lqr_pitch_hip = k[1][8] * l_leg->leg_state[6] + k[1][9] * l_leg->leg_state[7];
    
        r_leg->T_lqr_dist_wheel  = k[0][0] * r_leg->wheel_state[0] + k[0][1] * r_leg->wheel_state[1];
        r_leg->T_lqr_yaw_wheel   = k[0][2] * r_leg->wheel_state[2] + k[0][3] * r_leg->wheel_state[3];
        r_leg->T_lqr_theta_wheel = k[0][6] * r_leg->wheel_state[4] + k[0][7] * r_leg->wheel_state[5];
        r_leg->T_lqr_pitch_wheel = k[0][8] * r_leg->wheel_state[6] + k[0][9] * r_leg->wheel_state[7];
    
        r_leg->T_lqr_dist_hip  = k[1][0] * r_leg->leg_state[0] + k[1][1] * r_leg->leg_state[1];
        r_leg->T_lqr_yaw_hip   = k[1][2] * r_leg->leg_state[2] + k[1][3] * r_leg->leg_state[3];
        r_leg->T_lqr_theta_hip = k[1][6] * r_leg->leg_state[4] + k[1][7] * r_leg->leg_state[5]; // 注意右腿可能用不同索引
        r_leg->T_lqr_pitch_hip = k[1][8] * r_leg->leg_state[6] + k[1][9] * r_leg->leg_state[7];

        l_leg->T_lqr_wheel =
                        // l_leg->T_lqr_dist_wheel 
                           + l_leg->T_lqr_yaw_wheel 
                        //    + l_leg->T_lqr_theta_wheel 
                           + l_leg->T_lqr_pitch_wheel ;

        l_leg->T_lqr_hip   = 0;
                        //      l_leg->T_lqr_dist_hip ;
                        //    + l_leg->T_lqr_yaw_hip 
                        //    + l_leg->T_lqr_theta_hip 
                        //    + l_leg->T_lqr_pitch_hip;
    
        r_leg->T_lqr_wheel =
                        // r_leg->T_lqr_dist_wheel 
                           + r_leg->T_lqr_yaw_wheel 
                        //    + r_leg->T_lqr_theta_wheel 
                           + r_leg->T_lqr_pitch_wheel ;    

        r_leg->T_lqr_hip   =0;// r_leg->T_lqr_dist_hip ;
                        //    + r_leg->T_lqr_yaw_hip 
                        //    + r_leg->T_lqr_theta_hip 
                        //    + r_leg->T_lqr_pitch_hip;
    
}



// 3. calculate_leg_torgue
// 原本为 雅可比矩阵计算
// J[2][2]
// leg->jacobian[0][0] = (THIGH_LEN * sin(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / sin(leg->phi3 - leg->phi2);
// leg->jacobian[0][1] = (THIGH_LEN * sin(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / sin(leg->phi3 - leg->phi2);
// leg->jacobian[1][0] = (THIGH_LEN * cos(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
// leg->jacobian[1][1] = (THIGH_LEN * cos(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
// F =[F_leg, T_hip]
// 关节力矩与足端力的关系： τ = J^T * F
// F =[ Fleg ]
//    [ Thip ]
//    [ Tback ] = J^T * [ Fleg ]
//    [ Tfront]         [ Thip ]
//    得：Tback  = J[0][0]* Tleg / leg_len + J[1][0] * Thip
//    得：Tfront = J[0][1]* Tleg / leg_len + J[1][1] * Thip
//    得：    
//       float T_leg      = leg->F_leg * leg->leg_len;
//       leg->T_back = ( THIGH_LEN * sin(leg->phi1 - leg->phi2) *            T_leg * sin(leg->phi0 - leg->phi3)
//                   +   THIGH_LEN * sin(leg->phi1 - leg->phi2) *  leg->T_hip * cos(leg->phi0 - leg->phi3)) 
//                   /   leg->leg_len * sin(leg->phi3 - leg->phi2)
void calculate_leg_torgue(LegParam * leg)
{
    leg->T_back   = leg->jacobian[0][0]* leg->F_leg + leg->jacobian[1][0] * leg->T_hip;
    leg->T_front  = leg->jacobian[0][1]* leg->F_leg + leg->jacobian[1][1] * leg->T_hip;
}

void calculate_support_force(LegParam *leg, attitude_t *imu)
{
    static float accx, accy, accz;
    accx = imu->MotionAccel_b[0];
    accy = imu->MotionAccel_b[1];
    accz = imu->MotionAccel_b[2];

    static float pitch, roll;
    pitch = imu->pitch;
    roll = imu->roll;

    // 机体竖直方向加速度
    leg->zw_ddot = -sin(roll) * accx + cos(roll) * sin(pitch) * accy + cos(pitch) * cos(roll) * accz;

    static float P;
    P = leg->F_leg * cos(leg->theta) + leg->T_hip * sin(leg->theta) / leg->leg_len;
    leg->support_force = P + WHEEL_MASS * (leg->zw_ddot + 9.81f); 

    // // 离地检测
    if(leg->support_force < 20.0f)
        leg->fly_flag = 1;
    else
        leg->fly_flag = 0;
}
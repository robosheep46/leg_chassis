#include "Chassis_Calculate.h"
#include "arm_math.h"
#include "chassis.h"
#include "bmi088.h"
#include "robot_def.h"

#include <math.h>


































// 1. 计算腿长、腿与地面角度
void TranLeg(LegParam * leg,ChassisParam *chassis)
{
    // 计算B、D坐标
    leg->YD = THIGH_LEN * arm_sin_f32(leg->phi4);    
    leg->XD = THIGH_LEN * arm_cos_f32(leg->phi4);    //distance = 0
    leg->XB = THIGH_LEN * arm_cos_f32(leg->phi1);    
    leg->YB = THIGH_LEN * arm_sin_f32(leg->phi1);    
    
    // 使用atan2方法计算phi2
    // BD的平方
    leg->lBD_2 = (leg->XD - leg->XB) * (leg->XD - leg->XB) 
                    + (leg->YD - leg->YB) * (leg->YD - leg->YB);
    leg->A0 = 2 * CALF_LEN * (leg->XD - leg->XB);
    leg->B0 = 2 * CALF_LEN * (leg->YD - leg->YB);
    leg->C0 = leg->lBD_2  ;
    // 判别式
    float discriminant = leg->A0 * leg->A0 + leg->B0 * leg->B0 - leg->C0 * leg->C0;
    leg->phi2 = 2 * atan2((leg->B0 + sqrt(discriminant)), leg->A0 + leg->C0);

    // 计算末端C点坐标
    // （THIGH_LEN * cos(leg->phi1) + CALF_LEN * cos(phi2)）
    leg->XC = leg->XB + CALF_LEN * cos(leg->phi2);
    // （THIGH_LEN * sin(leg->phi1) + CALF_LEN * sin(phi2)）
    leg->YC = leg->YB + CALF_LEN * sin(leg->phi2);
    leg->phi3 = atan2(leg->YC - leg->YD , leg->XC - leg->XD);

    //求腿长、phi0
    leg->leg_len = sqrt((leg->XC ) * (leg->XC) + leg->YC * leg->YC);
    leg->phi0 = atan2(leg->YC, leg->XC);
    leg->theta = M_PI_2 - leg->phi0 - leg->pitch;
    leg->height = leg->leg_len * cos(leg->theta);
}

// leg->jacobian[0][0] = (THIGH_LEN * sin(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / sin(leg->phi3 - leg->phi2);
// leg->jacobian[0][1] = (THIGH_LEN * sin(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / sin(leg->phi3 - leg->phi2);
// leg->jacobian[1][0] = (THIGH_LEN * cos(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
// leg->jacobian[1][1] = (THIGH_LEN * cos(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
void CalcThetaw(LegParam * leg,ChassisParam *chassis)
{
    leg->jacobian[0][0] = (THIGH_LEN * sin(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / sin(leg->phi3 - leg->phi2);
    leg->jacobian[0][1] = (THIGH_LEN * sin(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / sin(leg->phi3 - leg->phi2);
    leg->jacobian[1][0] = (THIGH_LEN * cos(leg->phi0 - leg->phi3) * sin(leg->phi1 - leg->phi2)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
    leg->jacobian[1][1] = (THIGH_LEN * cos(leg->phi0 - leg->phi2) * sin(leg->phi3 - leg->phi4)) / (leg->leg_len * sin(leg->phi3 - leg->phi2));
    leg->phi0_w = leg->jacobian[1][0] * leg->phi1_w + leg->jacobian[1][1] * leg->phi4_w;
    leg->theta_w = - leg->phi0_w - leg->pitch_w;
}

// VMC 中  F_leg即腿长控制 pid 输出，
//         leg_len √ -> T_leg √
//          还差 Thip
// 2.接下来用LQR 计算 Thip 
static float testa[6];
void CalLeftT_Wheel(LegParam *leg, ChassisParam *chassis)
{
    static float k[2][6];
    float t1 = leg->leg_len;
    float t2 = leg->leg_len * leg->leg_len;
    float t3 = leg->leg_len * leg->leg_len * leg->leg_len;

    k[0][0] = -276.9355f*t3 + 318.5072f*t2-147.7479f*t1 + 0.3148f;
    k[0][1] = -1.7898f*t3 + 5.1048f*t2-9.6011f*t1 + 0.2912f;
    k[0][2] = -27.9394f*t3 + 28.0033f*t2-9.7795f*t1-0.2503f;
    k[0][3] = -42.4398f*t3 + 42.7436f*t2-15.3926f*t1-0.4394f;
    k[0][4] = -37.8587f*t3 + 59.4502f*t2-36.3017f*t1 + 10.6018f;
    k[0][5] = -4.2180f*t3 + 8.4305f*t2-6.0094f*t1 + 2.0718f;
    k[1][0] = 282.0810f*t3-242.2992f*t2 + 55.0414f*t1 + 9.4673f;
    k[1][1] = 24.7783f*t3-24.8609f*t2 + 7.9582f*t1 + 0.1660f;
    k[1][2] = -21.4404f*t3 + 27.8204f*t2-14.0966f*t1 + 3.2074f;
    k[1][3] = -31.8560f*t3 + 41.5448f*t2-21.2111f*t1 + 4.8747f;
    k[1][4] = 289.2883f*t3-296.0413f*t2 + 107.4000f*t1-0.7643f;
    k[1][5] = 55.9790f*t3-57.8950f*t2 + 21.3992f*t1-0.4533f;

    uint8_t is_take_off = leg->fly_flag;
    
    // 离地
    if (is_take_off) {
        // 轮力矩增益清零 - 离地时不需要轮子力矩
        for (int i = 0; i < 6; i++) {
            k[0][i] = 0;
        }
        k[1][2]  = 0   ;  
        k[1][3]  = 0   ;
        k[1][4]  = 0   ;
        k[1][5]  = 0   ;
    }

    float T[2] = {0}; // [0]: T_wheel, [1]: T_hip

    // 状态向量定义
    float x[6];
    
    // x[0] = 0;
    // x[1] = 0;
    // x[2] = 0;
    // x[3] = 0;
    // x[4] = 0 ;
    // x[5] = 0 ;

    x[0] =  leg->theta - 0;                            // 腿部角度偏差
    x[1] =  leg->theta_w - 0;                         // 腿部角速度偏差 ok
    x[2] =  chassis->dist - 0  ;                    // 机体位移偏差
    x[3] =  chassis->vel - chassis->target_v    ;                        // 机体速度偏差
    x[4] =  leg->pitch ;                                   // 机体俯仰角偏差
    x[5] =  leg->pitch_w ;                                 // 机体俯仰角速度偏差

    // 计算LQR输出
    T[0] = 0;

    for (int j = 0; j < 6; j++) 
    {
        T[0] += k[0][j] * x[j];
    }

    leg->T_wheel = T[0]; //算是最终输出
}

void CalRightT_Wheel(LegParam *leg, ChassisParam *chassis)
{
    static float k[2][6];
    float t1 = leg->leg_len;
    float t2 = leg->leg_len * leg->leg_len;
    float t3 = leg->leg_len * leg->leg_len * leg->leg_len;

    k[0][0] = -276.9355f*t3 + 318.5072f*t2-147.7479f*t1 + 0.3148f;
    k[0][1] = -1.7898f*t3 + 5.1048f*t2-9.6011f*t1 + 0.2912f;
    k[0][2] = -27.9394f*t3 + 28.0033f*t2-9.7795f*t1-0.2503f;
    k[0][3] = -42.4398f*t3 + 42.7436f*t2-15.3926f*t1-0.4394f;
    k[0][4] = -37.8587f*t3 + 59.4502f*t2-36.3017f*t1 + 10.6018f;
    k[0][5] = -4.2180f*t3 + 8.4305f*t2-6.0094f*t1 + 2.0718f;
    k[1][0] = 282.0810f*t3-242.2992f*t2 + 55.0414f*t1 + 9.4673f;
    k[1][1] = 24.7783f*t3-24.8609f*t2 + 7.9582f*t1 + 0.1660f;
    k[1][2] = -21.4404f*t3 + 27.8204f*t2-14.0966f*t1 + 3.2074f;
    k[1][3] = -31.8560f*t3 + 41.5448f*t2-21.2111f*t1 + 4.8747f;
    k[1][4] = 289.2883f*t3-296.0413f*t2 + 107.4000f*t1-0.7643f;
    k[1][5] = 55.9790f*t3-57.8950f*t2 + 21.3992f*t1-0.4533f;

    uint8_t is_take_off = leg->fly_flag;
    // 离地
    if (is_take_off) {
        // 轮力矩增益清零 - 离地时不需要轮子力矩
        for (int i = 0; i < 6; i++) {
            k[0][i] = 0;
        }
        k[1][2]  = 0   ;  
        k[1][3]  = 0   ;
        k[1][4]  = 0   ;
        k[1][5]  = 0   ;
    }

    float T[2] = {0}; // [0]: T_wheel, [1]: T_hip

    // 状态向量定义
    float x[6];
    
    // x[0] = 0;
    // x[1] = 0;
    
    // x[2] = 0 ;
    // x[3] = 0 ;
    // x[4] = 0 ;
    // x[5] = 0 ;


    x[0] =  leg->theta - 0;                     // 腿部角度偏差 ok
    x[1] =  leg->theta_w - 0;                     // 腿部角速度偏差 ok
    x[2] =  - chassis->dist - 0  ;                    // 机体位移偏差
    x[3] =  - chassis->vel -(-chassis->target_v)    ;                        // 机体速度偏差

    x[4] =  leg->pitch ;                                   // 机体俯仰角偏差
    x[5] =  leg->pitch_w ;                                 // 机体俯仰角速度偏差

    // 计算LQR输出
    T[0] = 0;
    for (int j = 0; j < 6; j++) {
        T[0] += k[0][j] * x[j];
    }

    leg->T_wheel = T[0]; //算是最终输出
}


void CalLeftT_Hip(LegParam *leg, ChassisParam *chassis)
{
    static float k[2][6];
    float t1 = leg->leg_len;
    float t2 = leg->leg_len * leg->leg_len;
    float t3 = leg->leg_len * leg->leg_len * leg->leg_len;
    k[0][0] = -276.9355f*t3 + 318.5072f*t2-147.7479f*t1 + 0.3148f;
    k[0][1] = -1.7898f*t3 + 5.1048f*t2-9.6011f*t1 + 0.2912f;
    k[0][2] = -27.9394f*t3 + 28.0033f*t2-9.7795f*t1-0.2503f;
    k[0][3] = -42.4398f*t3 + 42.7436f*t2-15.3926f*t1-0.4394f;
    k[0][4] = -37.8587f*t3 + 59.4502f*t2-36.3017f*t1 + 10.6018f;
    k[0][5] = -4.2180f*t3 + 8.4305f*t2-6.0094f*t1 + 2.0718f;
    k[1][0] = 282.0810f*t3-242.2992f*t2 + 55.0414f*t1 + 9.4673f;
    k[1][1] = 24.7783f*t3-24.8609f*t2 + 7.9582f*t1 + 0.1660f;
    k[1][2] = -21.4404f*t3 + 27.8204f*t2-14.0966f*t1 + 3.2074f;
    k[1][3] = -31.8560f*t3 + 41.5448f*t2-21.2111f*t1 + 4.8747f;
    k[1][4] = 289.2883f*t3-296.0413f*t2 + 107.4000f*t1-0.7643f;
    k[1][5] = 55.9790f*t3-57.8950f*t2 + 21.3992f*t1-0.4533f;

    uint8_t is_take_off = leg->fly_flag;
    
    // 离地
    if (is_take_off) {
        // 轮力矩增益清零 - 离地时不需要轮子力矩
        for (int i = 0; i < 6; i++) {
            k[0][i] = 0;
        }
        k[1][2]  = 0   ;  
        k[1][3]  = 0   ;
        k[1][4]  = 0   ;
        k[1][5]  = 0   ;
    }

    float T[2] = {0}; // [0]: T_wheel, [1]: T_hip

    // 状态向量定义
    float x[6];
    
    // x[0] = 0;
    // x[1] = 0;
    // // x[2] = 0 ;
    // // x[3] = 0 ;
    // x[4] = 0 ;
    // x[5] = 0 ;

    x[0] =  leg->theta - 0;    // 腿部角度偏差
    x[1] =  leg->theta_w - 0;  // 腿部角速度偏差
    x[2] =  0 - chassis->dist;                    // 机体位移偏差
    x[3] =  - chassis->target_v - chassis->vel  ; // 机体速度偏差
    x[4] =  leg->pitch ;                                   // 机体俯仰角偏差
    x[5] =  leg->pitch_w ;                                 // 机体俯仰角速度偏差

    // 计算LQR输出
    T[1] = 0;
    for (int j = 0; j < 6; j++) {
        T[1] += k[1][j] * x[j];
    }
    leg->T_real_hip = T[1];   //VMC用
}

void CalRightT_Hip(LegParam *leg, ChassisParam *chassis)
{
    static float k[2][6];
    float t1 = leg->leg_len;
    float t2 = leg->leg_len * leg->leg_len;
    float t3 = leg->leg_len * leg->leg_len * leg->leg_len;
    k[0][0] = -276.9355f*t3 + 318.5072f*t2-147.7479f*t1 + 0.3148f;
    k[0][1] = -1.7898f*t3 + 5.1048f*t2-9.6011f*t1 + 0.2912f;
    k[0][2] = -27.9394f*t3 + 28.0033f*t2-9.7795f*t1-0.2503f;
    k[0][3] = -42.4398f*t3 + 42.7436f*t2-15.3926f*t1-0.4394f;
    k[0][4] = -37.8587f*t3 + 59.4502f*t2-36.3017f*t1 + 10.6018f;
    k[0][5] = -4.2180f*t3 + 8.4305f*t2-6.0094f*t1 + 2.0718f;
    k[1][0] = 282.0810f*t3-242.2992f*t2 + 55.0414f*t1 + 9.4673f;
    k[1][1] = 24.7783f*t3-24.8609f*t2 + 7.9582f*t1 + 0.1660f;
    k[1][2] = -21.4404f*t3 + 27.8204f*t2-14.0966f*t1 + 3.2074f;
    k[1][3] = -31.8560f*t3 + 41.5448f*t2-21.2111f*t1 + 4.8747f;
    k[1][4] = 289.2883f*t3-296.0413f*t2 + 107.4000f*t1-0.7643f;
    k[1][5] = 55.9790f*t3-57.8950f*t2 + 21.3992f*t1-0.4533f;
    
    
    uint8_t is_take_off = leg->fly_flag;
    
    // 离地
    if (is_take_off) {
        // 轮力矩增益清零 - 离地时不需要轮子力矩
        for (int i = 0; i < 6; i++) {
            k[0][i] = 0;
        }
        k[1][2]  = 0   ;  
        k[1][3]  = 0   ;
        k[1][4]  = 0   ;
        k[1][5]  = 0   ;
    }

    float T[2] = {0}; // [0]: T_wheel, [1]: T_hip

    // 状态向量定义
    float x[6];
    
    // x[0] = 0;
    // x[1] = 0;
    // x[2] = 0 ;
    // x[3] = 0 ;
    // x[4] = 0 ;
    // x[5] = 0 ;

    x[0] =  leg->theta - 0;                                        // 腿部角度偏差
    x[1] =  leg->theta_w - 0;
    // // 腿部角速度偏差
    x[2] =  chassis->dist ;                               // 机体位移偏差
    x[3] =  chassis->vel + chassis->target_v;              // 机体速度偏差
    x[4] =  leg->pitch ;                                   // 机体俯仰角偏差
    x[5] =  leg->pitch_w ;                                 // 机体俯仰角速度偏差
    // a[0] = (-chassis->dist ) * k[1][2];
    // a[1] = (-chassis->vel - chassis->target_v )*k[1][3];
    // a[2] = k[1][3];

    // 计算LQR输出
    T[1] = 0;
    for (int j = 0; j < 6; j++) {
        T[1] += k[1][j] * x[j];
    }
    leg->T_real_hip = T[1];   //VMC用
}



// 3. VMC
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
void VMC(LegParam * leg)
{
    leg->T_back  = leg->jacobian[0][0]* leg->F_leg + leg->jacobian[1][0] * leg->T_real_hip;
    leg->T_front  = leg->jacobian[0][1]* leg->F_leg + leg->jacobian[1][1] * leg->T_real_hip;
}

void SupportForceSolve(LegParam *leg, imu_data_t *imu)
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

    static float F;
    F = leg->F_leg * cos(leg->theta) + leg->T_hip * sin(leg->theta) / leg->leg_len;
    leg->support_force = F + WHEEL_MASS * (leg->zw_ddot + 9.81f); 

    // 离地检测
    if(leg->support_force < 20.0f)
        leg->fly_flag = 1;
    else
        leg->fly_flag = 0;
}
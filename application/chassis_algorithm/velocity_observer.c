#include "ins_task.h"
#include "robot_def.h"
#include "user_lib.h"
#include "general_def.h"
#include "chassis.h"
#include "velocity_observer.h"
#include "kalman_filter.h"


void GetChassisAccel()
{

}
/**
 * @brief 使用卡尔曼滤波估计底盘速度
 * @todo 增加w和dw的滤波,当w和dw均小于一定值时,不考虑dw导致的角加速度
 *
 * @param lp 左侧腿
 * @param rp 右侧腿
 * @param cp 底盘
 * @param imu imu数据
 * @param delta_t 更新间隔
 */
void observe_speed_init(ChassisParam *cp)
{
    Kalman_Filter_Init(&cp->v_kf, 2, 0, 2);
    float F[4] = {1, 0.005, 0, 1};
    float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};
    float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
    float P[4] = {100000, 0, 0, 100000};
    float H[4] = {1, 0, 0, 1};
    memcpy(cp->v_kf.F_data, F, sizeof(F));
    memcpy(cp->v_kf.P_data, P, sizeof(P));
    memcpy(cp->v_kf.Q_data, Q, sizeof(Q));
    memcpy(cp->v_kf.R_data, R, sizeof(R));
    memcpy(cp->v_kf.H_data, H, sizeof(H));
}
void observe_speed(LegParam *lp, LegParam *rp, ChassisParam *cp, attitude_t *imu, float delta_t)
{
    // 修正轮速和距离
    lp->wheel_w = lp->w_ecd; // 减去和定子固连的phi2_w
    rp->wheel_w = rp->w_ecd;
    cp->vel_m = (lp->wheel_w - rp->wheel_w) * WHEEL_RADIUS / 2; // 机体速度(平动)为两侧速度的平均值
    // cp->vel = cp->vel_m;

    cp->v_kf.MeasuredVector[0] =  cp->vel_m;
    cp->v_kf.MeasuredVector[1] =  imu->MotionAccel_b[0];
    cp->v_kf.F_data[1]  = delta_t;

    Kalman_Filter_Update(&cp->v_kf);
    cp->vel = cp->v_kf.xhat_data[0];
    
    // 速度和位置分离，有速度输入时不进行位置闭环
    if(fabsf(cp->target_v) > 0.005)
    {
        cp->target_dist = 0;
        cp->dist = 0;
    }
    else
    {
        cp->dist += cp->vel * delta_t;
    }
}
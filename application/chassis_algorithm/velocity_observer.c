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
void SpeedObserver(LegParam *lp, LegParam *rp, ChassisParam *cp, attitude_t *imu, float delta_t)
{
    // 修正轮速和距离
    lp->wheel_w = lp->w_ecd *WHEEL_RADIUS *2*PI/60; // 减去和定子固连的phi2_w
    rp->wheel_w = rp->w_ecd *WHEEL_RADIUS *2*PI/60;
    cp->vel_m = (lp->wheel_w - rp->wheel_w) / 2; // 机体速度(平动)为两侧速度的平均值
    cp->vel = cp->vel_m;
    // cp->v_kf.MeasuredVector[0] =  cp->vel_m;
    // cp->v_kf.MeasuredVector[1] =  1;
    // cp->v_kf.F_data[1]  = delta_t;

    // Kalman_Filter_Update(&cp->v_kf);
    // cp->vel = cp->v_kf.xhat_data[0];
    
    // 速度和位置分离，有速度输入时不进行位置闭环
    if(abs(cp->target_v) < 0.005)
    {
        cp->dist += cp->vel * delta_t;
    }
    else
    {
        cp->target_dist = cp->dist = 0;
    }
}
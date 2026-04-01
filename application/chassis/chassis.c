//app
#include "chassis.h"
#include "fdcan.h"
#include "robot_def.h"
//module

#include "ins_task.h"
#include "general_def.h"
#include "controller.h"
//bsp
#include "bsp_dwt.h"
#include "arm_math.h"
#include "LK9025.h"
#include "dmmotor.h"
#include "spi.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
#include "user_lib.h"

#include "Chassis_Calculate.h"
#include "velocity_observer.h"
#include <math.h>
/************************************** CommUsed **************************************/
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;             // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data;     // 底盘回传的反馈数据

static LKMotorInstance *r_driven, *l_driven, *driven[2]; // 左右驱动轮
static DMMotorInstance *lf, *lb, *rf, *rb,*joint[4]; // 双马达
static attitude_t *chassis_imu_data;
// static Robot_Status_e chassis_status;


static ChassisParam chassis; 
static LegParam l_side, r_side; 

static uint32_t balance_dwt_cnt;
static float del_t;

// 只调pd
static PIDInstance leg_len_pid_l;     
static PIDInstance leg_len_pid_r; 
//   调p
static PIDInstance steer_p_pid;
static PIDInstance steer_v_pid;

static PIDInstance roll_compensate_pid; 
static PIDInstance anti_crash_pid;
static TaskHandle_t chassis_task_handle = NULL;
static QueueHandle_t chassis_recv_queue = NULL;
static QueueHandle_t chassis_feedback_queue = NULL;
static QueueHandle_t imu_queue = NULL;

const osThreadAttr_t chassis_task_attributes = {
    .name = "chassis_task",
    .stack_size = 1024*2,
    .priority = (osPriority_t) osPriorityNormal,
};

void ChassisInit(ChassisQueues_t *chassis_queue)
{
/***************************IMU_INIT******************************/
    chassis_task_handle = osThreadNew(ChassisTask, NULL,&chassis_task_attributes);
    chassis_recv_queue = chassis_queue->chassis_recv_queue;
    chassis_feedback_queue = chassis_queue->chassis_feedback_queue;
    

    chassis_imu_data = imu_init();


/***************************MOTOR_INIT******************************/
    // // 驱动轮电机初始化
    Motor_Init_Config_s driven_conf = {
        .can_init_config.can_handle = &hfdcan2,
        .motor_type = LK9025,
    };
    driven_conf.can_init_config.tx_id = 0x01;
    driven[0] = r_driven = LKMotorInit(&driven_conf);

    driven_conf.can_init_config.can_handle = &hfdcan1,
    driven_conf.can_init_config.tx_id = 0x02;
    driven[1] = l_driven = LKMotorInit(&driven_conf);
    // 关节电机
    Motor_Init_Config_s r_joint_conf = {
        // 写一个,剩下的修改方向和id即可
        .can_init_config = {
            .can_handle = &hfdcan2
        },
        .controller_param_init_config = {
            .angle_PID=
            {
                .Kp = 0 //100,// .Kp=60,
            },
            .speed_PID  =
            {
                .Kd = 0// 0.5 // .Kd=0.5,
            },
        },
        .mit_flag = 1, 
        .motor_type = DM8009};
    r_joint_conf.can_init_config.tx_id =  0x01;
    r_joint_conf.can_init_config.rx_id =  0x11;
    joint[RF] = rf = DMMotorInit(&r_joint_conf);
    r_joint_conf.can_init_config.tx_id = 0x02;
    r_joint_conf.can_init_config.rx_id = 0x12;
    joint[RB] = rb = DMMotorInit(&r_joint_conf);

    Motor_Init_Config_s l_joint_conf = {
        // 写一个,剩下的修改方向和id即可
        .can_init_config = {
            .can_handle = &hfdcan1
        },
        .controller_param_init_config = {
            .angle_PID=
            {
                .Kp= 0 //80, //角度/60,
            },
            .speed_PID = 
            {
                .Kd =0// 0.5,   //角度/0.5,
            },
        },
        .mit_flag = 1, 
        .motor_type = DM8009};
    l_joint_conf.can_init_config.tx_id = 0x03;
    l_joint_conf.can_init_config.rx_id = 0x13;
    joint[LB] = lb = DMMotorInit(&l_joint_conf);
    l_joint_conf.can_init_config.tx_id = 0x04;
    l_joint_conf.can_init_config.rx_id = 0x14;
    joint[LF] = lf = DMMotorInit(&l_joint_conf);


// /*******************************LEG_PID_INIT******************************* */
    // 腿长控制
    PID_Init_Config_s l_leg_length_pid_conf = {
        .Kp = 5,
        .Kd = 0,
        .Ki = 0,
        .MaxOut = 40,
        .DeadBand = 0.0001f,
        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&leg_len_pid_l, &l_leg_length_pid_conf);

    PID_Init_Config_s r_leg_length_pid_conf = {
        .Kp = 5,
        .Kd = 0,
        .Ki = 0,
        .MaxOut = 40,
        .DeadBand = 0.0001f,
        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&leg_len_pid_r, &r_leg_length_pid_conf);

    PID_Init_Config_s roll_compensate_pid_conf = {
        .Kp = 0.0008f,
        .Kd = 0.0002f,
        .Ki = 0.0f,
        .MaxOut = 0.05,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&roll_compensate_pid, &roll_compensate_pid_conf);

// /*******************************steer_PID_INIT******************************* */
//     // 航向控制
//     // 角度环
    // 航向控制
    // 角度环
    PID_Init_Config_s steer_p_pid_conf = {
        .Kp = 5,
        .Kd = 0,
        .Ki = 0.0f,
        .MaxOut = 3,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&steer_p_pid, &steer_p_pid_conf);
    // 速度环
    PID_Init_Config_s steer_v_pid_conf = {
        .Kp = 0.001 ,
        .Kd = 0.0f,
        .Ki = 0.0f,
        .MaxOut = 20,
        .DeadBand = 0.0f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&steer_v_pid, &steer_v_pid_conf);

    // 抗劈叉
    PID_Init_Config_s anti_crash_pid_conf = {
        .Kp = 1,         //15
        .Kd = 0,         //2
        .Ki = 0.0,
        .MaxOut = 30,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_ChangingIntegrationRate | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&anti_crash_pid, &anti_crash_pid_conf);

    chassis.target_v = 0;
    r_side.real_T_wheel = 0;
    r_side.real_T_front = 0;
    r_side.real_T_back = 0;
    l_side.real_T_front = 0;
    l_side.real_T_back = 0;
    l_side.real_T_wheel = 0;
    // chassis.vel_cov = 100;
}
/************************************** Motor Control **************************************/
static void EnableAllMotor()
{
    for (uint8_t i = 0; i < DRIVEN_CNT; i++) {
        LKMotorEnable(driven[i]);
    }
    for (uint8_t i = 0; i < JOINT_CNT; i++) {
        DMMotorEnable(joint[i]);
    }
}

static void ResetChassis()
{
    EnableAllMotor();
    
    // 重置状态
    chassis.target_v = 0;
    chassis.dist = chassis.target_dist = 0;
    chassis.target_yaw = chassis.yaw;
    
    // 停止所有电机
    LKMotorSetRef(l_driven, 0);
    // LKMotorSetRef(r_driven, 0);
    for (uint8_t i = 0; i < JOINT_CNT; i++)
    {
        DMMotorOuterLoop(joint[i], ANGLE_LOOP);
        // DMMotorSetRef(joint[i], 0);
    }    

}


/**
* @brief 将电机和imu的数据 放进LegParam结构体和chassisParam结构体
* 
* @note LK9025电机逆时针旋转为正
* 
*/
static void set_leg_data()
{
    chassis.yaw = chassis_imu_data->yaw_total_angle * DEGREE_2_RAD;
    // chassis.target_yaw = chassis.yaw + chassis_cmd_recv.offset_angle * DEGREE_2_RAD;
    chassis.wz = chassis_imu_data->gyro[2];
    chassis.roll = chassis_imu_data->roll * DEGREE_2_RAD;
    chassis.roll_w = chassis_imu_data->gyro[1];
    
    // DM8009 电机的角度是逆时针为正 ,这里顺时针转是增加角度
    // LK9025 电机的角度是逆时针为正 ，右轮电机速度为正。
    //小腿 phi1 180  大腿 phi4 0 
    l_side.phi1 = ( 180  + lf->measure.real_total_angle  -9)*PI/180 ;
    l_side.phi1_angle = 180 + lf->measure.real_total_angle -9;
    l_side.phi1_w =  lf->measure.velocity;
    l_side.phi4 = (  0 +  lb->measure.real_total_angle +8 )*PI/180 ;
    l_side.phi4_angle = 0 + lb->measure.real_total_angle +8;
    l_side.phi4_w =   lb->measure.velocity;

    l_side.w_ecd = l_driven->measure.speed_rads;
    l_side.pitch   =  chassis_imu_data->pitch * DEGREE_2_RAD;
    l_side.pitch_w =  chassis_imu_data->gyro[0];

    //小腿phi 0  大腿phi1 180
    r_side.phi1 = (180 + rb->measure.real_total_angle  -4) *PI/180;
    r_side.phi1_angle = (180 + rb->measure.real_total_angle  -10);
    r_side.phi1_w =  rb->measure.velocity;
    r_side.phi4 = ( 0 + rf->measure.real_total_angle + 9) *PI/180;
    r_side.phi4_angle = ( 0  + rf->measure.real_total_angle +9);
    r_side.phi4_w =   rf->measure.velocity;

    // LK9025电机顺时针为负 +
    r_side.w_ecd = r_driven->measure.speed_rads;
    
    r_side.pitch   = -(chassis_imu_data->pitch ) * DEGREE_2_RAD;
    r_side.pitch_w = -chassis_imu_data->gyro[0];
}


static void SynthesizeMotion() /* 腿部控制:抗劈叉; 轮子控制:转向 */
{
    // float p_ref = PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
    // PIDCalculate(&steer_v_pid, chassis.wz, p_ref);
    // l_side.real_T_wheel = l_side.real_T_wheel - steer_v_pid.Output;
    // r_side.real_T_wheel = r_side.real_T_wheel + steer_v_pid.Output;


    // // 抗劈叉
    // static float swerving_speed_ff, ff_coef = 3;
    // swerving_speed_ff = ff_coef * steer_v_pid.Output; // 用于抗劈叉的前馈
    // PIDCalculate(&anti_crash_pid, l_side.phi0 + r_side.phi0 - PI, 0);
    // l_side.T_real_hip = l_side.T_hip + anti_crash_pid.Output ;//- swerving_speed_ff;
    // r_side.T_real_hip = r_side.T_hip - anti_crash_pid.Output ;//- swerving_speed_ff;
}
 
/************************************** leg Control **************************************/

static void leg_control() /* 腿长控制和Roll补偿 */
{
    PIDCalculate(&roll_compensate_pid, chassis.roll, 0);
    l_side.target_len += roll_compensate_pid.Output;
    r_side.target_len -= roll_compensate_pid.Output;

    static float gravity_ff = 52.34;
    static float roll_extra_comp_p = 400;
    float roll_comp = roll_extra_comp_p * chassis.roll;
    l_side.F_leg = 90 + PIDCalculate(&leg_len_pid_l, l_side.height, chassis_cmd_recv.l_target_len);
    r_side.F_leg = 90 + PIDCalculate(&leg_len_pid_r, r_side.height, chassis_cmd_recv.r_target_len);
}


static void set_working_state()
{

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) 
    {
        chassis.yaw = chassis_cmd_recv.offset_angle;
        chassis.target_v = chassis_cmd_recv.vx;

        // 目标速度置0
        // chassis.target_v = 0;
        chassis.dist = chassis.target_dist = 0;
        // return;
    }
    else if(chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW)
    {
        chassis.target_v = chassis_cmd_recv.vx;

    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW) // 底盘跟随
    {
        chassis.target_yaw = chassis_cmd_recv.offset_angle;
    }
    // EnableAllMotor();
}


/************************************** Main Task **************************************/
void ChassisTask(void *argument)
{
    for(;;)
    {
        xQueueReceive(chassis_recv_queue, &chassis_cmd_recv, 0);
        // xQueueReceive(imu_queue, &Chassis_IMU_data, 0);
        // // DMMotorEnable(rf);
        // // DMMotorEnable(rb);
        // // static uint32_t safety_counter = 0;
            
        del_t = dwt_get_delta_time(&balance_dwt_cnt);
            
        // 1. 设置工作状态
        set_working_state();
        // 2. 观测数据
        set_leg_data();

        // 3. 运动学计算
        phi_transform_theta(&l_side, &chassis);
     
        phi_transform_theta(&r_side, &chassis);
        calculate_leg_theta_w(&l_side,&chassis);
        calculate_leg_theta_w(&r_side,&chassis);

        // 4. 通过卡尔曼滤波估计机体速度
        observe_speed(&l_side, &r_side, &chassis, &chassis_imu_data, del_t);
        
        // 5. lqr 算 T_Hip
        set_left_leg_six_states(&l_side, &chassis)  ;
        set_right_leg_six_states(&r_side, &chassis) ;

        calculate_wheel_torgue(&l_side,  &chassis); 
        calculate_wheel_torgue(&r_side, &chassis); 
        
        leg_control(); 


        calculate_leg_torgue(&l_side); 
        calculate_leg_torgue(&r_side); 

        SynthesizeMotion();


        // // 6. calculate_leg_torgue 算 T_front、T_back
        // calculate_leg_torgue(&l_side);
        // calculate_leg_torgue(&r_side);

        // SynthesizeMotion();
        // 7. 算支持力
        // calculate_support_force(&l_side, &Chassis_IMU_data);
        // calculate_support_force(&r_side, &Chassis_IMU_data);



        if(chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW)
        {
            // LKMotorSetRef(r_driven, 0) ; 
            // LKMotorSetRef(l_driven, 0) ; 
            // DMMotorSetFFTorque(lb, 0)  ; 
            // DMMotorSetFFTorque(lf, 0)  ; 
            // DMMotorSetFFTorque(rf, 0)  ; 
            // DMMotorSetFFTorque(rb, 0)  ; 

            LKMotorSetRef(l_driven,l_side.T_wheel*124.12);
            LKMotorSetRef(r_driven,r_side.T_wheel*124.12);

            DMMotorSetFFTorque(lb, l_side.T_back )  ;
            DMMotorSetFFTorque(lf, l_side.T_front ) ;
            DMMotorSetFFTorque(rf, r_side.T_back )  ; 
            DMMotorSetFFTorque(rb, r_side.T_front)  ;
        }
        else
        {
            LKMotorSetRef(r_driven, 0)   ;
            LKMotorSetRef(l_driven, 0)   ;
            DMMotorSetFFTorque(lb, 0) ;
            DMMotorSetFFTorque(lf, 0) ;
            DMMotorSetFFTorque(rf, 0 ) ; 
            DMMotorSetFFTorque(rb, 0)  ;
        }
        osDelay(1);
    }
}
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

static LKMotorInstance *driven_r, *driven_l, *driven[2]; // 左右驱动轮
static DMMotorInstance *joint_lf, *joint_lb, *joint_rf, *joint_rb,*joint[4]; // 双马达
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

static PIDInstance adaptive_pid_l;
static PIDInstance adaptive_pid_r;


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
    driven[0] = driven_r = LKMotorInit(&driven_conf);

    driven_conf.can_init_config.can_handle = &hfdcan1,
    driven_conf.can_init_config.tx_id = 0x02;
    driven[1] = driven_l = LKMotorInit(&driven_conf);
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
    joint[RF] = joint_rf = DMMotorInit(&r_joint_conf);
    r_joint_conf.can_init_config.tx_id = 0x02;
    r_joint_conf.can_init_config.rx_id = 0x12;
    joint[RB] = joint_rb = DMMotorInit(&r_joint_conf);

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
    joint[LB] = joint_lb = DMMotorInit(&l_joint_conf);
    l_joint_conf.can_init_config.tx_id = 0x04;
    l_joint_conf.can_init_config.rx_id = 0x14;
    joint[LF] = joint_lf = DMMotorInit(&l_joint_conf);


// /*******************************LEG_PID_INIT******************************* */
    // 腿长控制
    PID_Init_Config_s l_leg_length_pid_conf = {
        .Kp = 700,
        .Kd = 50,
        .Ki = 0,
        .MaxOut = 60,
        .DeadBand = 0.0001f,
        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&leg_len_pid_l, &l_leg_length_pid_conf);

    PID_Init_Config_s r_leg_length_pid_conf = {
        .Kp = 700,
        .Kd = 50,
        .Ki = 0,
        .MaxOut = 60,
        .DeadBand = 0.0001f,
        .Improve = PID_ChangingIntegrationRate | PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.05,
    };
    PIDInit(&leg_len_pid_r, &r_leg_length_pid_conf);

    PID_Init_Config_s roll_compensate_pid_conf = {
        .Kp = 0.008f,
        .Kd = 0.002f,
        .Ki = 0.0f,
        .MaxOut = 0.2,
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
        .Kp = 3 ,
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
        .Kp = 15,         //15
        .Kd = 0,         //2
        .Ki = 0.0,
        .MaxOut = 30,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_ChangingIntegrationRate | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&anti_crash_pid, &anti_crash_pid_conf);

    PID_Init_Config_s adaptive_pid_conf = {
        .Kp = 0,
        .Kd = 0,
        .Ki = 0.3,
        .MaxOut = 40,
        .IntegralLimit = 20,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit,
        .DeadBand = 0.0001f,
    };
    PIDInit(&adaptive_pid_l, &adaptive_pid_conf);
    PIDInit(&adaptive_pid_r, &adaptive_pid_conf);


    chassis.target_v = 0;
    r_side.real_T_wheel = 0;
    r_side.real_T_front = 0;
    r_side.real_T_back = 0;
    l_side.real_T_front = 0;
    l_side.real_T_back = 0;
    l_side.real_T_wheel = 0;
    // chassis.vel_cov = 100;
    observe_speed_init(&chassis);
}
/************************************** Motor Control **************************************/

static void set_leg_data()
{
    chassis.yaw = chassis_imu_data->yaw_total_angle * DEGREE_2_RAD;
    // chassis.target_yaw = chassis.yaw + chassis_cmd_recv.offset_angle * DEGREE_2_RAD;
    chassis.wz = chassis_imu_data->gyro[2];
    chassis.roll = chassis_imu_data->roll * DEGREE_2_RAD;
    chassis.roll_w = chassis_imu_data->gyro[1];
    
    // DM8009 电机的角度是逆时针为正 ,这里顺时针转是增加角度
    // LK9025 电机的角度是逆时针为正 ，右轮电机速度为正。
    //大腿 phi1 180  小腿 phi4 0 
    l_side.phi1 = ( 180  + joint_lf->measure.real_total_angle  -40)*PI/180 ;
    l_side.phi1_angle = 180 + joint_lf->measure.real_total_angle -40;
    l_side.phi1_w =  joint_lf->measure.velocity;
    l_side.phi4 = (  0 +  joint_lb->measure.real_total_angle )*PI/180 ;
    l_side.phi4_angle = 0 + joint_lb->measure.real_total_angle ;
    l_side.phi4_w =   joint_lb->measure.velocity;

    l_side.w_ecd = driven_l->measure.speed_rads;
    l_side.pitch   =  chassis_imu_data->pitch * DEGREE_2_RAD;
    l_side.pitch_w =  chassis_imu_data->gyro[0];


    //大腿phi4 0  小腿phi1 180
    r_side.phi1 = (180 + joint_rb->measure.real_total_angle -10) *PI/180;
    r_side.phi1_angle = (180 + joint_rb->measure.real_total_angle  -10);
    r_side.phi1_w =  joint_rb->measure.velocity;
    r_side.phi4 = ( 0 + joint_rf->measure.real_total_angle ) *PI/180;
    r_side.phi4_angle = ( 0  + joint_rf->measure.real_total_angle );
    r_side.phi4_w =   joint_rf->measure.velocity;

    // LK9025电机顺时针为负 +
    r_side.w_ecd = driven_r->measure.speed_rads;
    
    r_side.pitch   = -(chassis_imu_data->pitch ) * DEGREE_2_RAD;
    r_side.pitch_w = -chassis_imu_data->gyro[0];
}


static void SynthesizeMotion() /* 腿部控制:抗劈叉; 轮子控制:转向 */
{
    if(chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {

        float p_ref = PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
        PIDCalculate(&steer_v_pid, chassis.wz, p_ref);
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) // 小陀螺
    {
        PIDCalculate(&steer_v_pid, chassis.wz, 2);
    }
    float vel_error_l = l_side.wheel_state[3];
    float vel_error_r = r_side.wheel_state[3];
    if(fabsf(l_side.real_T_wheel) < 2)
    {
        PIDCalculate(&adaptive_pid_l, vel_error_l, 0);
    }
    if(fabsf(r_side.real_T_wheel) < 2)
    {
        PIDCalculate(&adaptive_pid_r, vel_error_r, 0);
    }

    if(l_side.fly_flag==1||r_side.fly_flag ==1)
    {
        l_side.real_T_wheel =0;
    }
    else
    {
        l_side.real_T_wheel = l_side.T_wheel - steer_v_pid.Output + adaptive_pid_l.Output;
        r_side.real_T_wheel = r_side.T_wheel - steer_v_pid.Output + adaptive_pid_r.Output;
    }


    // // 抗劈叉
    // static float swerving_speed_ff, ff_coef = 3;
    // swerving_speed_ff = ff_coef * steer_v_pid.Output; // 用于抗劈叉的前馈
    PIDCalculate(&anti_crash_pid, l_side.phi0 + r_side.phi0 - PI, 0);
    l_side.T_motion_hip = anti_crash_pid.Output ;//- swerving_speed_ff;
    r_side.T_motion_hip = anti_crash_pid.Output ;//- swerving_speed_ff;
}
 
/************************************** leg Control **************************************/
static void stop_state()
{
    chassis.yaw = chassis_imu_data->yaw_total_angle;
    chassis.target_v = 0;
    chassis.dist = chassis.target_dist = 0;

    l_side.F_leg = 0;
    r_side.F_leg = 0;

    l_side.T_hip = 0;
    r_side.T_hip = 0;

    l_side.T_wheel = 0 ;
    r_side.T_wheel = 0 ;
}
static void recover_leg_length()
{
    l_side.F_leg = 0 +  PIDCalculate(&leg_len_pid_l, l_side.leg_len, chassis_cmd_recv.l_target_len);
    r_side.F_leg = 0 +  PIDCalculate(&leg_len_pid_r, r_side.leg_len, chassis_cmd_recv.r_target_len);

    l_side.T_hip = 0;//l_side.T_lqr_hip;
    r_side.T_hip = 0;//r_side.T_lqr_hip;

    l_side.T_wheel = 0 ;
    r_side.T_wheel = 0 ;
}

static void follow_state()
{
    float p_ref = PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
    l_side.T_motion_wheel = PIDCalculate(&steer_v_pid, chassis.wz, p_ref);
    r_side.T_motion_wheel = PIDCalculate(&steer_v_pid, chassis.wz, p_ref);
}

static void rotate_state()
{
    l_side.T_motion_wheel = PIDCalculate(&steer_v_pid, chassis.wz, 2);
    r_side.T_motion_wheel = PIDCalculate(&steer_v_pid, chassis.wz, 2);
}
static void standup_state()
{
    chassis.yaw = chassis_imu_data->yaw_total_angle;
    chassis.target_v = 0;
    chassis.dist = chassis.target_dist = 0;

    l_side.F_leg = 50 +  PIDCalculate(&leg_len_pid_l, l_side.leg_len, chassis_cmd_recv.l_target_len);
    r_side.F_leg = 50 +  PIDCalculate(&leg_len_pid_r, r_side.leg_len, chassis_cmd_recv.r_target_len);

    l_side.T_hip = 0.1 * l_side.T_lqr_hip;
    r_side.T_hip = 0.1 * r_side.T_lqr_hip;

    l_side.T_wheel = l_side.T_lqr_wheel - l_side.T_motion_wheel;
    r_side.T_wheel = r_side.T_lqr_wheel - r_side.T_motion_wheel;
}

static void balance_state()
{
    static float roll_extra_comp_p = 200;
    float roll_comp=0;
    roll_comp = roll_extra_comp_p * chassis.roll;
    l_side.F_leg = 50 +  PIDCalculate(&leg_len_pid_l, l_side.leg_len, l_side.target_len) + roll_comp;
    r_side.F_leg = 50 +  PIDCalculate(&leg_len_pid_r, r_side.leg_len, r_side.target_len) - roll_comp;

    PIDCalculate(&roll_compensate_pid, chassis.roll, 0);
    l_side.target_len = chassis_cmd_recv.l_target_len - roll_compensate_pid.Output;
    r_side.target_len = chassis_cmd_recv.r_target_len + roll_compensate_pid.Output;

    PIDCalculate(&anti_crash_pid, l_side.phi0 + r_side.phi0 - PI, 0);
    l_side.T_motion_hip = anti_crash_pid.Output ;//- swerving_speed_ff;
    r_side.T_motion_hip = anti_crash_pid.Output ;//- swerving_speed_ff;
    
    l_side.T_hip = l_side.T_lqr_hip + l_side.T_motion_hip;
    r_side.T_hip = r_side.T_lqr_hip + r_side.T_motion_hip;

    if(fabsf(l_side.real_T_wheel) < 2)
    {
        PIDCalculate(&adaptive_pid_l, l_side.wheel_state[3], 0);
    }
    if(fabsf(r_side.real_T_wheel) < 2)
    {
        PIDCalculate(&adaptive_pid_r, r_side.wheel_state[3], 0);
    }

    
    l_side.T_wheel = l_side.T_lqr_wheel - l_side.T_motion_wheel + adaptive_pid_l.Output;
    r_side.T_wheel = r_side.T_lqr_wheel - r_side.T_motion_wheel + adaptive_pid_r.Output;
}

static void set_working_state()
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) 
    {
        stop_state();
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_STAND_UP) 
    {
        if(l_side.leg_len <=0.24 &&r_side.leg_len <=0.24)
        {
            if(fabsf(l_side.theta)<0.3&&fabsf(r_side.theta)<0.3)
            {
                // balance_state();
            }
            else
            {
                // standup_state();
            }
        }
        else
        {
            recover_leg_length();
        }
    }
    else if(chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis.target_v = chassis_cmd_recv.vx;
        chassis.target_yaw = chassis_cmd_recv.offset_angle;
        follow_state();
        balance_state();
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE) // 底盘跟随
    {
        rotate_state();
        balance_state();
    }
}


/************************************** Main Task **************************************/
void ChassisTask(void *argument)
{
    for(;;)
    {
        xQueueReceive(chassis_recv_queue, &chassis_cmd_recv, 0);
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

        calculate_leg_torgue(&l_side);
        calculate_leg_torgue(&r_side);

        SynthesizeMotion();
        // 7. 算支持力
        calculate_support_force(&l_side, chassis_imu_data);
        calculate_support_force(&r_side, chassis_imu_data);

        // if(chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW|| chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)
        // {
            // LKMotorSetRef(driven_r, 0)   ;
            // LKMotorSetRef(driven_l, 0)   ;
            // DMMotorSetFFTorque(joint_lb, 0)  ;
            // DMMotorSetFFTorque(joint_lf, 0)  ;
            // DMMotorSetFFTorque(joint_rf, 0)  ; 
            // DMMotorSetFFTorque(rb, 0)  ;

            LKMotorSetRef(driven_l,l_side.T_wheel*124.12);
            LKMotorSetRef(driven_r,r_side.T_wheel*124.12);

            DMMotorSetFFTorque(joint_lb, l_side.T_front )  ;
            DMMotorSetFFTorque(joint_lf, l_side.T_back ) ;
            DMMotorSetFFTorque(joint_rf, r_side.T_front )  ; 
            DMMotorSetFFTorque(joint_rb, r_side.T_back)  ;
        // }
        // else
        // {
            // LKMotorSetRef(driven_r, 0)   ;
            // LKMotorSetRef(driven_l, 0)   ;
            // DMMotorSetFFTorque(joint_lb, 0) ;
            // DMMotorSetFFTorque(joint_lf, 0) ;
            // DMMotorSetFFTorque(joint_rf, 0 ) ; 
            // DMMotorSetFFTorque(joint_rb, 0)  ;
        // }
        osDelay(1);
    }
}
// app
#include "buzzer.h"
#include "daemon.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "board2board.h"
// module
#include "remote_control.h"
#include "bmi088.h"
#include "minipc_comm.h"

#include "general_def.h"
#include "dji_motor.h"
#include "referee_pt.h"
// bsp
#include "bsp_dwt.h"
#include "usart.h"
#include <math.h>



/* cmd应用包含的模块实例指针和交互信息存储*/
/************************************** HuartUsed **************************************/
static RC_ctrl_t *rc_data;                      // 遥控器数据,初始化时返回
static Minipc_Recv_s *minipc_data;
static referee_ctrl_t *rfc_data;
/**************************************  CANUsed  **************************************/
static CANCommInstance *cmd_can_comm;

/************************************** ChassisUsed **************************************/
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;
static Shoot_Ctrl_Cmd_s shoot_cmd_send;
// static Gimbal_Upload_Data_s gimbal_fetch_data;

static TaskHandle_t robot_cmd_task_handle = NULL;
static BuzzerInstance *buzzer;

static QueueHandle_t cmd_control_chassis_queue;
static QueueHandle_t cmd_control_gimbal_queue;
static QueueHandle_t cmd_control_shoot_queue;

const osThreadAttr_t robot_cmd_task_attributes = {
    .name = "robot_cmd_task",
    .stack_size = 1024*2,
    .priority = (osPriority_t) osPriorityNormal,
};

/********************************************************************************************
***************************************      Init     ***************************************
*********************************************************************************************/
void RobotCMDInit(RobotCtrlQueues_t *control_queue)
{
/**************************************  HuartInit  **************************************/
    rc_data = RemoteControlInit(&huart5);       // 遥控器通信串口初始化
    // minipc_data = minipcInit(&huart1);
    // rfc_data = RefereeControlInit(&huart7);

    CreateUsartTask();
    // static QueueHandle_t buzzer_queue = NULL;
    // buzzer_queue = xQueueCreate(10, sizeof(Buzzer_Event_t));
    // buzzer = BuzzerRegister(buzzer_queue);
    // CreateDaemon(buzzer);
    gimbal_cmd_send.yaw = 30;
    chassis_cmd_send.l_target_len = 0.15;
    chassis_cmd_send.r_target_len = 0.15;
    cmd_control_gimbal_queue  = control_queue->control_gimbal_queue;
    #if defined (CHASSIS_BOARD) || defined (CHASSIS_BOARD_CONTROL_CHASSIS)
    cmd_control_chassis_queue = control_queue->control_chassis_queue;
    #endif
    cmd_control_shoot_queue = control_queue->control_shoot_queue;
    robot_cmd_task_handle = osThreadNew(RobotCMDTask,NULL,&robot_cmd_task_attributes);
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;

    //注册了一个can通信实例
    CANComm_Init_Config_s comm_conf = {
        .can_config = 
        {
            .can_handle = &hfdcan1,
            .tx_id = 0x300,
            .rx_id =  0x309,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

}
// static void CalcOffsetAngle()
// {
//         // @todo:相差一整圈时会出问题，待修复
//     // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
//     uint16_t yaw_chassis_align_ecd;
//     yaw_chassis_align_ecd = 2716;

//     static float angle, yaw_align_angle;
//     yaw_align_angle = yaw_chassis_align_ecd * ECD_ANGLE_COEF_DJI; // 从底盘获取的yaw电机对齐角度
//     angle = gimbal_fetch_data.yaw_motor_single_round_angle;       // 从云台获取的当前yaw电机单圈角度

//     if (yaw_chassis_align_ecd > 4096) // 如果大于180度
//     {
//         if (angle > yaw_align_angle)
//             chassis_cmd_send.offset_angle = angle - yaw_align_angle;
//         else if (angle <= yaw_align_angle && angle >= yaw_align_angle - 180.0f)
//             chassis_cmd_send.offset_angle = angle - yaw_align_angle;
//         else
//             chassis_cmd_send.offset_angle = angle - yaw_align_angle + 360.0f;
//     }
//     else
//     { // 小于180度
//         if (angle > yaw_align_angle && angle <= 180.0f + yaw_align_angle)
//             chassis_cmd_send.offset_angle = angle - yaw_align_angle;
//         else if (angle > 180.0f + yaw_align_angle)
//             chassis_cmd_send.offset_angle = angle - yaw_align_angle - 360.0f;
//         else
//             chassis_cmd_send.offset_angle = angle - yaw_align_angle;
//     }
// }

#ifdef CHASSIS_BOARD_CONTROL_CHASSIS
static void CalcOffsetAngle()
{

    if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        // chassis_cmd_send.r_target_len -=  0.000002f*rc_data->rc.rocker_left_y;
        // chassis_cmd_send.l_target_len -=  0.000002f*rc_data->rc.rocker_left_y;

        // chassis_cmd_send.vx = 0.003f * (float)rc_data[TEMP].rc.rocker_left_x;
        // chassis_cmd_send.l_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_right_x;
        // chassis_cmd_send.r_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_right_x;
        if (switch_is_down((rc_data[TEMP].rc.switch_left)))
        {
            
            chassis_cmd_send.change_length +=0.000004 * rc_data->rc.rocker_left_y;
            chassis_cmd_send.turn_position +=0.000004 * rc_data->rc.rocker_right_y;
        }

        if (switch_is_mid((rc_data[TEMP].rc.switch_left)))
        {
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        }
    }
    
    else
    {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        if(chassis_cmd_send.fly_flag)
        {

        }
        else
        {
            chassis_cmd_send.vx = 0.003f * (float)rc_data[TEMP].rc.rocker_left_x;
            chassis_cmd_send.l_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_right_x;
            chassis_cmd_send.r_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_right_x;
        }
    }
}
#endif

#ifdef GIMBAL_BOARD
static void BasicSet()
{
    //云台基本模式设定
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    // 云台软件限位
    if(gimbal_cmd_send.pitch<PITCH_MIN_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MIN_ANGLE;
    else if (gimbal_cmd_send.pitch>PITCH_MAX_ANGLE)
    gimbal_cmd_send.pitch=PITCH_MAX_ANGLE;
    else
    gimbal_cmd_send.pitch=gimbal_cmd_send.pitch;

    //发射基本模式设定
    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_ON;
    shoot_cmd_send.loader_mode = LOADER_STOP;
    //射频
    shoot_cmd_send.shoot_rate = 8;
}

static void RemoteControlSet()
{
    BasicSet();
    gimbal_cmd_send.yaw = 30 + 2* sin(DWT_GetTimeline_s()); 
    //底盘x、y速度设定：
    chassis_cmd_send.vx =10.0f * (float)rc_data[TEMP].rc.rocker_left_y; // _水平方向
    chassis_cmd_send.vy =-10.0f * (float)rc_data[TEMP].rc.rocker_left_x; // 竖直方向

    // 左侧开关状态[下],底盘跟随云台
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.yaw -= 0.0006f * (float)rc_data[TEMP].rc.rocker_right_x;//0
        gimbal_cmd_send.pitch += 0.0002f * (float)rc_data[TEMP].rc.rocker_right_y;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_left)) 
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.yaw   -= minipc_data->Vision.yaw;   //往右获得的yaw是减
        gimbal_cmd_send.pitch -= minipc_data->Vision.pitch;
    }
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial > 500)
    {
        shoot_cmd_send.loader_mode = LOADER_BURSTFIRE;
    }
    else
    {
        shoot_cmd_send.loader_mode = LOADER_STOP;
    }
}


static void RKeymouseSet()
{
    BasicSet();
    chassis_cmd_send.vx = 0.55 * (rfc_data[TEMP].key[KEY_PRESS].w - rfc_data[TEMP].key[KEY_PRESS].s) * 20000; // 系数待测，平移运动功率限制！
    chassis_cmd_send.vy = 0.55 * (rfc_data[TEMP].key[KEY_PRESS].a  - rfc_data[TEMP].key[KEY_PRESS].d )* 20000;
    gimbal_cmd_send.yaw   -= (float)rfc_data[TEMP].mouse.x / 660 * 3 ; 
    gimbal_cmd_send.pitch -= (float)rfc_data[TEMP].mouse.y / 660 * 3 ; 


    switch (rfc_data[TEMP].mouse.press_l) // 鼠标左键射击
    {
    case 0:
        shoot_cmd_send.loader_mode = LOADER_STOP;
        break;
    default:
        shoot_cmd_send.loader_mode = LOADER_BURSTFIRE;
        if(chassis_fetch_data.over_heat_flag==1)
        {
            shoot_cmd_send.loader_mode = LOADER_STOP;
            break;
        }
        break;
    }
    switch (rfc_data[TEMP].key_count[KEY_PRESS][Key_Q] % 2) // Q键设置底盘运动模式
    {
    case 0:
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        break;
    default:
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
        break;
    }
}

















#endif
/*********************************************************************************************
***************************************      TASK      ***************************************
**********************************************************************************************/
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask(void *argument)
{
    for(;;)
    {
        chassis_cmd_send.vx = 1;
        // CalcOffsetAngle();
        #ifdef GIMBAL_BOARD
        RemoteControlSet();
        #endif // GIMBAL_BOARD
        // xQueueSend(cmd_control_gimbal_queue, &gimbal_cmd_send, NULL);
        // xQueueSend(cmd_control_shoot_queue, &shoot_cmd_send, NULL);
        // CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
        // xQueueSend(cmd_control_chassis_queue, &chassis_cmd_send, NULL);
        osDelay(3);
    }
}
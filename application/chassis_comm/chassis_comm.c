// app
#include "chassis_comm.h"
#include "buzzer.h"
#include "daemon.h"
#include "robot_def.h"

#include "board2board.h"
#include "usart_task.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "minipc_comm.h"
#include "referee_mc_task.h"
#include "general_def.h"
#include "dji_motor.h"
#include "board2board.h"
// bsp
#include "bsp_dwt.h"
#include "usart.h"



/* cmd应用包含的模块实例指针和交互信息存储*/
/************************************** HuartUsed `**************************************/
static RC_ctrl_t *rc_data;                      // 遥控器数据,初始化时返回
static Minipc_Recv_s *minipc_data;
static referee_info_t * referee_data;
/**************************************  CANUsed  **************************************/
static CANCommInstance *chassis_can_comm;

/**************************************ChassisUsed**************************************/
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;
static Shoot_Ctrl_Cmd_s shoot_cmd_send;
// static Gimbal_Upload_Data_s gimbal_fetch_data;



static TaskHandle_t robot_cmd_task_handle = NULL;
static BuzzerInstance *buzzer;

static QueueHandle_t cmd_control_chassis_queue;

const osThreadAttr_t chassis_comm_task_attributes = {
    .name = "chassis_comm_task",
    .stack_size = 1024*2,
    .priority = (osPriority_t) osPriorityNormal,
};

/********************************************************************************************
***************************************      Init     ***************************************
*********************************************************************************************/
void ChassisCMDInit(RobotCtrlQueues_t *control_queue)
{
/**************************************  HuartInit  **************************************/
    #ifdef CHASSIS_BOARD_CONTROL_CHASSIS
    rc_data = RemoteControlInit(&huart5);       // 遥控器通信串口初始化
    #endif // DEBUG

    create_usart_task();

    static QueueHandle_t buzzer_queue = NULL;
    buzzer_queue = xQueueCreate(10, sizeof(Buzzer_Event_t));
    buzzer = BuzzerRegister(buzzer_queue);
    CreateDaemon(buzzer);

    chassis_cmd_send.l_target_len = 0.23;
    chassis_cmd_send.r_target_len = 0.22;
    #if defined (CHASSIS_BOARD) || defined (CHASSIS_BOARD_CONTROL_CHASSIS)
    cmd_control_chassis_queue = control_queue->control_chassis_queue;
    #endif

    robot_cmd_task_handle = osThreadNew(ChassisCMDTask,NULL,&chassis_comm_task_attributes);
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hfdcan1,
            .tx_id = 0x309,
            .rx_id = 0x300,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chassis_can_comm = CANCommInit(&comm_conf); // can comm初始化
}

#ifdef CHASSIS_BOARD_CONTROL_CHASSIS
static void BasicSet()
{
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        // chassis_cmd_send.motor_rb -=  0.000002f*rc_data->rc.rocker_left_y;
        // chassis_cmd_send.motor_rf -=  0.000002f*rc_data->rc.rocker_left_x;
        // chassis_cmd_send.motor_lf -=  0.000002f*rc_data->rc.rocker_right_y;
        // chassis_cmd_send.motor_lb -=  0.000002f*rc_data->rc.rocker_right_x;
        // chassis_cmd_send.l_target_len -= 0.000002f*rc_data->rc.rocker_right_y;
        // chassis_cmd_send.r_target_len -= 0.000002f*rc_data->rc.rocker_right_y;

        // chassis_cmd_send.turn_position -=  0.000002f*rc_data->rc.rocker_left_y;
        // chassis_cmd_send.change_length -=  0.000002f*rc_data->rc.rocker_right_y;
        chassis_cmd_send.vx = 0.002f * (float)rc_data[TEMP].rc.rocker_left_y;
        chassis_cmd_send.l_target_len -= 0.000002f*(float)rc_data[TEMP].rc.rocker_right_y;
        chassis_cmd_send.r_target_len -= 0.000002f*(float)rc_data[TEMP].rc.rocker_right_y;
        if(chassis_cmd_send.l_target_len>= 0.3)
        {
            chassis_cmd_send.l_target_len =0.3;
        }
        else if(chassis_cmd_send.l_target_len<= 0.18)
        {
            chassis_cmd_send.l_target_len =0.18;
        }
        else
        {
            chassis_cmd_send.l_target_len = chassis_cmd_send.l_target_len;
        }
        
        if(chassis_cmd_send.r_target_len>= 0.3)
        {
            chassis_cmd_send.r_target_len =0.3;
        }
        else if(chassis_cmd_send.r_target_len<= 0.21)
        {
            chassis_cmd_send.r_target_len =0.21;
        }
        else
        {
            chassis_cmd_send.r_target_len = chassis_cmd_send.r_target_len;
        }
        chassis_cmd_send.offset_angle -= 0.000002f * (float)rc_data[TEMP].rc.rocker_right_x;
        // chassis_cmd_send.l_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_right_x;
        // chassis_cmd_send.r_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_right_x;
        // chassis_cmd_send.l_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_left_y;
        // chassis_cmd_send.r_target_len += 0.000005f*(float)rc_data[TEMP].rc.rocker_right_y;
        // chassis_cmd_send.offset_angle +=  0.0005f*(float)rc_data[TEMP].rc.rocker_left_x;
        if (switch_is_down((rc_data[TEMP].rc.switch_left)))
        {
            chassis_cmd_send.chassis_mode  = CHASSIS_ZERO_FORCE;
            
        }
        if (switch_is_mid((rc_data[TEMP].rc.switch_left)))
        {
            chassis_cmd_send.chassis_mode  = CHASSIS_NO_FOLLOW;
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

        }
    }
}
#endif

#ifdef CHASSIS_BOARD


#endif
/*********************************************************************************************
***************************************      TASK      ***************************************
**********************************************************************************************/
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void ChassisCMDTask(void *argument)
{
    for(;;)
    {
        #ifdef CHASSIS_BOARD
        chassis_cmd_send = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chassis_can_comm);
        #endif

        #ifdef CHASSIS_BOARD_CONTROL_CHASSIS
        BasicSet();

        #endif // CHASSIS_BOARD_CONTROL_CHASSIS
        xQueueSend(cmd_control_chassis_queue, &chassis_cmd_send, NULL);

        osDelay(3);
    }
}
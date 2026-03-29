#include "bsp_init.h"
#include "robot.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "chassis.h"
#include "motor_task.h"
#include "gimbal.h"
#include "chassis_comm.h"
// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_DEF_PARAM_WARNING
#define ROBOT_DEF_PARAM_WARNING
#pragma message "check if you have configured the parameters in robot_def.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif // !ROBOT_DEF_PARAM_WARNING

#include "chassis.h"
#include "dmmotor.h"

void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    
    BSPInit();

    RobotCtrlQueues_t     cmd_queues;
    #if defined(CHASSIS_BOARD_CONTROL_CHASSIS)  ||defined (CHASSIS_BOARD)

    ChassisQueues_t   chassis_queues;
    cmd_queues.control_chassis_queue = xQueueCreate(10, sizeof(Chassis_Ctrl_Cmd_s));
    chassis_queues.chassis_recv_queue = cmd_queues.control_chassis_queue;
    ChassisInit(&chassis_queues);              
    ChassisCMDInit(&cmd_queues);
    MotorTaskInit();
    DMMotorControlInit();
    #endif // DEBUG

    #if defined (GIMBAL_BOARD) 

    GimbalQueues_t     gimbal_queues;
    ShootQueues_t       shoot_queues;
    cmd_queues.control_gimbal_queue = xQueueCreate(10, sizeof(Gimbal_Ctrl_Cmd_s));
    cmd_queues.control_shoot_queue = xQueueCreate(10, sizeof(Shoot_Ctrl_Cmd_s));
    gimbal_queues.gimbal_recv_queue = cmd_queues.control_gimbal_queue;
    shoot_queues.shoot_recv_queue   = cmd_queues.control_shoot_queue;
    // chassis_queues.chassis_feedback_queue = xQueueCreate(10, sizeof(Chassis_Upload_Data_s));
    // cmd_queues.chassis_fetch_queue = chassis_queues.chassis_feedback_queue;

    RobotCMDInit(&cmd_queues);
    GimbalInit(&gimbal_queues);


    #endif // DEBUG

}

// void RobotTask()
// {
//     // RobotCMDTask();
//     BalanceTask();
// }
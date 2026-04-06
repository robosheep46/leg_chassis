#include "bsp_init.h"
#include "robot.h"

#include "robot_def.h"
#include "chassis.h"
#include "motor_task.h"

#include "chassis_comm.h"


#include "chassis.h"
#include "dmmotor.h"

void RobotInit()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用dwt_delay()
    __disable_irq();
    
    BSPInit();

    RobotCtrlQueues_t     cmd_queues;
    ChassisQueues_t   chassis_queues;
    cmd_queues.control_chassis_queue = xQueueCreate(10, sizeof(Chassis_Ctrl_Cmd_s));
    chassis_queues.chassis_recv_queue = cmd_queues.control_chassis_queue;
    ChassisInit(&chassis_queues);
    ChassisCMDInit(&cmd_queues);
    MotorTaskInit();
    DMMotorControlInit();
}
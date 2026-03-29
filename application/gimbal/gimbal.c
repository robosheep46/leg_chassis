#include "gimbal.h"
#include "dji_motor.h"
#include "fdcan.h"
#include "gimbal_algorithm.h"


#include "motor_def.h"
#include "robot_def.h"
#include "general_def.h"
#include "buzzer.h"
#include "bmi088.h"
#include "stdbool.h"
// static attitude_t *gimbal_IMU_data; // 云台IMU数据
static BMI088Instance *gimbal_IMU_data;
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data;
static TaskHandle_t gimbal_task_handle = NULL;
static QueueHandle_t gimbal_recv_queue = NULL;
static QueueHandle_t gimbal_feedback_queue = NULL;

const osThreadAttr_t gimbal_task_attributes = {
    .name = "gimbal_task",
    .stack_size = 1024*2,
    .priority = (osPriority_t) osPriorityNormal,
};
void GimbalInit(GimbalQueues_t *gimbal_queue)
{
    gimbal_task_handle = osThreadNew(GimbalTask, NULL,&gimbal_task_attributes);
    gimbal_recv_queue = gimbal_queue->gimbal_recv_queue;
    gimbal_feedback_queue = gimbal_queue->gimbal_feedback_queue;

    gimbal_IMU_data = BMI088Register();

    Motor_Init_Config_s yaw_config = {
        .can_init_config = 
        {
            .can_handle = &hfdcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 1.5, // 2
                .Ki = 2,
                .Kd = 0.17,//0.0705
                .CoefA=5,
                .CoefB=8,
                .DeadBand = 0.01,
                .Improve = PID_ChangingIntegrationRate | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 250,  // 50
                .Ki = 0, // 200
                .Kd = 0,
                .Improve = PID_ChangingIntegrationRate | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->yaw_total_angle,
            
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    yaw_motor = DJIMotorInit(&yaw_config);

    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hfdcan2,
            .tx_id =  2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 2, // 10
                .Ki = 4,
                .CoefA=3,
                .CoefB=1,
                .Improve = PID_Derivative_On_Measurement|PID_ChangingIntegrationRate,
                .IntegralLimit = 100,
                .MaxOut = 5000,
            },
            .speed_PID = {
                .Kp = -250,  // 50
                .Ki = 0, // 350
                .Kd = 0,   // 0
                .Improve =PID_ChangingIntegrationRate | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config =
         {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    pitch_motor = DJIMotorInit(&pitch_config);
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */


void GimbalTask(void *argument)
{
    for(;;)
    {
        xQueueReceive(gimbal_recv_queue, &gimbal_cmd_recv, 0);
        float feedforward = Cal_FollowControl_Feedforward(*gimbal_IMU_data, gimbal_cmd_recv);
        float yaw_ref = Cal_FollowControl_Set(*gimbal_IMU_data,gimbal_cmd_recv);
        // xQueueSend(gimbal_feedback_queue, &gimbal_fetch_data, 0);
        switch (gimbal_cmd_recv.gimbal_mode)
        {
            // 停止
            case GIMBAL_ZERO_FORCE:
                DJIMotorStop(yaw_motor);
                DJIMotorStop(pitch_motor);

                break;
            // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
            case GIMBAL_GYRO_MODE: // 后续只保留此模式
                DJIMotorEnable(yaw_motor);
                DJIMotorEnable(pitch_motor);
                DJIMotorSetRef(yaw_motor, yaw_ref);
                DJIMotorSetSpeedFeedForward(yaw_motor, feedforward);
                break;
            default:
                break;
        }
    }
}


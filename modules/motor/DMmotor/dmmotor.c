#include "dmmotor.h"
#include "cmsis_os2.h"
#include "memory.h"
#include "general_def.h"
#include "motor_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include <stddef.h>
#include <stdint.h>
#include "stdio.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static TaskHandle_t dm_task_handle;

static uint8_t enable_flag[4]={0};

static CANInstance dm_sender_assignment[4] = {
    [0] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x01, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x02, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x03, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x04, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
};

static void group_motor(DMMotorInstance *motor, CAN_Init_Config_s *config)
{
    //直接用->运算符访问电机的成员再进行操作，开销会更多   
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    if(motor->motor_can_instace->tx_id == 0x01)
    {   
        motor_grouping = 0;
        motor->motor_can_instace->txconf.Identifier = 0x101;
        enable_flag[0]=1;
        dm_sender_assignment[0].tx_id = motor->motor_can_instace->tx_id;
    }
    else if(motor->motor_can_instace->tx_id == 0x02)
    {
        motor_grouping = 1;
        motor->motor_can_instace->txconf.Identifier = 0x102;
        enable_flag[1]=1;
        dm_sender_assignment[1].tx_id = motor->motor_can_instace->tx_id;
    }
    else if(motor->motor_can_instace->tx_id == 0x03)
    {
        motor_grouping = 2;
        motor->motor_can_instace->txconf.Identifier = 0x103;
        enable_flag[2]=1;
        dm_sender_assignment[2].tx_id = motor->motor_can_instace->tx_id;
    }
    else if(motor->motor_can_instace->tx_id == 0x04)
    {
        motor_grouping = 3;
        motor->motor_can_instace->txconf.Identifier = 0x104;
        enable_flag[3]=1;
        dm_sender_assignment[3].tx_id = motor->motor_can_instace->tx_id;
    }
    motor->sender_group = motor_grouping;
}



/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void set_dm_motor_mit_mode(dm_motor_mit_mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    can_transmit(motor->motor_can_instace, 1);
}

void dmmotor_set_control_mode(DMMotorInstance *motor,dm_motor_control_mode_e mode)
{
    motor->control_mode = mode;
}

static void dmmotor_decode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    measure->last_torque = measure->torque;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
    if (measure->position - measure->last_position > 12)
    {
        measure->total_round--;
    }
    else if (measure->position - measure->last_position < -12)
    {
        measure->total_round++;
    }
    measure->angle_single_round = ECD_ANGLE_COEF_DM * (float)measure->position;
    //总角度等于圈数*360°+当前角度
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
    if(motor->motor_type == DM8009)
    {
        measure->real_total_angle =  measure->total_angle*4;
        float temp_angle = measure->angle_single_round * 4;
        measure->real_total_round = (int32_t)(temp_angle / 360.0f);
        measure->real_angle_single_round = temp_angle - measure->real_total_round * 360.0f;
    }
    else if (motor->motor_type == DM4310)
    {
        measure->real_total_angle =  measure->total_angle*3;
        float temp_angle = measure->angle_single_round * 3;
        measure->real_total_round = (int32_t)(temp_angle / 360.0f);
        measure->real_angle_single_round = temp_angle - measure->real_total_round * 360.0f;
    }



    // 规范化到[-180, 180]范围
    if(measure->real_angle_single_round > 180.0f) 
    {
        measure->real_angle_single_round -= 360.0f;
        measure->real_total_round++;
    }
    else if(measure->real_angle_single_round < -180.0f) 
    {
        measure->real_angle_single_round += 360.0f;
        measure->real_total_round--;
    }
    if(motor->init_flag == 0)
    {
        motor->init_flag = 1;
        set_dm_motor_mit_mode(DM_CMD_MOTOR_MODE, motor);    
        dwt_delay(0.1);
    }
}

static void dmmotor_lost_callback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    // set_dm_motor_mit_mode(DM_CMD_MOTOR_MODE, motor);
    // dwt_delay(0.1);
    // motor->init_flag = 0;
    memset(&(motor->measure), 0, sizeof(motor->measure));
}

static void dm_motor_other_error_callback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    motor->other_error_flag = 1;
}

void dmmotor_cali_encoder(DMMotorInstance *motor)
{
    set_dm_motor_mit_mode(DM_CMD_ZERO_POSITION, motor);
    dwt_delay(0.1);
}
DMMotorInstance *dmmotor_init(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_type = config->motor_type;
    motor->mit_flag=config->mit_flag;
    motor->motor_can_instace->tx_id = config->can_init_config.tx_id+0x100;
    motor->motor_can_instace->rx_id = config->can_init_config.rx_id;
    config->can_init_config.can_module_callback = dmmotor_decode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = can_register(&config->can_init_config);
    group_motor(motor,&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .owner_callback = dmmotor_lost_callback,
        .other_modules_error_callback = dm_motor_other_error_callback,
        .owner_id = motor,
        .reload_count = 50,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    dmmotor_enable(motor);
    set_dm_motor_mit_mode(DM_CMD_MOTOR_MODE, motor);    
    dwt_delay(0.1);
    // dmmotor_cali_encoder(motor);
    dm_motor_instance[idx++] = motor;
    return motor;
}
void dmmotor_set_position(DMMotorInstance *motor, float position,float speed)
{
    dmmotor_set_control_mode(motor, POSITION_MODE);
    motor->motor_controller.angle_PID.Ref = position;
    motor->motor_controller.speed_PID.Ref = speed;
}

void dmmotor_set_torque(DMMotorInstance *motor, float Torque)
{
    if(Torque>=18)
    {
        motor->motor_controller.current_PID.Ref = 18;
    }
    else if(Torque<=-18)
    {
        motor->motor_controller.current_PID.Ref = -18;
    }
    else
    {
        motor->motor_controller.current_PID.Ref = Torque;
    }
}

void dmmotor_enable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void dmmotor_stop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->motor_controller.current_PID.Ref = 0;
    motor->stop_flag = MOTOR_STOP;
}

static void dmmotor_mit_mode(DMMotorInstance *motor)
{
    motor->motor_can_instace->txconf.Identifier = motor->motor_can_instace->tx_id;
    uint16_t torque_uint;
    if (motor->other_error_flag || motor->stop_flag == MOTOR_STOP) 
    {
        torque_uint = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
    }
    else 
    {
        torque_uint = float_to_uint(motor->motor_controller.current_PID.Ref, DM_T_MIN, DM_T_MAX, 12);
    }

    uint8_t *buf = motor->motor_can_instace->tx_buff;
    buf[0] = 0;
    buf[1] = 0;                                
    buf[2] = 0;                               
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    buf[6] = (torque_uint >> 8) & 0x0F;            // Torque 高 4 位
    buf[7] = torque_uint & 0xFF;                   // Torque 低 8 位
    if (motor->init_flag == 1)
    {
        can_transmit(motor->motor_can_instace, 1);
    }
}

static void dmmotor_position_mode(DMMotorInstance *motor)
{
    motor->motor_can_instace->txconf.Identifier = motor->motor_can_instace->tx_id + 0x100;

    float p_des = motor->motor_controller.angle_PID.Ref;
    float v_des = motor->motor_controller.speed_PID.Ref;

    // 若电机处于停止状态，强制速度给定为 0
    if (motor->stop_flag == MOTOR_STOP) 
    {
        v_des = 0.0f;
    }

    uint8_t *buf = motor->motor_can_instace->tx_buff;
    memcpy(buf,      &p_des, 4);
    memcpy(buf + 4,  &v_des, 4);
    // 发送 CAN 帧
    can_transmit(motor->motor_can_instace, 1);
}
void DMMotorTask(void *argument)
{
    for(;;)
    {
        DMMotorInstance *motor;

        float set;        // 电机控制CAN发送设定值
        Motor_Control_Setting_s *setting; // 电机控制参数
        Motor_Controller_s *motor_controller;   // 电机控制器
        DM_Motor_Measure_s *measure;           // 电机测量值
        float pid_measure, pid_ref;             // 电机PID测量值和设定值
        uint8_t group, num; // 电机组号和组内编号

        for (int i = 0; i < DM_MOTOR_CNT; i++)
        {
            DMMotorInstance *motor = dm_motor_instance[i];
            dmmotor_position_mode(motor);

            // // 根据每个电机的控制模式选择发送函数
            // if (motor->control_mode == MIT_MODE)
            // {
            //     // dmmotor_mit_mode(motor);
            // }
            // else if (motor->control_mode == POSITION_MODE)
            // {
            //     dmmotor_position_mode(motor);
            // }
        }
        osDelay(1);
    }
}

const osThreadAttr_t yaw_4310_task_attributes = {
    .name = "dm_task",
    .stack_size = 1024*2,
    .priority = (osPriority_t) osPriorityNormal,
};


void dmmotor_task_init()
{
    dm_task_handle = osThreadNew(DMMotorTask, NULL, &yaw_4310_task_attributes);

}
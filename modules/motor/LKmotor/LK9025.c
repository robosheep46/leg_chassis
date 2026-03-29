#include "LK9025.h"
#include "fdcan.h"
#include "stdlib.h"
#include "general_def.h"
#include "daemon.h"
#include "bsp_dwt.h"
#include <stdint.h>

static uint8_t idx;
static LKMotorInstance *lkmotor_instance[LK_MOTOR_MX_CNT] = {NULL};
static CANInstance lk_sender_assignment[2] = {
    [0] = {.can_handle = &hfdcan2, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},

};

static void MotorSenderGrouping(LKMotorInstance *motor, CAN_Init_Config_s *config)
{
    //直接用->运算符访问电机的成员再进行操作，开销会更多   
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    if(motor->motor_can_ins->tx_id == 0x141)
    {   
        motor_grouping = 0;
        lk_sender_assignment[0].txconf.Identifier = 0x141;
    }
    else if(motor->motor_can_ins->tx_id == 0x142)
    {
        motor_grouping = 1;

        lk_sender_assignment[1].txconf.Identifier = 0x142;
    }
    motor->sender_group = motor_grouping;

}

/**
 * @brief 电机反馈报文解析
 *
 * @param _instance 发生中断的caninstance
 */
static void LKMotorDecode(CANInstance *_instance)
{
    LKMotorInstance *motor = (LKMotorInstance *)_instance->id; // 通过caninstance保存的father id获取对应的motorinstance
    LKMotor_Measure_t *measure = &motor->measure;
    uint8_t *rx_buff = _instance->rx_buff;

    DaemonReload(motor->daemon); // 喂狗
    measure->feed_dt = dwt_get_delta_time(&measure->feed_dwt_cnt);

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)((rx_buff[7] << 8) | rx_buff[6]);

    measure->angle_single_round = ECD_ANGLE_COEF_LK * measure->ecd;

    measure->speed_rads = (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
                          DEGREE_2_RAD * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));
    measure->speed_angle =  (1 - SPEED_SMOOTH_COEF) * measure->speed_rads +
                          SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[5] << 8 | rx_buff[4]));
    measure->real_current = (1 - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[3] << 8 | rx_buff[2]));
    measure->temperature = rx_buff[1];

    if (measure->ecd - measure->last_ecd > 32768)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -32768)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void LKMotorLostCallback(void *motor_ptr)
{
    LKMotorInstance *motor = (LKMotorInstance *)motor_ptr;
}

LKMotorInstance *LKMotorInit(Motor_Init_Config_s *config)
{
    LKMotorInstance *motor = (LKMotorInstance *)malloc(sizeof(LKMotorInstance));
    motor = (LKMotorInstance *)malloc(sizeof(LKMotorInstance));
    memset(motor, 0, sizeof(LKMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;


    config->can_init_config.id = motor;
    config->can_init_config.can_module_callback = LKMotorDecode;
    config->can_init_config.rx_id = config->can_init_config.tx_id+0x140;
    config->can_init_config.tx_id = config->can_init_config.tx_id+0x140;
    motor->motor_can_ins = can_register(&config->can_init_config);
    MotorSenderGrouping(motor,config);
    LKMotorEnable(motor);
    dwt_get_delta_time(&motor->measure.feed_dwt_cnt);
    lkmotor_instance[idx++] = motor;

    Daemon_Init_Config_s daemon_config = {
        .callback = LKMotorLostCallback,
        .owner_id = motor,
        .reload_count = 50, // 50ms
    };
    motor->daemon = DaemonRegister(&daemon_config);

    return motor;
}

/* 第一个电机的can instance用于发送数据,向其tx_buff填充数据 */
void LKMotorControl()
{
    float pid_measure, pid_ref;
    int16_t set;
    LKMotorInstance *motor;
    LKMotor_Measure_t *measure;
    Motor_Control_Setting_s *setting;
    uint8_t group, num; // 电机组号和组内编号

    for (size_t i = 0; i < idx; ++i)
    {   
        motor = lkmotor_instance[i];
        pid_ref = motor->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        set = (int16_t)pid_ref;        
        group = motor->sender_group;
        // // 设置tx_buff[0]为0xa1
        lk_sender_assignment[group].tx_buff[0] = 0xA1 ;
        lk_sender_assignment[group].tx_buff[1] = 0;
        lk_sender_assignment[group].tx_buff[2] = 0;
        lk_sender_assignment[group].tx_buff[3] = 0;
        lk_sender_assignment[group].tx_buff[4] = (uint8_t)(set & 0x00ff);  // 低八位;
        lk_sender_assignment[group].tx_buff[5] = (uint8_t)(set >> 8);       
        lk_sender_assignment[group].tx_buff[6] = 0;
        lk_sender_assignment[group].tx_buff[7] = 0;
    }

    for(uint8_t i=0 ;i<2;i++)
    {
        can_transmit(&lk_sender_assignment[i], 1);
    }
}

void LKMotorStop(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void LKMotorEnable(LKMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void LKMotorSetRef(LKMotorInstance *motor, float ref)
{
    if(ref <= I_MIN)
    {
        ref = I_MIN;
    }
    else if(ref >= I_MAX)
    {
        ref = I_MAX;
    }
    else
    {
        ref = ref;
    }
    motor->pid_ref = ref;
}

uint8_t LKMotorIsOnline(LKMotorInstance *motor)
{
    return DaemonIsOnline(motor->daemon);
}

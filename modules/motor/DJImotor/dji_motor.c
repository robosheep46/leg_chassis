#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
// #include "bsp_log.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算
static float m2006_init_flag=1;

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用9个(3hfcan*3group)can_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  因为只用于发送,所以不需要在bsp_can中注册
 *
 */
static CANInstance sender_assignment[9] = {
    [0] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .txconf.Identifier =
    
    
     0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [6] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [7] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
    [8] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = 0x08, .tx_buff = {0}},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 */
static uint8_t sender_enable_flag[9] = {0};

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1;
    //直接用->运算符访问电机的成员再进行操作，开销会更多   
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type) 
    {

    /*因为2006电机和3508电机的控制(tx_id):0x1ff,0x200;所以将其分在一组
    因为一条指令可以控制4个电机,所以将句柄一致且id为1-4的电机放在同一组，
    5-7分为第二组,因为data从0开始,所以id为5-8的电机，motor_id要-4，确保从1开始*/
    case M2006:
    case M3508:
        if (motor_id <= 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            if(config->can_handle == &hfdcan1)
            {
                motor_grouping = 1;
            }
            else if(config->can_handle == &hfdcan2)
            {
                motor_grouping = 4;
            }
            else if(config->can_handle == &hfdcan3)
            {
                motor_grouping = 7;
            }
        }
        else
        {
            motor_send_num = motor_id - 4;
            if(config->can_handle == &hfdcan1)
            {
                motor_grouping = 0;
            }
            else if(config->can_handle == &hfdcan2)
            {
                motor_grouping = 3;
            }
            else if(config->can_handle == &hfdcan3)
            {
                motor_grouping = 6;
            }
        }

        //3508及2006电机反馈报文格式中的rx_id=0x200+id
        config->rx_id = 0x200 + motor_id+1;
        //只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
        sender_enable_flag[motor_grouping] = 1;
        //直接用指针的话，开销会更大     
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        // 检查是否发生id冲突
        // 注意 6020的id 1-4和2006/3508的id 5-8会发生冲突（因为0x200+5=0x204+1）
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                // LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                while (1)  
                // LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
                    ;
            }
        }
        break;

    case GM6020:
        if (motor_id <= 4)
        {
            motor_send_num = motor_id;
            if(config->can_handle == &hfdcan1)
            {
                motor_grouping = 0;
            }
            else if(config->can_handle == &hfdcan2)
            {
                motor_grouping = 3;
            }
            else if(config->can_handle == &hfdcan3)
            {
                motor_grouping = 6;
            }        
        }
        else
        {
            motor_send_num = motor_id - 4;
            if(config->can_handle == &hfdcan1)
            {
                motor_grouping = 2;
            }
            else if(config->can_handle == &hfdcan2)
            {
                motor_grouping = 5;
            }
            else if(config->can_handle == &hfdcan3)
            {
                motor_grouping = 8;
            }        
        }

        config->rx_id = 0x204 + motor_id+1 ;
        sender_enable_flag[motor_grouping] = 1;
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                while (1) // 6020的id 1-4和2006/3508的id 5-8会发生冲突(若有注册,即1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    ;
            }
        }
        break;
    default: // other motors should not be registered here
        while (1)
            ;
    }
}

/**
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    // 将can instance的id强制转换成DJIMotorInstance,从而获得电机的instance实例地址
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    // 再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    // measure要多次使用,保存指针减小访存开销
    DJI_Motor_Measure_s *measure = &motor->measure;
    //喂狗，程序卡死后还能重置
    //初始化已经将deamon的初始化信息传入到daemon中
    //20ms不起作用就进丢失回调即重启程序
    // DaemonReload(motor->daemon);
    //计算两次进入同一个函数的时间间隔
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);
    /******** 解析数据电机的反馈报文详见djmotor.md ********/
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    //将编码值改成角度值
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    //加个滤波，去除那些一瞬间的高频噪声，并将速度rpm转换成°/s
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    //加个滤波，去除那些一瞬间的高频噪声
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));

    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°
    //ecd范围是0-8191，（8191再加1等于0）
    //如果这一次比上一次还要再多的比4096（半圈）还多，说明他从一个很小的数变成了一个很大的数，说明他少了一圈
    if (measure->ecd - measure->last_ecd > 4096)
    {
        measure->total_round--;
    }
    //如果这一次比上一次还要再小的比4096还多，说明他从一个很大的数变成了一个很小的数，说明他多了一圈
    else if (measure->ecd - measure->last_ecd < -4096)
    {
        measure->total_round++;
    }
    //总角度等于圈数*360°+当前角度
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
    
    //主要用于计算拨盘转过的角度，进而算出发弹量
    if(m2006_init_flag==1 && motor->motor_type == M2006 )
    {
        measure->init_total_angle= motor->measure.total_angle;//记录初始化时2006电机角度
        m2006_init_flag++;
    }
}

//重新获取电机的指针
static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    DJIMotorEnable(motor);
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    // 对电机实例进行内存分配
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    // 将电机实例的数据清零
    memset(instance, 0, sizeof(DJIMotorInstance));
    
    // 6020 or 2006 or 3508
    instance->motor_type = config->motor_type;                         
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
 
    MotorSenderGrouping(instance, &config->can_init_config);

    //can_module_callback是函数指针，所以不用->进行访问，因为他已经规定了函数类型为(CANInstance *)，
    //所以直接等于DecodeDJIMotor即可
    config->can_init_config.can_module_callback = DecodeDJIMotor; 
    config->can_init_config.id = instance;
    // 注册电机到CAN总线                        
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // // 注册守护线程，未收到数据，调用DJIMotorLostCallback
    // Daemon_Init_Config_s daemon_config = {
    //     .callback = DJIMotorLostCallback,
    //     .owner_id = instance,
    //     .reload_count = 2, // 20ms未收到数据则丢失
    // };
    // instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    // else
    // ;
        // LOGERROR("[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的LOOP类型,或发生了指针越界
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    if(motor->motor_settings.control_algorithm==PID_MODE)
    motor->motor_controller.pid_ref = ref;
    else
    motor->motor_controller.lqr_ref = ref;
}

void DJIMotorSetSpeedFeedForward(DJIMotorInstance *motor, float speed_feedforward)
{
    *motor->motor_controller.speed_feedforward_ptr = speed_feedforward;
}

// 为所有电机实例计算三环PID,发送控制报文
void DJIMotorControl()
{
    // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值
    
    // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    {   
         // 减小访存开销,先保存指针引用
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if (motor_setting->control_algorithm==PID_MODE)
        {
            if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
            {
                if (motor_setting->angle_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                else
                    pid_measure = measure->total_angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
                // 更新pid_ref进入下一个环
                pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
            }
            pid_ref = pid_ref + *motor_controller->speed_feedforward_ptr; // 速度前馈

            // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
            if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            {
                if (motor_setting->speed_feedback_source == OTHER_FEED)
                    pid_measure = *motor_controller->other_speed_feedback_ptr;
                else // MOTOR_FEED
                    pid_measure = measure->speed_aps;
                // 更新pid_ref进入下一个环
                pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
            }
            
            if (motor_setting->close_loop_type & CURRENT_LOOP)
            {
                pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
            }

            if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                pid_ref *= -1;

            // 获取最终输出
            set = (int16_t)pid_ref;        
        }
        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP)
        memset(sender_assignment[group].tx_buff + 2 * num, 0, 16u);
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 8; ++i)
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}

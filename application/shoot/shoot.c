#include "shoot.h"
#include "robot_def.h"
#include "buzzer.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
//static  BuzzzerInstance *shoot_motor_error_buzzer;

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
static int heat_data[2];

/*弹量及热量计算*/
//拨盘电机数据
static float last_loader_total_angle_1;//用于储存上一次计算时角度
static float last_loader_total_angle;//用于储存上一次计算时角度
static float loader_total_angle;//当前角度

static uint8_t shoot_init_flag=1;//用于读取初始化电机角度
static uint8_t over_heat_flag=0;//用于判断是否超热量
static uint8_t first_flag=1;


//枪口热量数据
static uint16_t shoot_cooling_val;//每秒冷却值
static uint16_t shoot_heat=0;//枪口当前热量(计算值)
static uint16_t heat_k=1.2;//控制热量系数
static float last_bullet_speed = 0;
//读取裁判系统时间间隔
static float last_time;
static float time;

static TaskHandle_t shoot_task_handle = NULL;
static QueueHandle_t shoot_recv_queue = NULL;
static QueueHandle_t shoot_feedback_queue = NULL;
const osThreadAttr_t shoot_task_attributes = {
    .name = "shoot_task",
    .stack_size = 1024*2,
    .priority = (osPriority_t) osPriorityNormal,
};


void ShootInit(ShootQueues_t *shoot_queue)
{
    shoot_task_handle = osThreadNew(ShootTask, NULL,&shoot_task_attributes);
    shoot_recv_queue = shoot_queue->shoot_recv_queue;
    shoot_feedback_queue = shoot_queue->shoot_feedback_queue;






    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = 
        {
            .can_handle = &hfdcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 1,
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hfdcan1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 10, // 10
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 
    };

    loader_config.can_init_config.tx_id=4;
    loader = DJIMotorInit(&loader_config);

}

static void ShootSpeedSet()
{
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        DJIMotorSetRef(friction_l, 20000);
        DJIMotorSetRef(friction_r, 20000);
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
}


void ShootModeSet()
{
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }
}
void ShootRateSet()
{
    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.loader_mode)
    {
    // 停止拨盘
    case LOADER_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 连发模式,对速度闭环
    case LOADER_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 4);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环
    case LOADER_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, -1000);

        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
}
/* 机器人发射机构控制核心任务 */
void ShootTask(void *argument)
{
    for(;;)
    {
        xQueueReceive(shoot_recv_queue, &shoot_cmd_recv, 0);
    }
}
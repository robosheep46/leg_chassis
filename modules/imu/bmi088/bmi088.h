#ifndef __BMI088_H__ // 防止重复包含
#define __BMI088_H__

#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "controller.h"
#include "bsp_pwm.h"
#include "stdint.h"
#include <math.h>
#include "cmsis_os.h"
#include "queue.h"

#define X 0
#define Y 1
#define Z 2

typedef struct {
    float yaw;
    float pitch;
    float roll;
} attitude_t;

typedef enum {
    BMI088_RESULT_OK = 0,
    BMI088_RESULT_ERROR,
    BMI088_RESULT_NULL_PTR,
    BMI088_RESULT_COMM_ERROR,
    BMI088_RESULT_DATA_INVALID
} BMI088_Result_e;

// bmi088标定方式枚举,若使用预设标定参数,注意修改预设参数
typedef enum
{
    BMI088_CALIBRATE_ONLINE_MODE = 0, // 初始化时进行标定
    BMI088_LOAD_PRE_CALI_MODE,        // 使用预设标定参数,
} BMI088_Calibrate_Mode_e;

#pragma pack(1) // 1字节对齐
// typedef struct
// {
//     float Gyro[3];  // 角速度
//     float Accel[3]; // 加速度
//     float Roll;
//     float Pitch;
//     float Yaw;
//     float YawTotalAngle;
// } attitude_t; // 最终解算得到的角度,以及yaw转动的总角度(方便多圈控制)

typedef struct
{
    float q[4]; // 四元数估计值

    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数
    float DGyroLPF; // 角加速度低通滤波系数

    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;

    // IMU量测值
    float Gyro[3];  // 角速度
    float dgyro[3]; // 角加速度

    float Accel[3]; // 加速度
    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;

    uint8_t init;
} INS_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;


#pragma pack() // 恢复默认对齐,需要传输的结构体务必开启1字节对齐

/* BMI088实例结构体定义 */
typedef struct
{
    // 传输模式和工作模式控制
    BMI088_Calibrate_Mode_e cali_mode;
    // SPI接口
    SPIInstance *spi_gyro; // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    SPIInstance *spi_acc;  // 注意,SPIInstnace内部也有一个GPIOInstance,用于控制片选CS
    // EXTI GPIO,如果BMI088工作在中断模式,则需要配置中断引脚(有数据产生时触发解算)
    GPIOInstance *gyro_int;
    GPIOInstance *acc_int;
    // 温度控制
    PIDInstance heat_pid; // 恒温PID
    PWMInstance *heat_pwm; // 加热PWM
    // IMU数据
    float gyro[3];     // 陀螺仪数据,xyz
    float dgyro[3]; // 角加速度
    float acc[3];      // 加速度计数据,xyz
    float temperature; // 温度
    float MotionAccel_b[3]; // 机体坐标加速度

    // 标定数据
    float gyro_offset[3]; // 陀螺仪零偏
    float gNorm;          // 重力加速度模长,从标定获取
    float acc_coef;       // 加速度计原始数据转换系数
    // 传感器灵敏度,用于计算实际值(regNdef.h中定义)
    float BMI088_ACCEL_SEN;
    float BMI088_GYRO_SEN;
    // 用于计算两次采样的时间间隔
    uint32_t bias_dwt_cnt;
    // 数据更新标志位
    uint8_t imu_ready; // 1:IMU数据准备好,0:IMU数据未准备好(gyro+acc)
    float yaw;
    float pitch;
    float roll;
    float yaw_total_angle;
} BMI088Instance;

/* BMI088初始化配置 */
typedef struct
{
    BMI088_Calibrate_Mode_e cali_mode;
    SPI_Init_Config_s spi_gyro_config;
    SPI_Init_Config_s spi_acc_config;
    GPIO_Init_Config_s gyro_int_config;
    GPIO_Init_Config_s acc_int_config;
    PID_Init_Config_s heat_pid_config;
    PWM_Init_Config_s heat_pwm_config;
} BMI088_Init_Config_s;

/**
 * @brief 初始化BMI088,返回BMI088实例指针
 * @note  一般一个开发板只有一个BMI088,所以这里就叫BMI088Init而不是Register
 *
 * @param config bmi088初始化配置
 * @return BMI088Instance* 实例指针
 */
 QueueHandle_t BMI088Register();


/**
 * @brief 标定传感器.BMI088在初始化的时候会调用此函数. 提供接口方便标定离线数据
 * @attention @todo 注意,当操作系统开始运行后,此函数会和bmi088冲突.目前不允许在运行时调用此函数,后续加入标志位判断以提供运行时重新的标定功能
 *
 * @param _bmi088 待标定的实例
 */
void BMI088CalibrateIMU(BMI088Instance *_bmi088);
BaseType_t BMI088DataReceiveFromQueue(attitude_t *attitude, TickType_t wait_time);

void InsTask(void *argument);

#endif // !__BMI088_H__
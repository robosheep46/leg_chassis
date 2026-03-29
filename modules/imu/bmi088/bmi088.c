#include "bmi088_regNdef.h"
#include "bmi088.h"
#include "user_lib.h"
#include "daemon.h"
#include "QuaternionEKF.h"
#include "robot_def.h"
static BMI088Instance *bmi088;
static attitude_t attitude;
static INS_t INS;
static float dt = 0, t = 0;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};
static IMU_Param_t IMU_Param;
static uint32_t INS_DWT_Count = 0;
static PIDInstance TempCtrl = {0};
static float RefTemp = 40; // 恒温设定温度

static QueueHandle_t bmi088_send_queue = NULL;

// 创建 INS 任务
osThreadAttr_t ins_task_attributes = {
    .name = "InsTask",
    .priority = osPriorityAboveNormal,
    .stack_size = 1024
};

osThreadId_t ins_task_handle;

// ---------------------------以下私有函数,用于读写BMI088寄存器封装,blocking--------------------------------//
/**
 * @brief 读取BMI088寄存器Accel. BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg 待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static void BMI088AccelRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    if (len > 6)
        while (1)
            ;
    // 一次读取最多6个字节,加上两个dummy data    第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
    static uint8_t tx[8]; // 读取,第一个字节为0x80|reg ,第二个是dummy data,后面的没用都是dummy write
    static uint8_t rx[8]; // 前两个字节是dummy data,第三个开始是真正的数据
    tx[0] = 0x80 | reg;   // 静态变量每次进来还是上次的值,所以要每次都要给tx[0]赋值0x80
    SPITransRecv(bmi088->spi_acc, rx, tx, len + 2);
    memcpy(dataptr, rx + 2, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
}

/**
 * @brief 读取BMI088寄存器Gyro, BMI088要求在不释放CS的情况下连续读取
 *
 * @param bmi088 待读取的BMI088实例
 * @param reg  待读取的寄存器地址
 * @param dataptr 读取到的数据存放的指针
 * @param len 读取长度
 */
static void BMI088GyroRead(BMI088Instance *bmi088, uint8_t reg, uint8_t *dataptr, uint8_t len)
{
    if (len > 6)
        while (1)
            ;
    // 一次读取最多6个字节,加上一个dummy data  ,第一个字节的第一个位是读写位,1为读,0为写,1-7bit是寄存器地址
    static uint8_t tx[7] = {0x80}; // 读取,第一个字节为0x80 | reg ,之后是dummy data
    static uint8_t rx[7];          // 第一个是dummy data,第三个开始是真正的数据
    
    tx[0] = 0x80 | reg;
    SPITransRecv(bmi088->spi_gyro, rx, tx, len + 1);
    memcpy(dataptr, rx + 1, len); // @todo : memcpy有额外开销,后续可以考虑优化,在SPI中加入接口或模式,使得在一次传输结束后不释放CS,直接接着传输
}

/**
 * @brief 写accel寄存器.对spitransmit形式上的封装
 * @attention 只会向目标reg写入一个字节,因为只有1个字节所以直接传值(指针是32位反而浪费)
 *
 * @param bmi088 待写入的BMI088实例
 * @param reg  待写入的寄存器地址
 * @param data 待写入的数据(注意不是指针)
 */
static void BMI088AccelWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_acc, tx, 2);
}

//原代码使用的是硬件的spi，现使用软件spi
static void BMI088GyroWriteSingleReg(BMI088Instance *bmi088, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    SPITransmit(bmi088->spi_gyro, tx, 2);
}
// -------------------------以上为私有函数,封装了BMI088寄存器读写函数,blocking--------------------------------//

// -------------------------以下为私有函数,用于初始化BMI088acc和gyro的硬件和配置--------------------------------//
#define BMI088REG 0
#define BMI088DATA 1
#define BMI088ERROR 2
// BMI088初始化配置数组for accel,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};
// BMI088初始化配置数组for gyro,第一列为reg地址,第二列为写入的配置值,第三列为错误码(如果出错)
static uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};
// @attention : 以上两个数组配合各自的初始化函数使用. 若要修改请参照BMI088 datasheet

/**
 * @brief 初始化BMI088加速度计,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE if any problems here
 */
static uint8_t BMI088AccelInit(BMI088Instance *bmi088)
{
    uint8_t whoami_check = 0;

    // 加速度计以I2C模式启动,需要一次上升沿来切换到SPI模式,因此进行一次fake write
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    DWT_Delay(0.001);

    BMI088AccelWriteSingleReg(bmi088, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE); // 软复位
    DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME / 1000);

    // 检查ID,如果不是0x1E(bmi088 whoami寄存器值),则返回错误
    BMI088AccelRead(bmi088, BMI088_ACC_CHIP_ID, &whoami_check, 1);
    if (whoami_check != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001);
    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Accel_Init_Table) / sizeof(BMI088_Accel_Init_Table[0]); i++)
    {
        reg = BMI088_Accel_Init_Table[i][BMI088REG];
        data = BMI088_Accel_Init_Table[i][BMI088DATA];
        BMI088AccelWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.01);
        BMI088AccelRead(bmi088, reg, &data, 1); // 写完之后立刻读回检查
        DWT_Delay(0.01);
        if (data != BMI088_Accel_Init_Table[i][BMI088DATA])
            error |= BMI088_Accel_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,如果retry次数用完了,则返回error
    }
    return error;
}

/**
 * @brief 初始化BMI088陀螺仪,提高可读性分拆功能
 *
 * @param bmi088 待初始化的BMI088实例
 * @return uint8_t BMI088ERROR CODE
 */
static uint8_t BMI088GyroInit(BMI088Instance *bmi088)
{

    BMI088GyroWriteSingleReg(bmi088, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE); // 软复位
    DWT_Delay(0.08);

    // 检查ID,如果不是0x0F(bmi088 whoami寄存器值),则返回错误
    uint8_t whoami_check = 0;
    BMI088GyroRead(bmi088, BMI088_GYRO_CHIP_ID, &whoami_check, 1);
    if (whoami_check != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;
    DWT_Delay(0.001);

    // 初始化寄存器,提高可读性
    uint8_t reg = 0, data = 0;
    BMI088_ERORR_CODE_e error = 0;
    // 使用sizeof而不是magic number,这样如果修改了数组大小,不用修改这里的代码;或者使用宏定义
    for (uint8_t i = 0; i < sizeof(BMI088_Gyro_Init_Table) / sizeof(BMI088_Gyro_Init_Table[0]); i++)
    {
        reg = BMI088_Gyro_Init_Table[i][BMI088REG];
        data = BMI088_Gyro_Init_Table[i][BMI088DATA];
        BMI088GyroWriteSingleReg(bmi088, reg, data); // 写入寄存器
        DWT_Delay(0.001);
        BMI088GyroRead(bmi088, reg, &data, 1); // 写完之后立刻读回对应寄存器检查是否写入成功
        DWT_Delay(0.001);
        if (data != BMI088_Gyro_Init_Table[i][BMI088DATA])
            error |= BMI088_Gyro_Init_Table[i][BMI088ERROR];
        //{i--;} 可以设置retry次数,尝试重新写入.如果retry次数用完了,则返回error
    }

    return error;
}
// -------------------------以下为公有函数,用于注册BMI088,标定和数据读取--------------------------------//

/**
 * @brief
 *
 * @param bmi088
 * @return BMI088_Data_t
 */
static uint8_t BMI088Acquire(BMI088Instance *bmi088)
{

    static uint8_t buf[6] = {0}; // 最多读取6个byte(gyro/acc,temp是2)
    // 读取accel的x轴数据首地址,bmi088内部自增读取地址 // 3* sizeof(int16_t)
    BMI088AccelRead(bmi088, BMI088_ACCEL_XOUT_L, buf, 6);
    for (uint8_t i = 0; i < 3; i++)
        bmi088->acc[i] = bmi088->acc_coef * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]);
    BMI088GyroRead(bmi088, BMI088_GYRO_X_L, buf, 6); // 连续读取3个(3*2=6)轴的角速度
    for (uint8_t i = 0; i < 3; i++)
        bmi088->gyro[i] = bmi088->BMI088_GYRO_SEN * (float)(int16_t)(((buf[2 * i + 1]) << 8) | buf[2 * i]);
    BMI088AccelRead(bmi088, BMI088_TEMP_M, buf, 2); // 读温度,温度传感器在accel上
        bmi088->temperature = (float)(int16_t)(((buf[0] << 3) | (buf[1] >> 5))) * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return 1;
    if (bmi088->imu_ready == 0)
        return 0;
    return 1;
}

/* pre calibrate parameter to go here */
#define BMI088_PRE_CALI_ACC_X_OFFSET 0.0f
#define BMI088_PRE_CALI_ACC_Y_OFFSET 0.0f
#define BMI088_PRE_CALI_ACC_Z_OFFSET 0.0f
#define BMI088_PRE_CALI_G_NORM 9.805f

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    if (!init_q4) return;
    
    float acc_init[3] = {0.0f, 0.0f, 0.0f};
    float gravity_norm[3] = {0.0f, 0.0f, 1.0f};
    float axis_rot[3] = {0.0f, 0.0f, 0.0f};
    
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088Acquire(bmi088);
        acc_init[X] += bmi088->acc[X];
        acc_init[Y] += bmi088->acc[Y];
        acc_init[Z] += bmi088->acc[Z];
        DWT_Delay(0.001);
    }
    
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100.0f;
    
    // 检查向量是否有效
    float acc_magnitude = sqrtf(acc_init[X]*acc_init[X] + 
                               acc_init[Y]*acc_init[Y] + 
                               acc_init[Z]*acc_init[Z]);
    
    if (acc_magnitude < 0.1f || acc_magnitude > 20.0f) {
        // 如果加速度数据异常，使用默认值
        init_q4[0] = 1.0f;
        init_q4[1] = 0.0f;
        init_q4[2] = 0.0f;
        init_q4[3] = 0.0f;
        return;
    }
    
    // 归一化
    acc_init[X] /= acc_magnitude;
    acc_init[Y] /= acc_magnitude;
    acc_init[Z] /= acc_magnitude;
    
    // 限制点积范围
    float dot_product = acc_init[X]*gravity_norm[X] + 
                       acc_init[Y]*gravity_norm[Y] + 
                       acc_init[Z]*gravity_norm[Z];
    dot_product = fmaxf(-1.0f, fminf(1.0f, dot_product));
    
    float angle = acosf(dot_product);
    
    // 检查angle是否有效
    if (isnan(angle) || isinf(angle)) {
        init_q4[0] = 1.0f;
        init_q4[1] = 0.0f;
        init_q4[2] = 0.0f;
        init_q4[3] = 0.0f;
        return;
    }
    
    Cross3d(acc_init, gravity_norm, axis_rot);
    
    // 检查旋转轴
    float axis_magnitude = sqrtf(axis_rot[X]*axis_rot[X] + 
                                axis_rot[Y]*axis_rot[Y] + 
                                axis_rot[Z]*axis_rot[Z]);
    
    if (axis_magnitude > 0.001f) {
        axis_rot[X] /= axis_magnitude;
        axis_rot[Y] /= axis_magnitude;
        axis_rot[Z] /= axis_magnitude;
        
        init_q4[0] = cosf(angle / 2.0f);
        init_q4[1] = axis_rot[X] * sinf(angle / 2.0f);
        init_q4[2] = axis_rot[Y] * sinf(angle / 2.0f);
        init_q4[3] = axis_rot[Z] * sinf(angle / 2.0f);
    } else {
        // 如果旋转轴几乎为零，说明已经对齐
        init_q4[0] = 1.0f;
        init_q4[1] = 0.0f;
        init_q4[2] = 0.0f;
        init_q4[3] = 0.0f;
    }
    
    // 检查四元数是否有效
    for (int i = 0; i < 4; i++) {
        if (isnan(init_q4[i]) || isinf(init_q4[i])) {
            init_q4[0] = 1.0f;
            init_q4[1] = 0.0f;
            init_q4[2] = 0.0f;
            init_q4[3] = 0.0f;
            break;
        }
    }
}

/**
 * @brief BMI088 acc gyro 标定，该代码对应原代码void Calibrate_MPU_Offset(IMU_Data_t *bmi088)
 * @note 标定后的数据存储在bmi088->bias和gNorm中,用于后续数据消噪和单位转换归一化
 * @attention 标定精度和等待时间有关,目前使用线性回归.后续考虑引入非线性回归
 * @todo 将标定次数(等待时间)变为参数供设定
 * @section 整体流程为1.累加加速度数据计算gNrom() 2.累加陀螺仪数据计算零飘
 *          3. 如果标定过程运动幅度过大,重新标定  4.保存标定参数
 *
 */
void BMI088CalibrateIMU(BMI088Instance *_bmi088)
{
    if (_bmi088->cali_mode == BMI088_CALIBRATE_ONLINE_MODE) // bmi088在线标定,耗时6s
    {
        _bmi088->acc_coef = BMI088_ACCEL_6G_SEN;         // 标定完后要乘以9.805/gNorm
        _bmi088->BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN; // 后续改为从initTable中获取
        // 一次性参数用完就丢,不用static
        float startTime;                     // 开始标定时间,用于确定是否超时
        uint16_t CaliTimes = 10000;           // 标定次数(6s)
        float gyroMax[3], gyroMin[3];        // 保存标定过程中读取到的数据最大值判断是否满足标定环境
        float gNormTemp, gNormMax, gNormMin; // 同上,计算矢量范数(模长)
        float gyroDiff[3], gNormDiff;        // 每个轴的最大角速度跨度及其模长

        startTime = DWT_GetTimeline_s();
        // 循环继续的条件为标定环境不满足
        do // 用do while至少执行一次,省得对上面的参数进行初始化
        {  // 标定超时,直接使用预标定参数(如果有)
            if (DWT_GetTimeline_s() - startTime > 12.01)
            { // 两次都没有成功就切换标定模式,丢给下一个if处理,使用预标定参数
                _bmi088->cali_mode = BMI088_LOAD_PRE_CALI_MODE;
                break;
            }

            DWT_Delay(0.0005);
            _bmi088->gNorm = 0;
            for (uint8_t i = 0; i < 3; i++) // 重置gNorm和零飘
                _bmi088->gyro_offset[i] = 0;

            // @todo : 这里也有获取bmi088数据的操作,后续与BMI088Acquire合并.注意标定时的工作模式是阻塞,且offset和acc_coef要初始化成0和1,标定完成后再设定为标定值
            for (uint16_t i = 0; i < CaliTimes; ++i) // 提前计算,优化
            {
                BMI088Acquire(_bmi088);
                gNormTemp = NormOf3d(_bmi088->acc);
                _bmi088->gNorm += gNormTemp; // 计算范数并累加,最后除以calib times获取单次值
                for (uint8_t ii = 0; ii < 3; ii++)
                    _bmi088->gyro_offset[ii] += _bmi088->gyro[ii]; // 因为标定时传感器静止,所以采集到的值就是漂移,累加当前值,最后除以calib times获得零飘

                if (i == 0) // 避免未定义的行为(else中)
                {
                    gNormMax = gNormMin = gNormTemp; // 初始化成当前的重力加速度模长
                    for (uint8_t j = 0; j < 3; ++j)
                    {
                        gyroMax[j] = _bmi088->gyro[j];
                        gyroMin[j] = _bmi088->gyro[j];
                    }
                }
                else // 更新gNorm的Min Max和gyro的minmax
                {
                    gNormMax = gNormMax > gNormTemp ? gNormMax : gNormTemp;
                    gNormMin = gNormMin < gNormTemp ? gNormMin : gNormTemp;
                    for (uint8_t j = 0; j < 3; ++j)
                    {
                        gyroMax[j] = gyroMax[j] > _bmi088->gyro[j] ? gyroMax[j] : _bmi088->gyro[j];
                        gyroMin[j] = gyroMin[j] < _bmi088->gyro[j] ? gyroMin[j] : _bmi088->gyro[j];
                    }
                }

                gNormDiff = gNormMax - gNormMin; // 最大值和最小值的差
                for (uint8_t j = 0; j < 3; ++j)
                    gyroDiff[j] = gyroMax[j] - gyroMin[j]; // 分别计算三轴
                if (gNormDiff > 0.5f ||
                    gyroDiff[0] > 0.15f ||
                    gyroDiff[1] > 0.15f ||
                    gyroDiff[2] > 0.15f)
                    break;         // 超出范围了,重开! remake到while循环,外面还有一层
                DWT_Delay(0.0005); // 休息一会再开始下一轮数据获取,IMU准备数据需要时间
            }
            _bmi088->gNorm /= (float)CaliTimes; // 加速度范数重力
            for (uint8_t i = 0; i < 3; ++i)
                _bmi088->gyro_offset[i] /= (float)CaliTimes; // 三轴零飘
            // 这里直接存到temperature,可以另外增加BMI088Instance的成员变量TempWhenCalib
            _bmi088->temperature = _bmi088->temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET; // 保存标定时的温度,如果已知温度和零飘的关系
            // caliTryOutCount++; 保存已经尝试的标定次数?由你.
        } while (gNormDiff > 0.5f ||
                 fabsf(_bmi088->gNorm - 9.8f) > 0.5f ||
                 gyroDiff[0] > 0.15f ||
                 gyroDiff[1] > 0.15f ||
                 gyroDiff[2] > 0.15f ||
                 fabsf(_bmi088->gyro_offset[0]) > 0.01f ||
                 fabsf(_bmi088->gyro_offset[1]) > 0.01f ||
                 fabsf(_bmi088->gyro_offset[2]) > 0.01f); // 满足条件说明标定环境不好
    }

    // 离线标定
    if (_bmi088->cali_mode == BMI088_LOAD_PRE_CALI_MODE)
    {
        _bmi088->gyro_offset[0] = BMI088_PRE_CALI_ACC_X_OFFSET;
        _bmi088->gyro_offset[1] = BMI088_PRE_CALI_ACC_Y_OFFSET;
        _bmi088->gyro_offset[2] = BMI088_PRE_CALI_ACC_Z_OFFSET;
        _bmi088->gNorm = BMI088_PRE_CALI_G_NORM;
    }
    
    // 防止除零错误
    if (_bmi088->gNorm > 0.1f && _bmi088->gNorm < 20.0f) {
        _bmi088->acc_coef *= 9.805f / _bmi088->gNorm;
    } else {
        _bmi088->acc_coef = 9.6; // 或者设置一个合理的默认值
    }
}




/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

//0 1 0
//-1 0 0
//0 0 1
    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

static void IMUPWMSet(float pwm)
{
    PWMSetDutyRatio(bmi088->heat_pwm, pwm);
}

static void IMU_Temperature_Ctrl(void)
{
    PIDCalculate(&bmi088->heat_pid, bmi088->temperature, RefTemp);
    IMUPWMSet(float_constrain(float_rounding(bmi088->heat_pid.Output), 0, UINT32_MAX));
}

BaseType_t BMI088DataReceiveFromQueue(attitude_t *attitude, TickType_t wait_time)
{
    return xQueueReceive(bmi088_send_queue, attitude, wait_time);
}

BMI088_Init_Config_s bmi088_config = {
    .spi_gyro_config={
        .spi_handle = &hspi2,
        .GPIOx = GPIOC,
        .cs_pin = GPIO_PIN_3,
    },
    .spi_acc_config = {
        .spi_handle = &hspi2,
        .GPIOx = GPIOC,
        .cs_pin = GPIO_PIN_0,
    },
    .cali_mode = BMI088_CALIBRATE_ONLINE_MODE,
    .heat_pwm_config = {
        .htim = &htim3,
        .channel = TIM_CHANNEL_4,
    },
    .heat_pid_config ={
        .MaxOut =0.5,
        .IntegralLimit = 300,
        .DeadBand = 0,
        .Kp = 0.05,
        .Ki = 0,
        .Kd = 0,
        .Improve = 0x01},
};

//这个可在应用层调用，便于适配不同主控板，可在应用层自由设置引脚，与
QueueHandle_t BMI088Register()
{
    // 申请内存
    BMI088Instance *bmi088_instance = (BMI088Instance *)zmalloc(sizeof(BMI088Instance));
    // 根据参数选择工作模式
    bmi088_instance->spi_acc = SPIRegister(&bmi088_config.spi_acc_config);
    bmi088_instance->spi_gyro = SPIRegister(&bmi088_config.spi_gyro_config);
    bmi088_instance->heat_pwm = PWMRegister(&bmi088_config.heat_pwm_config);
    PIDInit(&bmi088_instance->heat_pid, &bmi088_config.heat_pid_config);

    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;
    bmi088=bmi088_instance;

    float init_quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 单位四元数作为默认值
    InitQuaternion(init_quaternion);
    
    // 检查初始化四元数是否有效
    float quat_magnitude = sqrtf(init_quaternion[0]*init_quaternion[0] +
                                init_quaternion[1]*init_quaternion[1] +
                                init_quaternion[2]*init_quaternion[2] +
                                init_quaternion[3]*init_quaternion[3]);
    
    if (quat_magnitude < 0.1f || quat_magnitude > 2.0f || 
        isnan(quat_magnitude) || isinf(quat_magnitude)) {
        // 如果四元数无效，使用单位四元数
        init_quaternion[0] = 1.0f;
        init_quaternion[1] = 0.0f;
        init_quaternion[2] = 0.0f;
        init_quaternion[3] = 0.0f;
    }

    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

    INS.AccelLPF = 0.0085;
    INS.DGyroLPF = 0.009;

    // 初始化acc和gyro
    BMI088_ERORR_CODE_e error = BMI088_NO_ERROR;

    /*****这里相当于之前的bmi088初始化*****/
    do
    {
        error = BMI088_NO_ERROR;
        error |= BMI088AccelInit(bmi088_instance);
        error |= BMI088GyroInit(bmi088_instance);
        // 可以增加try out times,超出次数则返回错误
    } while (error != 0);

    BMI088CalibrateIMU(bmi088_instance);                  

    ins_task_handle = osThreadNew(InsTask, NULL, &ins_task_attributes);
    bmi088_send_queue = xQueueCreate(10, sizeof(imu_data_t));
    return bmi088_send_queue;
}

void InsTask(void *argument)
{
    imu_data_t imu_data;
    for(;;)
    {
            static uint32_t count = 0;
            const float gravity[3] = {0, 0, 9.81f};

            dt = DWT_GetDeltaT(&INS_DWT_Count);
            t += dt;
            
            // 防止dt异常
            if (dt > 0.1f || dt <= 0.0f) {
                dt = 0.001f;
            }

            // ins update
            if ((count % 1) == 0)
            {
                BMI088Acquire(bmi088);

                INS.Accel[X] = bmi088->acc[X];
                INS.Accel[Y] = bmi088->acc[Y];
                INS.Accel[Z] = bmi088->acc[Z];
                INS.dgyro[X] = (bmi088->gyro[X] - INS.Gyro[X])/ (INS.DGyroLPF + dt) + INS.dgyro[X] * INS.DGyroLPF / (INS.DGyroLPF + dt);
                INS.dgyro[Y] = (bmi088->gyro[Y] - INS.Gyro[Y])/ (INS.DGyroLPF + dt) + INS.dgyro[Y] * INS.DGyroLPF / (INS.DGyroLPF + dt);
                INS.dgyro[Z] = (bmi088->gyro[Z] - INS.Gyro[Z])/ (INS.DGyroLPF + dt) + INS.dgyro[Z] * INS.DGyroLPF / (INS.DGyroLPF + dt);
                INS.Gyro[X] = bmi088->gyro[X];
                INS.Gyro[Y] = bmi088->gyro[Y];
                INS.Gyro[Z] = bmi088->gyro[Z];

                // 检查数据有效性
                if (isnan(INS.Accel[X]) || isnan(INS.Accel[Y]) || isnan(INS.Accel[Z]) ||
                    isnan(INS.Gyro[X]) || isnan(INS.Gyro[Y]) || isnan(INS.Gyro[Z])) {
                    // 如果数据无效，跳过这次更新
                    count++;
                    return;
                }

                IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

                // 核心函数,EKF更新四元数
                IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], 
                                    INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

                // 检查QEKF_INS.q是否有效后再复制
                uint8_t q_valid = 1;
                for (int i = 0; i < 4; i++) {
                    if (isnan(QEKF_INS.q[i]) || isinf(QEKF_INS.q[i])) {
                        q_valid = 0;
                        break;
                    }
                }
                
                if (q_valid) {
                    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));
                }

                // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
                if (q_valid) {
                    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
                    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
                    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

                    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
                    float gravity_b[3];
                    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
                    
                    // 检查gravity_b是否有效
                    if (!isnan(gravity_b[0]) && !isnan(gravity_b[1]) && !isnan(gravity_b[2])) {
                        for (uint8_t i = 0; i < 3; ++i) {
                            float denominator = INS.AccelLPF + dt;
                            if (denominator > 0.0001f) {
                                INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / denominator + 
                                                    INS.MotionAccel_b[i] * INS.AccelLPF / denominator;
                            }
                        }
                        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);
                    }
                }
                bmi088->MotionAccel_b[0]= INS.MotionAccel_b[0];
                bmi088->MotionAccel_b[1]= INS.MotionAccel_b[1];
                bmi088->MotionAccel_b[2]= INS.MotionAccel_b[2];

                bmi088->dgyro[0] =INS.dgyro[0];
                bmi088->dgyro[1] =INS.dgyro[1];
                bmi088->dgyro[2] =INS.dgyro[2];

                bmi088->gyro[0] =INS.Gyro[0];
                bmi088->gyro[1] =INS.Gyro[1];
                bmi088->gyro[2] =INS.Gyro[2];

                bmi088->yaw = QEKF_INS.Yaw;
                bmi088->pitch =  QEKF_INS.Pitch;
                bmi088->roll = QEKF_INS.Roll;
                bmi088->yaw_total_angle = QEKF_INS.YawTotalAngle;
                
                imu_data.yaw =bmi088->yaw;
                imu_data.pitch = bmi088->pitch;
                imu_data.roll =bmi088->roll;
                imu_data.yaw_total_angle = bmi088->yaw_total_angle;
                imu_data.gyro[0] = bmi088->gyro[0];
                imu_data.gyro[1] = bmi088->gyro[1];
                imu_data.gyro[2] = bmi088->gyro[2];
                imu_data.dgyro[0] = bmi088->dgyro[0];
                imu_data.dgyro[1] = bmi088->dgyro[1];
                imu_data.dgyro[2] = bmi088->dgyro[2];
                imu_data.MotionAccel_b[0] = bmi088->MotionAccel_b[0];
                imu_data.MotionAccel_b[1] = bmi088->MotionAccel_b[1];
                imu_data.MotionAccel_b[2] = bmi088->MotionAccel_b[2];

            }
            // temperature control
            if ((count % 2) == 0)
            {
                // 500hz
                IMU_Temperature_Ctrl();
            }

            count++;
            xQueueSend(bmi088_send_queue, &imu_data, NULL);

            osDelay(1);
    }
}
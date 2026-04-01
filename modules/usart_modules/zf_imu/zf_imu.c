#include "zf_imu.h"
#include "double_buffer.h"
#include "projdefs.h"
#include "robot_def.h"
#include "string.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "daemon.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "buzzer.h"
#include <stdint.h>
#include "robot_def.h"

// 志翔imu
static zf_imu_data_t zf_imu_data;
static imu_data_t imu_data;
static uint8_t zf_imu_init_flag = 0;
static uint8_t idx;
// 串口实例和守护进程
static USARTInstance *zf_imu_usart_instance = NULL;
static DaemonInstance *zf_imu_daemon_instance = NULL;

static TaskHandle_t zf_imu_task_handle = NULL;
static QueueHandle_t zf_imu_queue = NULL;
static QueueHandle_t zf_imu_daemon_queue = NULL;
const osThreadAttr_t zf_imu_task_attributes = {
    .name = "zf_imu_task",
    .stack_size = 512,
    .priority = (osPriority_t) osPriorityNormal,
};

static uint8_t zf_imuDecode(const uint8_t *sbus_buf)
{
    // 1. 校验帧头
    if (sbus_buf[0] != 0xAA || sbus_buf[1] != 0x55)
        return 0;

    // 2. 检查命令ID是否为0x03（角度推送包）
    uint16_t cmd_id = sbus_buf[2] | (sbus_buf[3] << 8);
    if (cmd_id == 0x03)
    {
        // 3. 解析数据（小端格式）
        // 频率：偏移6-7
        zf_imu_data.freq = sbus_buf[6] | (sbus_buf[7] << 8);
        // 角度：float32，偏移8-11（roll），12-15（pitch），16-19（yaw）
        // 使用memcpy安全地拷贝浮点数
        memcpy(&zf_imu_data.roll,  sbus_buf + 8,  4);
        imu_data.roll = zf_imu_data.roll;
        memcpy(&zf_imu_data.pitch, sbus_buf + 12, 4);
        imu_data.pitch = zf_imu_data.pitch;
        memcpy(&zf_imu_data.yaw,   sbus_buf + 16, 4);
        imu_data.yaw = zf_imu_data.yaw;

        // 四元数：float32，偏移20-23（q0），24-27（q1），28-31（q2），32-35（q3）
        memcpy(&zf_imu_data.q0, sbus_buf + 20, 4);
        memcpy(&zf_imu_data.q1, sbus_buf + 24, 4);
        memcpy(&zf_imu_data.q2, sbus_buf + 28, 4);
        memcpy(&zf_imu_data.q3, sbus_buf + 32, 4);

        return 1; // 解析成功
    }

}

// 后端任务处理函数
static void zf_imu_task(void *argument)
{
    usart_backend_packet_t *packet = NULL;
    // Daemon_Event_t *daemon_event =NULL;
    // 数据累积缓冲区
    zf_imu_data_buffer_t data_buffer;
    memset(&data_buffer, 0, sizeof(data_buffer));
    zf_imu_parse_state_t parse_state = ZF_DATA_STATE_IDLE;

    uint8_t frame_buffer[38];
    uint8_t frame_index = 0;
    while (1) 
    {
        // 等待前端任务发送的数据包
        if (xQueueReceive(zf_imu_queue, &packet, portMAX_DELAY) == pdPASS) 
        {
            if(packet->instance == zf_imu_usart_instance)
            {
                // 处理接收到的数据
                for (uint16_t i = 0; i < packet->length; i++) 
                {
                    uint8_t data = packet->data[i];
                    // 根据协议解析状态处理数据
                    switch (parse_state)
                    {
                        case ZF_DATA_STATE_IDLE:
                            // 寻找帧头
                            if (data == ZF_RECV_HEADER) {
                                parse_state = ZF_DATA_STATE_HEADER_FOUND;
                                frame_index = 0;
                                frame_buffer[frame_index++] = data;
                            }
                            break;

                        case ZF_DATA_STATE_HEADER_FOUND:
                            // 已经找到帧头，开始接收数据
                            parse_state = ZF_DATA_STATE_RECEIVING;

                        case ZF_DATA_STATE_RECEIVING:
                            // 收集帧数据
                            frame_buffer[frame_index++] = data;
                            
                            // 检查是否收到完整帧
                            if (frame_index == 37) {
                                // 尝试解析数据
                                if (zf_imuDecode(frame_buffer)) 
                                {
                                    // 解析成功
                                    parse_state = ZF_DATA_STATE_COMPLETE;
                                    // DaemonReload(ZF_DATA_daemon_instance);
                                }
                                else 
                                {
                                    parse_state = ZF_DATA_STATE_IDLE;
                                }
                                frame_index = 0;
                            }
                            break;
                            
                        case ZF_DATA_STATE_COMPLETE:
                            // 帧解析完成，准备接收下一帧
                            parse_state = ZF_DATA_STATE_IDLE;
                            break;
                    }
                }
                // 释放数据包内存
                if (packet->data) 
                {
                    free(packet->data);
                }
                free(packet);
            }
        }
    }
}

// 遥控器离线回调
void zf_imuLostCallback()
{
    memset(&zf_imu_data, 0 , sizeof(zf_imu_data_t));
}

// 初始化遥控器
zf_imu_data_t *ZFImuInit(UART_HandleTypeDef *rf_usart_handle)
{
    USART_Init_Config_s conf;
    conf.usart_handle = rf_usart_handle;
    conf.recv_buff_size = 255;
    zf_imu_queue = xQueueCreate(10, sizeof(usart_backend_packet_t *));
    conf.module_queue = zf_imu_queue;
    conf.circular_buffer_flag = 1;
    // 注册串口实例
    zf_imu_usart_instance = usart_register(&conf);
    // 创建后端任务
    zf_imu_task_handle = osThreadNew(zf_imu_task, zf_imu_usart_instance, &zf_imu_task_attributes);
    
    // Daemon_Init_Config_s daemon_conf = {
    //     .owner_id = rc_usart_instance,
    //     .reload_count = 30,
    //     .init_count = 30,
    // };
    // rc_daemon_instance = DaemonRegister(&daemon_conf);
    // rc_daemon_instance->callback = RCLostCallback;
    zf_imu_init_flag = 1;
    return &zf_imu_data;
}



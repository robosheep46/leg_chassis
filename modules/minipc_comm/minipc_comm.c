#include "minipc_comm.h"
#include "bsp_usart.h"
#include <string.h>
#include "bmi088.h"
#include "cmsis_os2.h"
#include "daemon.h"
#include "crc_ref.h"
#include "robot_def.h"
static USARTInstance *minipc_usart_instance;
static DaemonInstance *minipc_daemon_instance;
static Minipc_Send_s minipc_send_data;
static Minipc_Recv_s* minipc_recv_data;
osThreadId_t MinipcComm_TaskHandle;
static QueueHandle_t minipc_recv_queue;
const osThreadAttr_t minipc_comm_task_attr = {
    .name = "minipc_comm_task",
    .stack_size = 256,
    .priority = (osPriority_t) osPriorityNormal,
};

/*****SEND_DATA*****/


static void get_protocol_send_Minipc_data(Minipc_Send_s *tx_data,          // 待发送的float数据
                                            uint8_t *tx_buf)         // 待发送的数据帧     
{
    /*帧头部分*/
    tx_buf[0] = tx_data->header;
	memcpy(&tx_buf[1], &tx_data->Vision.detect_color, 1);
    memcpy( &tx_buf[2],&tx_data->Vision.roll, 4);
    memcpy( &tx_buf[6],&tx_data->Vision.pitch, 4);
    memcpy( &tx_buf[10],&tx_data->Vision.yaw, 4);
    Append_CRC16_Check_Sum(&tx_buf[0],16);
}

static void get_protocol_info_vision(uint8_t *rx_buf, Minipc_Recv_s *recv_data)
{
    // 检查帧头
    if (rx_buf[0] != MINIPC_FRAME_HEADER) 
    {
        memset(minipc_recv_data, 0, sizeof(Minipc_Recv_s));
    }
    else
    {
        memcpy(&recv_data->header, &rx_buf[0], 1);
        memcpy(&recv_data->Vision.pitch, &rx_buf[1], 4);
        memcpy(&recv_data->Vision.yaw, &rx_buf[5], 4);
        memcpy(&recv_data->Vision.shoot_flag, &rx_buf[9], 1);
        memcpy(&recv_data->Vision.time, &rx_buf[10], 4);
    }
}


static void DecodeMinipcData()
{
    DaemonReload(minipc_daemon_instance); // 喂狗
    get_protocol_info_vision(minipc_usart_instance->recv_buff,&minipc_recv_data);
}


// 遥控器离线回调
void MinipcLostCallback()
{
    memset(minipc_recv_data, 0, sizeof(Minipc_Recv_s));
}


void Minipc_Comm_Task(void* argument)
{
    //发送
    USARTInstance *instance = (USARTInstance *)argument;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1);
    static uint8_t send_buff[Minipc_Send_SIZE];
    imu_data_t imu_data;
    Enemy_Color_e color;

    while (1) 
    {   
        // 从陀螺仪队列获取数据
        BaseType_t attitude_status = BMI088DataReceiveFromQueue(&imu_data, 0);
        if (attitude_status == pdPASS )
        {
            minipc_send_data.header = MINIPC_FRAME_HEADER;
            minipc_send_data.Vision.yaw = imu_data.yaw;
            minipc_send_data.Vision.pitch = imu_data.pitch;
            minipc_send_data.Vision.roll = imu_data.roll;
            minipc_send_data.Vision.detect_color = 1;
            // 只有成功接收到新数据才发送
            get_protocol_send_Minipc_data(&minipc_send_data, send_buff);
            USARTSend(minipc_usart_instance, send_buff, Minipc_Send_SIZE, USART_TRANSFER_DMA);
        }
    }
}


Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle)
{
    minipc_recv_data = (Minipc_Recv_s *)malloc(sizeof(Minipc_Recv_s));
    USART_Init_Config_s conf;
    conf.recv_buff_size = Minipc_Send_SIZE;
    conf.usart_handle = _handle;
    minipc_usart_instance = USARTRegister(&conf);    

    MinipcComm_TaskHandle = osThreadNew(Minipc_Comm_Task, 
                                            minipc_usart_instance, 
                                            &minipc_comm_task_attr);
    conf.module_callback = DecodeMinipcData;
    Daemon_Init_Config_s daemon_conf = {
        .owner_id = minipc_usart_instance,
        .reload_count = 30,
        .init_count = 30,
    };
    minipc_daemon_instance = DaemonRegister(&daemon_conf);
    minipc_daemon_instance->callback = MinipcLostCallback;
    return minipc_recv_data;
}
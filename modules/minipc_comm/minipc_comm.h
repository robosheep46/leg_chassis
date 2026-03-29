#ifndef MINIPC_COMM_H
#define MINIPC_COMM_H

#include "bsp_usart.h"
#include <stdint.h>

#define Minipc_Recv_SIZE 9u 
#define Minipc_Send_SIZE 16u
#define MINIPC_FRAME_HEADER 0XA5
#define MINIPC_RECV_HEADER 0X5A

typedef enum {
    MINIPC_DATA_STATE_IDLE,
    MINIPC_DATA_STATE_HEADER_FOUND,
    MINIPC_DATA_STATE_RECEIVING,
    MINIPC_DATA_STATE_COMPLETE
} minipc_parse_state_t;


typedef struct {
    uint8_t buffer[Minipc_Recv_SIZE];
    uint8_t index;
    uint8_t ready;  // 标志是否累积了完整的一帧
} minipc_data_buffer_t;
//1+4+4+1+4=14
#pragma pack(1)
typedef struct
{
	uint8_t header;  // 帧头，固定为0x5A
	struct
	{
		float pitch;
		float yaw;
		int8_t shoot_flag;
		int32_t time;
	}Vision;
} __attribute__((packed)) Minipc_Recv_s;

typedef enum
{
	COLOR_BLUE = 1,
	COLOR_RED = 0,
} Enemy_Color_e;
//1+1+4+4+4+2
typedef struct
{
	uint8_t header;  // 帧头，固定为0x5A
	struct
	{
		uint8_t detect_color;
		float roll;
		float pitch;
		float yaw;
	}Vision;
} __attribute__((packed)) Minipc_Send_s;

#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
/**
 * @brief 发送视觉数据
 *
 */
void SendMinipcData();

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_Vision_data(
                            Minipc_Send_s *tx_data,          // 待发送的float数据
                            uint8_t float_length,    // float的数据长度
                            uint8_t *tx_buf,         // 待发送的数据帧
                            uint16_t *tx_buf_len) ;   // 待发送的数据帧长度




void Minipc_Comm_Task(void* argument);
void MinipcLostCallback();
Minipc_Recv_s *minipcInit(UART_HandleTypeDef *_handle);


#endif // !MASTER_PROCESS_H
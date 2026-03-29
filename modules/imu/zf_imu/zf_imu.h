#include "general_def.h"
#include "bsp_usart.h"

#define ZF_RECV_HEADER 0xAA
typedef struct {
    uint16_t freq;        // 推送频率 (Hz)
    float roll;           // 滚转角 (度)
    float pitch;          // 俯仰角 (度)
    float yaw;            // 偏航角 (度)
    float q0, q1, q2, q3; // 四元数
} zf_imu_data_t;

// 数据累积缓冲区
typedef struct {
    uint8_t buffer[255];
    uint8_t index;
    uint8_t ready;  // 标志是否累积了完整的一帧
}zf_imu_data_buffer_t;

typedef enum {
    ZF_DATA_STATE_IDLE,
    ZF_DATA_STATE_HEADER_FOUND,
    ZF_DATA_STATE_RECEIVING,
    ZF_DATA_STATE_COMPLETE
} zf_imu_parse_state_t;

zf_imu_data_t *ZFImuInit(UART_HandleTypeDef *rf_usart_handle);
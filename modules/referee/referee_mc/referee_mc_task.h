#ifndef REFEREE_MC_H
#define REFEREE_MC_H

#include "rm_referee.h"
#include "robot_def.h"

typedef enum {
    REFEREE_DATA_STATE_IDLE,
    REFEREE_DATA_STATE_HEADER_FOUND,
    REFEREE_DATA_STATE_RECEIVING,
    REFEREE_DATA_STATE_COMPLETE
} referee_parse_state_t;

// 数据累积缓冲区
typedef struct {
    uint8_t buffer[254];
    uint8_t index;
    uint8_t ready;  // 标志是否累积了完整的一帧
} referee_data_buffer_t;

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *RefereeTaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit();

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
 void RefereeInfoTask(void *argument);

#endif // REFEREE_H

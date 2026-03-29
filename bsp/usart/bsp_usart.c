#include "bsp_usart.h"
#include "stdlib.h"
#include "memory.h"


/* usart service instance, modules' info would be recoreded here using usart_register() */
/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};


USARTInstance *usart_register(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            ;

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                ;

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    // 为两个缓冲区动态分配内存（大小 = recv_buff_size）
    for (int i = 0; i < USART_BUFFER_COUNT; i++) {
        instance->rx_buffers[i] = pvPortMalloc(instance->recv_buff_size);
        if (instance->rx_buffers[i] == NULL) while(1);
        memset(instance->rx_buffers[i], 0, instance->recv_buff_size);
    }

    // 创建同步对象
    instance->xEventGroup = xEventGroupCreate();
    instance->xMutex = xSemaphoreCreateMutex();

    // 启动第一次 DMA 接收（使用缓冲区0）
    instance->active_buffer = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle,
                                 instance->rx_buffers[0],
                                 instance->recv_buff_size);
    __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);

    usart_instance[idx++] = instance;
    return instance;
}

void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! check your code context! 检查定义instance的代码上下文,可能出现指针越界
        break;
    }
}

/* 串口发送时,gstate会被设为BUSY_TX */
uint8_t USARTIsReady(USARTInstance *_instance)
{
    if (_instance->usart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return 0;
    else
        return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i) 
    {
        if (huart == usart_instance[i]->usart_handle) 
        {
            USARTInstance *inst = usart_instance[i];
            uint8_t full_buffer = inst->active_buffer;   // 当前完成的缓冲区索引

            // 保存本次接收的长度
            inst->rx_len[full_buffer] = Size;
            inst->buffer_full[full_buffer] = 1;          // 标记该缓冲区已满

            // 切换到另一个缓冲区
            uint8_t next_buffer = (full_buffer + 1) % USART_BUFFER_COUNT;
            inst->active_buffer = next_buffer;

            // 启动下一次 DMA 接收（使用新缓冲区）
            HAL_UARTEx_ReceiveToIdle_DMA(huart,
                                         inst->rx_buffers[next_buffer],
                                         inst->recv_buff_size);
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

            // 通知通信任务
            EventBits_t bit = (full_buffer == 0) ? USART_BUFFER0_FULL_BIT : USART_BUFFER1_FULL_BIT;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xEventGroupSetBitsFromISR(inst->xEventGroup, bit, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return;
        }
    }
}

/**
 * @brief 当串口发送/接收出现错误时,会调用此函数,此时这个函数要做的就是重新启动接收
 *
 * @note  最常见的错误:奇偶校验/溢出/帧错误
 *
 * @param huart 发生错误的串口
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // for (uint8_t i = 0; i < idx; ++i)
    // {
    //     if (huart == usart_instance[i]->usart_handle)
    //     {
    //         HAL_UARTEx_ReceiveToIdle_DMA(usart_instance[i]->usart_handle, usart_instance[i]->recv_buff, usart_instance[i]->recv_buff_size);
    //         __HAL_DMA_DISABLE_IT(usart_instance[i]->usart_handle->hdmarx, DMA_IT_HT);
    //         return;
    //     }
    // }
}
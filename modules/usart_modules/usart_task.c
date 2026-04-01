#include "usart_task.h"

extern USARTInstance *usart_instance[DEVICE_USART_CNT];
osThreadId usart_task_handle;

const osThreadAttr_t usart_task_attributes = {
    .name = "usart_task",
    .stack_size = 1024*2,
    .priority = (osPriority_t) osPriorityNormal,
};


void create_usart_task()
{
    usart_task_handle = osThreadNew(usart_task, NULL, &usart_task_attributes);
}

void usart_task(void *argument)
{
    EventBits_t bits;
    USARTInstance *inst;
    uint8_t buffer_idx;

    for (;;) 
    {
        for (uint8_t i = 0; i < 1; i++) 
        {
            inst = usart_instance[i];
            bits = xEventGroupGetBits(inst->xEventGroup);
            if (bits & (USART_BUFFER0_FULL_BIT | USART_BUFFER1_FULL_BIT)) {
                // 确定是哪个缓冲区
                if (bits & USART_BUFFER0_FULL_BIT) 
                {
                    buffer_idx = 0;
                }
                else 
                {
                    buffer_idx = 1;
                }
                // 清除事件位
                xEventGroupClearBits(inst->xEventGroup,
                                     (buffer_idx==0) ? USART_BUFFER0_FULL_BIT : USART_BUFFER1_FULL_BIT);

                // 调用模块回调（此时数据已就绪）
                if (inst->module_callback != NULL) 
                {
                    inst->module_callback(inst->rx_buffers[buffer_idx], inst->rx_len[buffer_idx]);
                }
                inst->buffer_full[buffer_idx] = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
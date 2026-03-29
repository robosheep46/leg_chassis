#include "daemon.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "buzzer.h"
#include "cmsis_os2.h"
#include "stdlib.h"
#include "memory.h"
#include "queue.h"

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

static QueueHandle_t daemon_queue = NULL;
static TaskHandle_t daemon_TaskHandle = NULL;
static BuzzerInstance *buzzer_instance = NULL;
const osThreadAttr_t daemon_Task_attributes = {
  .name = "daemon_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    instance->callback = config->owner_callback;
    instance->other_modules_error_callback = config->other_modules_error_callback;

    instance->temp_count = config->init_count == 0 ? 100 : config->init_count; // 默认值为100,初始计数

    instance->temp_count = config->reload_count;
    daemon_instances[idx++] = instance;
    return instance;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
}
static uint32_t all_normal_count = 0; // 用于记录所有daemon都正常的次数

void DaemonTask(void *argument)
{
    uint8_t normal_flag = 0;
    for(;;)
    {
        DaemonInstance *dins;
        uint8_t all_normal = 1;
        uint8_t timed_out_indices[DAEMON_MX_CNT] = {0};
        uint8_t timeout_count = 0;

        for (size_t i = 0; i < idx; i++)
        {
            dins = daemon_instances[i];
            
            if (dins->temp_count > 0)
            {
                // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
                dins->temp_count--;
            }
        }


        for (size_t i = 0; i < idx; i++)
        {
            dins = daemon_instances[i];
            
            if (dins->temp_count <= 0)
            {
                // 自己超时，调用callback（即自己的错误处理回调）
                if (dins->callback) 
                {
                    dins->callback(dins->owner_id); // 自己模块超时处理
                }
            }
        }


        for (size_t i = 0; i < idx; i++)
        {
            dins = daemon_instances[i];
            
            // 只有注册了other_modules_error_callback的模块才会受到影响
            if (dins->other_modules_error_callback != NULL)
            {
                // 检查是否有其他模块超时
                for (size_t j = 0; j < idx; j++)
                {
                    if (i != j && daemon_instances[j]->temp_count <= 0)
                    {
                        // 第j个模块超时了，通知第i个模块
                        dins->other_modules_error_callback(daemon_instances[i]->owner_id);
                    }
                }
            }
        }
        osDelay(11);
    }
}

void CreateDaemon(BuzzerInstance * buzzer)
{
    daemon_TaskHandle = osThreadNew(DaemonTask, NULL, &daemon_Task_attributes);
    buzzer_instance = buzzer;
}






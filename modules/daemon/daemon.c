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
    instance->callback = config->callback;
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
        for (size_t i = 0; i < idx; i++)
        {
            dins = daemon_instances[i];
            
            if (dins->temp_count > 0)
            {
                // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
                dins->temp_count--;
            }
            else
            {
                // 如果有一个daemon超时，标记为不正常
                all_normal = 0;
                
                if (dins->callback) // 等于零说明超时了,调用回调函数(如果有的话)
                {
                    dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
                    BuzzerSendWorkModeEvent(buzzer_instance, WARNING1_MODE);
                }
            }
        }
        
        // 检查是否所有daemon都正常
        if (all_normal && idx > 0) // idx>=0确保至少有一个daemon实例
        {
            all_normal_count++;
        
            if (all_normal_count >= 200)
            {
                if(normal_flag == 0)
                {
                    // 发送消息队列
                    BuzzerSendWorkModeEvent(buzzer_instance, START_SONG_MODE);
                }

                normal_flag = 1;
            }
        }
        else
        {
            normal_flag = 0;
            all_normal_count = 0;
        }
        osDelay(11);
    }
}

void CreateDaemon(BuzzerInstance * buzzer)
{
    daemon_TaskHandle = osThreadNew(DaemonTask, NULL, &daemon_Task_attributes);
    buzzer_instance = buzzer;
}






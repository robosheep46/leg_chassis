#include "buzzer.h"
#include "bsp_pwm.h"
#include "stdlib.h"
#include "string.h"
#include "tim.h"
#include <stdint.h>
#include "cmsis_os.h"
#include "queue.h"
static BuzzerInstance *buzzer_instance;

osThreadId_t              buzzer_TaskHandle;
const osThreadAttr_t buzzer_Task_attributes = {
  .name = "buzzer_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) 1,
};

static QueueHandle_t buzzer_queue;

static Buzzer_Init_Config_s buzzer_config = {
    .buzzer_pwm_init_config = {
      .htim = &htim12,
      .channel = TIM_CHANNEL_2,
    },
    .work_mode = START_SONG_MODE,
    .loudness = 0.5,
};

// 启动音
const Note_t start_song[] = {
    {523.25*2, 300},  
    {587.33*2, 200},  
    {783.99*2, 500},  
};

// 瞄准提示音
const Note_t aim_song[] = 
{
    {1000, 100}
};

// 警报音1
const Note_t Warning1_song[] = {
    {1000, 200},
    {0, 10000},
};

const Note_t Warning2_song[] = {
    {1000, 200},
    {0, 10000},
};

const Note_t Warning3_song[] = {
    {1000, 200},
    {0, 10000},
};

//短按切换音
const Note_t short_switch_song[] = {
    {1000, 100},
};

const Note_t long_switch_song[] = {
    {1000, 100},
    {0, 100},
    {1000, 100},
};
BuzzerInstance *BuzzerRegister(QueueHandle_t queue)
{
    BuzzerInstance *instance = (BuzzerInstance *)malloc(sizeof(BuzzerInstance));
    memset(instance, 0, sizeof(BuzzerInstance));
    
    instance->buzzer_pwm = pwm_register(&buzzer_config.buzzer_pwm_init_config);
    
    instance->frequency = buzzer_config.frequency;
    instance->work_mode = buzzer_config.work_mode;
    instance->loudness = buzzer_config.loudness;
    if (queue != NULL) 
    {
        instance->event_queue = queue;
    }

    buzzer_TaskHandle = osThreadNew(BuzzerTask, (void*)instance, &buzzer_Task_attributes);
    
    buzzer_instance = instance;
    return instance;
}

//将原来初始化的活放到这个函数中
void BuzzerSetMode(BuzzerInstance *_instance, Buzzer_Work_Mode_e mode)
{
    if (_instance == NULL) return;

    if (_instance->work_mode == mode && _instance->last_work_mode == mode) 
    {
        return;
    }

    _instance->work_mode = mode;
    _instance->song.init_flag = 0;  // 重置初始化标志
    _instance->song.current_note = 0;
    _instance->last_work_mode= mode;

    // 根据模式设置对应的歌曲和播放模式
    switch (mode)
    {
        case START_SONG_MODE:
            _instance->song.song = start_song;
            _instance->song.note_count = sizeof(start_song) / sizeof(Note_t);
            _instance->song.play_mode = PLAY_ONCE;
            break;
            
        case AIM_MODE:
            _instance->song.song = aim_song;
            _instance->song.note_count = sizeof(aim_song) / sizeof(Note_t);
            _instance->song.play_mode = PLAY_LOOP;  // 改为循环播放
            break;
            
        case WARNING1_MODE:
            _instance->song.song = Warning1_song;
            _instance->song.note_count = sizeof(Warning1_song) / sizeof(Note_t);
            _instance->song.play_mode = PLAY_LOOP;
            break;
        case WARNING2_MODE:
            _instance->song.song = Warning2_song;
            _instance->song.note_count = sizeof(Warning2_song) / sizeof(Note_t);
            _instance->song.play_mode = PLAY_LOOP;
            break;
        case LONG_SWITCH_MODE:
            _instance->song.song = long_switch_song;
            _instance->song.note_count = sizeof(long_switch_song) / sizeof(Note_t);
            _instance->song.play_mode = PLAY_ONCE;
            break;
        case SHORT_SWITCH_MODE:
            _instance->song.song = short_switch_song;
            _instance->song.note_count = sizeof(short_switch_song) / sizeof(Note_t);
            _instance->song.play_mode = PLAY_ONCE;
            break;
        case BUZZER_OFF:
        default:
            pwm_stop(_instance->buzzer_pwm);
            _instance->song.song = NULL;
            _instance->song.note_count = 0;
            break;
    }
}

static void BuzzerSetFrequency(BuzzerInstance * _instance, uint32_t freq)
{   
    _instance->frequency = freq;
    
    if (freq == 0) 
    {
        pwm_stop(_instance->buzzer_pwm);
        return;
    }
    
    pwm_start(_instance->buzzer_pwm);
    
    // 计算周期：周期 = 1 / 频率
    float period = 1.0f / freq;
    pwm_set_period(_instance->buzzer_pwm, period);
    
    // 重新设置占空比
    pwm_set_duty_ratio(_instance->buzzer_pwm, _instance->loudness);
}

void BuzzerSetLoudness(BuzzerInstance * _instance, float loudness)
{   
    _instance->loudness = loudness;
}

//重置音，以便可以再次播放
void BuzzerRestartSong(BuzzerInstance *_instance)
{
    if (_instance == NULL) return;
    
    _instance->song.finished=0;
}

/* 开始播放指定音符 */
static void SongStartNote(BuzzerInstance *_instance, uint16_t note_index)
{
    const Note_t *note = &_instance->song.song[note_index];
    _instance->song.note_start_time = HAL_GetTick();
    BuzzerSetFrequency(_instance, (uint32_t)note->freq);
}

static void SongNextNote(BuzzerInstance *_instance)
{
    _instance->song.current_note++;
    
    // 检查是否到达歌曲末尾
    if (_instance->song.current_note >= _instance->song.note_count) 
    {
        if (_instance->song.play_mode == PLAY_LOOP) 
        {
            // 循环播放：回到第一个音符
            _instance->song.current_note = 0;
            SongStartNote(_instance, 0);
        }
        else 
        {
            // 播放一次：停止播放并关闭蜂鸣器
            _instance->work_mode = BUZZER_OFF;
            _instance->song.init_flag = 0;
            pwm_stop(_instance->buzzer_pwm);
            _instance->song.finished=1;
        }
    }
    else 
    {
        // 播放下一个音符
        SongStartNote(_instance, _instance->song.current_note);
    }
}

/* 开始播放歌曲 */
static void BuzzerPlaySong(BuzzerInstance *_instance)
{
    if (_instance->song.song == NULL || _instance->song.note_count == 0||_instance->song.finished==1)
     return;
    
    if (_instance->song.init_flag == 0)
    {
        _instance->song.current_note = 0;
        _instance->song.init_flag = 1;        
        // 开始播放第一个音符
        SongStartNote(_instance, 0);
    }
}

void SetBuzzerPlayMode(BuzzerInstance *_instance, Play_Mode_e mode)
{
    _instance->song.play_mode = mode;
}




/************************队列操作函数************************/

/**
* @brief 发送Buzzer事件到队列
*/

void BuzzerSendEventToQueue(BuzzerInstance *instance, Buzzer_Event_t *event)
{
    if (instance == NULL || instance->event_queue == NULL || event == NULL) {
        return;
    }
    
    event->timestamp = HAL_GetTick();
    
    if (xQueueSend(instance->event_queue, event, 0) != pdTRUE) {
        // 队列已满，可以处理错误
    }
}
 
/**
* @brief 从队列接收Buzzer事件
*/
BaseType_t BuzzerReceiveEventFromQueue(BuzzerInstance *instance, Buzzer_Event_t *event, TickType_t wait_time)
{
    if (instance == NULL || instance->event_queue == NULL || event == NULL) {
        return pdFALSE;
    }
    
    return xQueueReceive(instance->event_queue, event, wait_time);
}


/**
* @brief 发送设置模式事件
*/
void BuzzerSendWorkModeEvent(BuzzerInstance *instance, Buzzer_Work_Mode_e mode)
{
    if (instance == NULL) return;
    
    Buzzer_Event_t event = {
        .type = BUZZER_EVENT_SET_WORK_MODE,
        .data.work_mode = mode,
        .timestamp = 0
    };
    BuzzerSendEventToQueue(instance, &event);
}
 
/**
* @brief 发送播放模式事件
*/
void BuzzerSendPlayModeEvent(BuzzerInstance *instance, Play_Mode_e play_mode)
{
    if (instance == NULL) return;
    
    Buzzer_Event_t event = {
        .type = BUZZER_EVENT_SET_PLAY_MODE,
        .data.play_mode = play_mode,
        .timestamp = 0
    };
    
    BuzzerSendEventToQueue(instance, &event);
}
 
/**
* @brief 发送播放模式事件
*/
void BuzzerSendRestartEvent(BuzzerInstance *instance)
{
    if (instance == NULL) return;
    
    Buzzer_Event_t event = {
        .type = BUZZER_EVENT_SET_PLAY_MODE,
        .data.finished = 0,
        .timestamp = 0
    };
    
    BuzzerSendEventToQueue(instance, &event);
}




void BuzzerTask(void *argument)
{
    BuzzerInstance *buzzer = (BuzzerInstance*)argument;
    Buzzer_Event_t event;

    for (;;) 
    {
        while (BuzzerReceiveEventFromQueue(buzzer, &event, 0) == pdTRUE) 
        {
            switch (event.type) 
            {
                case BUZZER_EVENT_SET_WORK_MODE:
                    BuzzerSetMode(buzzer, event.data.work_mode);
                    BuzzerRestartSong(buzzer);
                    break;
                case BUZZER_EVENT_SET_PLAY_MODE:
                    SetBuzzerPlayMode(buzzer, event.data.play_mode);
                    break;
                default:
                    break;
            }   
        }

            // 如果暂停或关闭模式，直接返回
            if (buzzer->work_mode == BUZZER_OFF) {
                // return;
            }
            // 歌曲模式处理
            if (buzzer->work_mode >= START_SONG_MODE && buzzer->work_mode <= LONG_SWITCH_MODE)
            {
                if (buzzer->song.play_mode == PLAY_LOOP) 
                {
                    buzzer->song.finished=0;
                }
                // 先初始化歌曲播放
                BuzzerPlaySong(buzzer);
                
                // 检查当前音符是否播放完成
                if (buzzer->song.init_flag == 1 && buzzer->song.current_note < buzzer->song.note_count)
                {
                    uint32_t current_time = HAL_GetTick();
                    const Note_t *current_note = & buzzer->song.song[buzzer->song.current_note];
                    uint32_t elapsed_time = current_time - buzzer->song.note_start_time;
                    
                    if (elapsed_time >= current_note->duration) 
                    {
                        SongNextNote(buzzer);
                    }
                }
            }
        }
        osDelay(20);
}



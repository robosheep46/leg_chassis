#ifndef BUZZER_H
#define BUZZER_H

#include "cmsis_os.h"
#include "bsp_pwm.h"
#include <stdint.h>
#include "queue.h"

// 蜂鸣器工作模式
typedef enum
{
    BUZZER_OFF = 0,         
    START_SONG_MODE,        
    AIM_MODE,        
    WARNING1_MODE,
    WARNING2_MODE,
    WARNING3_MODE,
    SHORT_SWITCH_MODE,
    LONG_SWITCH_MODE,
} Buzzer_Work_Mode_e;

// 播放模式
typedef enum
{
    PLAY_ONCE = 0,          // 播放一次
    PLAY_LOOP,              // 循环播放
} Play_Mode_e;

typedef struct 
{
    float freq;             // 频率(Hz)
    uint32_t duration;      // 持续时间(ms)
} Note_t;

typedef struct 
{
    const Note_t *song;    
    uint16_t note_count;   
    uint16_t current_note;  
    uint32_t note_start_time; 
    uint8_t init_flag; 
    // 在原有基础上增加播放模式
    // 为了让警报音和识别目标音可循环播放。
    // 启动音可单次播放
    Play_Mode_e play_mode;
    //单次播放完成标志  
    uint8_t finished;
} Song_t;

typedef enum 
{
    BUZZER_EVENT_SET_WORK_MODE = 0,    // 设置工作模式
    BUZZER_EVENT_SET_PLAY_MODE,        // 设置播放模式
    BUZZER_EVENT_RESTART_SONG
}Buzzer_Event_Type_e;



// BUZZER事件结构体
typedef struct 
{
    Buzzer_Event_Type_e type;     // 事件类型
    //为了节省内存，写成union
    union 
    {
        Buzzer_Work_Mode_e work_mode;     // 工作模式
        Play_Mode_e play_mode;            // 播放模式
        uint8_t finished;
    } data;
    uint32_t timestamp;                // 时间戳
} Buzzer_Event_t;


/* buzzer实例结构体定义 */
typedef struct
{
    PWMInstance  *buzzer_pwm;
    Buzzer_Work_Mode_e work_mode;
    //这里可以增加一个上一次工作模式，用于判断是否需要更新工作模式
    //没更新就不用再调用函数，以免出现一直播放第一个音符的情况
    Buzzer_Work_Mode_e last_work_mode;

    float loudness;
    uint32_t frequency;
    Song_t song;
    QueueHandle_t event_queue;
} BuzzerInstance;

/* buzzer初始化配置 */
typedef struct
{
    Buzzer_Work_Mode_e work_mode;
    float loudness;
    uint32_t frequency;
    PWM_Init_Config_s buzzer_pwm_init_config;
} Buzzer_Init_Config_s;


BuzzerInstance *BuzzerRegister(QueueHandle_t queue);
void BuzzerSetMode(BuzzerInstance *_instance, Buzzer_Work_Mode_e mode);
void BuzzerTask(void *argument);
void BuzzerSetLoudness(BuzzerInstance *_instance, float loudness);
void BuzzerRestartSong(BuzzerInstance *_instance);
void SetBuzzerPlayMode(BuzzerInstance *_instance, Play_Mode_e mode);
/**
* @brief 发送设置模式事件
*/
void BuzzerSendWorkModeEvent(BuzzerInstance *instance, Buzzer_Work_Mode_e mode);
/**
* @brief 发送播放模式事件
*/
void BuzzerSendPlayModeEvent(BuzzerInstance *instance, Play_Mode_e play_mode);

void BuzzerSendRestartEvent(BuzzerInstance *instance);

#endif // !BUZZER_H
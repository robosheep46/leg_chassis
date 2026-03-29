#include "referee_pt.h"

#include "remote_control.h"
#include "projdefs.h"
#include "string.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "daemon.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "buzzer.h"
#include <stdint.h>


// 图传数据
static referee_ctrl_t rf_ctrl[2];
static uint8_t rfc_init_flag = 0;
static uint8_t idx;
// 串口实例和守护进程
static USARTInstance *rfc_usart_instance = NULL;
static DaemonInstance *rfc_daemon_instance = NULL;

static TaskHandle_t rfc_task_handle = NULL;
static QueueHandle_t rfc_queue = NULL;
static QueueHandle_t rfc_daemon_queue = NULL;

// 矫正摇杆值
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i) {
        if (abs(*(&rf_ctrl[TEMP].rc.rocker_left_x + i)) > 660) {
            *(&rf_ctrl[TEMP].rc.rocker_left_x + i) = 0;
        }
    }
}

// SBUS协议解析
/**
 * @brief 解析新版19字节遥控器协议
*/
static void rfcDecode(const uint8_t *sbus_buf)
{
    // 帧头校验
    if (sbus_buf[0] != 0xA9 || sbus_buf[1] != 0x53)
    {
        memset(rf_ctrl, 0 , sizeof(rf_ctrl));
    }
    // ---------- 摇杆通道（11位，减去中值变成有符号） ----------
    // 右摇杆X (起始位16)
    rf_ctrl[TEMP].rc.rocker_right_x = (int16_t)(((sbus_buf[2] | ((uint16_t)sbus_buf[3] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    // 右摇杆Y (起始位27)
    rf_ctrl[TEMP].rc.rocker_right_y = (int16_t)((((sbus_buf[3] >> 3) | ((uint16_t)sbus_buf[4] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    // 左摇杆Y (起始位38)
    rf_ctrl[TEMP].rc.rocker_left_y  = (int16_t)((((sbus_buf[4] >> 6) | ((uint16_t)sbus_buf[5] << 2) | 
                                        (((uint16_t)sbus_buf[6] & 0x01) << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    // 左摇杆X (起始位49)
    rf_ctrl[TEMP].rc.rocker_left_x  = (int16_t)((((sbus_buf[6] >> 1) | (((uint16_t)sbus_buf[7] & 0x0F) << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET);

    // ---------- 拨轮 (起始位65，11位) ----------
    rf_ctrl[TEMP].rc.dial = (int16_t)((((sbus_buf[8] >> 1) | (((uint16_t)sbus_buf[9] & 0x0F) << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET);

    // 摇杆死区/非线性校正（沿用你的现有函数）
    RectifyRCjoystick();

    // ---------- 挡位开关（只有一个2位开关） ----------
    uint8_t sw = (sbus_buf[7] >> 4) & 0x03;   // 起始位60，长度2
    // 原结构体有左右两个开关，此处均赋值为同一个值（可根据实际需要调整）
    rf_ctrl[TEMP].rc.rc_switch = sw;

    // ---------- 新增功能按键 ----------
    rf_ctrl[TEMP].rc.pause       = (sbus_buf[7] >> 6) & 0x01;   // 暂停键，起始位62
    rf_ctrl[TEMP].rc.custom_left = (sbus_buf[7] >> 7) & 0x01;   // 自定义左，起始位63
    rf_ctrl[TEMP].rc.custom_right= (sbus_buf[8] >> 0) & 0x01;   // 自定义右，起始位64
    rf_ctrl[TEMP].rc.trigger     = (sbus_buf[9] >> 4) & 0x01;   // 扳机键，起始位76

    // ---------- 鼠标 ----------
    rf_ctrl[TEMP].mouse.x = (int16_t)(sbus_buf[10] | ((uint16_t)sbus_buf[11] << 8));  // 起始位80
    rf_ctrl[TEMP].mouse.y = (int16_t)(sbus_buf[12] | ((uint16_t)sbus_buf[13] << 8));  // 起始位96
    rf_ctrl[TEMP].mouse.z = (int16_t)(sbus_buf[14] | ((uint16_t)sbus_buf[15] << 8));  // 起始位112（新增鼠标滚轮）

    // 鼠标按键（从2位字段中取最低有效位）
    rf_ctrl[TEMP].mouse.press_l =  sbus_buf[16]       & 0x01;   // 左键，起始位128
    rf_ctrl[TEMP].mouse.press_r = (sbus_buf[16] >> 2) & 0x01;   // 右键，起始位130
    rf_ctrl[TEMP].mouse.press_m = (sbus_buf[16] >> 4) & 0x01;   // 中键，起始位132（新增）

    // ---------- 键盘按键（16位位图） ----------
    *(uint16_t *)&rf_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[17] | ((uint16_t)sbus_buf[18] << 8)); // 起始位136

    // ---------- 组合键处理（Ctrl/Shift）----------
    if (rf_ctrl[TEMP].key[KEY_PRESS].ctrl) {
        rf_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rf_ctrl[TEMP].key[KEY_PRESS];
    } else {
        memset(&rf_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    }

    if (rf_ctrl[TEMP].key[KEY_PRESS].shift) {
        rf_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rf_ctrl[TEMP].key[KEY_PRESS];
    } else {
        memset(&rf_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    }

    // ---------- 更新按键计数（完全保留原有逻辑）----------
    uint16_t key_now    = rf_ctrl[TEMP].key[KEY_PRESS].keys;
    uint16_t key_last   = rf_ctrl[LAST].key[KEY_PRESS].keys;
    uint16_t key_with_ctrl  = rf_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys;
    uint16_t key_with_shift = rf_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys;
    uint16_t key_last_with_ctrl  = rf_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys;
    uint16_t key_last_with_shift = rf_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys;

    for (uint16_t i = 0, j = 0x01; i < 16; j <<= 1, i++) {
        if (i == 4 || i == 5) continue; // 跳过ctrl和shift位
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rf_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rf_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rf_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    // 保存当前帧为上一帧
    memcpy(&rf_ctrl[LAST], &rf_ctrl[TEMP], sizeof(referee_ctrl_t));
}

static void RefereeControlRxCallback()
{
    DaemonReload(rfc_daemon_instance);         // 先喂狗
    rfcDecode(rfc_usart_instance->recv_buff); // 进行协议解析
}

// 遥控器离线回调
void RFCLostCallback()
{
    memset(rf_ctrl, 0 , sizeof(rf_ctrl));
}

// 初始化遥控器
referee_ctrl_t *RefereeControlInit(UART_HandleTypeDef *rf_usart_handle)
{
    
    USART_Init_Config_s conf;
    conf.usart_handle = rf_usart_handle;
    conf.recv_buff_size = CUSTOM_CONTROLLER_FRAME_SIZE;
    conf.module_callback =RefereeControlRxCallback;
    rfc_usart_instance = USARTRegister(&conf);
    
    Daemon_Init_Config_s daemon_conf = {
        .owner_id = rfc_usart_instance,
        .reload_count = 30,
        .init_count = 30,
    };
    rfc_daemon_instance = DaemonRegister(&daemon_conf);
    rfc_daemon_instance->callback = RFCLostCallback;
    rfc_init_flag = 1;
    return rf_ctrl;
}


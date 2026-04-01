#include "general_def.h"
#include "bsp_usart.h"

#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

typedef enum {
    RFPT_DATA_STATE_IDLE,
    RFPT_DATA_STATE_HEADER_FOUND,
    RFPT_DATA_STATE_RECEIVING,
    RFPT_DATA_STATE_COMPLETE
} rfpt_parse_state_t;

#define RFPT_RECV_HEADER 0xA9
#define RFC_RECV_HEADER 0x53

#define CUSTOM_CONTROLLER_FRAME_SIZE 39      // 

/* ----------------------- Data Struct ------------------------------------- */
typedef union
{
    struct // 用于访问键盘状态
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t d : 1;
        uint16_t a : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys; // 用于memcpy而不需要进行强制类型转换
} Key_t;

typedef struct
{
    uint8_t header0;
    uint8_t header1;
    struct
    {
        int16_t rocker_left_x; // 左水平
        int16_t rocker_left_y; // 左竖直
        int16_t rocker_right_x; // 右水平
        int16_t rocker_right_y; // 右竖直
        int16_t dial;      // 侧边拨轮

        uint8_t rc_switch;  // 左侧开关
        uint8_t pause;
        uint8_t custom_left;
        uint8_t custom_right;
        uint8_t trigger;

    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
        uint8_t press_m;
    } mouse;

    Key_t key[3]; 

    uint8_t key_count[3][16];
} referee_ctrl_t;

// 数据累积缓冲区
typedef struct {
    uint8_t buffer[254];
    uint8_t index;
    uint8_t ready;  // 标志是否累积了完整的一帧
} rfc_data_buffer_t;

referee_ctrl_t *RefereeControlInit(UART_HandleTypeDef *rf_usart_handle);


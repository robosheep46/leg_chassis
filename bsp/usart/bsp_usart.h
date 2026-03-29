#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>
#include "main.h"


#include "cmsis_os.h"
#include "event_groups.h"   
#include "semphr.h"      

#define DEVICE_USART_CNT 3     // C板至多分配3个串口
#define USART_RXBUFF_LIMIT 256 // 如果协议需要更大的buff,请修改这里
#define USART_BUFFER_COUNT 2 

// 事件组位定义（用于中断与任务同步）
#define USART_BUFFER0_FULL_BIT  (1 << 0)   // 缓冲区0已满待处理
#define USART_BUFFER1_FULL_BIT  (1 << 1)   // 缓冲区1已满待处理
#define USART_BUFFER_CONSUMED_BIT (1 << 2) // 消费完成

// 模块回调函数,用于解析协议
typedef void (*usart_module_callback)(uint8_t *data, uint16_t len);

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

// 串口实例结构体,每个module都要包含一个实例.
typedef struct
{
    // 双缓冲相关
    uint8_t *rx_buffers[USART_BUFFER_COUNT];   // 指向动态分配的缓冲区
    volatile uint8_t active_buffer;            // 当前DMA使用的缓冲区索引
    volatile uint8_t buffer_full[USART_BUFFER_COUNT]; // 缓冲区是否已满
    uint16_t rx_len[USART_BUFFER_COUNT];       // 每个缓冲区实际接收的长度

    // 同步对象
    EventGroupHandle_t xEventGroup;            // 事件组
    SemaphoreHandle_t xMutex;                  // 保护共享资源
    
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint8_t recv_buff_size;                // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;

/**
 * @brief 注册一个串口实例,返回一个串口实例指针
 *
 * @param init_config 传入串口初始化结构体
 */
USARTInstance *usart_register(USART_Init_Config_s *init_config);

/**
 * @brief 启动串口服务,需要传入一个usart实例.一般用于lost callback的情况(使用串口的模块daemon)
 *
 * @param _instance
 */
void USARTServiceInit(USARTInstance *_instance);


/**
 * @brief 通过调用该函数可以发送一帧数据,需要传入一个usart实例,发送buff以及这一帧的长度
 * @note 在短时间内连续调用此接口,若采用IT/DMA会导致上一次的发送未完成而新的发送取消.
 * @note 若希望连续使用DMA/IT进行发送,请配合USARTIsReady()使用,或自行为你的module实现一个发送队列和任务.
 * @todo 是否考虑为USARTInstance增加发送队列以进行连续发送?
 * 
 * @param _instance 串口实例
 * @param send_buf 待发送数据的buffer
 * @param send_size how many bytes to send
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size,USART_TRANSFER_MODE mode);

/**
 * @brief 判断串口是否准备好,用于连续或异步的IT/DMA发送
 *
 * @param _instance 要判断的串口实例
 * @return uint8_t ready 1, busy 0
 */
uint8_t USARTIsReady(USARTInstance *_instance);
extern USARTInstance *usart_instance[DEVICE_USART_CNT];
#endif

#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
// #include "bsp_log.h"
#include "stdint.h"
#include <stdint.h>

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hfdcan和rxid与发生中断的实例相同的那个,调用其回调函数
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* --------------------两个注册CAN实例的基础功能-------------------- */
/**
* @brief 主要实现两个功能，一个用于添加过滤器、另外一个用于启用CAN服务。
*/

/**
 * @note 添加两个过滤器，一个是用来过滤标准帧的，一个是用来过滤扩展帧的
 */
static void can_add_filter(CANInstance *_instance)
{
    //声明一个can过滤器的初始化信息结构体
     FDCAN_FilterTypeDef can_filter_config;
    //只接收标准帧，相当于F4的IDE
    can_filter_config.IdType = FDCAN_STANDARD_ID;
    //使用0号过滤器进行信息过滤
    can_filter_config.FilterIndex = 0;
    //运用掩码模式
    can_filter_config.FilterType = FDCAN_FILTER_MASK;
    //全部接收
    can_filter_config.FilterID1 = 0x00000000;
    can_filter_config.FilterID2 = 0x00000000; 
    //将过滤到的标准帧的信息存入到FIFO0队列
    can_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    //过滤器初始化，就是将前面配好的信息全部填进里面
    HAL_FDCAN_ConfigFilter(_instance->can_handle, &can_filter_config);
    //将不匹配的标准帧和扩展帧进行过滤，对遥控标准帧及遥控扩展帧进行过滤
    HAL_FDCAN_ConfigGlobalFilter(_instance->can_handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

static void CANAddExtendedFilter(CANInstance *_instance)
{
    //声明一个can过滤器的初始化信息结构体
     FDCAN_FilterTypeDef can_filter_config;
    can_filter_config.IdType = FDCAN_EXTENDED_ID;
    //使用1号过滤器进行信息过滤
    can_filter_config.FilterIndex = 1;
    //运用掩码模式
    can_filter_config.FilterType = FDCAN_FILTER_MASK;
    //全部接收
    can_filter_config.FilterID1 = 0x00000000;
    can_filter_config.FilterID2 = 0x00000000; 
    //将过滤到的扩展帧信息存入到FIF01队列
    can_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    //过滤器初始化，就是将前面配好的信息全部填进里面
    HAL_FDCAN_ConfigFilter(_instance->can_handle, &can_filter_config);
    //将不匹配的标准帧和扩展帧进行过滤，对遥控标准帧及遥控扩展帧进行过滤
    HAL_FDCAN_ConfigGlobalFilter(_instance->can_handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}

/** 
 * @note 此函数会启动CAN1、CAN2、CAN3，并开启FIFO0溢出通知 (此处基本与F4开发板相同)
 */
static void can_service_init()
{
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* ----------------------- CAN注册函数 -----------------------*/
/**
* @brief 以下是CAN的注册函数，在运用到CAN的电机或需要两个开发板进行通信的CAN初始化被调用
*/
CANInstance *can_register(CAN_Init_Config_s *config)
{
    if (!idx)    //如果还没有这个CAN，则初始化CAN服务
    {
        can_service_init(); 
        // LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
        ;
            // LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
            ;
                // LOGERROR("[}bsp_can] CAN id crash ,tx [%d] or rx [%d] already registered", &config->tx_id, &config->rx_id);
        }
    }
    //分配内存空间并置0
    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); 
    memset(instance, 0, sizeof(CANInstance));                           
    // 进行发送报文的配置
    // 一般来说，发送数据帧（遥控帧几乎不用）相当于F4的RTR
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;
    // 默认发送长度为8,相当于F4的DLC   
    instance->txconf.DataLength = 0x08;            
    //出现错误时 ESI位为1
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;  // 错误状态指示器
    //经典CAN不需要使用到位速率切换
    instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;
    //设置为经典CAN
    instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;
    //不用记录发送事件，减少内存开销        
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0;
    //设置句柄（&hfdcan1/&hfdcan2/&hfdcan3）                     
    instance->can_handle = config->can_handle;
    instance->tx_id =config->tx_id;
    //设置rx_id(注意id不能重复！！！！)
    instance->rx_id = config->rx_id;
    // 设置回调函数(当我接收到数据时触发这个函数，如：当我接收到电机数据时，对电机数据进行解码)
    instance->can_module_callback = config->can_module_callback;
    //用于区分是开发板互相通信的模块还是电机模块
    instance->id = config->id;

    can_add_filter(instance);
    // 将实例保存到can_instance中         
    can_instance[idx++] = instance; 
    // 返回can实例指针
    return instance; 
}

/* ----------------------- CAN发送函数 -----------------------*/
uint8_t can_transmit(CANInstance *_instance, float timeout)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    // _instance->txconf.Identifier = _instance->tx_id;
    TxHeader.Identifier = _instance->tx_id;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = _instance->txconf.DataLength;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle,&_instance->txconf, _instance->tx_buff))
    {
        // LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
        // busy_count++;
        return 0;
    }
    return 1; // 发送成功
}

void can_set_data_length(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        while (1)
        ;
        // LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
    _instance->txconf.DataLength = length;
}

/* -----------------------CAN的接收回调函数--------------------------*/

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,如果有回调函数，就调用该实例的回调函数
 *
 * @param _hfdcan
 * @param fifox passed to HAL_FDCAN_GetRxMessage() to get mesg from a specific fifo
 */
static void can_callback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    static FDCAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        if (rxconf.IdType == FDCAN_STANDARD_ID) 
        {
            for (size_t i = 0; i < idx; ++i)
            { // 两者相等说明这是要找的实例
                {
                    if (_hfdcan == can_instance[i]->can_handle
                     && rxconf.Identifier == can_instance[i]->rx_id)
                    {
                        if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
                        {
                            can_instance[i]->rx_len = rxconf.DataLength;                      // 保存接收到的数据长度
                            memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DataLength); // 消息拷贝到对应实例
                            can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
                        }
                        return;
                    }
                }
            }
        }
    }
}

/**
 * @brief FIFO0和FIFO1的接收回调函数
 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can_callback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can_callback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}

// void can_check_timeout(uint32_t timeout_ms)
// {
//     uint32_t current_time = dwt_get_time_ms();
//     for (size_t i = 0; i < idx; i++)
//     {
//         if ((current_time - last_time) > 2000)
//         {
//             // 清零接收缓冲区
//             memset(can_instance[i]->rx_buff, 0, sizeof(can_instance[i]->rx_buff));            
//         }
//     }
// }
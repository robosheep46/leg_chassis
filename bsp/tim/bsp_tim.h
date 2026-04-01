#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "tim.h"
#include "stdint.h"


#define TIM_DEVICE_CNT 16 // 最大支持的TIM实例数量

/* tim实例结构体 */
typedef struct tim
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    void (*callback)(struct tim *); 
} TIMInstance;

typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM句柄  
    void (*callback)(TIMInstance*);
} TIM_Init_Config_s;

/**
 * @brief 注册一个tim实例
 *
 * @param config 初始化配置
 * @return TIMInstance*
 */
TIMInstance *tim_register(TIM_Init_Config_s *TIM_config);

#endif // BSP_TIM_H
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"

#define PWM_DEVICE_CNT 16 // 最大支持的PWM实例数量

#define HCLK 640000000   //最大时钟频率

/* pwm实例结构体 */
typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    float period;                         // 周期
    float dutyratio;                      // 占空比
} PWMInstance;

typedef struct
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    float period;                         // 周期
    float dutyratio;                      // 占空比
} PWM_Init_Config_s;


PWMInstance *PWMRegister(PWM_Init_Config_s *PWM_config);
void PWMStart(PWMInstance *pwm);
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);
void PWMStop(PWMInstance *pwm);
void PWMSetPeriod(PWMInstance *pwm, float period);

 
 #endif // BSP_PWM_H
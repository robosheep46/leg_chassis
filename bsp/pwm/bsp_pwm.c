#include "bsp_pwm.h"
#include <string.h>
#include <stdlib.h>

static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL}; // 所有的pwm instance保存于此
static uint8_t idx;

PWMInstance *PWMRegister(PWM_Init_Config_s *PWM_config)
{
    PWMInstance *pwm = (PWMInstance *)malloc(sizeof(PWMInstance));
    memset(pwm, 0, sizeof(PWMInstance));

    pwm->htim = PWM_config->htim;
    pwm->channel=PWM_config->channel;
    // 启动PWM
    HAL_TIM_PWM_Start(pwm->htim,pwm->channel);

    pwm_instance[idx++] = pwm;
    return pwm;
}

void PWMStart(PWMInstance *_instance)
{
    HAL_TIM_PWM_Start(_instance->htim, _instance->channel);
}

void PWMStop(PWMInstance *_instance)
{
    HAL_TIM_PWM_Stop(_instance->htim, _instance->channel);
}

void PWMSetDutyRatio(PWMInstance *_instance, float dutyratio)
{
    __HAL_TIM_SetCompare(_instance->htim,_instance->channel, dutyratio * (_instance->htim->Instance->ARR));
}

void PWMSetPeriod(PWMInstance *_instance, float period)
{
    //HCLK为最大时钟频率 72MHZ，Prescaler为PSC的值
    __HAL_TIM_SetAutoreload(_instance->htim,period * HCLK /(_instance->htim->Init.Prescaler+1));
}

 
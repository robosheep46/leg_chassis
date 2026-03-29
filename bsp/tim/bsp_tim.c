#include "bsp_tim.h"
#include <string.h>
#include <stdlib.h>

static TIMInstance *tim_instance[TIM_DEVICE_CNT] = {NULL}; // 所有的tim instance保存于此,用于callback时判断中断来源
static uint8_t idx;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
      HAL_IncTick();
    }
    
    TIMInstance *tim;
    for (uint8_t i = 0; i < idx; i++)
    {
        tim=tim_instance[i];
        if (tim->htim == htim&&tim->callback!=NULL)
        {
            tim->callback(tim);
            return;
        }
    }
}
TIMInstance *TIMRegister(TIM_Init_Config_s *TIM_config)
{
    TIMInstance *tim = (TIMInstance *)malloc(sizeof(TIMInstance));
    memset(tim, 0, sizeof(TIMInstance));

    tim->htim = TIM_config->htim;
    tim->callback = TIM_config->callback;
    // 启动TIM中断
    HAL_TIM_Base_Start_IT(tim->htim);

    tim_instance[idx++] = tim;
    return tim;
}


#include "bsp_adc.h"
#include "stdlib.h"
#include <string.h>

// 配合中断以及初始化
static uint8_t idx;
static ADCInstance *adc_instance[ADC_DEVICE_CNT] = {NULL}; // 所有的ADC instance保存于此,用于callback时判断中断来源

static void ADCStartDMA(ADC_HandleTypeDef *adc_handle, uint32_t *pData, uint32_t Size)
{
    HAL_ADC_Start_DMA(adc_handle, pData, Size);
}

void ADCStart(ADC_HandleTypeDef *adc_handle,uint32_t *recv_buf)
{
    HAL_ADCEx_Calibration_Start(adc_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    ADCStartDMA(adc_handle,recv_buf,1);
}

void USER_ADC_Voltage_Update(ADCInstance *_instance)
{
    uint32_t *rxbuff = _instance->rx_buff; 
    _instance->Voltage = (rxbuff[0]*3.3f/65535)*11.0f;
}   

ADCInstance *ADCRegister(ADC_Init_Config_s *config)
{
    if (idx >= ADC_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    ADCInstance *adc = (ADCInstance *)malloc(sizeof(ADCInstance));
    memset(adc, 0, sizeof(ADCInstance));

    adc->adc_handle = config->adc_handle;
    adc->callback =config->callback;
    adc->size=config->size;
    return adc;
}

ADCInstance *ADC_Init(ADC_Init_Config_s *config)
{
    ADCInstance *instance = (ADCInstance *)malloc(sizeof(ADCInstance));

    ADC_Init_Config_s adc_config = {
        .callback = USER_ADC_Voltage_Update,
        .adc_handle=config->adc_handle,
        .size=config->size,
    };
    instance = ADCRegister(&adc_config);
}

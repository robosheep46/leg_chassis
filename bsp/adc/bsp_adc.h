#include "adc.h"
#include "stdint.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h723xx.h"

#define ADC_DEVICE_CNT 3 // 最大支持的adc实例数量

typedef struct adc_
{
    ADC_HandleTypeDef *adc_handle;                 // adc句柄

    void (*callback)(struct adc_ *); // DMA传输完成回调函数
    uint32_t size;
    uint32_t rx_buff[1];              // 接收缓存,最大消息长度为8
    float Voltage;
} ADCInstance;

typedef struct
{
    ADC_HandleTypeDef *adc_handle;                 // adc句柄

    void (*callback)(ADCInstance*); // DMA传输完成回调函数
    uint32_t size;
    uint32_t rx_buff[1];              // 接收缓存,最大消息长度为8

} ADC_Init_Config_s;

void ADCStart(ADC_HandleTypeDef *adc_handle,uint32_t *send_buf);
ADCInstance *ADC_Init(ADC_Init_Config_s *config);

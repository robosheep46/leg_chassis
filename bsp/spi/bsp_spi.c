#include "bsp_spi.h"
#include "memory.h"
#include "stdlib.h"

/* 所有的spi instance保存于此,用于callback时判断中断来源*/
static SPIInstance *spi_instance[SPI_DEVICE_CNT] = {NULL};
static uint8_t idx = 0;                         // 配合中断以及初始化

SPIInstance *spi_register(SPI_Init_Config_s *conf)
{
    if (idx >= MX_SPI_BUS_SLAVE_CNT) // 超过最大实例数
        while (1)
            ;
    SPIInstance *instance = (SPIInstance *)malloc(sizeof(SPIInstance));
    memset(instance, 0, sizeof(SPIInstance));

    instance->spi_handle = conf->spi_handle;
    instance->GPIOx = conf->GPIOx;
    instance->cs_pin = conf->cs_pin;

    spi_instance[idx++] = instance;
    return instance;
}

void spi_transmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // 拉低片选,开始传输(选中从机)
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    spi_ins->CS_State =HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
    HAL_SPI_Transmit(spi_ins->spi_handle, ptr_data, len, 1000); // 默认50ms超时
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
    spi_ins->CS_State =HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
}

void spi_receive(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // 用于稍后回调使用
    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data;
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    spi_ins->CS_State =HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
    HAL_SPI_Receive(spi_ins->spi_handle, ptr_data, len, 1000);
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
    spi_ins->CS_State =HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
}

void spi_transmit_receive(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
{
    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data_rx;
    
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    spi_ins->CS_State =HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
    HAL_SPI_TransmitReceive(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len, 1000); // 默认50ms超时
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
    spi_ins->CS_State = HAL_GPIO_ReadPin(spi_ins->GPIOx, spi_ins->cs_pin);
}
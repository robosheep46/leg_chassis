#include "spi.h"
#include "stdint.h"
#include "gpio.h"

/* 根据开发板引出的spi引脚以及CubeMX中的初始化配置设定 */
#define SPI_DEVICE_CNT 2       
#define MX_SPI_BUS_SLAVE_CNT 4 // 单个spi总线上挂载的从机数目


/* SPI实例结构体定义 */
typedef struct spi_ins_temp
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    uint8_t rx_size;               // 本次接收的数据长度
    uint8_t *rx_buffer;            // 本次接收的数据缓冲区
    uint8_t CS_State;              // 

} SPIInstance;


typedef struct
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
} SPI_Init_Config_s;

/**
 * @brief 注册一个spi instance
 *
 * @param conf 传入spi配置
 * @return SPIInstance* 返回一个spi实例指针,之后通过该指针操作spi外设
 */
SPIInstance *spi_register(SPI_Init_Config_s *conf);

/**
 * @brief 通过spi向对应从机发送数据
 *
 * @param spi_ins spi实例指针
 * @param ptr_data 要发送的数据
 * @param len 待发送的数据长度
 */
void spi_transmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);

/**
 * @brief 通过spi从从机获取数据 * 
 * @param spi_ins spi实例指针
 * @param ptr_data 接受数据buffer的首地址
 * @param len 待接收的长度
 */
void spi_receive(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);

/**
 * @brief 通过spi利用移位寄存器同时收发数据
 * 
 * @param spi_ins spi实例指针
 * @param ptr_data_rx 接收数据地址
 * @param ptr_data_tx 发送数据地址
 * @param len 接收&发送的长度
 */
void spi_transmit_receive(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);

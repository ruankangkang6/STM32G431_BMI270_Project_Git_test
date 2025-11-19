
#ifndef __SPI_DRIVER_H__
#define __SPI_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif


// #include "main.h"
#include "stm32g4xx_hal.h"
#include "spi.h"
#include "gpio.h"

#include <string.h>

extern SPI_HandleTypeDef hspi3;

#define BMI270_CS_GPIO_PORT   GPIOA
#define BMI270_CS_PIN         GPIO_PIN_15

// Set CS pin to  high level
#define BMI270_CS_LOW()    HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_RESET)

// Set CS pin to  low level
#define BMI270_CS_HIGH()  HAL_GPIO_WritePin(BMI270_CS_GPIO_PORT, BMI270_CS_PIN, GPIO_PIN_SET)

#define BMI270_SPI_W              ( 0x00 )
#define BMI270_SPI_R              ( 0x80 )

void BMI270_spi_read_register(uint8_t reg, uint8_t *data);
void BMI270_spi_write_register(uint8_t reg, uint8_t value);
int BMI270_read_rawdata(uint8_t reg, uint8_t *data, uint16_t len);
void BMI270_spi_write_registers(uint8_t reg, const uint8_t *data, uint16_t len);
void BMI270_read_ACC_gyr_fifo_data(uint8_t reg, uint8_t *data, uint16_t len);
void BMI270_read_fifo_register(uint8_t reg, uint8_t *read_buff, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BMI270_H__ */


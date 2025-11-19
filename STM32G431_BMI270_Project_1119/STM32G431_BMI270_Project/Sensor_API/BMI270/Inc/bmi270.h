
#ifndef __BMI270_H__
#define __BMI270_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "spi_driver.h"
#include "bmi270_config_file.h"
#include "gpio.h"
#include <string.h>

extern SPI_HandleTypeDef hspi3;

#define BMI270_TIMEOUT_COUNT      ( 0x00FF )  

#define BMI270_CHIP_ID            ( 0x00 )
#define BMI270_PWR_CONF           ( 0x7C )
#define BMI270_PWR_CTRL           ( 0x7D )
#define BMI270_INIT_CTRL          ( 0x59 )
#define BMI270_INIT_DATA          ( 0x5E )
#define BMI270_INT_STA            ( 0x21 )
#define BMI270_ACC_ADDRESS        ( 0x0C )
#define BMI270_GYRO_ADDRESS       ( 0x12 )
#define BMI270_ACC_CONF           ( 0x40 )
#define BMI270_ACC_RANGE          ( 0x41 )
#define BMI270_GYR_CONF           ( 0x42 )
#define BMI270_GYR_RANGE          ( 0x43 )

// Acceleration range setting
#define BMI270_ACC_RANGE_2G     0x00
#define BMI270_ACC_RANGE_4G     0x01
#define BMI270_ACC_RANGE_8G     0x02
#define BMI270_ACC_RANGE_16G    0x03

// gyroscope range setting
#define BMI270_GYRO_RANGE_2000_DPS	0	// ±2000 °/s
#define BMI270_GYRO_RANGE_1000_DPS	1	// ±1000 °/s
#define BMI270_GYRO_RANGE_500_DPS 	2	// ±500  °/s
#define BMI270_GYRO_RANGE_250_DPS 	3	// ±250  °/s
#define BMI270_GYRO_RANGE_125_DPS 	4	// ±125  °/s 

// BMI270 Command
#define BMI270_SOFT_RESET_CMD   0xB6
#define BMI270_ACC_ENABLE_CMD   0x04

// FIFO
#define BMI270_FIFO_DATA_REG    0x24

// // Acc register 
// #define BMI270_ACC_X_LSB_REG    0x0C


#define MOVING_AVERAGE_LENGTH 16

typedef struct {
    float buffer[MOVING_AVERAGE_LENGTH];
    uint8_t index;
    int32_t sum;
} MovingAverageFilter;

#define FILTER_LENGTH    16

uint8_t BMI270_Init(void);
int Output_acceleration_data(float *accel_x, float *accel_y, float *accel_z);
int Output_gyroscope_data(float *gyro_x, float *gyro_y, float *gyro_z);

void BMI270_read_ACC_gyr_fifo_data(uint8_t reg, uint8_t *data, uint16_t len);
int16_t MovingAverage_Update(float *acc_x, float *acc_y, float *acc_z);

int16_t first_order_lpf_6axis(float data[6]);
int BMI270_ReadSensorData(void);

int get_acc_gyr_data(float *physical_data, uint8_t len);

int convert_physical_data(float *physical_data, uint8_t len, const int16_t *raw_data);

void BMI270_ReadGyroscope(int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif

#endif /* __BMI270_H__ */


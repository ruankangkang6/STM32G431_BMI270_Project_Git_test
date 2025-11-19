
#include "spi_driver.h"

static uint8_t read_acc_gyro_fifo[512] = {0};

/**
  * @brief  Read single register
  * @param  reg: Register address
  * @retval Read data value
  */
void BMI270_spi_read_register(uint8_t reg, uint8_t *data)
{
  uint8_t send_data = reg | BMI270_SPI_R; // Set read flag (MSB=1)
  uint8_t recv_data[2] = {0};
  
  BMI270_CS_LOW() ;
  HAL_SPI_Transmit(&hspi3, &send_data, 1, 100);
  HAL_SPI_Receive(&hspi3, recv_data, 2, 100);
  BMI270_CS_HIGH();

  *data = recv_data[1];

}

/**
  * @brief  Read FIFO register
  * @param  reg: Register address
  * @retval Read data value
  */
void BMI270_read_fifo_register(uint8_t reg, uint8_t *read_buff, uint16_t len)
{
  uint8_t send_data[5] = {0}; 
  uint8_t recv_data[5] = {0};

  send_data[0] = reg | BMI270_SPI_R; 
  
  BMI270_CS_LOW();
  HAL_SPI_Transmit(&hspi3, send_data, 1, 100);
  HAL_SPI_Receive(&hspi3, read_buff, len+1, 100);
  BMI270_CS_HIGH();

}

/**
  * @brief  Write to single register
  * @param  reg: Register address
  * @param  value: Value to write
  */
void BMI270_spi_write_register(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] ={0};
    
    // Set write flag (MSB=0)
    txData[0] = reg & 0x7F; 
    txData[1] = value;
    
    BMI270_CS_LOW() ;
    HAL_SPI_Transmit(&hspi3, txData, 2, HAL_MAX_DELAY);
    BMI270_CS_HIGH();

}

/**
  * @brief  Read multiple consecutive registers
  * @param  reg: Starting register address
  * @param  data: Data buffer for storage
  * @param  len: Number of bytes to read
  */
int BMI270_read_rawdata(uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t temp_data[20] = {0};
  uint8_t send_data = reg | BMI270_SPI_R; // Read acc register

  BMI270_CS_LOW() ;
  HAL_SPI_Transmit(&hspi3, &send_data, 1, HAL_MAX_DELAY); 
  HAL_SPI_Receive(&hspi3, temp_data, len+1, HAL_MAX_DELAY); 
  BMI270_CS_HIGH();

  memcpy(data, temp_data + 1, len);
	
	return 0;

}


/**
  * @brief  Read acceleration and gyroscope FIFO registers
  * @param  reg: Starting register address
  * @param  data: Data buffer for storage
  * @param  len: Number of bytes to read
  */
void BMI270_read_ACC_gyr_fifo_data(uint8_t reg, uint8_t *data, uint16_t len)
{
  uint8_t read_data = 0;
  uint16_t fifo_len = 0;

  uint8_t fifo_reg = reg | BMI270_SPI_R; // Read acc register

    // Read fifo len.
    uint8_t lsb = 0;
    uint8_t msb = 0;

    BMI270_spi_read_register(0x24, &lsb);
    BMI270_spi_read_register(0x25, &msb);
    fifo_len = (msb << 8) | lsb;
    // printf("FIFO len: %d\n", fifo_len);

  // Init acceleration and gyroscope fifo 
  memset(read_acc_gyro_fifo, 0, sizeof(read_acc_gyro_fifo));

  BMI270_CS_LOW() ;
  // Send start address
  HAL_SPI_Transmit(&hspi3, &fifo_reg, 1, HAL_MAX_DELAY); 
  // Receive Data
  HAL_SPI_Receive(&hspi3, read_acc_gyro_fifo, fifo_len+1, HAL_MAX_DELAY);   
  BMI270_CS_HIGH();

}

/**
  * @brief  Write to multiple consecutive registers
  * @param  reg: Starting register address
  * @param  data: Data to write
  * @param  len: Number of bytes to write
  */
void BMI270_spi_write_registers(uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint8_t txData = reg & 0x7F; // Set write flag (MSB=0)
   
    BMI270_CS_LOW() ;
    HAL_SPI_Transmit(&hspi3, &txData, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi3, data, len, HAL_MAX_DELAY);
    BMI270_CS_HIGH();

}




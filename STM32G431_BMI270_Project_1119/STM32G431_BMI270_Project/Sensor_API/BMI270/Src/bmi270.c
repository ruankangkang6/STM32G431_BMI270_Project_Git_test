#include "bmi270.h"
#include "angle_calc.h"

#include "idcu_hill_start_assist.h"

static uint8_t read_acc_gyro_fifo[512] = {0};

int16_t bmi270_gyro_x = 0, bmi270_gyro_y = 0, bmi270_gyro_z = 0;   
int16_t bmi270_acc_x = 0, bmi270_acc_y = 0, bmi270_acc_z = 0; 

// Global buffer for raw sensor data
static int16_t raw_sensor_data[6] = {0};

/**
  * @brief  Check bmi270
  * @param  void
  */
static uint8_t bmi270_self_check (void)
{
  uint8_t dat = 0, return_state = 0;
  uint16_t timeout_count = 0;

  do
  {
      if(BMI270_TIMEOUT_COUNT < timeout_count ++)
      {
          return_state = 1;
          break;
      }
      BMI270_spi_read_register(BMI270_CHIP_ID, &dat);
      // printf("chip id:%02x\n",dat);

      HAL_Delay(1);
  }while(0x24 != dat);    // BIM270 chip id is 0x24

  return return_state;
}

/**
  * @brief  Initialize BMI270 sensor
  * @retval Initialization status (BMI270_OK or error code)
  */
uint8_t BMI270_Init(void)
{
    uint8_t return_state = 0;
    uint8_t read_data = 0;

    // BMI270 check
    if(bmi270_self_check())   
    {
        // printf("bmi270 self check error.\n");
        return_state = 1;
    }

    // Power mode config
    BMI270_spi_write_register(BMI270_PWR_CONF, 0x00); 
    HAL_Delay(1);
    // Start initialization
    BMI270_spi_write_register(BMI270_INIT_CTRL, 0x00);
    // Write to the configuration file
    BMI270_spi_write_registers(BMI270_INIT_DATA, bmi270_config_file, sizeof(bmi270_config_file));
    // Initialization configuration finished
    BMI270_spi_write_register(BMI270_INIT_CTRL, 0x01); 
    HAL_Delay(10);

    // Whether the read status is configured success
    BMI270_spi_read_register(BMI270_INT_STA, &read_data);
    if(read_data == 0) 
    {
        // printf("bmi270 init error\n");
        return_state = 1;
    }

    // Enable gyroscope and acceleration; Disable temperature
    BMI270_spi_write_register(BMI270_PWR_CTRL, 0x06);
    // Acceleration sampling 1.6KHz
    // BMI270_spi_write_register(BMI270_ACC_CONF, 0xAC);   // normal mode
    BMI270_spi_write_register(BMI270_ACC_CONF, 0xEC);    // 64 samples
    // Gyroscope sampling 1.6KHz
    BMI270_spi_write_register(BMI270_GYR_CONF, 0xAC); 
    // Acceleration range: 16G
    BMI270_spi_write_register(BMI270_ACC_RANGE, BMI270_ACC_RANGE_16G);
    // Gyroscope range: 2000dps
    BMI270_spi_write_register(BMI270_GYR_RANGE, BMI270_GYRO_RANGE_2000_DPS);

    return return_state;
}

/**
  * @brief  Read raw acceleration data
  * @param  x: Pointer for X-axis acceleration
  * @param  y: Pointer for Y-axis acceleration
  * @param  z: Pointer for Z-axis acceleration
  */
void BMI270_ReadAcceleration(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[7] = {0};
    
    // Read all accelerometer registers data
    BMI270_read_rawdata(BMI270_ACC_ADDRESS, buffer, 6);
    // Combine data (LSB first)
    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);
}

/**
  * @brief  Read raw Gyroscope data
  * @param  x: Pointer for X-axis Gyroscope
  * @param  y: Pointer for Y-axis Gyroscope
  * @param  z: Pointer for Z-axis Gyroscope
  */
void BMI270_ReadGyroscope(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buffer[7] = {0};
    
    // Read all gyroscope register data
    BMI270_read_rawdata(BMI270_GYRO_ADDRESS, buffer, 6);
    
    // Combine data (LSB first)
    *x = (int16_t)((buffer[1] << 8) | buffer[0]);
    *y = (int16_t)((buffer[3] << 8) | buffer[2]);
    *z = (int16_t)((buffer[5] << 8) | buffer[4]);
}

/**
  * @brief  Read both accelerometer and gyroscope data to global buffer
  * @retval 0: success, -1: read failed
  */
int BMI270_ReadSensorData(void)
{
    uint8_t buffer[12] = {0};

    // Read both accelerometer and gyroscope data
    if (BMI270_read_rawdata(BMI270_ACC_ADDRESS, buffer, sizeof(buffer)) != 0) 
        return -1;  // SPI read failed

    // Parse raw data (little-endian)
    for (int i = 0; i < 6; i++) 
    {
        raw_sensor_data[i] = (int16_t)((buffer[i*2 + 1] << 8) | buffer[i*2]);
    }

    return 0;
}

/**
  * @brief  Read both accelerometer and gyroscope data to provided buffer
  * @param  sensor_data: Pointer to buffer for storing sensor data
  * @param  len: Length of buffer (should be at least 6)
  */
void BMI270_read_gyr_acc_data(int16_t *sensor_data, uint8_t len)
{
    uint8_t buffer[12] = {0};

    // Get BMI270 acceleration gyroscope raw data
    BMI270_read_rawdata(BMI270_ACC_ADDRESS, buffer, sizeof(buffer));

    for (int i= 0; i < 6; i++)
    {
        *(sensor_data + i) = (int16_t)((buffer[(i*2) + 1] << 8) | buffer[i*2]);
    }
}

/**
  * @brief  Get pointer to raw sensor data buffer
  * @retval Pointer to raw sensor data array
  */
int16_t* BMI270_GetRawData(void)
{
    return raw_sensor_data;
}

/**
  * @brief  Convert raw sensor data to physical values and apply filtering
  * @param  physical_data: Output array for physical values [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z]
  * @param  len: Length of output array (should be at least 6)
  * @param  raw_data: Input raw sensor data array
  * @retval 0: success, -1: invalid parameters
  */
int convert_physical_data(float *physical_data, uint8_t len, const int16_t *raw_data)
{
    if (!physical_data || !raw_data || len < 6) 
    {
        return -1;  // Invalid parameters
    }

    float lsb_per_g = 0.0; 
    float lsb_per_dps;  // LSB per degree per second

    uint8_t acc_range = BMI270_ACC_RANGE_16G;
    uint8_t gyr_range = BMI270_GYRO_RANGE_2000_DPS;

    float temp_xyz_buff[6] = {0.0f};

    // Determine the sensitivity (LSB per g) based on the selected range
    switch (acc_range) 
    {
        case BMI270_ACC_RANGE_2G:   lsb_per_g = 16384.0f;  break;
        case BMI270_ACC_RANGE_4G:   lsb_per_g = 8192.0f;   break;
        case BMI270_ACC_RANGE_8G:   lsb_per_g = 4096.0f;   break;
        case BMI270_ACC_RANGE_16G:  lsb_per_g = 2048.0f;   break;
        default:    return -1;  // Invalid range provided
    }

    // Set sensitivity based on full-scale range
    switch (gyr_range) 
    {
        case BMI270_GYRO_RANGE_2000_DPS:   lsb_per_dps = 16.4f;    break;
        case BMI270_GYRO_RANGE_1000_DPS:   lsb_per_dps = 32.8f;    break;
        case BMI270_GYRO_RANGE_500_DPS:    lsb_per_dps = 65.6f;    break;
        case BMI270_GYRO_RANGE_250_DPS:    lsb_per_dps = 131.1f;   break;
        case BMI270_GYRO_RANGE_125_DPS:    lsb_per_dps = 262.2f;   break;
        default:   return -1;  // Invalid range
    }

    // Convert raw ADC values to physical acceleration in 'g'
    for (int i = 0; i < 3; i++)
    {
        temp_xyz_buff[i] = (float)raw_data[i] / lsb_per_g;      // Acceleration data
        temp_xyz_buff[i+3] = (float)raw_data[i+3] / lsb_per_dps;  // Gyroscope data
    }

    // Apply filtering
    first_order_lpf_6axis(temp_xyz_buff);
    memcpy(physical_data, temp_xyz_buff, 6 * sizeof(float));

    return 0;  // Success
}


/**
 * @brief Convert raw accelerometer data to physical acceleration values in 'g'
 *
 * @param accel_x   Pointer to float where the converted X-axis acceleration (in g) will be stored
 * @param accel_y   Pointer to float where the converted Y-axis acceleration (in g) will be storednin
 * @param accel_z   Pointer to float where the converted Z-axis acceleration (in g) will be stored
 *
 * @return int      Returns 0 on success, -1 if the range is invalid
 */
int Output_acceleration_data(float *accel_x,
                             float *accel_y,
                             float *accel_z)
{
    float lsb_per_g = 0.0; 
    int16_t raw_x, raw_y, raw_z;
    uint8_t range = BMI270_ACC_RANGE_16G;

    float temp_x,temp_y,temp_z; 

    BMI270_ReadAcceleration(&raw_x, &raw_y, &raw_z);

    // Determine the sensitivity (LSB per g) based on the selected range
    switch (range) {
        case BMI270_ACC_RANGE_2G:
            lsb_per_g = 16384.0f;  // ±2g: 16384 LSB per g
            break;
        case BMI270_ACC_RANGE_4G:
            lsb_per_g = 8192.0f;   // ±4g: 8192 LSB per g
            break;
        case BMI270_ACC_RANGE_8G:
            lsb_per_g = 4096.0f;   // ±8g: 4096 LSB per g
            break;
        case BMI270_ACC_RANGE_16G:
            lsb_per_g = 2048.0f;   // ±16g: 2048 LSB per g
            break;
        default:
            return -1;  // Invalid range provided
    }

    // Convert raw ADC values to physical acceleration in 'g'
    temp_x = (float)raw_x / lsb_per_g;
    temp_y = (float)raw_y / lsb_per_g;
    temp_z = (float)raw_z / lsb_per_g;

    // MovingAverage_Update(&temp_x, &temp_y, &temp_z);
    *accel_x = temp_x;
    *accel_y = temp_y;
    *accel_z = temp_z;

    printf("%f,%f,%f\n",temp_x,temp_y,temp_z);

    return 0;  // Success
}

/**
 * @brief Convert raw gyroscope data to physical angular rate (in degrees per second, dps)
 *
 * This function converts the raw 16-bit signed output from the BMI270 gyroscope
 * into actual angular velocity values based on the selected full-scale range.
 *
 * @param gyro_x    Pointer to float for storing converted X-axis angular rate (dps)
 * @param gyro_y    Pointer to float for storing converted Y-axis angular rate (dps)
 * @param gyro_z    Pointer to float for storing converted Z-axis angular rate (dps)
 *
 * @return int      Returns 0 on success, -1 if invalid range is provided
 */
int Output_gyroscope_data(float *gyro_x, float *gyro_y, float *gyro_z)
{
    float lsb_per_dps;  // LSB per degree per second
    uint8_t range = BMI270_GYRO_RANGE_2000_DPS;
    int16_t raw_x, raw_y, raw_z;

    float temp_x,temp_y,temp_z; 

    BMI270_ReadGyroscope(&raw_x, &raw_y, &raw_z);

    // Set sensitivity based on full-scale range
    switch (range) {
        case BMI270_GYRO_RANGE_2000_DPS:
            lsb_per_dps = 16.4f;   // 16.4 LSB/(°/s)
            break;
        case BMI270_GYRO_RANGE_1000_DPS:
            lsb_per_dps = 32.8f;   // 32.8 LSB/(°/s)
            break;
        case BMI270_GYRO_RANGE_500_DPS:
            lsb_per_dps = 65.6f;   // 65.6 LSB/(°/s)
            break;
        case BMI270_GYRO_RANGE_250_DPS:
            lsb_per_dps = 131.1f;  // 131.1 LSB/(°/s)
            break;
        case BMI270_GYRO_RANGE_125_DPS:
            lsb_per_dps = 262.2f;  // 262.2 LSB/(°/s)
            break;
        default:
            return -1;  // Invalid range
    }

    // Convert raw values to physical angular rate in degrees per second (dps)
    temp_x = (float)raw_x / lsb_per_dps;
    temp_y = (float)raw_y / lsb_per_dps;
    temp_z = (float)raw_z / lsb_per_dps;

    // MovingAverage_Update(&temp_x, &temp_y, &temp_z);

    *gyro_x = temp_x;
    *gyro_y = temp_y;
    *gyro_z = temp_z;

    return 0;  // Success
}

/**
  * @brief  Read raw Gyroscope data
  * @param  x: Pointer for X-axis Gyroscope
  * @param  y: Pointer for Y-axis Gyroscope
  * @param  z: Pointer for Z-axis Gyroscope
  */
void BMI270_Read_fifo_data()
{
    uint8_t buffer[7] = {0};

    uint8_t fifo_data[96]; // 8 samples * 12 bytes
    uint16_t fifo_length;

    uint8_t read_data;
    uint8_t len_l,len_h;

    BMI270_spi_read_register(0x24, &len_l);
    BMI270_spi_read_register(0x25, &len_h);

    fifo_length = (len_h << 8) | len_l;
    printf("fifo_len=%d, 0x24:%02x, 0x25:%02x\n",fifo_length,len_l,len_h);
}

static uint32_t buff_index = 0;
static uint8_t mov_aver_count = 0;

MovingAverageFilter acc_x_filter, acc_y_filter, acc_z_filter;

// Filter interface
int16_t moving_average_acc(float *acc_x, float *acc_y, float *acc_z)
{
    // static uint8_t index = 0;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    if (mov_aver_count < FILTER_LENGTH)   // Fill the array
    {
        acc_x_filter.buffer[buff_index] = *acc_x;
        acc_y_filter.buffer[buff_index] = *acc_y;
        acc_z_filter.buffer[buff_index] = *acc_z;
        mov_aver_count++;
    }
    else    // Acceleration value filtering
    {
        //  Update buff 
        acc_x_filter.buffer[buff_index] = *acc_x;
        acc_y_filter.buffer[buff_index] = *acc_y;
        acc_z_filter.buffer[buff_index] = *acc_z;

        // sum
        for (int i = 0; i < FILTER_LENGTH; i++) 
        {
            sum_x += acc_x_filter.buffer[i];
            sum_y += acc_y_filter.buffer[i];
            sum_z += acc_z_filter.buffer[i];
        }

        *acc_x = sum_x / FILTER_LENGTH ;
        *acc_y = sum_y / FILTER_LENGTH ;
        *acc_z = sum_z / FILTER_LENGTH ;
    }
    buff_index = (buff_index + 1) % FILTER_LENGTH;
    
    return 0;
}

// Define filter coefficients. Higher value: faster response; lower value: smoother output
#define LPF_ACC_ALPHA  0.20f    // Filter coefficient for accelerometer: 
#define LPF_GYR_ALPHA  0.30f    // Filter coefficient for gyroscope

// Static variables to store previous filtered output values (initialized to 0)
static float lpf_acc_x = 0.0f;
static float lpf_acc_y = 0.0f;
static float lpf_acc_z = 0.0f;
static float lpf_gyr_x = 0.0f;
static float lpf_gyr_y = 0.0f;
static float lpf_gyr_z = 0.0f;

/**
 * @brief  Apply first-order low-pass filter to 6-axis IMU data (accelerometer + gyroscope)
 * @param  data[6]  Input/output array: [ax, ay, az, gx, gy, gz]
 * @retval 0 on success
 */
int16_t first_order_lpf_6axis(float data[6])
{
    // Update accelerometer channels
    lpf_acc_x = LPF_ACC_ALPHA * data[0] + (1.0f - LPF_ACC_ALPHA) * lpf_acc_x;
    lpf_acc_y = LPF_ACC_ALPHA * data[1] + (1.0f - LPF_ACC_ALPHA) * lpf_acc_y;
    lpf_acc_z = LPF_ACC_ALPHA * data[2] + (1.0f - LPF_ACC_ALPHA) * lpf_acc_z;

    // Update gyroscope channels
    lpf_gyr_x = LPF_GYR_ALPHA * data[3] + (1.0f - LPF_GYR_ALPHA) * lpf_gyr_x;
    lpf_gyr_y = LPF_GYR_ALPHA * data[4] + (1.0f - LPF_GYR_ALPHA) * lpf_gyr_y;
    lpf_gyr_z = LPF_GYR_ALPHA * data[5] + (1.0f - LPF_GYR_ALPHA) * lpf_gyr_z;

    // Write back filtered values to the input array
    data[0] = lpf_acc_x;
    data[1] = lpf_acc_y;
    data[2] = lpf_acc_z;
    data[3] = lpf_gyr_x;
    data[4] = lpf_gyr_y;
    data[5] = lpf_gyr_z;

    return 0;
}


//定义姿态角结构体实例
Attitude_t current_attitude = {0.0f, 0.0f, 0.0f};
static uint32_t last_updata_time = 0;

//extern iDCU_SystemStatus_s iDCU_SystemStatus;

/**
  * @brief  Get accelerometer and gyroscope data (uses global raw data)
  * @param  physical_data: Output array for physical values
  * @param  len: Length of output array
  * @retval Processing status
  */
int get_acc_gyr_data(float *physical_data, uint8_t len)
{
    convert_physical_data(physical_data, len, raw_sensor_data);

    //计算采样周期（单位：秒）
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_updata_time) / 1000.0f;
    last_updata_time = current_time;

    //使用互补滤波计算姿态角
    Calculate_Attitude(physical_data[0], physical_data[1], physical_data[2],
                       physical_data[3], physical_data[4], physical_data[5],
                       dt, &current_attitude);

    iDCU_SystemStatus.slope_angle = current_attitude.pitch_y;   // roll
    iDCU_SystemStatus.roll_angle = current_attitude.roll_x;   // roll

    return 0;
}
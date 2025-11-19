#include "angle_calc.h"
#include <stdint.h>
#include <math.h>
#include <bmi270.h>

//互补滤波系数（0-1）
#define ALPHA 0.96f
//静止检测阈值
#define MOTION_THRESHOLD 0.1f
//重置条件计数阈值
#define RESET_COUNT_THRESHOLD 50

static float gyro_offset_x = 0.0f;
static float gyro_offset_y = 0.0f;
static float gyro_offset_z = 0.0f;
static uint16_t reset_counter = 0;

void Calculate_Attitude(float ax, float ay, float az, 
                        float gx, float gy, float gz, 
                       float dt,Attitude_t *attitude)
{
    //计算合加速度，用于检测静止状态
    float acc_magnitude = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_magnitude = sqrtf(gx*gx + gy*gy + gz*gz);
    
    //检测静止状态
    if(fabsf(acc_magnitude - 1.0f) < MOTION_THRESHOLD && 
       gyro_magnitude < MOTION_THRESHOLD) {
        reset_counter++;
    } else {
        reset_counter = 0;
    }
    
    //使用加速度计计算静态角度
    float roll_acc = atan2f(ay , az) * 57.3f;//横滚角
    float pitch_acc = atan2f(-ax , sqrtf(ay*ay + az *az))*57.3f;//俯仰角

    //补偿陀螺仪零偏
    gx -= gyro_offset_x;
    gy -= gyro_offset_y;
    gz -= gyro_offset_z;
    
    //使用陀螺仪积分计算动态角度
    float roll_gyro = attitude->roll_x + gx * dt; //横滚角
    float pitch_gyro = attitude->pitch_y + gy * dt; //俯仰角

    //当检测到持续静止时，逐渐增加加速度计权重
    float alpha = ALPHA;
    if(reset_counter > RESET_COUNT_THRESHOLD) {
        alpha = ALPHA * (1.0f - (float)(reset_counter - RESET_COUNT_THRESHOLD) / 100.0f);
        if(alpha < 0.5f) alpha = 0.5f;
    }

    //互补滤波融合两种传感器数据
    attitude->roll_x = alpha * roll_gyro + (1.0f - alpha) * roll_acc;
    attitude->pitch_y = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    //偏航角仅由陀螺仪积分得到
    attitude->yaw_z += gz * dt;

    //限制偏航角在-180 到 180度之间
    if(attitude->yaw_z > 180.0f)
        attitude->yaw_z -= 360.0f;
    else if(attitude->yaw_z < -180.0f)
        attitude->yaw_z += 360.0f;
}

// 陀螺仪校准函数
void Calibrate_Gyro(uint16_t samples, uint16_t delay_ms)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    
    // 清零当前的零偏值
    gyro_offset_x = 0.0f;
    gyro_offset_y = 0.0f;
    gyro_offset_z = 0.0f;
    
    // 采集多次数据取平均值
    for(uint16_t i = 0; i < samples; i++)
    {
        // 注意：这里需要根据实际的BMI270读取函数进行修改
        int16_t gx, gy, gz;
        //从BMI270读取陀螺仪数据
        BMI270_ReadGyroscope(&gx, &gy, &gz);
        
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        
        // 延时
        HAL_Delay(delay_ms);
    }
    
    // 计算平均值作为零偏
    gyro_offset_x = sum_x / samples;
    gyro_offset_y = sum_y / samples;
    gyro_offset_z = sum_z / samples;
}

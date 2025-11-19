#ifndef ANGLE_CALC_H_
#define ANGLE_CALC_H_   

#ifdef __cplusplus
extern "C" {
#endif

#include<math.h>
#include <stdint.h>  


//姿态角结构体
typedef struct
{
    volatile float roll_x;   //横滚角
    volatile float pitch_y;  //俯仰角
    volatile float yaw_z;    //偏航角
}Attitude_t;

//计算互补滤波的姿态角
void Calculate_Attitude(float ax, float ay, float az, 
                        float gx, float gy, float gz, 
                       float dt,Attitude_t *attitude);
												
void Calibrate_Gyro(uint16_t samples, uint16_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* ANGLE_CALC_H_ */

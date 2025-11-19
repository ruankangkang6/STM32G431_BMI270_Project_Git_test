/**
 * @file    idcu_sensor_data.h
 * @brief   
 * @details 
 */

#ifndef __IDCU_SENSOR_DATA_H__
#define __IDCU_SENSOR_DATA_H__

#include "idcu_system_types.h"

/* Vehicle system status structure for sharing input data across modules */
typedef struct {
    volatile float slope_angle;      	// Current slope angle,pitch-y (negative = uphill,  positive = downhill)
    volatile float roll_angle;      	// roll_angle
    float   wheel_speed;             	// Current vehicle speed (km/h)  Throttle speed
    uint16_t throttle_speed;         	//Throttle speed (speed:%)
    bool  brake_pressed;             	// Brake status(Brake:true; No brake: false)
    bool  throttle_input_valid;      	// Throttle signal validity
    float throttle_pedal_perc;       	// Throttle pedal percentage (0~100%)
    iDCU_GearLevel_t current_gear;   	// Current transmission gear
    bool  fault_flag;                	// Fault flag (true = fault present, disable all functions)
    bool  brake_just_released;       	// Brake just released event flag (for HSA/HHC triggering)
} iDCU_SystemStatus_s;


/* Extern global variable */
extern iDCU_SystemStatus_s iDCU_SystemStatus;

#endif /* __IDCU_SENSOR_DATA_H__ */
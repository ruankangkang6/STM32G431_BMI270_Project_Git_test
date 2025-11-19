
#ifndef __IDCU_API_H__
#define __IDCU_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// parameter
#define SENSOR_HORIZONTAL           0           // Horizontal value
#define SENSOR_VERTICAL             245         // Maximum value (90°)
#define MAX_SPEED                   0x03E8      // maximum speed
#define MIN_SENSOR_THRESHOLD        20          // Minimum slope threshold (10°)
#define MAX_SENSOR_THRESHOLD        100         // Maximum slope threshold (37°)

#define WHEEL_CONST 287.98f  // Constant for speed calculation
#define WHEEL_DIAMETER_INCH 10.0f  // Wheel diameter in inches


extern int controller_update_message_process();
void Hill_Start_Assist_speed_output(uint16_t speed);
void vehicle_start_speed_output(const uint16_t start_speed);
int HDA_ABS_output(const uint16_t start_speed);
void iDCU_SkidControl_Handler(void);

int idcu_main();
void idcu_init();


#ifdef __cplusplus
}
#endif

#endif /* __IDCU_API_H__ */


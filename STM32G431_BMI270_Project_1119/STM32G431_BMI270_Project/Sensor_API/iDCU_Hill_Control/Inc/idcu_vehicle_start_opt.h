// iDCU_VehicleStartOpt.h

#ifndef __IDCU_VEHICLE_START_OPT_H__
#define __IDCU_VEHICLE_START_OPT_H__

#include "idcu_system_types.h"

//First gear
#define VSO_MAX_SPEED_VARIATION_GEAR1    (6U)   // Maximum throttle speed variation
#define START_SPEED_GEAR1  (120)     // Speed below which Eco mode resets last output speed

//Second gear
#define VSO_MAX_SPEED_VARIATION_GEAR2    (5U)   // Maximum throttle speed variation
#define START_SPEED_GEAR2  (125)     // Speed below which Eco mode resets last output speed

/**
 * @brief Whether to enable this feature only in GEAR_1 or GEAR_2
 * 
 * - Set to 1: Only effective in GEAR_1 / GEAR_2
 * - Set to 0: Effective in all gears
 */
#define VSO_GEAR_1_OR_2_ONLY        (1)

void iDCU_VSO_Init(void);   // Init VSO 

uint16_t iDCU_VSO_GetOutputThrottle(void);

#endif /* IDCU_VEHICLE_START_OPT_H */
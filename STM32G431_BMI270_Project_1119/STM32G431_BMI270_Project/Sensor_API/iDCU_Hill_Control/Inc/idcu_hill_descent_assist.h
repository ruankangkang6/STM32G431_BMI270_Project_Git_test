// idcu_hill_descent_assist.h
// Description: Interface declarations for Hill Descent Assist (HDA) function

#ifndef __IDCU_HILL_DESCENT_ASSIST_H__
#define __IDCU_HILL_DESCENT_ASSIST_H__

#include "idcu_system_types.h"

// Function activation condition parameters
#define HDA_MIN_SLOPE_TO_ENTER_deg      (8.0f)   // Min slope (8°)
#define HDA_MAX_SLOPE_TO_EXIT_deg       (5.0f)   // Maximum slope for exit (<5°)
#define HDA_MIN_SPEED_TO_ENTER_kph      (10.0f)   // Minimum vehicle speed for entry
#define HDA_MAX_SPEED_TO_EXIT_kph       (10.0f)   // Maximum vehicle speed for exit (≤20 kph)
#define HDA_MAX_THROTTLE_TO_ENTER_perc  (200)     // Maximum throttle input allowed for entry (10%)
#define HDA_MIN_THROTTLE_TO_EXIT_perc   (200)     // Throttle exit threshold (≥15%)

#define HDA_SLOPE_ANGLE_10     (10.0f)   // Slope level 1 -> ABS_LEVEL_1    
#define HDA_SLOPE_ANGLE_20     (12.0f)   // Slope level 2 -> ABS_LEVEL_2
#define HDA_SLOPE_ANGLE_30     (15.0f)   // Slope level 3 -> ABS_LEVEL_3

// HDA state 
typedef enum {
    HDA_STATE_IDLE,      // Idle state
    HDA_STATE_ACTIVE,    // Active state
    HDA_STATE_EXITING    // Exiting state
} iDCU_HDA_State_e;

/* ABS braking level enumeration */
typedef enum {
    ABS_LEVEL_0      = 0,  // ABS minimum braking
    ABS_LEVEL_1      = 1,  // Light braking force (gentle slope)
    ABS_LEVEL_2      = 2,  // Medium braking force (moderate slope)
    ABS_LEVEL_3      = 3,  // Strong braking force (steep slope)
    ABS_LEVEL_CLOSE  = 4   // ABS disabled
} iDCU_HDA_BrakeLevel;

extern iDCU_HDA_BrakeLevel enHDA_BrakeLevel;

void iDCU_HDA_Init(void);
void iDCU_HDA_MainFunction(void);
iDCU_HDA_State_e iDCU_HDA_GetState(void);    
bool iDCU_HDA_IsActive(void);             
iDCU_HDA_BrakeLevel iDCU_HDA_GetBrakeLevel(void);

static iDCU_HDA_BrakeLevel prvCalculateBrakeLevel(float fSlopeDeg);

#endif /* __IDCU_HILL_DESCENT_ASSIST_H__ */
// iDCU_hill_start_assist.h

#ifndef __IDCU_HILL_START_ASSIST_H__
#define __IDCU_HILL_START_ASSIST_H__

#include <stdint.h>
#include "idcu_system_types.h"  
#include "idcu_sensor_data.h" 

/* Function Parameter Configuration */
#define HSA_MIN_SLOPE                       (-3.0f)     // Minimum slope angle (10°)
#define HSA_MAX_SLOPE_DEG                   (-30.0f)     // Maximum slope angle (30°)

#define HSA_MAX_SPEED_KPH                   (8.0f)    ///< Vehicle speed threshold: entry ≤5 kph; exit ≥5 kph
#define HSA_CAN_THROTTLE_ENTRY_THRESHOLD    (50U)   // Entry condition: ≤ 5%
#define HSA_CAN_THROTTLE_EXIT_THRESHOLD     (200U)  // Exit condition: ≥ 20%
#define HSA_MIN_THROTTLE_0_1000             (80U)   // Minimum slope throttle (80 / 1000 = 8%)
#define HSA_MAX_THROTTLE_0_1000             (280U)  // Maximum slope throttle (280 / 1000 = 28%)
#define HSA_MAX_HOLD_TIME                   (3000U)   ///< Maximum hold duration: 3000ms (3 seconds)


#define ratio_adjust                        (0.9f)   // Entry condition: ≤ 5%

/* HSA State */
typedef enum {
    HSA_STATE_IDLE,      // Inactive state
    HSA_STATE_ACTIVE,    // Active state
    HSA_STATE_EXITING    // Exit state
} iDCU_HSA_State_e;


void iDCU_HSA_Init(void);
void iDCU_HSA_OnBrakeReleaseEvent(void);
void idcu_hsa_interface(void);
iDCU_HSA_State_e iDCU_HSA_GetState(void);
bool iDCU_HSA_IsActive(void);
uint16_t iDCU_HSA_GetOverrideThrottle_0_1000(void);

void check_HSA_status(void);

#endif /* __IDCU_HILL_START_ASSIST_H__ */
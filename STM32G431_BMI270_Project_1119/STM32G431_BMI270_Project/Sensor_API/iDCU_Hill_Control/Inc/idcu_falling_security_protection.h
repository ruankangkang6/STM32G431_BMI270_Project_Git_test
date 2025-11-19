// idcu_falling_security_protection.h

#ifndef __IDCU_FALLING_SECURITY_PROTECTION_H__
#define __IDCU_FALLING_SECURITY_PROTECTION_H__

#include "idcu_system_types.h"

// Function activation threshold > 40°
#define FSP_TRIGGER_ANGLE_THRESHOLD_DEG    (40.0f)
// Function deactivation threshold <= 25°
#define FSP_EXIT_ANGLE_THRESHOLD_DEG       (25.0f)

// FSP states
typedef enum {
    FSP_STATE_IDLE,    // Inactive
    FSP_STATE_ACTIVE,  // Triggered: power cut + buzzer alarm
    FSP_STATE_EXITING  // Exiting
} iDCU_FSP_State_e;


void iDCU_FSP_Init(void);
void iDCU_FSP_MainFunction(void);      
iDCU_FSP_State_e iDCU_FSP_GetState(void);
bool iDCU_FSP_IsActive(void);

extern volatile bool g_FSP_Request_PowerCut;   
extern volatile bool g_FSP_Alarm_Active;   

#endif  /* __IDCU_FALLING_SECURITY_PROTECTION_H__ */
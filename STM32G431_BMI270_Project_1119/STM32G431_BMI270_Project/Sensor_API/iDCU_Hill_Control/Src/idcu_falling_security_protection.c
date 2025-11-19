
/**
 * @file idcu_falling_security_protection.c
 * @brief Fall Security Protection (FSP) Function Implementation
 * @details
 *   - When vehicle rollover is detected (>40°), immediately cut power and trigger alarm
 */

#include "idcu_falling_security_protection.h"
#include "idcu_sensor_data.h"  
#include "idcu_system_types.h"

volatile bool g_FSP_Request_PowerCut = false;
volatile bool g_FSP_Alarm_Active = false;

// Initialize IDLE state
static iDCU_FSP_State_e enFSP_State = FSP_STATE_IDLE;

// Check if fall is detected
static inline bool prv_is_fall_detected(float y_angle)
{
    return (y_angle > FSP_TRIGGER_ANGLE_THRESHOLD_DEG) ||
           (y_angle < -FSP_TRIGGER_ANGLE_THRESHOLD_DEG);
}

// Check if stability is restored
static inline bool prv_is_stable(float y_angle)
{
    return (y_angle <= FSP_EXIT_ANGLE_THRESHOLD_DEG) &&
           (y_angle >= -FSP_EXIT_ANGLE_THRESHOLD_DEG);
}

// FSP initialization
void iDCU_FSP_Init(void)
{
    enFSP_State = FSP_STATE_IDLE;
    g_FSP_Request_PowerCut = false;
    g_FSP_Alarm_Active = false;
}

void iDCU_FSP_MainFunction(void)
{
    const iDCU_SystemStatus_s* psSys = &iDCU_SystemStatus;
    float fXAngle = psSys->roll_angle;
    bool should_exit = false;

    /* ACTIVE state: Check if exit is needed */ 
    if (enFSP_State == FSP_STATE_ACTIVE) 
    {
        if (prv_is_stable(fXAngle)) {
            should_exit = true; // Fall event ended
        } else if (psSys->fault_flag) {
            should_exit = true; // System fault
        } else if (!iDCU_FSP_Enable) {
            should_exit = true; // Function disabled
        }

        if (should_exit) 
        {
            enFSP_State = FSP_STATE_IDLE;
            g_FSP_Request_PowerCut = false;
            g_FSP_Alarm_Active = false;
        }
        return;
    }

    /* IDLE state: Check if triggered */ 
    if (enFSP_State == FSP_STATE_IDLE) {
        if (!iDCU_FSP_Enable) {
            return; // Function not enabled
        }
        if (psSys->fault_flag) {
            return; // System fault
        }
        if (!prv_is_fall_detected(fXAngle)) {
            return; // No fall detected (|X| <= 40°)
        }

        // Enable FSP
        enFSP_State = FSP_STATE_ACTIVE;
        g_FSP_Request_PowerCut = true;  // Cut power
        g_FSP_Alarm_Active = true;      // Trigger alarm
        return;
    }

    // Set IDLE state
    enFSP_State = FSP_STATE_IDLE;
}

// Query interface
iDCU_FSP_State_e iDCU_FSP_GetState(void)
{
    return enFSP_State;
}

bool iDCU_FSP_IsActive(void)
{
    return (enFSP_State == FSP_STATE_ACTIVE);
}


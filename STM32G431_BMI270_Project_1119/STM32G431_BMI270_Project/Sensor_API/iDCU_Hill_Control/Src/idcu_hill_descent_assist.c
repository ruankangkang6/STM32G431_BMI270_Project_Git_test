
/**
 * @file idcu_hill_descent_assist.c
 * @brief Hill Descent Assist (HDA) function
 * @details
 *   - 
 */

#include "idcu_hill_descent_assist.h"
#include "idcu_sensor_data.h"
#include "idcu_falling_security_protection.h"   

#include "idcu_system_types.h"   

// Static variable: stores current state
static iDCU_HDA_State_e enHDA_State = HDA_STATE_IDLE;
iDCU_HDA_BrakeLevel enHDA_BrakeLevel = ABS_LEVEL_CLOSE;  

// Hill Descent Assist initialization
void iDCU_HDA_Init(void)
{
    enHDA_State = HDA_STATE_IDLE;
    enHDA_BrakeLevel = ABS_LEVEL_CLOSE;
}

// Main function
void iDCU_HDA_MainFunction(void)
{
    const iDCU_SystemStatus_s* psSys = &iDCU_SystemStatus;

    /* Process ACTIVE or EXITING state: Check if exit is needed */
    if (enHDA_State == HDA_STATE_ACTIVE || enHDA_State == HDA_STATE_EXITING) 
    {
        bool should_exit = false;

        // Exit HDA if any condition is met
        if (psSys->throttle_pedal_perc >= HDA_MIN_THROTTLE_TO_EXIT_perc) {
            should_exit = true; // Throttle ≥15%
        } else if (psSys->slope_angle < HDA_MAX_SLOPE_TO_EXIT_deg) {
            should_exit = true; // Slope <5°
        } 
//				else if (psSys->wheel_speed <= HDA_MAX_SPEED_TO_EXIT_kph) {
//            should_exit = true; // Vehicle speed ≤10 kph
//        } 
				else if (psSys->brake_pressed) {
            should_exit = true; // Brake pressed
        } else if (psSys->fault_flag) {
            should_exit = true; // System fault
        } else if (!iDCU_HDA_Enable) {
            should_exit = true; // Function disabled
        }

        if (should_exit) 
        {
            enHDA_State = HDA_STATE_EXITING;
            enHDA_BrakeLevel = ABS_LEVEL_CLOSE;
        } 
        else 
        {
            // Update ABS level
            enHDA_BrakeLevel = prvCalculateBrakeLevel(psSys->slope_angle);
        }
        return; 
    }

    /* IDLE state: Check if entry conditions are met */
    if (enHDA_State == HDA_STATE_IDLE) 
    {
        // Check HDA function entry conditions
        if (!iDCU_HDA_Enable) {
            return; // Function disabled
        }
        if (psSys->fault_flag) {
            return; // Fault present
        }
        if (psSys->brake_pressed) {
            return; // Brake pressed
        }
        if (psSys->throttle_pedal_perc > HDA_MAX_THROTTLE_TO_ENTER_perc) {
            return; // Throttle >10%
        }
        // if (psSys->wheel_speed < HDA_MIN_SPEED_TO_ENTER_kph) {
        //     return; // Vehicle speed <10 kph
        // }
        if (psSys->slope_angle <= HDA_MIN_SLOPE_TO_ENTER_deg) {
            return; // Slope ≤8°
        }

        // Enable ABS function
        enHDA_State = HDA_STATE_ACTIVE;
        enHDA_BrakeLevel = prvCalculateBrakeLevel(psSys->slope_angle);
        return;
    }

    enHDA_State = HDA_STATE_IDLE;
}

// Calculate brake level based on slope
static iDCU_HDA_BrakeLevel prvCalculateBrakeLevel(float fSlopeDeg)
{
    if (fSlopeDeg >= HDA_SLOPE_ANGLE_30) 
    {
        return ABS_LEVEL_3;
    } 
    else if (fSlopeDeg >= HDA_SLOPE_ANGLE_20) 
    {
        return ABS_LEVEL_2;
    } 
    else if (fSlopeDeg > HDA_SLOPE_ANGLE_10) 
    {
        return ABS_LEVEL_1;
    } 
    else 
    {
        return ABS_LEVEL_CLOSE;
    }
}

// Get current state
iDCU_HDA_State_e iDCU_HDA_GetState(void) 
{
    return enHDA_State;
}

// Check if HDA is active
bool iDCU_HDA_IsActive(void) 
{
    return (enHDA_State == HDA_STATE_ACTIVE);
}

// Get current brake level
iDCU_HDA_BrakeLevel iDCU_HDA_GetBrakeLevel(void)  
{
    return enHDA_BrakeLevel;
}
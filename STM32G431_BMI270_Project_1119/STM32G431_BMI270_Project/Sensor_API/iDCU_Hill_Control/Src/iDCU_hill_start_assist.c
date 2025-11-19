
/**
 * @file idcu_hill_start_assist.c
 * @brief Hill Start Assist (HAC) function implementation
 * @details
 *   - When entering this function, adjust throttle based on slope angle.
 */

#include "idcu_hill_start_assist.h"
#include "idcu_sensor_data.h"  

 iDCU_HSA_State_e HSA_state = HSA_STATE_IDLE;  //static  				
static uint32_t HSA_start_time_ms = 0U;                 			
static volatile uint16_t HSA_override_throttle_0_1000 = 0U;        

iDCU_SystemStatus_s iDCU_SystemStatus;

/**
 * @brief Calculate required driving force (0~1000) based on current slope angle
 * @param slope_deg: Current slope angle in degrees
 * @return Output throttle value (0~1000)
 * @note  Mapping relationship:
 *        - 10° → 80
 *        - 30° → 280
 *        - Linear interpolation in between
 */
static uint16_t prv_calculate_throttle_0_1000(float slope_deg) 
{
    float slope_value = (slope_deg < 0) ? -slope_deg : slope_deg;

    // Return minimum throttle if slope ≤ minimum threshold
    if (slope_deg >= HSA_MIN_SLOPE) {
        return HSA_MIN_THROTTLE_0_1000;
    }
    // Return maximum throttle if slope ≥ maximum threshold
    if (slope_deg <= HSA_MAX_SLOPE_DEG) {
        return HSA_MAX_THROTTLE_0_1000;
    }

    // Map slope to throttle signal
    // ratio = (slope - min) / (max - min)
    float ratio = (slope_value - HSA_MIN_SLOPE) / ( HSA_MIN_SLOPE - HSA_MAX_SLOPE_DEG );
    
    // throttle = min + ratio * (max - min)
    float throttle_f = (float)HSA_MIN_THROTTLE_0_1000 + 
                       ratio * ratio_adjust * ((float)HSA_MAX_THROTTLE_0_1000 - (float)HSA_MIN_THROTTLE_0_1000);
    
    // Round to uint type
    uint16_t throttle = (uint16_t)(throttle_f + 0.5f);
    
    // Maximum speed judgment
    if (throttle > 1000U)   throttle = 1000U;

    return throttle;
}

/**
 * @brief Initialize HSA module
 * @note  Clear state, ensure no initial output
 */
void iDCU_HSA_Init(void) 
{
    HSA_state = HSA_STATE_IDLE;
    HSA_start_time_ms = 0U;
    HSA_override_throttle_0_1000 = 0U;
}

/** 
 * @brief Brake release event handler
 * @note  Called when brake changes from "pressed" to "released"
 */
void iDCU_HSA_OnBrakeReleaseEvent(void) 
{
    const iDCU_SystemStatus_s* ps = &iDCU_SystemStatus;

    // Condition 1: HSA function enabled and no system fault
    if (!iDCU_HSA_Enable || ps->fault_flag) {
        return;
    }

    // Condition 2: Slope ≥ 10°
    if (ps->slope_angle > HSA_MIN_SLOPE) { 
        return;
    }

    // Condition 3: Vehicle speed ≤ 5 kph
    if (ps->wheel_speed > HSA_MAX_SPEED_KPH) {
        return;
    }

    // Condition 4: Current CAN throttle signal ≤ 5%
    if (ps->throttle_speed > HSA_CAN_THROTTLE_ENTRY_THRESHOLD) {
        return;
    }

    // All conditions met: Start HSA
    HSA_override_throttle_0_1000 = prv_calculate_throttle_0_1000(ps->slope_angle);
    HSA_state = HSA_STATE_ACTIVE;
    HSA_start_time_ms = HAL_GetTick();
}

/**
 * @brief HSA main loop function
 */
void idcu_hsa_interface(void) 
{
    if (HSA_state != HSA_STATE_ACTIVE) {
        return;
    }

    const iDCU_SystemStatus_s* ps = &iDCU_SystemStatus;
    uint32_t elapsed_ms = HAL_GetTick() - HSA_start_time_ms;
    volatile bool should_exit = false;

    // Check any exit condition
    if (ps->throttle_speed >= HSA_CAN_THROTTLE_EXIT_THRESHOLD) {
        should_exit = true; // User throttle ≥ 20% (200/1000)
    } else if (elapsed_ms >= HSA_MAX_HOLD_TIME) {
        should_exit = true; // Timeout (3 seconds)
    } else if (ps->slope_angle > HSA_MIN_SLOPE) {
        should_exit = true; // Slope becomes gentle
    } else if (ps->brake_pressed) {
        should_exit = true; // Brake pressed again
    } else if (ps->fault_flag || !iDCU_HSA_Enable) {
        should_exit = true; // Fault or disabled
    } else if (ps->wheel_speed >= HSA_MAX_SPEED_KPH) {
        should_exit = true; // Vehicle has started moving
    }

    if (should_exit) 
    {
        HSA_state = HSA_STATE_EXITING;
        HSA_override_throttle_0_1000 = 0U;
        should_exit = false;
    }
}

void check_HSA_status(void) 
{
    static bool last_brake_state = true;

    bool current_brake = iDCU_SystemStatus.brake_pressed;

    // Detect "brake release" edge: from 1 → 0
    if (last_brake_state && !current_brake)
    {
        iDCU_HSA_OnBrakeReleaseEvent();
    }

    last_brake_state = current_brake;
    idcu_hsa_interface();
}

/* External Query Interface */
// Get HSA state
iDCU_HSA_State_e iDCU_HSA_GetState(void) 
{
    return HSA_state;
}

bool iDCU_HSA_IsActive(void) 
{
    return (HSA_state == HSA_STATE_ACTIVE);
}

// Get HSA override throttle speed
uint16_t iDCU_HSA_GetOverrideThrottle_0_1000(void) 
{
    return HSA_override_throttle_0_1000;
}
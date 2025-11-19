
/**
 * @file idcu_vehicle_start_opt.c
 * @brief IDCU Vehicle Start output function implementation
 * @details
 *   - 
 */

#include "idcu_vehicle_start_opt.h"
#include "idcu_system_types.h"

#include "idcu_hill_start_assist.h"


bool iDCU_VSO_Enable_Flag = true;
static uint16_t s_last_raw_throttle = 0;
static uint16_t s_current_output_throttle = 0;
static uint8_t  Start_Prepare_flag = 0;

/**
 * @brief Initialize VSO module
 */
void iDCU_VSO_Init(void) 
{
    s_last_raw_throttle = 0;
    s_current_output_throttle = 0;
    iDCU_SystemStatus.fault_flag = false;   // true: fault present
}

/**
 * @brief Get the processed throttle_speed output value after VSO processing
 * @param 
 * @return uint16_t Processed throttle_speed (0~1000)
 */
uint16_t iDCU_VSO_GetOutputThrottle(void) 
{
    const iDCU_SystemStatus_s* ps = &iDCU_SystemStatus;

    // Get throttle speed from CAN data
    uint16_t current_raw = ps->throttle_speed;

    // Default output is raw value
    uint16_t output_throttle = current_raw;

    // Determine if VSO feature should be enabled
    bool should_apply_vso = 
        iDCU_VSO_Enable_Flag &&          // Feature enabled
        !ps->fault_flag;                 // No system fault

    // If configured to only take effect in GEAR_1/GEAR_2
    if (VSO_GEAR_1_OR_2_ONLY) 
    {
        should_apply_vso = should_apply_vso &&
            (ps->current_gear == GEAR_1 || ps->current_gear == GEAR_2); // (1)
    }

    // Execute VSO logic
    if (should_apply_vso) 
    {
        // Calculate throttle_speed variation (current - last)
        int16_t delta = (int16_t)current_raw - (int16_t)s_last_raw_throttle;

        if(current_raw <= 2)
        {
            Start_Prepare_flag = 1;
        }

        if(Start_Prepare_flag == 1 && current_raw >= 5 && ps->current_gear == GEAR_1)
        {
            current_raw = START_SPEED_GEAR1;
            s_last_raw_throttle = current_raw;
            Start_Prepare_flag = 0;
        }
        else if(Start_Prepare_flag == 1 && current_raw >= 5 && ps->current_gear == GEAR_2)
        {
            current_raw = START_SPEED_GEAR2;
            s_last_raw_throttle = current_raw;
            Start_Prepare_flag = 0;
        }

        // Acceleration process judgment
        if (ps->current_gear == GEAR_1 &&
            (delta > (int16_t)VSO_MAX_SPEED_VARIATION_GEAR1) &&
            (current_raw > START_SPEED_GEAR1))
        {
            output_throttle = s_last_raw_throttle + VSO_MAX_SPEED_VARIATION_GEAR1;
        } 
        else if (ps->current_gear == GEAR_2 &&
            (delta > (int16_t)VSO_MAX_SPEED_VARIATION_GEAR2) &&
            (current_raw > START_SPEED_GEAR2))
        {
            output_throttle = s_last_raw_throttle + VSO_MAX_SPEED_VARIATION_GEAR2;
        }
        else 
        {
            // Normal case: use raw speed
            output_throttle = current_raw;
        }
        // Max speed is input speed
        if (output_throttle > current_raw) 
        {
            output_throttle = current_raw;
        }

    }

    // Update internal state
    s_last_raw_throttle = output_throttle;

    // s_current_output_throttle = output_throttle;

    return output_throttle;
}
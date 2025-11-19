// iDCU_SystemTypes.h
/**
 * @file    iDCU_SystemTypes.h
 * @brief   Common data types, enumerations and global enable flags for the project
 */

#ifndef __IDCU_SYSTEM_TYPES_H__
#define __IDCU_SYSTEM_TYPES_H__

#include "stm32g4xx_hal.h"

#include <stdint.h>  
#include <stdbool.h> 

/* System timestamp type definition */
typedef uint32_t TickType_t;

/* The current gear of the vehicle */
typedef enum {
    GEAR_INVALID = 0,  // Drive assist gear 
    GEAR_1       = 1,  // First gear
    GEAR_2       = 2,  // Second gear
    GEAR_3       = 3,  // Third gear
} iDCU_GearLevel_t;

extern bool iDCU_HSA_Enable;  
extern bool iDCU_HDA_Enable; 
extern bool iDCU_VSO_Enable;  
extern bool iDCU_FSP_Enable;

#endif /* __IDCU_SYSTEM_TYPES_H__ */
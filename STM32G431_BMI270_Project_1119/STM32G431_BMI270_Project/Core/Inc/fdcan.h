/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */

// FDCAN initialization function
void FDCAN_Init(void);
// Send a CAN frame
HAL_StatusTypeDef CAN_send_message(uint32_t can_id, uint8_t *data, uint8_t len);

extern void CAN_Receive_Callback(uint32_t can_id, uint8_t *data, uint8_t len);

// FDCAN Communication Structure
typedef struct {
    volatile uint8_t data_buffer[8];
    uint8_t data_length;         // Actual data length (0-8)
    
    // CAN Identifier
    uint32_t can_id;             // CAN ID (11-bit or 29-bit)
    bool is_extended_id;         // true: Extended ID (29-bit), false: Standard ID (11-bit)
    
    volatile bool tx_complete;            // Transmission complete flag
    bool tx_error;               // Transmission error flag

    volatile bool rx_complete;            // Reception complete flag
    bool rx_error;               // Reception error flag
    
    // Timestamps
    uint32_t last_tx_time;       // Last transmission timestamp
    uint32_t last_rx_time;       // Last reception timestamp
    
} CAN_Comm_Struct_t;

extern CAN_Comm_Struct_t fdcan_comm;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */


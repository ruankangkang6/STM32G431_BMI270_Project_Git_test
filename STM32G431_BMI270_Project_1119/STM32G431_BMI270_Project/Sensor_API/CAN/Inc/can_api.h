
#ifndef __CAN_API_H__
#define __CAN_API_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "fdcan.h"
#include "usart.h"

#include "stm32g4xx_hal_fdcan.h"



void CAN_Receive_Callback(uint32_t can_id, uint8_t *data, uint8_t len);

int can_send_message_to_hmi();
void hmi_update_message_process();
void uart_rxdata_correction();



// // CAN通信结构体
// typedef struct {
//     uint8_t data_buffer[8];
//     uint8_t data_length;         // 实际数据长度 (0-8)
    
//     // CAN标识符
//     uint32_t can_id;             // CAN ID (11位或29位)
//     bool is_extended_id;         // true=扩展帧(29位), false=标准帧(11位)
    
//     // 发送相关标记
//     bool tx_complete;            // 发送完成标记
//     bool tx_error;               // 接收错误标记

//     // 接收相关标记
//     bool rx_ready;               // 接收完成标记
//     bool rx_error;               // 接收错误标记
    
//     // 时间戳
//     uint32_t last_tx_time;       // 最后发送时间戳
//     uint32_t last_rx_time;       // 最后接收时间戳
    
// } CAN_Comm_Struct_t;


#ifdef __cplusplus
}
#endif

#endif /* __CAN_API_H__ */


#ifndef __USART_DRIVER_H__
#define __USART_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif


// #include "main.h"
#include "stm32g4xx_hal.h"
#include "spi.h"
#include "gpio.h"

#include "usart.h"

#include <string.h>


// UART Communication Structure
typedef struct {
    // Transmit Buffer (length 21)
    uint8_t tx_buff[20];
    uint16_t tx_length;          // Actual length of data to be transmitted
    bool tx_complete;            // Transmission complete flag

    // Receive Buffer (length 14)
    uint8_t rx_buff[14];
    uint16_t rx_length;          // Actual length of received data
    bool rx_complete;            // Reception complete flag
    bool rx_error;               // Reception error flag

    // Timeout Control
    uint32_t last_rx_time;       // Timestamp of last received data
    uint32_t timeout_ms;         // Receive timeout duration (milliseconds)

} UART_Comm_Struct_t;

uint8_t calc_xor_checksum(const uint8_t *data, uint32_t length);
void preparer_data_uart_Tx(const uint8_t *sent_buff,  uint8_t length);

void controller_buff_init();

#ifdef __cplusplus
}
#endif

#endif 


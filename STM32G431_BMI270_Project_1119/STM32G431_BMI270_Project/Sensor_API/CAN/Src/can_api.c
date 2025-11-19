#include "fdcan.h"
#include "can_api.h" 
#include "usart.h"

#include <stdint.h>

#include "idcu_hill_start_assist.h"

// // FDcan struct
uint8_t  uart_rx_data_mod[14] = {0};

// Example implementation of user callback
void CAN_Receive_Callback(uint32_t can_id, uint8_t *data, uint8_t len)
{    
    // preparer_data_uart_Tx(data, len);
}

extern UART_Comm_Struct_t idcu_uart_comm;

extern iDCU_HSA_State_e HSA_state;


uint32_t CAN_send_count; 
/**
 * @brief  Receives a CAN message and updates the screen display accordingly.
 * 
 * @return 0 on success, -1 on failure.
 */
int can_send_message_to_hmi()       //Can send messages to HMI
{
    uint8_t temp;
    uint8_t brake_signal;

    uint8_t data_buffer[8] = {0};
    uint8_t data_buffer_speed[8] = {0};
    uint8_t data_buffer_power[8] = {0};
    uint32_t can_id;
    uint32_t can_id_speed = 0x0CFE01EF;//Id of speed
    uint32_t can_id_power = 0x18FF02F4;//Id of power

    uart_rxdata_correction();
    temp = uart_rx_data_mod[4];     // Brake state
    if(temp & 0x20)
    {
        // Brake 
        iDCU_SystemStatus.brake_pressed = true;     // 1
    }
    else
    {
        // No Brake 
        iDCU_SystemStatus.brake_pressed = false;    // 0
    }

    if(idcu_uart_comm.tx_buff[18] == 0x20 || HSA_state == HSA_STATE_ACTIVE) 
         data_buffer_speed[0] = 0xC0;
    else 
         data_buffer_speed[0] = 0x80;

    data_buffer_speed[1] = uart_rx_data_mod[12];    //High byte speed
    data_buffer_speed[2] = uart_rx_data_mod[11];    //Low byte speed
    
    data_buffer_power[1] = uart_rx_data_mod[10];    //power

    // Send multiple messages to the screen
    CAN_send_message(can_id_speed, data_buffer_speed, sizeof(data_buffer_speed));    // Message_1 speed
    CAN_send_message(can_id_power, data_buffer_power, sizeof(data_buffer_power));    // Message_2 power
    //CAN_send_message(can_id, data_buffer, sizeof(data_buffer));    // Message_3
    CAN_send_count++;

    return 0;
}


void hmi_update_message_process()
{
    can_send_message_to_hmi();
}

void uart_rxdata_correction()
{ 
    uint8_t  fir_indexes;  
    int      found = 0;   
    
    for(fir_indexes = 0; fir_indexes < sizeof(idcu_uart_comm.rx_buff) - 2; fir_indexes++)
    {
        if(idcu_uart_comm.rx_buff[fir_indexes] == 0x02 && 
           idcu_uart_comm.rx_buff[fir_indexes+1] == 0x0E && 
           idcu_uart_comm.rx_buff[fir_indexes+2] == 0x01)
        {
            found = 1;
            break; 
        }
    }

    if(found == 1)  
    {
        for (int i = 0; i < 15; i++) 
        {
            int source_index = (fir_indexes + i) % 14;
            uart_rx_data_mod[i] = idcu_uart_comm.rx_buff[source_index];
        }

         memcpy(idcu_uart_comm.rx_buff, uart_rx_data_mod, sizeof(uart_rx_data_mod));
    }

}
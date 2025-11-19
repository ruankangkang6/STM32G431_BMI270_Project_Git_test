
#include "usart_driver.h"


// UART structure
UART_Comm_Struct_t idcu_uart_comm;

uint8_t calc_xor_checksum(const uint8_t *data, uint32_t length)
{
    if (data == NULL || length == 0)
        return 0;

    uint8_t xor_sum = 0;
    for (uint32_t i = 0; i < length; i++)
    {
        xor_sum ^= data[i];  
    }
    return xor_sum;
}


void preparer_data_uart_Tx(const uint8_t *sent_buff,  uint8_t length)
{
    // Copy CAN information to uart buffer
	idcu_uart_comm.tx_buff[4] =  sent_buff[1] * 5;  //5、10、15 corresponding to the first, second and third levels
	idcu_uart_comm.tx_buff[16] = sent_buff[7];  //High byte speed
    idcu_uart_comm.tx_buff[17] = sent_buff[6];  //Low byte speed
	
    // idcu_uart_comm.tx_buff[19] = calc_xor_checksum(idcu_uart_comm.tx_buff, 19);		  
    idcu_uart_comm.tx_complete = true;
}


void controller_buff_init()
{
    // UART send buff init.
	idcu_uart_comm.tx_buff[0] = 0x01;   //Address
	idcu_uart_comm.tx_buff[1] = 0x14;   //Length
	idcu_uart_comm.tx_buff[2] = 0x01;   //Command
	idcu_uart_comm.tx_buff[3] = 0x01;   //Type of drive
	idcu_uart_comm.tx_buff[4] = 0x05;   //Driving gear   and  Single or dual drive
	idcu_uart_comm.tx_buff[5] = 0x80;   //Controller settings 1
	idcu_uart_comm.tx_buff[6] = 0x50;   //Speed measurement magnetic steel count
	idcu_uart_comm.tx_buff[7] = 0x00;   //Wheel Diameter
	idcu_uart_comm.tx_buff[8] = 0x3c;   //Wheel Diameter
	idcu_uart_comm.tx_buff[9] = 0x03;   //Enhance sensitivity
	idcu_uart_comm.tx_buff[10] = 0x30;  //Electronic braking force
	idcu_uart_comm.tx_buff[11] = 0x06;  //Speed measurement magnetic steel count
	idcu_uart_comm.tx_buff[12] = 0x03;  //speed limit values

    idcu_uart_comm.tx_buff[13] = 0x00;  //
	idcu_uart_comm.tx_buff[14] = 0x00;  //
	idcu_uart_comm.tx_buff[15] = 0x00;  //
	idcu_uart_comm.tx_buff[16] = 0x00;  // speed
	idcu_uart_comm.tx_buff[17] = 0x00;  // speed
	idcu_uart_comm.tx_buff[18] = 0x00;  // bi5 start brake
	idcu_uart_comm.tx_buff[19] = 0x00;	// crc  

}


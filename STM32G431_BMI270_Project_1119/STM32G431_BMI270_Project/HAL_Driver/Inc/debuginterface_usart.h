#ifndef DEBUGINTERFACE_USART_H
#define DEBUGINTERFACE_USART_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
	
#define CMD_BUFF_LENGTH 64
#define RSPS_BUFF_LENGTH 64

#define PERIOD_DATA_BUFF_LENGTH 256

#define DBG_I_UART 					USART1 
#define DBG_I_DMA      			DMA1 
#define DBG_I_DMA_LLRXCH  	LL_DMA_CHANNEL_3 
#define DBG_I_DMA_LLTXCH  	LL_DMA_CHANNEL_4 


extern uint8_t MC_DbgI_CMDData_Buff[CMD_BUFF_LENGTH];
extern uint8_t MC_DbgI_RespData_Buff[RSPS_BUFF_LENGTH];
extern uint8_t MC_DbgI_PeriodData_Buff[RSPS_BUFF_LENGTH + PERIOD_DATA_BUFF_LENGTH];


void MC_DebugInterface_HardwareInit(void);
void MC_DebugInterface_IRQHandler(void);

#endif /* DEBUGINTERFACE_USART_H */

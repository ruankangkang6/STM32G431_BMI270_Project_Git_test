
#include "debuginterface_usart.h"												

uint8_t MC_DbgI_CMDData_Buff[CMD_BUFF_LENGTH];
uint8_t MC_DbgI_RespData_Buff[RSPS_BUFF_LENGTH];
uint8_t MC_DbgI_PeriodData_Buff[RSPS_BUFF_LENGTH + PERIOD_DATA_BUFF_LENGTH];

static uint8_t Dbg_Parsed_CMD_Buff[CMD_BUFF_LENGTH];

//static uint32_t LastCMDBuffIdx = 0;

void MC_DebugInterface_HardwareInit(void)
{
	LL_DMA_SetPeriphAddress(DBG_I_DMA, DBG_I_DMA_LLRXCH, LL_USART_DMA_GetRegAddr(DBG_I_UART,LL_USART_DMA_REG_DATA_RECEIVE));
  LL_DMA_SetMemoryAddress(DBG_I_DMA, DBG_I_DMA_LLRXCH, (uint32_t)MC_DbgI_CMDData_Buff);
  LL_DMA_SetDataLength(DBG_I_DMA, DBG_I_DMA_LLRXCH, ARRAY_LEN(MC_DbgI_CMDData_Buff));
	
//	LastCMDBuffIdx = ARRAY_LEN(MC_DbgI_CMDData_Buff);
	
	LL_DMA_SetPeriphAddress(DBG_I_DMA, DBG_I_DMA_LLTXCH, LL_USART_DMA_GetRegAddr(DBG_I_UART, LL_USART_DMA_REG_DATA_TRANSMIT));
  
	LL_USART_EnableDMAReq_TX(DBG_I_UART);  
	LL_USART_EnableDMAReq_RX(DBG_I_UART);  
	
	LL_DMA_EnableChannel(DBG_I_DMA, DBG_I_DMA_LLRXCH);
	
	LL_USART_EnableIT_IDLE(DBG_I_UART);
}

void MC_DebugInterface_IRQHandler(void)
{
	static uint32_t LastCMDBuffIdx = ARRAY_LEN(MC_DbgI_CMDData_Buff);
	uint32_t flags;
//  uint32_t activeIdleFlag;
//  uint32_t isEnabledIdleFlag;
	uint32_t tempLength=0;
	uint8_t* source_buff_idx;
	uint8_t* target_buff_idx;
	
  if (0U == LL_USART_IsActiveFlag_IDLE(DBG_I_UART))
  {
    /* Nothing to do */
  }
  else
  {
    LL_USART_ClearFlag_IDLE(DBG_I_UART);
		if(LastCMDBuffIdx == LL_DMA_GetDataLength(DBG_I_DMA,DBG_I_DMA_LLRXCH))
		{
			/* Nothing to do */
		}
		else if(LastCMDBuffIdx > LL_DMA_GetDataLength(DBG_I_DMA,DBG_I_DMA_LLRXCH))
		{
			memset(Dbg_Parsed_CMD_Buff, 0, CMD_BUFF_LENGTH);
			source_buff_idx = MC_DbgI_CMDData_Buff + (CMD_BUFF_LENGTH - LastCMDBuffIdx);
			tempLength = LastCMDBuffIdx - LL_DMA_GetDataLength(DBG_I_DMA,DBG_I_DMA_LLRXCH);
			target_buff_idx = Dbg_Parsed_CMD_Buff;
			memcpy(target_buff_idx, source_buff_idx, tempLength);
		}
		else if(LastCMDBuffIdx < LL_DMA_GetDataLength(DBG_I_DMA,DBG_I_DMA_LLRXCH))
		{
			memset(Dbg_Parsed_CMD_Buff, 0, CMD_BUFF_LENGTH);
			source_buff_idx = MC_DbgI_CMDData_Buff + (CMD_BUFF_LENGTH - LastCMDBuffIdx);
			tempLength = LastCMDBuffIdx;
			target_buff_idx = Dbg_Parsed_CMD_Buff;
			memcpy(target_buff_idx, source_buff_idx, tempLength);
			target_buff_idx = Dbg_Parsed_CMD_Buff + tempLength;
			tempLength = CMD_BUFF_LENGTH - LL_DMA_GetDataLength(DBG_I_DMA,DBG_I_DMA_LLRXCH);
			if((tempLength > 0) && (tempLength < CMD_BUFF_LENGTH))
			{
				source_buff_idx = MC_DbgI_CMDData_Buff;
				memcpy(target_buff_idx, source_buff_idx, tempLength);
			}
		}
		
		LastCMDBuffIdx = LL_DMA_GetDataLength(DBG_I_DMA,DBG_I_DMA_LLRXCH);
  }

//  if (0U == LL_USART_IsActiveFlag_TC(USARTA))
//  {
//    /* Nothing to do */
//  }
//  else
//  {
//    /* Disable the DMA channel to prepare the next chunck of data*/
//    LL_DMA_DisableChannel(DMA_TX_A, DMACH_TX_A);
//    LL_USART_ClearFlag_TC(USARTA);
//    /* Data Sent by UART*/
//    /* Need to free the buffer, and to check pending transfer*/
//    ASPEP_HWDataTransmittedIT(&aspepOverUartA);
//  }
  uint32_t oreFlag;
  uint32_t feFlag;
  uint32_t neFlag;
  uint32_t errorMask;

  oreFlag = LL_USART_IsActiveFlag_ORE(DBG_I_UART);
  feFlag = LL_USART_IsActiveFlag_FE(DBG_I_UART);
  neFlag = LL_USART_IsActiveFlag_NE(DBG_I_UART);
  errorMask = LL_USART_IsEnabledIT_ERROR(DBG_I_UART);

  flags = ((oreFlag | feFlag | neFlag) & errorMask);
  if (0U == flags)
  {
    /* Nothing to do */
  }
  else
  { /* Stopping the debugger will generate an OverRun error*/
    WRITE_REG(DBG_I_UART->ICR, USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF);

    /* We disable ERROR interrupt to avoid to trig one Overrun IT per additional byte recevied*/
    LL_USART_DisableIT_ERROR(DBG_I_UART);
    LL_USART_EnableIT_IDLE(DBG_I_UART);
  }

//  activeIdleFlag = LL_USART_IsActiveFlag_IDLE(USARTA);
//  isEnabledIdleFlag = LL_USART_IsEnabledIT_IDLE(USARTA);

//  flags = activeIdleFlag & isEnabledIdleFlag;
//  if (0U == flags)
//  {
//    /* Nothing to do */
//  }
//  else
//  { /* Stopping the debugger will generate an OverRun error*/
//    LL_USART_DisableIT_IDLE(USARTA);
//    /* Once the complete unexpected data are received, we enable back the error IT*/
//    LL_USART_EnableIT_ERROR(USARTA);
//    /* To be sure we fetch the potential pending data*/
//    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
//    LL_USART_DisableDMAReq_RX(USARTA);
//    (void)LL_USART_ReceiveData8(USARTA);
//    LL_USART_EnableDMAReq_RX(USARTA);
//    /* Clear pending DMA TC to process only new received packet */
//    LL_DMA_ClearFlag_TC(DMA_RX_A, DMACH_RX_A);
//    ASPEP_HWReset(&aspepOverUartA);
//  }

  /* USER CODE BEGIN USART2_IRQHandler 1 */

  /* USER CODE END USART2_IRQHandler 1 */
}

void MC_DebugInterface_CMDHandler(const uint8_t *cmd)
{
	
}

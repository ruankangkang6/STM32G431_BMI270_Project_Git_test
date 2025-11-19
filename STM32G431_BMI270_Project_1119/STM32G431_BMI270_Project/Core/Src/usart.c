/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include <string.h>
#include "angle_calc.h"
#include "idcu_api.h"

uint8_t dma_tx_flag = 0;
extern float acc_gyro_buffer[6]; 

uint8_t uart_rx_buffer[RX_BUFFER_LEN];  
uint16_t rx_data_size = 0;             

uint8_t uart_send_buff_test[32] = {0};  

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;//1500000
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1500000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel1;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel2;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA1     ------> USART2_DE
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA1     ------> USART2_DE
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void UART1_Receive_Start(void)
{
    // Start DMA reception (circular mode)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, RX_BUFFER_LEN);

    // Clear the IDLE flag and enable the IDLE interrupt
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

}
uint32_t uart_rx_count;
extern UART_Comm_Struct_t idcu_uart_comm;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // if (huart->Instance == USART1)
    if (huart == &huart1) 
    {
        // Size: Actual received data length
        rx_data_size = Size;

        // uart_send_buff_test
        // memcpy(uart_send_buff_test, uart_rx_buffer, 4);
        // memcpy(idcu_uart_comm.rx_buff, uart_rx_buffer, sizeof(uart_rx_buffer));
        // memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));

        // Start DMA idle receive.
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, RX_BUFFER_LEN);

        memcpy(idcu_uart_comm.rx_buff, uart_rx_buffer, sizeof(uart_rx_buffer));

        // Receive Data completed
        idcu_uart_comm.rx_complete = true;
        uart_rx_count++;
    }
}


extern float acc_gyro_buffer[6];
uint8_t uart_tx_buffer[64];

#if 0

/******************************************************
* Brief     : DMA send function tset
* Parameter : 
*           *pData: Send buff
* Return    : None.
*******************************************************/
void User_uartdma_Transmit(const char *pData)
{ 
 HAL_UART_Transmit_DMA(&huart2,(uint8_t *)pData, strlen(pData));
}


int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

int fgetc(FILE *f)
{
	uint8_t ch;
	HAL_UART_Receive(&huart2, (uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) 
    {
        dma_tx_flag = 0;
    }
}

#else

/******************************************************
* Brief     : DMA send function tset
* Parameter : 
*           *pData: Send buff
* Return    : None.
*******************************************************/
void User_uartdma_Transmit(const char *pData)
{ 
 //HAL_UART_Transmit_DMA(&huart1,(uint8_t *)pData, strlen(pData));
}


int fputc(int ch, FILE *f)
{
	//HAL_UART_Transmit(&huart1, (uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}

int fgetc(FILE *f)
{
	uint8_t ch;
	HAL_UART_Receive(&huart1, (uint8_t *)&ch,1,HAL_MAX_DELAY);
	return ch;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) 
    {
        dma_tx_flag = 0;
    }
}

#endif

extern uint8_t uart1_send_buff[20];

void motor_control_buff_init()
{
	uart1_send_buff[0] = 0x01;
	uart1_send_buff[1] = 0x14;
	uart1_send_buff[2] = 0x01;
	uart1_send_buff[3] = 0x01;
	uart1_send_buff[4] = 0x00;
	uart1_send_buff[5] = 0x80;
	uart1_send_buff[6] = 0x28;
	uart1_send_buff[7] = 0x01;
	uart1_send_buff[8] = 0x31;
	uart1_send_buff[9] = 0x03;
	uart1_send_buff[10] = 0x03;
	uart1_send_buff[11] = 0x06;
	uart1_send_buff[12] = 0x03;
	
	uart1_send_buff[13] = 0x00;
	uart1_send_buff[14] = 0x00;
	uart1_send_buff[15] = 0x00;
	uart1_send_buff[16] = 0x00;     // speed
	uart1_send_buff[17] = 0x00;     // speed
	uart1_send_buff[18] = 0x05;
	uart1_send_buff[19] = 0x00;		// end

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // 打印错误信息，方便调试。实例地址可以帮助我们判断是哪个UART出了问题。
    printf("UART Error! Instance: %p\r\n", huart->Instance);

    // 重新启动UART的DMA接收。这是解决问题的核心！
    // 无论UART之前处于何种错误状态，我们都尝试通过重新启动DMA接收来使其恢复。
    if(huart->Instance == USART1)
    {
        
			 // 重新配置DMA
			 HAL_DMA_DeInit(&hdma_usart1_rx);
			 HAL_DMA_Init(&hdma_usart1_rx);
			  // 关键步骤：重新初始化UART1的DMA接收，使其脱离错误状态并重新监听数据。
       HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_rx_buffer, RX_BUFFER_LEN);
    }


    // 注意：在更复杂的错误处理场景中，可能还需要：
    // 1. 清除特定的错误标志位（如果HAL库没有自动处理）。
    // 2. 执行HAL_UART_AbortReceive() 或 HAL_UART_DeInit() / HAL_UART_Init() 进行更彻底的复位。
    //    但对于热插拔导致的常见错误，重新启动DMA接收通常就足够了。
}
/* USER CODE END 1 */


/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "fsm.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN UART */
UART_HandleTypeDef huart;
DMA_HandleTypeDef hdma_usart_rx;
DMA_HandleTypeDef hdma_usart_tx;

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart.Instance = USART6;
  huart.Init.BaudRate = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}
/**
  * @brief UART MSP Initialization
  * This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART6)
  {
    /* USER CODE BEGIN USART6_MspInit 0 */

    /* USER CODE END USART6_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart_rx.Instance = DMA2_Stream2;
    hdma_usart_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmarx,hdma_usart_rx);

    /* USART6_TX Init */
    hdma_usart_tx.Instance = DMA2_Stream7;
    hdma_usart_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart_tx.Init.Mode = DMA_NORMAL;
    hdma_usart_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(huart,hdmatx,hdma_usart_tx);

    /* USART6 interrupt Init */

		
    HAL_NVIC_SetPriority(USART6_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
		

    /* USER CODE BEGIN USART6_MspInit 1 */

    /* USER CODE END USART6_MspInit 1 */

  }

}

/**
  * @brief UART MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART6)
  {
    /* USER CODE BEGIN USART6_MspDeInit 0 */

    /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART6 interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
    /* USER CODE BEGIN USART6_MspDeInit 1 */

    /* USER CODE END USART6_MspDeInit 1 */
  }

}

//
int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart, (uint8_t*)&ch, 1, 0xffff);
  return 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huartx)
{
  /* Prevent unused argument(s) compilation warning */
  char c = Serial2RxBuffer[0];
  update_fsm(&state, c);
	HAL_UART_Receive_IT(huartx, (uint8_t *)Serial2RxBuffer, 1);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

/* USER CODE END UART */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




/* Includes ------------------------------------------------------------------*/
#include "uart.h"
#include "fsm.h"
/* USER CODE BEGIN Includes */



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



#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_uart.h"



/* USER CODE END Includes */

//typedef struct {
//    uint8_t header;       
//    uint8_t len;          

//    float Angle;
//    float Vel;
//    float Pos;
//    float Current[3];
//	
//} usart_tx_t;

//#define RX_SIZE					20

//#define HEADER 					0xAA
//#define ID_HEADER				0
//#define ID_MODE 				3
//#define ID_VEL 					(ID_MODE + 1)
//#define ID_POS 					(ID_VEL + 4)
//#define ID_TOR 					(ID_POS + 4)

//#define ID_CRC 					(RX_SIZE - 4)


//typedef struct {
//	uint8_t rx_buff[RX_SIZE];

//}usart_rx_t;

extern uint8_t Serial2RxBuffer[1];
/* USER CODE BEGIN UART */
extern UART_HandleTypeDef huart;
extern DMA_HandleTypeDef hdma_usart_rx;
extern DMA_HandleTypeDef hdma_usart_tx;

void MX_USART6_UART_Init(void);



/* USER CODE END UART */


#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



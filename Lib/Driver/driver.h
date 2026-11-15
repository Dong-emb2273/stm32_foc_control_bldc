#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "main.h"

// ==========================================================
// 				CONFIG TIM & PWM ( EDIT HERE )
// ==========================================================
#ifndef __TIM_CONFIG__
#define __TIM_CONFIG__

#define TIM_PWM					TIM1
#define TIM_COUNT_US			TIM10
#define TIM_IRQ_MS				TIM3

#define TIM_PWM_CHA_PIN			GPIO_PIN_8
#define TIM_PWM_CHA_PORT		GPIOA
#define TIM_PWM_CHB_PIN			GPIO_PIN_9
#define TIM_PWM_CHB_PORT		GPIOA
#define TIM_PWM_CHC_PIN			GPIO_PIN_10
#define TIM_PWM_CHC_PORT		GPIOA

#define TIM_PWM_CHAN_PIN		GPIO_PIN_13
#define TIM_PWM_CHAN_PORT		GPIOB
#define TIM_PWM_CHBN_PIN		GPIO_PIN_14
#define TIM_PWM_CHBN_PORT		GPIOB
#define TIM_PWM_CHCN_PIN		GPIO_PIN_15
#define TIM_PWM_CHCN_PORT		GPIOB
#define TIM_PWM_GPIO_AF			GPIO_AF1_TIM1

#define TIM_PWM_CHA             TIM_PWM->CCR1
#define TIM_PWM_CHB             TIM_PWM->CCR2
#define TIM_PWM_CHC             TIM_PWM->CCR3

                         
void TIM_PWM_DMA_CONFIG();

void TIM_COUNT_US_CONFIG(void);

void TIM_COUNTER_ENABLE(TIM_TypeDef *Timx);

void TIM_IQR_MS_CONGFIG();

#endif



// ==========================================================
//				CONFIG ADC & DMA ( EDIT HERE )
// ==========================================================
#ifndef __ADC_CONFIG__
#define __ADC_CONFIG__



typedef struct {
    volatile uint16_t ia;
    volatile uint16_t ib;
    volatile uint16_t ic;
    volatile uint16_t vbus;
} adc_raw_t;

/* ADC logical signals */
#define ADC_IA      0U
#define ADC_IB      1U
#define ADC_IC      2U
#define ADC_VBUS    3U
#define ADC_TEMP    4U

/* ADC hardware mapping */

/* Phase A current */
#define ADC_IA_INSTANCE                     ADC1
#define ADC_IA_CHANNEL                      0U
#define ADC_IA_GPIO_PORT                    GPIOA
#define ADC_IA_GPIO_PIN                     GPIO_PIN_0
#define ADC_IA_DATA                         ADC1->JDR1

/* Phase B current */               
#define ADC_IB_INSTANCE                     ADC2
#define ADC_IB_CHANNEL                      1U
#define ADC_IB_GPIO_PORT                    GPIOA
#define ADC_IB_GPIO_PIN                     GPIO_PIN_1
#define ADC_IB_DATA                         ADC2->JDR1

/* Phase C current */               
#define ADC_IC_INSTANCE                     ADC3
#define ADC_IC_CHANNEL                      2U
#define ADC_IC_GPIO_PORT                    GPIOA
#define ADC_IC_GPIO_PIN                     GPIO_PIN_2
#define ADC_IC_DATA                         ADC3->JDR1


#define ADC_VBUS_PIN						GPIO_PIN_3
#define ADC_VBUS_PORT						GPIOA
#define ADC_VBUS_DATA                       ADC1->JDR2




#ifndef LINK_ADC_H
#define LINK_ADC_H

#endif

void ADC_DMA_CONFIG(void);

#endif



// ==========================================================
// 					CONFIG SPI & DMA ( EDIT HERE )
// ==========================================================
#ifndef __SPI_CONFIG__
#define __SPI_CONFIG__

extern SPI_HandleTypeDef                hspi_in;
extern SPI_HandleTypeDef                hspi_ex;
extern DMA_HandleTypeDef                hdma_spi_in_rx;
extern DMA_HandleTypeDef                hdma_spi_in_tx;
extern DMA_HandleTypeDef                hdma_spi_ex_rx;
extern DMA_HandleTypeDef                hdma_spi_ex_tx;

#define SPI_IN_SCK_PIN                  GPIO_PIN_3
#define SPI_IN_SCK_PORT                 GPIOB       
#define SPI_IN_MISO_PIN                 GPIO_PIN_4
#define SPI_IN_MISO_PORT                GPIOB
#define SPI_IN_MOSI_PIN                 GPIO_PIN_5
#define SPI_IN_MOSI_PORT                GPIOB
#define SPI_IN_GPIO_AF                  GPIO_AF5_SPI1

#define SPI_IN_X						SPI1
#define SPI_IN_SCALE					SPI_BAUDRATEPRESCALER_16
#define SPI_IN_IRQ						SPI1_IRQn
#define SPI_IN_IRQ_FUNCTION				SPI1_IRQHandler
#define SPI_IN_IRQ_PRIO					10, 0

#define SPI_IN_DMA_RX_STREAM			DMA2_Stream0
#define SPI_IN_DMA_RX_CHANNEL			DMA_CHANNEL_3
#define SPI_IN_DMA_RX_IRQ				DMA2_Stream0_IRQn
#define SPI_IN_DMA_RX_IRQ_FUNCTION	    DMA2_Stream0_IRQHandler
#define SPI_IN_DMA_RX_IRQ_PRIO			7, 1

#define SPI_IN_DMA_TX_STREAM			DMA2_Stream3
#define SPI_IN_DMA_TX_CHANNEL			DMA_CHANNEL_3
#define SPI_IN_DMA_TX_IRQ				DMA2_Stream3_IRQn
#define SPI_IN_DMA_TX_IRQ_FUNCTION	    DMA2_Stream3_IRQHandler
#define SPI_IN_DMA_TX_IRQ_PRIO			7, 2


#define SPI_EX_SCK_PIN					GPIO_PIN_10
#define SPI_EX_SCK_PORT					GPIOB                         
#define SPI_EX_MISO_PIN					GPIO_PIN_2 
#define SPI_EX_MISO_PORT				GPIOC
#define SPI_EX_MOSI_PIN					GPIO_PIN_3
#define SPI_EX_MOSI_PORT				GPIOC
#define SPI_EX_GPIO_AF					GPIO_AF5_SPI2

#define SPI_EX_X						SPI2
#define SPI_EX_SCALE					SPI_BAUDRATEPRESCALER_16
#define SPI_EX_IRQ						SPI2_IRQn
#define SPI_EX_IRQ_FUNCTION				SPI2_IRQHandler
#define SPI_EX_IRQ_PRIO					10, 1

#define SPI_EX_DMA_RX_STREAM			DMA1_Stream3   
#define SPI_EX_DMA_RX_CHANNEL			DMA_CHANNEL_0
#define SPI_EX_DMA_RX_IRQ				DMA1_Stream3_IRQn
#define SPI_EX_DMA_RX_IRQ_FUNCTION	    DMA1_Stream3_IRQHandler
#define SPI_EX_DMA_RX_IRQ_PRIO			7, 3

#define SPI_EX_DMA_TX_STREAM			DMA1_Stream4
#define SPI_EX_DMA_TX_CHANNEL			DMA_CHANNEL_0
#define SPI_EX_DMA_TX_IRQ				DMA1_Stream4_IRQn
#define SPI_EX_DMA_TX_IRQ_FUNCTION	    DMA1_Stream4_IRQHandler
#define SPI_EX_DMA_TX_IRQ_PRIO			7, 4

void SPI1_DMA_CONFIG(void);
void SPI2_DMA_CONFIG(void);

#endif



// ==========================================================
//				CONFIG USART COM & DMA ( EDIT HERE )
// ==========================================================
#ifndef __UART_CONFIG__
#define __UART_CONFIG__

 extern UART_HandleTypeDef huart;
 extern DMA_HandleTypeDef hdma_usart_rx;
 extern DMA_HandleTypeDef hdma_usart_tx;

#define USART_TX_PIN					GPIO_PIN_6
#define USART_TX_PORT					GPIOC
#define USART_RX_PIN					GPIO_PIN_7
#define USART_RX_PORT					GPIOC
#define USART_GPIO_AF					GPIO_AF8_USART6

#define USART_X							USART6
#define USART_BAUDRATE					115200UL
#define USART_IRQ						USART6_IRQn
#define USART_IRQ_FUNCTION				USART6_IRQHandler
#define USART_IRQ_PRIO					14, 0

#define USART_DMA_RX_STREAM				DMA2_Stream2
#define USART_DMA_RX_CHANNEL			DMA_CHANNEL_5
#define USART_DMA_RX_IRQ				DMA2_Stream2_IRQn
#define USART_DMA_RX_IRQ_FUNCTION		DMA2_Stream2_IRQHandler
#define USART_DMA_RX_IRQ_PRIO			7, 10

#define USART_DMA_TX_STREAM				DMA2_Stream7
#define USART_DMA_TX_CHANNEL			DMA_CHANNEL_5
#define USART_DMA_TX_IRQ				DMA2_Stream7_IRQn
#define USART_DMA_TX_IRQ_FUNCTION		DMA2_Stream7_IRQHandler
#define USART_DMA_TX_IRQ_PRIO			7, 9    

#endif



// ==========================================================
//				CONFIG CAN ( EDIT HERE )
// ==========================================================
#ifndef __CAN_CONFIG__
#define __CAN_CONFIG__

extern CAN_HandleTypeDef hcan;


#define CAN_TX_PIN                      GPIO_PIN_12
#define CAN_TX_PORT                     GPIOA
#define CAN_RX_PIN                      GPIO_PIN_11
#define CAN_RX_PORT				    	GPIOA
#define CAN_GPIO_AF						GPIO_AF9_CAN1

#define CAN_X                           CAN1
#define CAN_FILTER_FIFO                 CAN_FILTER_FIFO0
#define CAN_X_IRQ                       CAN1_RX0_IRQn
#define CAN_X_IRQ_FUNCTION              CAN1_RX0_IRQHandler
#define CAN_X_IRQ_PRIO                  8, 0

#endif



// ==========================================================
//				CONFIG I2C & DMA ( EDIT HERE )
// ==========================================================
#ifndef __I2C_CONFIG__
#define __I2C_CONFIG__

#define I2C_SDA_PIN					GPIO_PIN_9
#define I2C_SDA_PORT				GPIOC	
#define I2C_SCL_PIN					GPIO_PIN_8
#define I2C_SCL_PORT				GPIOC
#define I2C_GPIO_AF					GPIO_AF4_I2C1

#define I2C_X						I2C1
#define I2C_BAUDRATE				100000UL
#define I2C_IRQ						I2C1_EV_IRQn
#define I2C_IRQ_FUNCTION			I2C1_EV_IRQHandler
#define I2C_IRQ_PRIO				5, 0

#define I2C_DMA_TX_STREAM
#define I2C_DMA_TX_CHANNEL
#define I2C_DMA_TX_IRQ
#define I2C_DMA_TX_IRQ_FUNCTION	
#define I2C_DMA_TX_IRQ_PRIO



#define I2C_DMA_RX_STREAM
#define I2C_DMA_RX_CHANNEL
#define I2C_DMA_RX_IRQ
#define I2C_DMA_RX_IRQ_FUNCTION
#define I2C_DMA_RX_IRQ_PRIO

#endif



// ==========================================================
//				CONFIG CRC & DMA ( EDIT HERE )
// ==========================================================
#ifndef __CRC_CONFIG__
#define __CRC_CONFIG__

#define CRC_DMA_STREAM                  DMA2_Stream5
#define CRC_DMA_CHANNEL                 0             
#define CRC_DMA_TCIF_FLAG               DMA_HISR_TCIF5     
#define CRC_DMA_CTCIF_FLAG              DMA_HIFCR_CTCIF5 
#define CRC_DMA_STAT_REG                DMA2->HISR         
#define CRC_DMA_CLR_REG                 DMA2->HIFCR 

void CRC_DMA_CONFIG(void);

uint32_t CRC_Calculate_DMA(uint8_t *pBuffer, uint32_t length);

#endif



// ==========================================================
//				CONFIG GPIO ( EDIT HERE )
// ==========================================================
#ifndef __GPIO_CONFIG__
#define __GPIO_CONFIG__


#define GPIO_DRV8323_CS_PIN					GPIO_PIN_12
#define GPIO_DRV8323_CS_PORT				GPIOC
#define GPIO_DRV8323_ENABLE_PIN				GPIO_PIN_11
#define GPIO_DRV8323_ENABLE_PORT			GPIOC
#define GPIO_DRV8323_FAULT_PIN				GPIO_PIN_15
#define GPIO_DRV8323_FAULT_PORT				GPIOA

#define GPIO_AS5048A_IN_CS_PIN			    GPIO_PIN_9
#define GPIO_AS5048A_IN_CS_PORT				GPIOB
#define GPIO_AS5048A_EX_CS_PIN				    GPIO_PIN_11
#define GPIO_AS5048A_EX_CS_PORT				    GPIOB

#define GPIO_LED_GREEN_PIN					GPIO_PIN_13
#define GPIO_LED_GREEN_PORT					GPIOC
#define GPIO_LED_RED_PIN					GPIO_PIN_14
#define GPIO_LED_RED_PORT					GPIOC

void GPIO_INIT (void);

#endif



// ==========================================================
//				CONFIG LINK DRIVER ( EDIT HERE )
// ==========================================================
#ifndef __LINK_DRIVER__
#define __LINK_DRIVER__

#define FLASH_SECTOR_ADDR  ((uint32_t)0x080E0000)
#define FLASH_SECTOR_NUM   FLASH_SECTOR_11


#endif



#endif
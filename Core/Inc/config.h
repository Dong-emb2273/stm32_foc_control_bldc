#ifndef __CONFIG_H__
#define __CONFIG_H__
#include "main.h"


void setdefaul();

// ==========================================================
//						 CONFIG SPI & DMA ( EDIT HERE )
// ==========================================================

#define SPI1_SCK_PIN  					GPIO_PIN_3
#define SPI1_SCK_PORT						GPIOB       
#define SPI1_MISO_PIN 					GPIO_PIN_4
#define SPI1_MISO_PORT					GPIOB
#define SPI1_MOSI_PIN 					GPIO_PIN_5
#define SPI1_MOSI_PORT      		GPIOB

#define SPI2_SCK_PIN   					GPIO_PIN_10
#define SPI2_SCK_PORT						GPIOB                         
#define SPI2_MISO_PIN 					GPIO_PIN_2 
#define SPI2_MISO_PORT					GPIOC
#define SPI2_MOSI_PIN 					GPIO_PIN_3
#define SPI2_MOSI_PORT					GPIOC

#define SPI1_DMA_RX_STREAM      DMA2_Stream0
#define SPI1_DMA_CHANNEL    		0x03                  
#define SPI1_DMA_RX_TCIF_FLAG   DMA_LISR_TCIF0     
#define SPI1_DMA_RX_CTCIF_FLAG  DMA_LIFCR_CTCIF0 
#define SPI1_DMA_RX_STAT_REG    DMA2->LISR         
#define SPI1_DMA_RX_CLR_REG     DMA2->LIFCR        
					
#define SPI1_DMA_TX_STREAM      DMA2_Stream3                  
#define SPI1_DMA_TX_TCIF_FLAG   DMA_LISR_TCIF3
#define SPI1_DMA_TX_CTCIF_FLAG  DMA_LIFCR_CTCIF3          
#define SPI1_DMA_TX_STAT_REG    DMA2->LISR          
#define SPI1_DMA_TX_CLR_REG     DMA2->LIFCR  

#define SPI2_DMA_RX_STREAM      DMA1_Stream3
#define SPI2_DMA_CHANNEL		    0x00                  
#define SPI2_DMA_RX_TCIF_FLAG   DMA_LISR_TCIF3     
#define SPI2_DMA_RX_CTCIF_FLAG  DMA_LIFCR_CTCIF3   
#define SPI2_DMA_RX_STAT_REG    DMA1->LISR         
#define SPI2_DMA_RX_CLR_REG     DMA1->LIFCR        

#define SPI2_DMA_TX_STREAM      DMA1_Stream4                  
#define SPI2_DMA_TX_TCIF_FLAG   DMA_HISR_TCIF4      
#define SPI2_DMA_TX_CTCIF_FLAG  DMA_HIFCR_CTCIF4    
#define SPI2_DMA_TX_STAT_REG    DMA1->HISR          
#define SPI2_DMA_TX_CLR_REG     DMA1->HIFCR 


void SPI1_DMA_CONFIG(void);
void SPI2_DMA_CONFIG(void);

void SPI1_DMA_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t len);
void SPI2_DMA_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t len);



//========================= END ===============================







// Dat dinh nghia nay o dau file .c hoac trong file .h chung
typedef enum {
    I2C_DMA_OK       = 0, // Thanh cong
    I2C_DMA_TIMEOUT  = 1, // Loi do qua thoi gian cho
    I2C_DMA_NACK     = 2  // Loi do Slave khong phan hoi (NACK)
} I2C_DmaStatus_t;



// ==========================================================
// CONFIG TIM & PWM ( EDIT HERE )
// ==========================================================

#define TIM_PWM									TIM1
#define TIM_COUNT_US						TIM10
#define TIM_IRQ_MS							TIM3

#define TIM_PWM_CHA_PIN  				GPIO_PIN_8
#define TIM_PWM_CHA_PORT				GPIOA
#define TIM_PWM_CHB_PIN 				GPIO_PIN_9
#define TIM_PWM_CHB_PORT				GPIOA
#define TIM_PWM_CHC_PIN 				GPIO_PIN_10
#define TIM_PWM_CHC_PORT 				GPIOA

#define TIM_PWM_CHAN_PIN  			GPIO_PIN_13
#define TIM_PWM_CHAN_PORT				GPIOB
#define TIM_PWM_CHBN_PIN 				GPIO_PIN_14
#define TIM_PWM_CHBN_PORT				GPIOB
#define TIM_PWM_CHCN_PIN 				GPIO_PIN_15
#define TIM_PWM_CHCN_PORT 			GPIOB

                         
void TIM1_PWM_DMA_CONFIG();

void TIM_COUNT_US_CONFIG(void);

void TIM_COUNTER_ENABLE(TIM_TypeDef *Timx);

void TIM_IQR_MS_CONGFIG();

// ========================= END ============================



// ==========================================================
// 							CONFIG USART COM & DMA ( EDIT HERE )
// ==========================================================
				
#define USART_TX_PIN  						GPIO_PIN_6
#define USART_TX_PORT							GPIOC
#define USART_RX_PIN  						GPIO_PIN_7
#define USART_RX_PORT							GPIOC
				
#define APB2_CLOCK_FREQ         	84000000UL 

#define USART_X										USART6
#define USART_BAUDRATE          	256000UL
#define USART_BRR_VAL           	((APB2_CLOCK_FREQ + (USART_BAUDRATE / 2)) / USART_BAUDRATE)

#define USART_DMA_RX_STREAM     	DMA2_Stream2
#define USART_DMA_CHANNEL		    	0x05                     
#define USART_DMA_RX_CTCIF_FLAG  	DMA_LIFCR_CTCIF2         
#define USART_DMA_RX_CLR_REG     	DMA2->LIFCR        

#define USART_DMA_TX_STREAM      	DMA2_Stream7                       
#define USART_DMA_TX_CTCIF_FLAG  	DMA_HIFCR_CTCIF7            
#define USART_DMA_TX_CLR_REG     	DMA2->HIFCR 
     
typedef struct {
	uint8_t header;       
	uint8_t len;          
	uint8_t reserved[2];  
	
	float Angle;
	float Vel;
	float Pos;
	float Current[3];
	
	uint32_t Crc;
	
} usart_tx_t;

#define RX_SIZE					20

#define HEADER 					0xAA
#define ID_HEADER				0
#define ID_MODE 				3
#define ID_VEL 					(ID_MODE + 1)
#define ID_POS 					(ID_VEL + 4)
#define ID_TOR 					(ID_POS + 4)

#define ID_CRC 					(RX_SIZE - 4)


typedef struct {
	uint8_t rx_buff[RX_SIZE];

}usart_rx_t;

void USART6_DMA_CONFIG();

void USART_COM_RX_DMA(uint8_t *rx_buff, uint16_t length);

void USART_COM_TX_DMA(uint8_t *tx_buff, uint16_t length);

void USART_Get_Angle(usart_tx_t *usart_com, float angle);

void USART_Get_Current(usart_tx_t *usart_com, float ia, float ib, float ic);

void USART_Get_Vel(usart_tx_t *usart_com, float vel);

void USART_Get_Pos(usart_tx_t *usart_com, float pos);

void USART_TRANSMIT(usart_tx_t *usart_com);



// ========================= END ============================


// ==========================================================
// 							CONFIG CRC & DMA ( EDIT HERE )
// ==========================================================
	



#define CRC_DMA_STREAM      DMA2_Stream5
#define CRC_DMA_CHANNEL    	0             
#define CRC_DMA_TCIF_FLAG   DMA_HISR_TCIF5     
#define CRC_DMA_CTCIF_FLAG  DMA_HIFCR_CTCIF5 
#define CRC_DMA_STAT_REG    DMA2->HISR         
#define CRC_DMA_CLR_REG     DMA2->HIFCR 




void CRC_DMA_CONFIG(void);

uint32_t CRC_Calculate_DMA(uint8_t *pBuffer, uint32_t length);

// ========================= END ============================





void ADC1_DMA_CONFIG3(void);
void ADC1_DMA_CONFIG2(void);
void ADC1_DMA_CONFIG(void);



void I2C1_DMA_CONFIG();

void I2C2_DMA_CONFIG();

I2C_DmaStatus_t I2C1_DMA_Read_From_Reg(uint8_t slave_address, uint8_t reg_address, uint8_t* pRxData, uint16_t len, uint32_t timeout_ms);

I2C_DmaStatus_t TMP117_Read_Temp(float* temperature_celsius);

#endif
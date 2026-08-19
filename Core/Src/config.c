#include "config.h"




GPIO_InitTypeDef GPIO_Init = {0};

void setdefaul(){
	GPIO_Init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_Init.Mode = GPIO_MODE_ANALOG;
	GPIO_Init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_Init);
	
	GPIO_Init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_Init.Mode = GPIO_MODE_ANALOG;
	GPIO_Init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_Init);
	
	GPIO_Init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13| GPIO_PIN_14 | GPIO_PIN_15 ;
	GPIO_Init.Mode = GPIO_MODE_ANALOG;
	GPIO_Init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_Init);
	
	GPIO_Init.Pin = GPIO_PIN_2;
	GPIO_Init.Mode = GPIO_MODE_ANALOG;
	GPIO_Init.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_Init);
	
	
	
}


// ==========================================================
// CONFIG SPI & DMA ( EDIT HERE )
// ==========================================================

void SPI1_DMA_CONFIG(void) {
	// ---- BUOC 1: CAU HINH CLOCK (Da sua loi) ----
	// Bat clock cho GPIO B, C, D va DMA2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_DMA2EN;
	// Bat clock cho SPI1 (tren bus APB2)
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	// ---- BUOC 2: CAU HINH GPIO ----

	GPIO_Init.Pin = SPI1_SCK_PIN;
	GPIO_Init.Mode = GPIO_MODE_AF_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(SPI1_SCK_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = SPI1_MISO_PIN;
	HAL_GPIO_Init(SPI1_MISO_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = SPI1_MOSI_PIN;
	HAL_GPIO_Init(SPI1_MOSI_PORT, &GPIO_Init);
	
	

	// ---- BUOC 3: CAU HINH DMA  ----
	SPI1_DMA_TX_STREAM->CR = 0; 
	while(SPI1_DMA_TX_STREAM->CR & DMA_SxCR_EN);
	SPI1_DMA_TX_STREAM->PAR = (uint32_t)&(SPI1->DR);
	SPI1_DMA_TX_STREAM->CR |= (SPI1_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) |
														(1 << DMA_SxCR_DIR_Pos) | // Huong: Memory-to-Peripheral
														(1 << DMA_SxCR_MINC_Pos);   // Tang dia chi bo nho
	
	// SPI1_RX: DMA2, Stream 0, Channel 3
	SPI1_DMA_RX_STREAM->CR = 0; // Reset
	while(SPI1_DMA_RX_STREAM->CR & DMA_SxCR_EN);
	SPI1_DMA_RX_STREAM->PAR = (uint32_t)&(SPI1->DR);
	SPI1_DMA_RX_STREAM->CR |= (SPI1_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) |
														(0 << DMA_SxCR_DIR_Pos) | // Huong: Peripheral-to-Memory
														(1 << DMA_SxCR_MINC_Pos);  // Tang dia chi bo nho

	// ---- BUOC 4: CAU HINH SPI1 ----
	// Tat SPI de cau hinh
	SPI1->CR1 &= ~SPI_CR1_SPE;
	// Cau hinh thanh ghi CR1
	SPI1->CR1 = 0; // Reset
	// Dat che do SPI Mode 1 (CPOL=0, CPHA=1)
	SPI1->CR1 |= SPI_CR1_CPHA; 
	// Dat toc do (PCLK2 / 128)
	SPI1->CR1 |= (0b110 << SPI_CR1_BR_Pos); 
	// Dat che do Master
	SPI1->CR1 |= SPI_CR1_MSTR;
	// Quan ly chan CS bang phan mem
	SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
	// Cau hinh thanh ghi CR2
	SPI1->CR2 = 0; // Reset
	// Bat yeu cau DMA cho ca truyen (TX) va nhan (RX)
	SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	// Bat SPI len
	SPI1->CR1 |= SPI_CR1_SPE;
}

//

void SPI2_DMA_CONFIG(void) {
	// ---- BUOC 1: CAU HINH CLOCK ----
	// Bat clock cho GPIO B, C va DMA1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_DMA1EN;
	// Bat clock cho SPI2 
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	
	// ---- BUOC 2: CAU HINH GPIO ----
	GPIO_Init.Pin = SPI2_SCK_PIN;
	GPIO_Init.Mode = GPIO_MODE_AF_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(SPI2_SCK_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = SPI2_MISO_PIN;
	HAL_GPIO_Init(SPI2_MISO_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = SPI2_MOSI_PIN;
	HAL_GPIO_Init(SPI2_MOSI_PORT, &GPIO_Init);
	

	// ---- BUOC 3: CAU HINH DMA ----
	
	SPI2_DMA_TX_STREAM->CR = 0; 
	while(SPI2_DMA_TX_STREAM->CR & DMA_SxCR_EN);
	SPI2_DMA_TX_STREAM->PAR = (uint32_t)&(SPI2->DR);
	SPI2_DMA_TX_STREAM->CR |= (SPI2_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) | // Kenh 0
														(1 << DMA_SxCR_DIR_Pos) | // Huong: Memory-to-Peripheral
														(1 << DMA_SxCR_MINC_Pos);   // Tang dia chi bo nho
	
	SPI2_DMA_RX_STREAM->CR = 0; 
	while(SPI2_DMA_RX_STREAM->CR & DMA_SxCR_EN);
	SPI2_DMA_RX_STREAM->PAR = (uint32_t)&(SPI2->DR);
	SPI2_DMA_RX_STREAM->CR |= (SPI2_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) | // Kenh 0
														(1 << DMA_SxCR_TCIE_Pos) |
														(1 << DMA_SxCR_MINC_Pos);   // Tang dia chi bo nho
														
	NVIC_EnableIRQ(DMA1_Stream3_IRQn); 
  NVIC_SetPriority(DMA1_Stream3_IRQn, 1);

	// ---- BUOC 4: CAU HINH SPI2 ----
	
	// Tat SPI de cau hinh
	SPI2->CR1 &= ~SPI_CR1_SPE;
	// Cau hinh thanh ghi CR1
	SPI2->CR1 = 0; // Reset
	// Dat che do Master
	SPI2->CR1 |= SPI_CR1_MSTR;
	// Quan ly chan CS bang phan mem
	SPI2->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;

	// PCLK1 tren F405 thuong la 42MHz. 42MHz/2 
	SPI2->CR1 |= (0b011 << SPI_CR1_BR_Pos); 
	// Dat che do SPI Mode 1 (CPOL=0, CPHA=1)
	SPI2->CR1 |= SPI_CR1_CPHA; 
	// Cau hinh thanh ghi CR2
	SPI2->CR2 = 0; // Reset
	// Bat yeu cau DMA cho ca truyen (TX) va nhan (RX)
	SPI2->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;
	// Bat SPI len
	SPI2->CR1 |= SPI_CR1_SPE;

}




//

void SPI1_DMA_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t len) {
  
	SPI1_DMA_RX_STREAM->CR &= ~DMA_SxCR_EN; 
	while(SPI1_DMA_RX_STREAM->CR & DMA_SxCR_EN); 
	
	SPI1_DMA_RX_CLR_REG |= SPI1_DMA_RX_CTCIF_FLAG;
	
	SPI1_DMA_RX_STREAM->M0AR = (uint32_t)pRxData; 
	SPI1_DMA_RX_STREAM->NDTR = len;
	
	SPI1_DMA_TX_STREAM->CR &= ~DMA_SxCR_EN; 
	while(SPI1_DMA_TX_STREAM->CR & DMA_SxCR_EN); 

	SPI1_DMA_TX_CLR_REG |= SPI1_DMA_TX_CTCIF_FLAG;

	SPI1_DMA_TX_STREAM->M0AR = (uint32_t)pTxData; 
	SPI1_DMA_TX_STREAM->NDTR = len;

	SPI1_DMA_RX_STREAM->CR |= DMA_SxCR_EN;
	SPI1_DMA_TX_STREAM->CR |= DMA_SxCR_EN;

	while (!(SPI1_DMA_RX_STAT_REG & SPI1_DMA_RX_TCIF_FLAG));
	while (SPI1->SR & SPI_SR_BSY);
	
	SPI1_DMA_RX_CLR_REG |= SPI1_DMA_RX_CTCIF_FLAG;
	SPI1_DMA_TX_CLR_REG |= SPI1_DMA_TX_CTCIF_FLAG;
}
//

void SPI2_DMA_TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t len) {
  
	SPI2_DMA_RX_STREAM->CR &= ~DMA_SxCR_EN; 
	while(SPI2_DMA_RX_STREAM->CR & DMA_SxCR_EN); 
	
	SPI2_DMA_RX_CLR_REG |= SPI2_DMA_RX_CTCIF_FLAG;
	
	SPI2_DMA_RX_STREAM->M0AR = (uint32_t)pRxData; 
	SPI2_DMA_RX_STREAM->NDTR = len;
	
	SPI2_DMA_TX_STREAM->CR &= ~DMA_SxCR_EN; 
	while(SPI2_DMA_TX_STREAM->CR & DMA_SxCR_EN); 

	SPI2_DMA_TX_CLR_REG |= SPI2_DMA_TX_CTCIF_FLAG;

	SPI2_DMA_TX_STREAM->M0AR = (uint32_t)pTxData; 
	SPI2_DMA_TX_STREAM->NDTR = len;

	SPI2_DMA_RX_STREAM->CR |= DMA_SxCR_EN;
	SPI2_DMA_TX_STREAM->CR |= DMA_SxCR_EN;

//	while (!(SPI2_DMA_RX_STAT_REG & SPI2_DMA_RX_TCIF_FLAG));
//	while (SPI2->SR & SPI_SR_BSY);
//	
//	SPI2_DMA_RX_CLR_REG |= SPI2_DMA_RX_CTCIF_FLAG;
//	SPI2_DMA_TX_CLR_REG |= SPI2_DMA_TX_CTCIF_FLAG;
}
//

// ========================= END ============================


// ==========================================================
// CONFIG TIM & PWM ( EDIT HERE )
// ==========================================================

void TIM1_PWM_DMA_CONFIG(void) {
	// 1. B?t Clock 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; 
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// 2. GPIO Config 

	GPIO_Init.Pin = TIM_PWM_CHA_PIN;
	GPIO_Init.Mode = GPIO_MODE_AF_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(TIM_PWM_CHA_PORT, &GPIO_Init);

	GPIO_Init.Pin = TIM_PWM_CHB_PIN;
	HAL_GPIO_Init(TIM_PWM_CHB_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = TIM_PWM_CHC_PIN;
	HAL_GPIO_Init(TIM_PWM_CHC_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = TIM_PWM_CHAN_PIN;
	HAL_GPIO_Init(TIM_PWM_CHAN_PORT, &GPIO_Init);

	GPIO_Init.Pin = TIM_PWM_CHBN_PIN;
	HAL_GPIO_Init(TIM_PWM_CHBN_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = TIM_PWM_CHCN_PIN;
	HAL_GPIO_Init(TIM_PWM_CHCN_PORT, &GPIO_Init);
	
	TIM1->CR1 &= ~TIM_CR1_CEN;
	

	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 |= TIM_CR1_CMS_0; 
	
	// Auto-reload preload enable (ARPE)
	TIM1->CR1 |= TIM_CR1_ARPE;

	TIM1->RCR = 0;

	TIM1->CCMR1 = 0;
	TIM1->CCMR2 = 0;
// CH4 ở chế độ PWM Mode 1 (110)
	TIM1->CCMR2 &= ~TIM_CCMR2_OC4M;
	TIM1->CCMR2 |= (6U << TIM_CCMR2_OC4M_Pos);
	// CH1: PWM Mode 1 (110), Preload Enable (OC1PE)
	TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
	// CH2: PWM Mode 1 (110), Preload Enable (OC2PE)
	TIM1->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
	// CH3: PWM Mode 1 (110), Preload Enable (OC3PE)
	TIM1->CCMR2 |= (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR2_OC3PE; 

	// 5. Channel Enable (CCER)
	// CH1, CH1N, CH2, CH2N, CH3, CH3N, CH4

	TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
								 TIM_CCER_CC2E | TIM_CCER_CC2NE | 
								 TIM_CCER_CC3E | TIM_CCER_CC3NE |
								 TIM_CCER_CC4E);

	// 6. Break & Deadtime (BDTR)
	TIM1->BDTR = 0; // Reset
	
	TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR;

	// DEADTIME (DTG):
	TIM1->BDTR &= ~TIM_BDTR_DTG; 
	TIM1->BDTR |= (5 << TIM_BDTR_DTG_Pos);
	// MOE: Main Output Enable 
	TIM1->BDTR |= TIM_BDTR_MOE;
	// 7. Interrupt & DMA Request
	TIM1->EGR |= TIM_EGR_UG;
    
	// 2. PHẢI XÓA CỜ NGẮT NGAY LẬP TỨC
	// Để tránh việc CPU nhảy vào ngắt ngay khi cho phép NVIC
	TIM1->SR = 0; // Xóa sạch mọi cờ báo trạng thái (đặc biệt là TIM_SR_UIF)

	// 3. Bây giờ mới được phép bật ngắt trong Timer
//	TIM1->DIER |= TIM_DIER_UIE;
// 9. Enable Counter
//	TIM1->CR1 |= TIM_CR1_CEN;
	
	// NVIC Config
//	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 4);
//	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}
//

void TIM_COUNTER_ENABLE(TIM_TypeDef *Timx){
	Timx->CR1 |= TIM_CR1_CEN;
	
}
//

void TIM_COUNT_US_CONFIG(void){
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;

	TIM10->PSC = 167;
	TIM10->ARR = 0xFFFF;
	TIM10->EGR |= TIM_EGR_UG;

}
//

void TIM_IQR_MS_CONGFIG(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->PSC = 839;  
	TIM3->ARR = 999;  
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->SR &= ~TIM_SR_UIF;
	TIM3->DIER |= TIM_DIER_UIE; 
	NVIC_SetPriority(TIM3_IRQn, 6);
	NVIC_EnableIRQ(TIM3_IRQn);     
}
//

// ========================= END ============================


void ADC1_DMA_CONFIG3(void) {
	// 1. Bật Clock cho cả 3 ADC
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;

	// 2. Cấu hình GPIO (PA0, PA1, PA2) -> Analog
	GPIOA->MODER |= 0x3FFF; 
	GPIOA->PUPDR &= ~0x3FFF;

	// 3. Cấu hình chung cho khối ADC (ADC_CCR)
	// ADCPRE = 01 (Chia 4 -> 21MHz)
	// MULTI = 10101 (Triple Injected simultaneous mode only) - Xem RM0090 Table 62
	// DMA = 0 (Không cần DMA cho Injected)
	ADC->CCR = (1 << ADC_CCR_ADCPRE_Pos) | 
						 (0x15 << ADC_CCR_MULTI_Pos); 

	// --- CẤU HÌNH ADC1 (Master - Pha A) ---
	ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE; // Bật ngắt JEOC ở Master
	ADC1->CR2 = 0; // Trigger cài bên dưới
	
	// Trigger từ TIM1_CC4, Rising Edge
	ADC1->CR2 |= (0 << ADC_CR2_JEXTSEL_Pos) | (2 << ADC_CR2_JEXTEN_Pos); 
	
	// Chuỗi Injected: 3 chuyển đổi (JL = 2)
	ADC1->JSQR = (2 << ADC_JSQR_JL_Pos)   |  // JL = 2 (Đọc 3 kênh)
	             (0 << ADC_JSQR_JSQ2_Pos) |  // Lượt 1: Channel 0 (PA0)
	             (4 << ADC_JSQR_JSQ3_Pos) |  // Lượt 2: Channel 4 (PA4)
	             (6 << ADC_JSQR_JSQ4_Pos);   // Lượt 3: Channel 6 (PA6)
	
	// Sample Time: 15 Cycles (010)
	ADC1->SMPR2 = (3 << ADC_SMPR2_SMP0_Pos) | 
	              (3 << ADC_SMPR2_SMP4_Pos) | 
	              (3 << ADC_SMPR2_SMP6_Pos);


	// --- CẤU HÌNH ADC2 (Slave 1 - Pha B) ---
	ADC2->CR1 = ADC_CR1_SCAN; // Không cần bật ngắt ở Slave
	ADC2->CR2 = 0; // Slave không cần cài Trigger (nó nghe theo Master)
	
	// Chuỗi Injected: 3 chuyển đổi (JL = 2)
	ADC2->JSQR = (2 << ADC_JSQR_JL_Pos)   |  // JL = 2 (Đọc 3 kênh)
	             (1 << ADC_JSQR_JSQ2_Pos) |  // Lượt 1: Channel 1 (PA1)
	             (5 << ADC_JSQR_JSQ3_Pos) |  // Lượt 2: Channel 5 (PA5)
	             (5 << ADC_JSQR_JSQ4_Pos);   // Lượt 3: Channel 5 (Lặp lại cho đủ chuỗi)
	
	// Sample Time: 15 Cycles
	ADC2->SMPR2 = (3 << ADC_SMPR2_SMP1_Pos) | 
	              (3 << ADC_SMPR2_SMP5_Pos);


	// --- CẤU HÌNH ADC3 (Slave 2 - Pha C) ---
	ADC3->CR1 = ADC_CR1_SCAN;
	ADC3->CR2 = 0;
	
	// Chuỗi Injected: 3 chuyển đổi (JL = 2)
	ADC3->JSQR = (2 << ADC_JSQR_JL_Pos)   |  // JL = 2 (Đọc 3 kênh)
	             (2 << ADC_JSQR_JSQ2_Pos) |  // Lượt 1: Channel 2 (PA2)
	             (3 << ADC_JSQR_JSQ3_Pos) |  // Lượt 2: Channel 3 (PA3)
	             (3 << ADC_JSQR_JSQ4_Pos);   // Lượt 3: Channel 3 (Lặp lại cho đủ chuỗi)
	
	// Sample Time: 15 Cycles
	ADC3->SMPR2 = (3 << ADC_SMPR2_SMP2_Pos) | 
	              (3 << ADC_SMPR2_SMP3_Pos);

	// ---- BẬT VÀ KÍCH HOAT ----
	NVIC_SetPriority(ADC_IRQn, 6);
	NVIC_EnableIRQ(ADC_IRQn);

	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;
}
//

// ==========================================================
// 							CONFIG USART COM & DMA ( EDIT HERE )
// ==========================================================
		
void USART6_DMA_CONFIG(){
	//============== CLOCK CONFIG ==============
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	
	//============== GPIO PIN CONFIG ===========
	/*Configure GPIO pin */
	GPIO_Init.Pin = USART_TX_PIN;
	GPIO_Init.Mode = GPIO_MODE_AF_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_Init.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(USART_TX_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = USART_RX_PIN;
	GPIO_Init.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(USART_RX_PORT, &GPIO_Init);
	

		// ---- BUOC 3: CAU HINH DMA  ----
	USART_DMA_RX_STREAM->CR = 0; 
	while(USART_DMA_RX_STREAM->CR & DMA_SxCR_EN);
	USART_DMA_RX_STREAM->PAR  = (uint32_t)&(USART_X->DR);
	USART_DMA_RX_STREAM->CR  |= (USART_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
													 |	(0 << DMA_SxCR_DIR_Pos) 
													 | 	(1 << DMA_SxCR_CIRC_Pos)
													 |	(1 << DMA_SxCR_MINC_Pos); 
	
	USART_DMA_TX_STREAM->CR = 0; // Reset
	while(USART_DMA_TX_STREAM->CR & DMA_SxCR_EN);
	USART_DMA_TX_STREAM->PAR  = (uint32_t)&(USART_X->DR);
	USART_DMA_TX_STREAM->CR  |= (USART_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos) 
													 |	(1<< DMA_SxCR_DIR_Pos)  
													 |	(1 << DMA_SxCR_MINC_Pos); 
										
	//============== USART6 CONFIG ================
	USART_X->CR1 |= USART_CR1_UE;  	

	USART_X->BRR |= USART_BRR_VAL;  

  USART_X->CR3 |= USART_CR3_DMAR   
               |  USART_CR3_DMAT;
							 
	USART_X->CR1 |= USART_CR1_TE 
							 |  USART_CR1_IDLEIE
               |  USART_CR1_RE;	
	NVIC_SetPriority(USART6_IRQn, 7);							 
	NVIC_EnableIRQ(USART6_IRQn);
}
//

void USART_COM_RX_DMA(uint8_t *rx_buff, uint16_t length){
  USART_DMA_RX_STREAM->CR &= ~DMA_SxCR_EN;
	while(USART_DMA_RX_STREAM->CR & DMA_SxCR_EN);
	USART_DMA_RX_CLR_REG |= USART_DMA_RX_CTCIF_FLAG;
	
  USART_DMA_RX_STREAM->M0AR = (uint32_t)rx_buff;
	USART_DMA_RX_STREAM->NDTR = length;
  USART_DMA_RX_STREAM->CR |= DMA_SxCR_EN;
}
//

void USART_COM_TX_DMA(uint8_t *tx_buff, uint16_t length){
  USART_DMA_TX_STREAM->CR &= ~DMA_SxCR_EN;
	while(USART_DMA_TX_STREAM->CR & DMA_SxCR_EN);
	USART_DMA_TX_CLR_REG |= USART_DMA_TX_CTCIF_FLAG;
	
  USART_DMA_TX_STREAM->M0AR = (uint32_t)tx_buff;
	USART_DMA_TX_STREAM->NDTR = length;
  USART_DMA_TX_STREAM->CR |= DMA_SxCR_EN;
}
//

void USART_Get_Angle(usart_tx_t *usart_com, float angle){
	usart_com->Angle = angle;
	
}


//

void USART_Get_Current(usart_tx_t *usart_com, float ia, float ib, float ic){
	usart_com->Current[0] = ia;
	usart_com->Current[1] = ib;
	usart_com->Current[2] = ic;
}
//

void USART_Get_Vel(usart_tx_t *usart_com, float vel){
	usart_com->Vel = vel;
}
//

void USART_Get_Pos(usart_tx_t *usart_com, float pos){
	usart_com->Pos = pos;
}
//

void USART_TRANSMIT(usart_tx_t *usart_com){
	
	usart_com->header = 0xAA;
	
	usart_com->reserved[0] = 0;
  usart_com->reserved[1] = 0;
	
	usart_com->len = sizeof(usart_tx_t);
	
	usart_com->Crc = CRC_Calculate_DMA((uint8_t*)usart_com,7);
	
	USART_COM_TX_DMA((uint8_t*)usart_com, usart_com->len);
	
}


//



// ========================= END ============================


// ==========================================================
// 							CONFIG CRC & DMA ( EDIT HERE )
// ==========================================================
	
void CRC_DMA_CONFIG(void){
   
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	CRC->CR |= CRC_CR_RESET;
	
	CRC_DMA_STREAM->CR = 0;
	while(CRC_DMA_STREAM->CR & DMA_SxCR_EN);

	CRC_DMA_STREAM->M0AR = (uint32_t)&(CRC->DR);
	CRC_DMA_STREAM->CR |= (0 << DMA_SxCR_CHSEL_Pos) | // Kênh nào cũng được trong M2M
												(2 << DMA_SxCR_PL_Pos)    | // Priority: High (để tính cho nhanh)
												(2 << DMA_SxCR_MSIZE_Pos) | // Đích (M0AR): 32-bit (Word)
												(2 << DMA_SxCR_PSIZE_Pos) | // Nguồn (PAR): 32-bit (Word)
												(0 << DMA_SxCR_MINC_Pos)  | // Đích (CRC->DR): KHÔNG tăng địa chỉ (MINC=0)
												(1 << DMA_SxCR_PINC_Pos)  | // Nguồn (Buffer): CÓ tăng địa chỉ (PINC=1)
												(2 << DMA_SxCR_DIR_Pos);    // DIR = 10: Memory-to-Memory Mode
												
}
//

uint32_t CRC_Calculate_DMA(uint8_t *pBuffer, uint32_t length){
	CRC->CR |= CRC_CR_RESET;

	CRC_DMA_STREAM->CR &= ~DMA_SxCR_EN;
	while(CRC_DMA_STREAM->CR & DMA_SxCR_EN);

	CRC_DMA_CLR_REG |= CRC_DMA_CTCIF_FLAG;
	CRC_DMA_STREAM->PAR = (uint32_t)pBuffer;
	CRC_DMA_STREAM->NDTR = length;
	CRC_DMA_STREAM->CR |= DMA_SxCR_EN;

	while (!(CRC_DMA_STAT_REG & CRC_DMA_TCIF_FLAG));
	
	return CRC->DR;
}
//


// ========================= END ============================






// ========================= END ============================


// ============= I2C ================
//
void I2C1_DMA_CONFIG(){
	I2C_HandleTypeDef hi2c1;
	
  //============ CLOCK CONFIG =============
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	
	//============ GPIO CONFIG =============
  GPIO_Init.Pin = GPIO_PIN_6 | GPIO_PIN_7; // SCL, SDA
	GPIO_Init.Mode = GPIO_MODE_AF_OD;  
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_Init);
	
	//============ DMA2 STREAM 0 CONFIG =============
	//------------ I2C1 RX ----------------
	DMA1_Stream0->CR  = 0;
	DMA1_Stream0->CR |= (1UL << DMA_SxCR_CHSEL_Pos)   // Chon kenh DMA (Channel select)
									 |  (0UL << DMA_SxCR_PBURST_Pos)  // Che do burst cho Peripheral
									 |  (0UL << DMA_SxCR_CT_Pos)      // Chon buffer hien tai khi double buffer
									 |  (0UL << DMA_SxCR_DBM_Pos)     // Double buffer mode enable
									 |  (0UL << DMA_SxCR_PL_Pos)      // Do uu tien (Priority level)
									 |  (0UL << DMA_SxCR_PINCOS_Pos)  // Peripheral increment offset size
									 |  (0UL << DMA_SxCR_MSIZE_Pos)   // Kich thuoc data Memory (MSIZE)
									 |  (0UL << DMA_SxCR_PSIZE_Pos)   // Kich thuoc data Peripheral (PSIZE)
									 |  (1UL << DMA_SxCR_MINC_Pos)    // Memory increment enable
									 |  (0UL << DMA_SxCR_PINC_Pos)    // Peripheral increment enable
									 |  (0UL << DMA_SxCR_CIRC_Pos)    // Circular mode enable
									 |  (0UL << DMA_SxCR_DIR_Pos)     // Huong truyen (00: P->M, 01: M->P, 10: M->M)
									 |  (0UL << DMA_SxCR_PFCTRL_Pos)  // Chon Peripheral la DMA flow controller
									 |  (0UL << DMA_SxCR_TCIE_Pos)    // Ngat khi truyen xong (Transfer complete interrupt)
									 |  (0UL << DMA_SxCR_HTIE_Pos)    // Ngat khi truyen nua (Half transfer interrupt)
									 |  (0UL << DMA_SxCR_TEIE_Pos)    // Ngat loi truyen (Transfer error interrupt)
									 |  (0UL << DMA_SxCR_DMEIE_Pos)   // Ngat loi FIFO (Direct mode error interrupt)
									 |  (0UL << DMA_SxCR_EN_Pos)      // Enable DMA stream
	                 ;  
	DMA1_Stream0->PAR = (uint32_t)&(I2C1->DR);
	
	//============ I2C1 CONFIG =============
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	hi2c1.Init.OwnAddress1 = 0x00;
	hi2c1.Init.OwnAddress2 = 0;
	HAL_I2C_Init(&hi2c1);
	
	I2C1->CR2 |= I2C_CR2_DMAEN;
	
}
//

void I2C2_DMA_CONFIG(){
	I2C_HandleTypeDef hi2c2;
	
  //============ CLOCK CONFIG =============
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	
	//============ GPIO CONFIG =============
  GPIO_Init.Pin = GPIO_PIN_10 | GPIO_PIN_11; // SCL, SDA
	GPIO_Init.Mode = GPIO_MODE_AF_OD;  
	GPIO_Init.Pull = GPIO_PULLUP;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_Init.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOB, &GPIO_Init);
	
	//============ DMA2 STREAM 0 CONFIG =============
	//------------ I2C1 RX ----------------
	DMA1_Stream2->CR  = 0;
	DMA1_Stream2->CR |= (7UL << DMA_SxCR_CHSEL_Pos)   // Chon kenh DMA (Channel select)
									 |  (0UL << DMA_SxCR_PBURST_Pos)  // Che do burst cho Peripheral
									 |  (0UL << DMA_SxCR_CT_Pos)      // Chon buffer hien tai khi double buffer
									 |  (0UL << DMA_SxCR_DBM_Pos)     // Double buffer mode enable
									 |  (0UL << DMA_SxCR_PL_Pos)      // Do uu tien (Priority level)
									 |  (0UL << DMA_SxCR_PINCOS_Pos)  // Peripheral increment offset size
									 |  (0UL << DMA_SxCR_MSIZE_Pos)   // Kich thuoc data Memory (MSIZE)
									 |  (0UL << DMA_SxCR_PSIZE_Pos)   // Kich thuoc data Peripheral (PSIZE)
									 |  (1UL << DMA_SxCR_MINC_Pos)    // Memory increment enable
									 |  (0UL << DMA_SxCR_PINC_Pos)    // Peripheral increment enable
									 |  (1UL << DMA_SxCR_CIRC_Pos)    // Circular mode enable
									 |  (0UL << DMA_SxCR_DIR_Pos)     // Huong truyen (00: P->M, 01: M->P, 10: M->M)
									 |  (0UL << DMA_SxCR_PFCTRL_Pos)  // Chon Peripheral la DMA flow controller
									 |  (0UL << DMA_SxCR_TCIE_Pos)    // Ngat khi truyen xong (Transfer complete interrupt)
									 |  (0UL << DMA_SxCR_HTIE_Pos)    // Ngat khi truyen nua (Half transfer interrupt)
									 |  (0UL << DMA_SxCR_TEIE_Pos)    // Ngat loi truyen (Transfer error interrupt)
									 |  (0UL << DMA_SxCR_DMEIE_Pos)   // Ngat loi FIFO (Direct mode error interrupt)
									 |  (0UL << DMA_SxCR_EN_Pos)      // Enable DMA stream
	                 ;  
	
	//============ I2C1 CONFIG =============
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	hi2c2.Init.OwnAddress1 = 0x00;
	hi2c2.Init.OwnAddress2 = 0;
	HAL_I2C_Init(&hi2c2);
	
	I2C2->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
	I2C2->CR1 |= I2C_CR1_PE;
	
}
//



//================ SPI =========================



void test111(){

	DMA1_Stream0->CR|= (0UL << DMA_SxCR_CHSEL_Pos)   // Chon kenh DMA (Channel select)
									|  (0UL << DMA_SxCR_PBURST_Pos)  // Che do burst cho Peripheral
									|  (0UL << DMA_SxCR_CT_Pos)      // Chon buffer hien tai khi double buffer
									|  (0UL << DMA_SxCR_DBM_Pos)     // Double buffer mode enable
									|  (0UL << DMA_SxCR_PL_Pos)      // Do uu tien (Priority level)
									|  (0UL << DMA_SxCR_PINCOS_Pos)  // Peripheral increment offset size
									|  (0UL << DMA_SxCR_MSIZE_Pos)   // Kich thuoc data Memory (MSIZE)
									|  (0UL << DMA_SxCR_PSIZE_Pos)   // Kich thuoc data Peripheral (PSIZE)
									|  (0UL << DMA_SxCR_MINC_Pos)    // Memory increment enable
									|  (0UL << DMA_SxCR_PINC_Pos)    // Peripheral increment enable
									|  (0UL << DMA_SxCR_CIRC_Pos)    // Circular mode enable
									|  (0UL << DMA_SxCR_DIR_Pos)     // Huong truyen (00: P->M, 01: M->P, 10: M->M)
									|  (0UL << DMA_SxCR_PFCTRL_Pos)  // Chon Peripheral la DMA flow controller
									|  (0UL << DMA_SxCR_TCIE_Pos)    // Ngat khi truyen xong (Transfer complete interrupt)
									|  (0UL << DMA_SxCR_HTIE_Pos)    // Ngat khi truyen nua (Half transfer interrupt)
									|  (0UL << DMA_SxCR_TEIE_Pos)    // Ngat loi truyen (Transfer error interrupt)
									|  (0UL << DMA_SxCR_DMEIE_Pos)   // Ngat loi FIFO (Direct mode error interrupt)
									|  (0UL << DMA_SxCR_EN_Pos)      // Enable DMA stream
	                ;                     

     

}

//

/**
 * @brief  Doc nhieu byte tu mot thanh ghi cua thiet bi tren I2C1 bang DMA.
 * @note   Ham nay duoc viet rieng cho I2C1 (RX: DMA1 Stream 0, Channel 1).
 * @param  slave_address Dia chi 7-bit cua thiet bi slave.
 * @param  reg_address   Dia chi thanh ghi can doc.
 * @param  pRxData       Con tro toi buffer de luu du lieu doc ve.
 * @param  len           So luong byte can doc.
 * @param  timeout_ms    Thoi gian cho doi toi da (mili-giay).
 * @retval I2C_DmaStatus_t Ket qua cua giao dich.
 */
I2C_DmaStatus_t I2C1_DMA_Read_From_Reg(uint8_t slave_address, uint8_t reg_address, uint8_t* pRxData, uint16_t len, uint32_t timeout_ms){
	uint32_t start_tick;
	
	// 1. Doi bus I2C ranh
	start_tick = HAL_GetTick();
	while (I2C1->SR2 & I2C_SR2_BUSY) {
			if ((HAL_GetTick() - start_tick) > timeout_ms) return I2C_DMA_TIMEOUT;
	}

	// 2. Gui dieu kien START
	I2C1->CR1 |= I2C_CR1_START;
	start_tick = HAL_GetTick();
	while (!(I2C1->SR1 & I2C_SR1_SB)) {
			if ((HAL_GetTick() - start_tick) > timeout_ms) { I2C1->CR1 |= I2C_CR1_STOP; return I2C_DMA_TIMEOUT; }
	}

	// 3. Gui dia chi slave + bit GHI (0)
	I2C1->DR = (slave_address << 1) & ~1;
	start_tick = HAL_GetTick();
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
			if ((HAL_GetTick() - start_tick) > timeout_ms) { I2C1->CR1 |= I2C_CR1_STOP; return I2C_DMA_TIMEOUT; }
			// Kiem tra NACK (loi khong tim thay slave)
			if (I2C1->SR1 & I2C_SR1_AF) {
					I2C1->CR1 |= I2C_CR1_STOP;
					I2C1->SR1 &= ~I2C_SR1_AF;
					return I2C_DMA_NACK;
			}
	}
	(void)I2C1->SR1; (void)I2C1->SR2; // Xoa co ADDR

	// 4. Gui dia chi thanh ghi can doc
	I2C1->DR = reg_address;
	start_tick = HAL_GetTick();
	while (!(I2C1->SR1 & I2C_SR1_TXE)) {
			if ((HAL_GetTick() - start_tick) > timeout_ms) { I2C1->CR1 |= I2C_CR1_STOP; return I2C_DMA_TIMEOUT; }
	}

	// ---- PHAN 2: DOC DU LIEU (Dung DMA) ----
	
	// 5. Gui dieu kien REPEATED START
	I2C1->CR1 |= I2C_CR1_START;
	start_tick = HAL_GetTick();
	while (!(I2C1->SR1 & I2C_SR1_SB)) {
			if ((HAL_GetTick() - start_tick) > timeout_ms) { I2C1->CR1 |= I2C_CR1_STOP; return I2C_DMA_TIMEOUT; }
	}

	// 6. Gui dia chi slave + bit DOC (1)
	I2C1->DR = (slave_address << 1) | 1;
	start_tick = HAL_GetTick();
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
			if ((HAL_GetTick() - start_tick) > timeout_ms) { I2C1->CR1 |= I2C_CR1_STOP; return I2C_DMA_TIMEOUT; }
			if (I2C1->SR1 & I2C_SR1_AF) {
					I2C1->CR1 |= I2C_CR1_STOP;
					I2C1->SR1 &= ~I2C_SR1_AF;
					return I2C_DMA_NACK;
			}
	}

	// 7. Cau hinh DMA (I2C1_RX = DMA1, Stream 0, Channel 1)
	DMA1_Stream0->CR &= ~DMA_SxCR_EN; // Tat luong
	while(DMA1_Stream0->CR & DMA_SxCR_EN);
	
	// Xoa co cua Stream 0 (dung LIFCR)
	DMA1->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0); 
	
	DMA1_Stream0->M0AR = (uint32_t)pRxData; // Dia chi bo nho
	DMA1_Stream0->NDTR = len;               // So luong byte
	
	// Cau hinh I2C de hoat dong voi DMA
	I2C1->CR1 |= I2C_CR1_ACK;  // Bat che do tu dong ACK
	I2C1->CR2 |= I2C_CR2_LAST; // Bat che do NACK o byte cuoi cung
	I2C1->CR2 |= I2C_CR2_DMAEN; // Bat yeu cau DMA
	
	// Bat luong DMA len
	DMA1_Stream0->CR |= DMA_SxCR_EN;

	// 8. Xoa co ADDR de bat dau nhan du lieu vao thanh ghi DR
	(void)I2C1->SR1; (void)I2C1->SR2;

	// 9. Doi DMA hoan thanh (Doi co TCIF0)
	start_tick = HAL_GetTick();
	while (!(DMA1->LISR & DMA_LISR_TCIF0)) {
			if ((HAL_GetTick() - start_tick) > timeout_ms) {
					DMA1_Stream0->CR &= ~DMA_SxCR_EN; // Tat DMA
					I2C1->CR1 |= I2C_CR1_STOP; // Gui Stop
					return I2C_DMA_TIMEOUT;
			}
	}
	
	// 10. Giao dich hoan tat, gui STOP
	I2C1->CR1 |= I2C_CR1_STOP;
	
	// Don dep
	I2C1->CR1 &= ~I2C_CR1_ACK; // Tat ACK
	I2C1->CR2 &= ~I2C_CR2_LAST;
	I2C1->CR2 &= ~I2C_CR2_DMAEN;
	
	return I2C_DMA_OK;
}


/**
 * @brief  Doc nhiet do tu cam bien TMP117 su dung I2C1 va DMA.
 * @note   Ham nay gia dinh ban da co ham I2C1_DMA_Read_From_Reg.
 * @param  temperature_celsius Con tro toi mot bien float de luu gia tri nhiet do.
 * @retval I2C_DmaStatus_t Tra ve I2C_DMA_OK neu thanh cong, hoac ma loi neu that bai.
 */
// Dinh nghia dia chi I2C mac dinh cua TMP117 (khi chan ADD0 noi dat)
#define TMP117_I2C_ADDRESS 0x48

// Dinh nghia thanh ghi nhiet do (Temperature Register)
#define TMP117_TEMP_REGISTER 0x00
I2C_DmaStatus_t TMP117_Read_Temp(float* temperature_celsius)
{
	// Buffer de luu 2 byte du lieu tho doc tu cam bien
	uint8_t rx_buffer[2];
	
	// 1. Goi ham DMA doc 2 byte tu thanh ghi 0x00 cua TMP117
	I2C_DmaStatus_t status = I2C1_DMA_Read_From_Reg(
																	TMP117_I2C_ADDRESS,     // Dia chi slave 0x48
																	TMP117_TEMP_REGISTER,   // Dia chi thanh ghi 0x00
																	rx_buffer,              // Luu vao buffer
																	2,                      // Doc 2 byte
																	100);                   // Timeout 100ms
	
	// 2. Kiem tra xem DMA co doc thanh cong khong
	if (status != I2C_DMA_OK)
	{
			// Neu doc khong thanh cong, tra ve ma loi ngay lap tuc
			return status;
	}
	
	// 3. Xu ly du lieu neu doc thanh cong
	// Ghep 2 byte lai (MSB truoc) thanh so 16-bit co dau (signed)
	// vi nhiet do co the am
	int16_t raw_temp = (int16_t)((rx_buffer[0] << 8) | rx_buffer[1]);
	
	// 4. Chuyen doi gia tri tho sang do C
	// Theo datasheet cua TMP117, 1 LSB = 0.0078125 �C
	*temperature_celsius = (float)raw_temp * 0.0078125f;
	
	// 5. Tra ve trang thai thanh cong
	return I2C_DMA_OK;
}



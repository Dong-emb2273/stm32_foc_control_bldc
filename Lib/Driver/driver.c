#include "driver.h"

// ==========================================================
//				  CONFIG GPIO ( EDIT HERE )
// ==========================================================
#ifdef __GPIO_CONFIG__

static void GPIO_CLOCK_ENABLE(GPIO_TypeDef *GPIOx){
  if(GPIOx == GPIOA)      __HAL_RCC_GPIOA_CLK_ENABLE();
  else if(GPIOx == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
  else if(GPIOx == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
  else if(GPIOx == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
  else if(GPIOx == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
  else if(GPIOx == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
  else if(GPIOx == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
  else if(GPIOx == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
}

static void GPIO_CLOCK_DISABLE(GPIO_TypeDef *GPIOx){
  if(GPIOx == GPIOA)      __HAL_RCC_GPIOA_CLK_DISABLE();
  else if(GPIOx == GPIOB) __HAL_RCC_GPIOB_CLK_DISABLE();
  else if(GPIOx == GPIOC) __HAL_RCC_GPIOC_CLK_DISABLE();
  else if(GPIOx == GPIOD) __HAL_RCC_GPIOD_CLK_DISABLE();
  else if(GPIOx == GPIOE) __HAL_RCC_GPIOE_CLK_DISABLE();
  else if(GPIOx == GPIOF) __HAL_RCC_GPIOF_CLK_DISABLE();
  else if(GPIOx == GPIOG) __HAL_RCC_GPIOG_CLK_DISABLE();
  else if(GPIOx == GPIOH) __HAL_RCC_GPIOH_CLK_DISABLE();
}

static void GPIO_SPI_IN_INIT (void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(SPI_IN_SCK_PORT);
  GPIO_CLOCK_ENABLE(SPI_IN_MISO_PORT);
  GPIO_CLOCK_ENABLE(SPI_IN_MOSI_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = SPI_IN_GPIO_AF;

  GPIO_InitStruct.Pin = SPI_IN_SCK_PIN;
  HAL_GPIO_Init(SPI_IN_SCK_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SPI_IN_MISO_PIN;
  HAL_GPIO_Init(SPI_IN_MISO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SPI_IN_MOSI_PIN;
  HAL_GPIO_Init(SPI_IN_MOSI_PORT, &GPIO_InitStruct);

}

static void GPIO_SPI_EX_INIT (void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(SPI_EX_SCK_PORT);
  GPIO_CLOCK_ENABLE(SPI_EX_MISO_PORT);
  GPIO_CLOCK_ENABLE(SPI_EX_MOSI_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = SPI_EX_GPIO_AF;

  GPIO_InitStruct.Pin = SPI_EX_SCK_PIN;
  HAL_GPIO_Init(SPI_EX_SCK_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SPI_EX_MISO_PIN;
  HAL_GPIO_Init(SPI_EX_MISO_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = SPI_EX_MOSI_PIN;
  HAL_GPIO_Init(SPI_EX_MOSI_PORT, &GPIO_InitStruct);

}

static void GPIO_USART_INIT (void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(USART_TX_PORT);
  GPIO_CLOCK_ENABLE(USART_RX_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = USART_GPIO_AF;

  GPIO_InitStruct.Pin = USART_TX_PIN;
  HAL_GPIO_Init(USART_TX_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = USART_RX_PIN;
  HAL_GPIO_Init(USART_RX_PORT, &GPIO_InitStruct);

}

static void GPIO_CAN_INIT (void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(CAN_TX_PORT);
  GPIO_CLOCK_ENABLE(CAN_RX_PORT);
  
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = CAN_GPIO_AF;

  GPIO_InitStruct.Pin = CAN_TX_PIN;
  HAL_GPIO_Init(CAN_TX_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = CAN_RX_PIN;
  HAL_GPIO_Init(CAN_RX_PORT, &GPIO_InitStruct);

}

static void GPIO_I2C_INIT (void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(I2C_SCL_PORT);
  GPIO_CLOCK_ENABLE(I2C_SDA_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = I2C_GPIO_AF;

  GPIO_InitStruct.Pin = I2C_SCL_PIN;
  HAL_GPIO_Init(I2C_SCL_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = I2C_SDA_PIN;
  HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);

}

static void GPIO_TIM_PWM_INIT (void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  
  GPIO_CLOCK_ENABLE(TIM_PWM_CHA_PORT);
  GPIO_CLOCK_ENABLE(TIM_PWM_CHB_PORT);
  GPIO_CLOCK_ENABLE(TIM_PWM_CHC_PORT);
  GPIO_CLOCK_ENABLE(TIM_PWM_CHAN_PORT);
  GPIO_CLOCK_ENABLE(TIM_PWM_CHBN_PORT);
  GPIO_CLOCK_ENABLE(TIM_PWM_CHCN_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = TIM_PWM_GPIO_AF;

	GPIO_InitStruct.Pin = TIM_PWM_CHA_PIN;
	HAL_GPIO_Init(TIM_PWM_CHA_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = TIM_PWM_CHB_PIN;
	HAL_GPIO_Init(TIM_PWM_CHB_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = TIM_PWM_CHC_PIN;
	HAL_GPIO_Init(TIM_PWM_CHC_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = TIM_PWM_CHAN_PIN;
	HAL_GPIO_Init(TIM_PWM_CHAN_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = TIM_PWM_CHBN_PIN;
	HAL_GPIO_Init(TIM_PWM_CHBN_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = TIM_PWM_CHCN_PIN;
	HAL_GPIO_Init(TIM_PWM_CHCN_PORT, &GPIO_InitStruct);

}

static void GPIO_DRV8323_INIT (void){

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(GPIO_DRV8323_CS_PORT);
  GPIO_CLOCK_ENABLE(GPIO_DRV8323_ENABLE_PORT);
  GPIO_CLOCK_ENABLE(GPIO_DRV8323_FAULT_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = GPIO_DRV8323_CS_PIN;
  HAL_GPIO_Init(GPIO_DRV8323_CS_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_DRV8323_ENABLE_PIN;
  HAL_GPIO_Init(GPIO_DRV8323_ENABLE_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIO_DRV8323_ENABLE_PORT, GPIO_DRV8323_ENABLE_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIO_DRV8323_CS_PORT, GPIO_DRV8323_CS_PIN, GPIO_PIN_SET);

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = GPIO_DRV8323_FAULT_PIN;
  HAL_GPIO_Init(GPIO_DRV8323_FAULT_PORT, &GPIO_InitStruct);

}

static void GPIO_AS5048A_INIT (void){
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(GPIO_AS5048A_IN_CS_PORT);
  GPIO_CLOCK_ENABLE(GPIO_AS5048A_EX_CS_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = GPIO_AS5048A_IN_CS_PIN;
  HAL_GPIO_Init(GPIO_AS5048A_IN_CS_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_AS5048A_EX_CS_PIN;
  HAL_GPIO_Init(GPIO_AS5048A_EX_CS_PORT, &GPIO_InitStruct);

}

static void GPIO_LED_INIT (void){
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_CLOCK_ENABLE(GPIO_LED_GREEN_PORT);
  GPIO_CLOCK_ENABLE(GPIO_LED_RED_PORT);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = GPIO_LED_GREEN_PIN;
  HAL_GPIO_Init(GPIO_LED_GREEN_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_LED_RED_PIN;
  HAL_GPIO_Init(GPIO_LED_RED_PORT, &GPIO_InitStruct);

}

void GPIO_DRV832xS_CS_EN(void){
  GPIO_DRV8323_CS_PORT->BSRR = GPIO_DRV8323_CS_PIN;
}

void GPIO_DRV832xS_CS_DIS(void){
  GPIO_DRV8323_CS_PORT->BSRR = GPIO_DRV8323_CS_PIN << 16U;
}

void GPIO_AS5048A_IN_CS_EN(void){
  GPIO_AS5048A_IN_CS_PORT->BSRR = GPIO_AS5048A_IN_CS_PIN;
}

void GPIO_AS5048A_IN_CS_DIS(void){
  GPIO_AS5048A_IN_CS_PORT->BSRR = GPIO_AS5048A_IN_CS_PIN << 16U;
}

void GPIO_AS5048A_EX_CS_EN(void){
  GPIO_AS5048A_EX_CS_PORT->BSRR = GPIO_AS5048A_EX_CS_PIN;
}

void GPIO_AS5048A_EX_CS_DIS(void){
  GPIO_AS5048A_EX_CS_PORT->BSRR = GPIO_AS5048A_EX_CS_PIN << 16U;
}

void GPIO_LED_GREEN_EN(void){
  GPIO_LED_GREEN_PORT->BSRR = GPIO_LED_GREEN_PIN;
}

void GPIO_LED_GREEN_DIS(void){
  GPIO_LED_GREEN_PORT->BSRR = GPIO_LED_GREEN_PIN << 16U;
}

void GPIO_LED_RED_EN(void){
  GPIO_LED_RED_PORT->BSRR = GPIO_LED_RED_PIN;
}

void GPIO_LED_RED_DIS(void){
  GPIO_LED_RED_PORT->BSRR = GPIO_LED_RED_PIN << 16U;
}


void GPIO_INIT (void){

  GPIO_DRV8323_INIT();
  GPIO_AS5048A_INIT();

}

#endif



// =================================================1=========
// 					CONFIG TIM & PWM ( EDIT HERE )
// ==========================================================
#ifdef __TIM_CONFIG__

void TIM_PWM_DMA_CONFIG(void) {
	// 1. B?t Clock 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; 
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// 2. GPIO Config 

	GPIO_TIM_PWM_INIT();
	
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
#endif



// ==========================================================
//				  CONFIG ADC & DMA ( EDIT HERE )
// ==========================================================
#ifdef __ADC_CONFIG__

void ADC_DMA_CONFIG(void) {
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

#endif




// ==========================================================
//				  CONFIG SPI & DMA ( EDIT HERE )
// ==========================================================
#ifdef __SPI_CONFIG__

SPI_HandleTypeDef hspi_in;
SPI_HandleTypeDef hspi_ex;
DMA_HandleTypeDef hdma_spi_in_rx;
DMA_HandleTypeDef hdma_spi_in_tx;
DMA_HandleTypeDef hdma_spi_ex_rx;
DMA_HandleTypeDef hdma_spi_ex_tx;

static void SPI_CLOCK_ENABLE(SPI_TypeDef *SPIx) {
  if(SPIx == SPI1)        __HAL_RCC_SPI1_CLK_ENABLE();
  else if(SPIx == SPI2)   __HAL_RCC_SPI2_CLK_ENABLE();
  else if(SPIx == SPI3)   __HAL_RCC_SPI3_CLK_ENABLE();
}
static void SPI_CLOCK_DISABLE(SPI_TypeDef *SPIx) {
  if(SPIx == SPI1)        __HAL_RCC_SPI1_CLK_DISABLE();
  else if(SPIx == SPI2)   __HAL_RCC_SPI2_CLK_DISABLE();
  else if(SPIx == SPI3)   __HAL_RCC_SPI3_CLK_DISABLE();
}

void MX_SPI_IN_Init(void)
{
  hspi_in.Instance = SPI_IN_X;
  hspi_in.Init.Mode = SPI_MODE_MASTER;
  hspi_in.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_in.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_in.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_in.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_in.Init.NSS = SPI_NSS_SOFT;
  hspi_in.Init.BaudRatePrescaler = SPI_IN_SCALE;
  hspi_in.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_in.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_in.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_in.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi_in) != HAL_OK)
  {
    Error_Handler();
  }

}

void MX_SPI_EX_Init(void)
{
  hspi_ex.Instance = SPI_EX_X;
  hspi_ex.Init.Mode = SPI_MODE_MASTER;
  hspi_ex.Init.Direction = SPI_DIRECTION_2LINES;
  hspi_ex.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi_ex.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi_ex.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi_ex.Init.NSS = SPI_NSS_SOFT;
  hspi_ex.Init.BaudRatePrescaler = SPI_EX_SCALE;
  hspi_ex.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi_ex.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi_ex.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi_ex.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi_ex) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI_IN_X)
  {
    SPI_CLOCK_ENABLE(hspi->Instance);

    /* SPI_IN DMA Init */
    /* SPI_IN_RX Init */
    hdma_spi_in_rx.Instance = SPI_IN_DMA_RX_STREAM;
    hdma_spi_in_rx.Init.Channel = SPI_IN_DMA_RX_CHANNEL;
    hdma_spi_in_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_in_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_in_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_in_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_in_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi_in_rx.Init.Mode = DMA_NORMAL;
    hdma_spi_in_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_spi_in_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi_in_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi_in_rx);

    /* SPI_IN_TX Init */
    hdma_spi_in_tx.Instance = SPI_IN_DMA_TX_STREAM;
    hdma_spi_in_tx.Init.Channel = SPI_IN_DMA_TX_CHANNEL;
    hdma_spi_in_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi_in_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_in_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_in_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_in_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi_in_tx.Init.Mode = DMA_NORMAL;
    hdma_spi_in_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_spi_in_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi_in_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi_in_tx);

    /* SPI_IN interrupt Init */
    HAL_NVIC_SetPriority(SPI_IN_IRQ, SPI_IN_IRQ_PRIO);
    HAL_NVIC_EnableIRQ(SPI_IN_IRQ);
  }
  else if(hspi->Instance==SPI_EX_X)
  {
    SPI_CLOCK_ENABLE(hspi->Instance);

    /* SPI_EX DMA Init */
    /* SPI_EX_RX Init */
    hdma_spi_ex_rx.Instance = SPI_EX_DMA_RX_STREAM;
    hdma_spi_ex_rx.Init.Channel = SPI_EX_DMA_RX_CHANNEL;
    hdma_spi_ex_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_ex_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_ex_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_ex_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_ex_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi_ex_rx.Init.Mode = DMA_NORMAL;
    hdma_spi_ex_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi_ex_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi_ex_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmarx,hdma_spi_ex_rx);

    /* SPI_EX_TX Init */
    hdma_spi_ex_tx.Instance = SPI_EX_DMA_TX_STREAM;
    hdma_spi_ex_tx.Init.Channel = SPI_EX_DMA_TX_CHANNEL;
    hdma_spi_ex_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi_ex_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_ex_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_ex_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_ex_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi_ex_tx.Init.Mode = DMA_NORMAL;
    hdma_spi_ex_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi_ex_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi_ex_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi_ex_tx);

    /* SPI_EX interrupt Init */
    HAL_NVIC_SetPriority(SPI_EX_IRQ, SPI_EX_IRQ_PRIO);
    HAL_NVIC_EnableIRQ(SPI_EX_IRQ);
  }

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI_IN_X)
  {
    /* Peripheral clock disable */
    SPI_CLOCK_DISABLE(hspi->Instance);

	/* SPI GPIO Configuration */
    HAL_GPIO_DeInit(SPI_IN_SCK_PORT, SPI_IN_SCK_PIN);
    HAL_GPIO_DeInit(SPI_IN_MISO_PORT, SPI_IN_MISO_PIN);
    HAL_GPIO_DeInit(SPI_IN_MOSI_PORT, SPI_IN_MOSI_PIN);

    /* SPI DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);

    /* SPI interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI_IN_IRQ);
  }
  else if(hspi->Instance==SPI_EX_X)
  {
    /* Peripheral clock disable */
    SPI_CLOCK_DISABLE(hspi->Instance);

    /* SPI GPIO Configuration */
    HAL_GPIO_DeInit(SPI_EX_SCK_PORT, SPI_EX_SCK_PIN);
    HAL_GPIO_DeInit(SPI_EX_MISO_PORT, SPI_EX_MISO_PIN);
    HAL_GPIO_DeInit(SPI_EX_MOSI_PORT, SPI_EX_MOSI_PIN);

    /* SPI DMA DeInit */
    HAL_DMA_DeInit(hspi->hdmarx);
    HAL_DMA_DeInit(hspi->hdmatx);

    /* SPI interrupt DeInit */
    HAL_NVIC_DisableIRQ(SPI_EX_IRQ);
  }

}

void SPI_IN_TransmitReceive_DMA(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size) {
  HAL_SPI_TransmitReceive_DMA(&hspi_in, tx_buffer, rx_buffer, size);
}

void SPI_EX_TransmitReceive_DMA(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size) {
  HAL_SPI_TransmitReceive_DMA(&hspi_ex, tx_buffer, rx_buffer, size);
}

void SPI_IN_TransmitReceive(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size) {
  HAL_SPI_TransmitReceive(&hspi_in, tx_buffer, rx_buffer, size, 100);
}

void SPI_EX_TransmitReceive(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size) {
  HAL_SPI_TransmitReceive(&hspi_ex, tx_buffer, rx_buffer, size, 100);
}

#endif



// ==========================================================
//				  CONFIG USART COM & DMA ( EDIT HERE )
// ==========================================================
#ifdef __UART_CONFIG__

UART_HandleTypeDef huart;
DMA_HandleTypeDef hdma_usart_rx;
DMA_HandleTypeDef hdma_usart_tx;

static void USART_CLOCK_ENABLE(USART_TypeDef *USARTx){
  if(USARTx == USART1)        __HAL_RCC_USART1_CLK_ENABLE();
  else if(USARTx == USART2)   __HAL_RCC_USART2_CLK_ENABLE();
  else if(USARTx == USART3)   __HAL_RCC_USART3_CLK_ENABLE();
  else if(USARTx == UART4)    __HAL_RCC_UART4_CLK_ENABLE();
  else if(USARTx == UART5)    __HAL_RCC_UART5_CLK_ENABLE();
  else if(USARTx == USART6)   __HAL_RCC_USART6_CLK_ENABLE();
}

static void USART_CLOCK_DISABLE(USART_TypeDef *USARTx){
  if(USARTx == USART1)        __HAL_RCC_USART1_CLK_DISABLE();
  else if(USARTx == USART2)   __HAL_RCC_USART2_CLK_DISABLE();
  else if(USARTx == USART3)   __HAL_RCC_USART3_CLK_DISABLE();
  else if(USARTx == UART4)    __HAL_RCC_UART4_CLK_DISABLE();
  else if(USARTx == UART5)    __HAL_RCC_UART5_CLK_DISABLE();
  else if(USARTx == USART6)   __HAL_RCC_USART6_CLK_DISABLE();
}

void MX_USART_UART_Init(void)
{
  huart.Instance = USART_X;
  huart.Init.BaudRate = USART_BAUDRATE;
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

}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_USART_INIT();

  if(huart->Instance==USART_X)
  {
    /* Peripheral clock enable */
    USART_CLOCK_ENABLE(USART_X);

    /* USART DMA Init */
    /* USART_RX Init */
    hdma_usart_rx.Instance = USART_DMA_RX_STREAM;
    hdma_usart_rx.Init.Channel = USART_DMA_RX_CHANNEL;
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

    /* USART_TX Init */
    hdma_usart_tx.Instance = USART_DMA_TX_STREAM;
    hdma_usart_tx.Init.Channel = USART_DMA_TX_CHANNEL;
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

    /* USART interrupt Init */
    HAL_NVIC_SetPriority(USART_IRQ, USART_IRQ_PRIO);
    HAL_NVIC_EnableIRQ(USART_IRQ);

  }

}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART_X)
  {
    /* Peripheral clock disable */
    USART_CLOCK_DISABLE(USART_X);

    HAL_GPIO_DeInit(USART_TX_PORT, USART_TX_PIN);
    HAL_GPIO_DeInit(USART_RX_PORT, USART_RX_PIN);

    /* USART DMA DeInit */
    HAL_DMA_DeInit(huart->hdmarx);
    HAL_DMA_DeInit(huart->hdmatx);

    /* USART interrupt DeInit */
    HAL_NVIC_DisableIRQ(USART_IRQ);
  }

}

void USART_Transmit_DMA(uint8_t *pData, uint16_t Size) {
  HAL_UART_Transmit_DMA(&huart, pData, Size);
}

void USART_Receive_DMA(uint8_t *pData, uint16_t Size) {
  HAL_UART_Receive_DMA(&huart, pData, Size);
}


#endif




// ==========================================================
//				CONFIG CAN ( EDIT HERE )
// ==========================================================
#ifdef __CAN_CONFIG__

CAN_HandleTypeDef hcan;

static void CAN_CLOCK_ENABLE(CAN_TypeDef *CANx){
  if(CANx == CAN1)        __HAL_RCC_CAN1_CLK_ENABLE();
  else if(CANx == CAN2)   __HAL_RCC_CAN2_CLK_ENABLE();
}

static void CAN_CLOCK_DISABLE(CAN_TypeDef *CANx){
  if(CANx == CAN1)        __HAL_RCC_CAN1_CLK_DISABLE();
  else if(CANx == CAN2)   __HAL_RCC_CAN2_CLK_DISABLE();
}

/* CAN init function */
void MX_CAN_Init(void)
{
  hcan.Instance = CAN_X;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN_X){
	/* CAN clock enable */
	CAN_CLOCK_ENABLE(canHandle->Instance);

	/* CAN GPIO Configuration */
  GPIO_CAN_INIT();
	
	/* CAN interrupt Init */
	HAL_NVIC_SetPriority(CAN_X_IRQ, CAN_X_IRQ_PRIO);
	HAL_NVIC_EnableIRQ(CAN_X_IRQ);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN_X){
    /* Peripheral clock disable */
    CAN_CLOCK_DISABLE(canHandle->Instance);

    /* CAN GPIO Configuration */
    HAL_GPIO_DeInit(CAN_TX_PORT, CAN_TX_PIN);
		HAL_GPIO_DeInit(CAN_RX_PORT, CAN_RX_PIN);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN_X_IRQ);
  }
}

#endif



// ==========================================================
//				  CONFIG CRC & DMA ( EDIT HERE )
// ==========================================================
#ifdef __CRC_CONFIG__

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
#endif



#ifdef __LINK_DRIVER__

static uint32_t get_timer_clock(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1 || htim->Instance == TIM8 || 
      htim->Instance == TIM9 || htim->Instance == TIM10 || 
      htim->Instance == TIM11) {
    // Timer in APB2
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
      return pclk * 2;  // Timer clock doubled if APB prescaler != 1
    else
      return pclk;
  } else {
    // Timer in APB1
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
      return pclk * 2;
    else
      return pclk;
  }
}

void link_set_pwm_freq(TIM_TypeDef *timx, uint32_t freq) {
  const uint32_t timer_clock = get_timer_clock(timx);

  // prescaler and period for center-aligned mode
  uint32_t prescaler = 0;
  uint32_t period = (timer_clock / (2 * freq)) - 1;

  if (period > 0xFFFF) {
    prescaler = period / 0xFFFF;
    period = (timer_clock / (2 * freq * (prescaler + 1))) - 1;
  }

  timx->PSC = prescaler;
  timx->ARR = period;
}

uint32_t link_get_max_pwm(TIM_TypeDef *timx){
  return timx->ARR;
}

int link_enable_pwm(TIM_TypeDef *timx){
  if (timx == NULL) return -1;

  /* Enable CH1, CH2, CH3 */
  timx->CCER  |= TIM_CCER_CC1E
              | TIM_CCER_CC2E
              | TIM_CCER_CC3E;

  /* Enable complementary CH1N, CH2N, CH3N */
  timx->CCER  |= TIM_CCER_CC1NE
              | TIM_CCER_CC2NE
              | TIM_CCER_CC3NE;

  /* Enable CH4 for ADC trigger */
  timx->CCER |= TIM_CCER_CC4E;

  /* Set CH4 compare value for ADC trigger */
  timx->CCR4 = timx->ARR - 1U;

  /* Enable main output (TIM1/TIM8 advanced timers) */
  timx->BDTR |= TIM_BDTR_MOE;

  /* Enable timer counter */
  timx->CR1 |= TIM_CR1_CEN;

  return 0;
}

int link_disable_pwm(TIM_TypeDef *timx){
  if (timx == NULL) return -1;

  /* Disable main output */
  timx->BDTR &= ~TIM_BDTR_MOE;

  /* Disable CH1, CH2, CH3 */
  timx->CCER &= ~(TIM_CCER_CC1E  |
                  TIM_CCER_CC1NE |
                  TIM_CCER_CC2E  |
                  TIM_CCER_CC2NE |
                  TIM_CCER_CC3E  |
                  TIM_CCER_CC3NE |
                  TIM_CCER_CC4E);

  return 0;
}

int link_enable_adc(ADC_TypeDef *adc){
  if (adc == NULL) return -1;

  /* Enable ADC */
  adc->CR2 |= ADC_CR2_ADON;

  /* Clear previous injected conversion flag */
  adc->SR &= ~ADC_SR_JEOC;

  /* Enable injected end of conversion interrupt */
  adc->CR1 |= ADC_CR1_JEOCIE;

  /* Software trigger if external trigger is disabled */
  if ((adc->CR2 & ADC_CR2_JEXTEN) == 0U)
  {
      adc->CR2 |= ADC_CR2_JSWSTART;
  }

  return 0;
}

int link_disable_adc(ADC_TypeDef *adc){
  if (adc == NULL) return -1;

  /* Disable injected interrupt */
  adc->CR1 &= ~ADC_CR1_JEOCIE;

  /* Disable external injected trigger */
  adc->CR2 &= ~ADC_CR2_JEXTEN;

  /* Clear injected conversion flag */
  adc->SR &= ~ADC_SR_JEOC;

  return 0;
}

int link_write_flash(void *data, uint32_t len){
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  
  HAL_FLASH_Unlock();

  EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector       = FLASH_SECTOR_NUM;
  EraseInitStruct.NbSectors    = 1;

  status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    return -1;
  }

  uint32_t address = FLASH_SECTOR_ADDR;
  uint8_t *src = (uint8_t *)data;

  for (uint32_t i = 0; i < len; i += 4) {
    uint32_t word = *(uint32_t*)(src + i);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word);
    if (status != HAL_OK) {
      HAL_FLASH_Lock();
      return -1;
    }
    address += 4;
  }

  HAL_FLASH_Lock();
  return 0;
}

int link_get_adc(adc_raw_t*adc_raw){

  adc_raw->ia    = ADC_IA_DATA;
  adc_raw->ib    = ADC_IB_DATA;
  adc_raw->ic    = ADC_IC_DATA;
  adc_raw->vbus  = ADC_VBUS_DATA;

  return 0;
}



#endif

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

#define TMP117_I2C_ADDRESS 0x48
#define TMP117_TEMP_REGISTER 0x00



#include "drv8323s.h"
#include "config.h"
#include "user_config.h"
#include "cmsis_os.h"

extern GPIO_InitTypeDef GPIO_Init;

void DRV8323RS_Cs_Ena_faul_Config_Pin(void){
	
	GPIO_Init.Pin = DRV8323RS_CS_PIN;
	GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init.Pull = GPIO_NOPULL;
	GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DRV8323RS_CS_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = DRV8323RS_ENABLE_PIN;
	HAL_GPIO_Init(DRV8323RS_ENABLE_PORT, &GPIO_Init);
	
	GPIO_Init.Pin = DRV8323RS_FAUL_PIN;
	GPIO_Init.Mode = GPIO_MODE_IT_FALLING;
	HAL_GPIO_Init(DRV8323RS_FAUL_PORT, &GPIO_Init);
	
}
//



//

uint16_t DRV8323_WriteRegister(uint8_t addr, uint16_t data){
	uint8_t tx_buffer[2];
	uint8_t rx_buffer[2];

	uint16_t command = DRV8323_WRITE_CMD(addr, data);
	

	tx_buffer[0] = (command >> 8) & 0xFF; // Byte cao
	tx_buffer[1] = command & 0xFF;        // Byte thap

	DRV8323RS_cs_reset;

	HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, 100);

	DRV8323RS_cs_set;

	return ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];
}
//

uint16_t DRV8323_ReadRegister(uint8_t addr){
	uint8_t tx_buffer[2];
	uint8_t rx_buffer[2];
	uint16_t command;
	uint16_t received_data;

	command = DRV8323_READ_CMD(addr); 

	tx_buffer[0] = (command >> 8) & 0xFF;
	tx_buffer[1] = command & 0xFF;

	DRV8323RS_cs_reset;
	
	HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 2, 100);

	DRV8323RS_cs_set;

	received_data = ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];

	return (received_data & 0x07FF);
}
//

int DRV8323_Stop_PWM(DRV8323_t *cfg){
	if (cfg == NULL || cfg->Timx == NULL) {
		return 0;
	}

	TIM_TypeDef *tim = cfg->Timx;

	tim->BDTR &= ~TIM_BDTR_MOE;

	tim->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

	tim->CCER &= ~(TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

	return 1;
}
//

int DRV8323_Start_PWM(DRV8323_t *cfg){
	if (cfg == NULL || cfg->Timx == NULL) {
			return 0;
	}
	TIM_TypeDef *tim = cfg->Timx;

	tim->CCR1 = 0;
	tim->CCR2 = 0;
	tim->CCR3 = 0;
	
	tim->CCR4 = tim->ARR - 1;
	tim->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
	tim->CCER |= (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

	tim->BDTR |= TIM_BDTR_MOE;
	tim->CR1 |= TIM_CR1_CEN;

	return 1;
}
//

void DRV8323_Set_PWM(DRV8323_t *cfg, uint32_t pwma, uint32_t pwmb, uint32_t pwmc){
    cfg->Timx->CCR1 = pwma;
    cfg->Timx->CCR2 = pwmb;
    cfg->Timx->CCR3 = pwmc;
}
//

int DRV8323_Current_Sens_Config(DRV8323_t *cfg, float gain, float R_shunt){
	if (cfg == NULL || gain > 40.0 || R_shunt <= 0) return 0;

	cfg->gain = gain;
	cfg->R_shunt = R_shunt;

	cfg->v_to_currenta = 1.0f / (cfg->gain * (cfg->R_shunt)); 
	cfg->v_to_currentb = 1.0f / (cfg->gain * (cfg->R_shunt)); 
	cfg->v_to_currentc = 1.0f / (cfg->gain * (cfg->R_shunt)); 

	return 1;
}


void DRV8323_Calibrate_Current_Offset(DRV8323_t *cfg){
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;
    uint32_t sum_c = 0;
    const int num_samples = 2000;

    printf("Starting Current Sensor Calibration...\r\n");

	DRV8323_Set_PWM(cfg, 0, 0, 0);
	osDelay(100);
    for (int i = 0; i < num_samples; i++) {
        sum_a += ADC1->JDR1;
        sum_b += ADC2->JDR1;
        sum_c += ADC3->JDR1;
        osDelay(1);
    }

    V_OFFSET_A = (float)sum_a / (float)num_samples;
    V_OFFSET_B = (float)sum_b / (float)num_samples;
    V_OFFSET_C = (float)sum_c / (float)num_samples;

    printf("Calibrated Offsets - A: %.2f, B: %.2f, C: %.2f\r\n", V_OFFSET_A, V_OFFSET_B, V_OFFSET_C);
}

#define MAX_CURRENT_JUMP 		30
//
uint16_t spike_filter(uint16_t new_val, uint16_t prev_val) {
    float delta = new_val - prev_val;

    if (delta > MAX_CURRENT_JUMP) {
        return prev_val + MAX_CURRENT_JUMP; 
    } else if (delta < -MAX_CURRENT_JUMP) {
        return prev_val - MAX_CURRENT_JUMP; 
    }
    return new_val;
}

//

void DRV8323_Get_Current(DRV8323_t *cfg, float *ia, float *ib, float *ic){
	// Static filter state (retain between calls)
	// static float ia_filtered = 0.0f;
    // static float ib_filtered = 0.0f;
    // static float ic_filtered = 0.0f;
    
    cfg->adc_raw[0] = ADC1->JDR1; 
    cfg->adc_raw[1] = ADC2->JDR1; 
    cfg->adc_raw[2] = ADC3->JDR1; 

    float ia_raw = ((float)cfg->adc_raw[0] - V_OFFSET_A) * ADC_2_VOLT * cfg->v_to_currenta;
    float ib_raw = ((float)cfg->adc_raw[1] - V_OFFSET_B) * ADC_2_VOLT * cfg->v_to_currentb;
    float ic_raw = ((float)cfg->adc_raw[2] - V_OFFSET_C) * ADC_2_VOLT * cfg->v_to_currentc;
    
    // IIR Low Pass Filter
    // ia_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ia_filtered + CURRENT_FILTER_ALPHA * ia_raw;
    // ib_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ib_filtered + CURRENT_FILTER_ALPHA * ib_raw;
    // ic_filtered = (1.0f - CURRENT_FILTER_ALPHA) * ic_filtered + CURRENT_FILTER_ALPHA * ic_raw;

    // Output
    *ia = ia_raw;
    *ib = ib_raw;
    *ic = ic_raw;
}
//

int DRV8302_TIMER_config(DRV8323_t *cfg, TIM_TypeDef *timer, uint32_t freq) {
	if (cfg == NULL || timer == NULL) return 0;
	ADC1->SR &= ~ADC_SR_OVR; 

  	ADC1->CR2 |= ADC_CR2_SWSTART;

	uint32_t prescaler = 0;
	uint32_t period = (168000000 / (2 * freq)) - 1;
	
	cfg->Timx = timer;
	cfg->Timx->PSC = prescaler;
	cfg->Timx->ARR = period;
  	cfg->Timx->CCR4 = period - 1;  
	cfg->pwm_resolution = period;

	return 1;
}
//

int DRV8323_V_Ofset(DRV8323_t *hbldc){
	uint32_t ofseta = 0, ofsetb = 0,ofsetc = 0;
	DRV8323_Stop_PWM(hbldc);
	
	DRV8323_WriteRegister(0x06, 0x02DF);
	
	for(uint16_t i = 1; i <= 500; i++){
		
		ofseta += hbldc->adc_raw[0];
		ofsetb += hbldc->adc_raw[1];
		ofsetc += hbldc->adc_raw[2];
		HAL_Delay(1);
	}
	hbldc->v_offset_a = ofseta/500;
	hbldc->v_offset_b = ofsetb/500;
	hbldc->v_offset_c = ofsetc/500;

	DRV8323_WriteRegister(0x06, 0x02C3);
	DRV8323_Start_PWM(hbldc);
	
	return 1;
}
//

void bldc_config(DRV8323_t *hbldc){
	
	DRV8323RS_Cs_Ena_faul_Config_Pin();
	
	DRV8323RS_Enable;
	HAL_Delay(20);
	DRV8323_WriteRegister(0x02, 0x0001);
	HAL_Delay(20);
	DRV8323_WriteRegister(0x02, 0x0001);
	DRV8323_WriteRegister(0x03, 0x03AA);
	DRV8323_WriteRegister(0x04, 0x07AA);
	DRV8323_WriteRegister(0x05, 0x0159);
	
  DRV8302_TIMER_config(hbldc, BLDC_TIMX, BLDC_PWM_FREQ);
	
  DRV8323_Current_Sens_Config(hbldc, GAIN, R_SHUNT);
	

}
//













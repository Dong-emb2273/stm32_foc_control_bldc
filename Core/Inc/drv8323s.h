#ifndef __DRV8323S_H__
#define __DRV8323S_H__

#include "main.h"
//#include "config.h"
#include "spi.h"


#define DRV8323RS_CS_PIN				GPIO_PIN_12
#define DRV8323RS_CS_PORT				GPIOC
#define DRV8323RS_ENABLE_PIN			GPIO_PIN_11
#define DRV8323RS_ENABLE_PORT			GPIOC
#define DRV8323RS_FAUL_PIN				GPIO_PIN_15
#define DRV8323RS_FAUL_PORT				GPIOA


#define DRV8323_WRITE_CMD(addr, data) 	((0x0000) | ((addr & 0x0F) << 11) | (data & 0x7FF))
#define DRV8323_READ_CMD(addr)        (	(0x8000) | ((addr & 0x0F) << 11))

#define DRV8323RS_cs_set				(GPIOC->BSRR = GPIO_PIN_12)
#define DRV8323RS_cs_reset				(GPIOC->BSRR = GPIO_PIN_12 << 16)
#define DRV8323RS_Enable				(DRV8323RS_ENABLE_PORT->BSRR = DRV8323RS_ENABLE_PIN)
#define DRV8323RS_Disnable				(DRV8323RS_ENABLE_PORT->BSRR = DRV8323RS_ENABLE_PIN << 16)

#define ADC_RES							4096
#define AVDD							3.3f


#define ADC_2_VOLT						0.0008056640625f // = AVDD / ADC_RES

#define ADC_2_POWER_VOLT				0.01651611328125f

#define CURRENT_FILTER_ALPHA			0.71539f //0.466512f //0.71539f 

#define BLDC_PWM_FREQ					25000
#define BLDC_TIMX						TIM1

#define R_HIGHT							82000.0f
#define R_LOW							10000.0f
// #define R_SHUNT	


#define R_SHUNT							0.002f
#define GAIN							40.0f


#define SPEED_CONTROL_CYCLE				10
#define FOC_TS							(1.0f / (float)BLDC_PWM_FREQ)





typedef struct {

	TIM_TypeDef *Timx;
	
	uint32_t pwm_freq;
	uint32_t pwm_resolution;
	
	volatile uint16_t adc_raw[3];

	

	float gain;
	float R_shunt;
	
	uint16_t adc_rawPev[3];
	uint16_t v_offset_a;
	uint16_t v_offset_b;
	uint16_t v_offset_c;
	
	float v_to_currenta; //pre-calculate conversion from voltaeg(V) to current(A)
	float v_to_currentb;
	float v_to_currentc;
	
	
	
}DRV8323_t;

void DRV8323RS_Cs_Ena_faul_Config_Pin(void);

uint16_t DRV8323_ReadRegister(uint8_t addr);

uint16_t DRV8323_WriteRegister(uint8_t addr, uint16_t data);

int DRV8323_Stop_PWM(DRV8323_t *cfg);

int DRV8323_Start_PWM(DRV8323_t *cfg);

int DRV8323_TIMER_ADC_config(DRV8323_t *cfg, TIM_TypeDef *timer, uint32_t freq);

int DRV8302_TIMER_config(DRV8323_t *cfg, TIM_TypeDef *timer, uint32_t freq);

void DRV8323_Set_PWM(DRV8323_t *cfg, uint32_t pwma, uint32_t pwmb, uint32_t pwmc);

int DRV8323_Current_Sens_Config(DRV8323_t *cfg, float gain, float R_shunt);

uint16_t spike_filter(uint16_t new_val, uint16_t prev_val);

void DRV8323_Calibrate_Current_Offset(void);

void DRV8323_Get_Current(DRV8323_t *cfg, float *ia, float *ib, float *ic);

int DRV8323_V_Ofset(DRV8323_t *hbldc);

void bldc_config(DRV8323_t *hbldc);



#endif
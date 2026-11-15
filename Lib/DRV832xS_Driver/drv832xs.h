#ifndef __DRV8323S_H__
#define __DRV8323S_H__

#include "main.h"
#include "driver.h"




#define FS1									0x00
#define FS1_VDS_LC							(1U << 0U)
#define FS1_VDS_HC							(1U << 1U)
#define FS1_VDS_LB							(1U << 2U)
#define FS1_VDS_HB							(1U << 3U)
#define FS1_VDS_LA							(1U << 4U)
#define FS1_VDS_HA							(1U << 5U)
#define FS1_OTSD							(1U << 6U)
#define FS1_UVLO							(1U << 7U)
#define FS1_GDF								(1U << 8U)
#define FS1_VDS_OCP							(1U << 9U)
#define FS1_FAULT							(1U << 10U)

#define FS2									0x01
#define FS2_VGS_LC							(1U << 0U)
#define FS2_VGS_HC							(1U << 1U)
#define FS2_VGS_LB							(1U << 2U)
#define FS2_VGS_HB							(1U << 3U)
#define FS2_VGS_LA							(1U << 4U)
#define FS2_VGS_HA							(1U << 5U)
#define FS2_CPUV							(1U << 6U)
#define FS2_OTW								(1U << 7U)
#define FS2_SC_OC							(1U << 8U)
#define FS2_SB_OC							(1U << 9U)
#define FS2_SA_OC							(1U << 10U)

#define DRV8323S_DC							0x02

#define DC_CLR_FLT_pos						0U
#define DC_CLR_FLT							(1U << DC_CLR_FLT_pos)
#define DC_BRAKE_Pos 						1U
#define DC_BRAKE							(1U << DC_BRAKE_Pos)
#define DC_COAST_Pos 						2U
#define DC_COAST							(1U << DC_COAST_Pos)
#define DC_1PWM_Pos 						3U
#define DC_1PWM								(1U << DC_1PWM_Pos)
#define DC_1PWM_COM_Pos 					4U
#define DC_1PWM_COM 						(1U << DC_1PWM_COM_Pos)
#define DC_PWM_MODE_Pos 					5U
#define DC_PWM_MODE_Mask 					3U
#define DC_PWM_MODE_Indep					(3U << DC_PWM_MODE_Pos)
#define DC_PWM_MODE_1PWM 					(2U << DC_PWM_MODE_Pos)
#define DC_PWM_MODE_3PWM 					(1U << DC_PWM_MODE_Pos)
#define DC_PWM_MODE_6PWM 					(0U << DC_PWM_MODE_Pos)
#define DC_OTW_REP_Pos 						7U
#define DC_OTW_REP							(1U << DC_OTW_REP_Pos)
#define DC_DIS_GDF_Pos 						8U
#define DC_DIS_GDF_EN						(1U << DC_DIS_GDF_Pos)
#define DC_DIS_CPUV_Pos 					9U
#define DC_DIS_CPUV_EN 						(1U << DC_DIS_CPUV_Pos)

#define DRV8323S_GDHS 						0x03
#define GDHS_IDRIVEN_HS_Pos 				0U
#define GDHS_IDRIVEN_HS_Mask 				15U
#define GDHS_IDRIVEN_HS_10mA 				(0U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_30mA 				(1U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_60mA 				(2U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_80mA 				(3U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_120mA 				(4U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_140mA 				(5U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_170mA 				(6U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_190mA 				(7U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_260mA 				(8U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_330mA 				(9U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_370mA 				(10U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_440mA 				(11U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_570mA 				(12U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_680mA 				(13U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_820mA 				(14U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEN_HS_1000mA 				(15U << GDHS_IDRIVEN_HS_Pos)
#define GDHS_IDRIVEP_HS_Pos					4U
#define GDHS_IDRIVEP_HS_Mask 				15U
#define GDHS_IDRIVEP_HS_10mA 				(0U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_30mA 				(1U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_60mA 				(2U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_80mA 				(3U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_120mA 				(4U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_140mA 				(5U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_170mA 				(6U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_190mA 				(7U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_260mA 				(8U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_330mA 				(9U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_370mA 				(10U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_440mA 				(11U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_570mA 				(12U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_680mA 				(13U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_820mA 				(14U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_IDRIVEP_HS_1000mA 				(15U << GDHS_IDRIVEP_HS_Pos)
#define GDHS_LOCK_Pos 						8U
#define GDHS_LOCK_Mask 						7U
#define GDHS_LOCK							(6U << GDHS_LOCK_Pos)
#define GDHS_UNLOCK 						(3U << GDHS_LOCK_Pos)
			
#define DRV8323S_GDLS 						0x04
#define GDHS_IDRIVEN_LS_Pos 				0U
#define GDHS_IDRIVEN_LS_Mask 				15U
#define GDHS_IDRIVEN_LS_10mA 				(0U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_30mA 				(1U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_60mA 				(2U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_80mA 				(3U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_120mA 				(4U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_140mA 				(5U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_170mA 				(6U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_190mA 				(7U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_260mA 				(8U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_330mA 				(9U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_370mA 				(10U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_440mA 				(11U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_570mA 				(12U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_680mA 				(13U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_820mA 				(14U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEN_LS_1000mA 				(15U << GDHS_IDRIVEN_LS_Pos)
#define GDHS_IDRIVEP_LS_Pos 				4U
#define GDHS_IDRIVEP_LS_Mask 				15U
#define GDHS_IDRIVEP_LS_10mA 				(0U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_30mA 				(1U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_60mA 				(2U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_80mA 				(3U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_120mA 				(4U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_140mA 				(5U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_170mA 				(6U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_190mA 				(7U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_260mA 				(8U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_330mA 				(9U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_370mA 				(10U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_440mA 				(11U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_570mA 				(12U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_680mA 				(13U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_820mA 				(14U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_IDRIVEP_LS_1000mA 				(15U << GDHS_IDRIVEP_LS_Pos)
#define GDHS_TDRIVE_Pos 					8U
#define GDHS_TDRIVE_Mask					3U
#define GDHS_TDRIVE_500ns					(0U << GDHS_TDRIVE_Pos)
#define GDHS_TDRIVE_1000ns					(1U << GDHS_TDRIVE_Pos)
#define GDHS_TDRIVE_2000ns					(2U << GDHS_TDRIVE_Pos)
#define GDHS_TDRIVE_4000ns					(3U << GDHS_TDRIVE_Pos)
#define GDHS_CBC_Pos						10U
#define GDHS_CBC 							(1U << GDHS_CBC_Pos)

#define DRV8323S_OCPC 						0x05
#define OCPC_VDS_LVL_Pos 					0U
#define OCPC_VDS_LVL_Mask					0x0FU
#define OCPC_VDS_LVL_0_06V					(0x00U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_13V					(0x01U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_2V					(0x02U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_26V					(0x03U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_31V					(0x04U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_45V					(0x05U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_53V					(0x06U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_6V					(0x07U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_68V					(0x08U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_75V					(0x09U << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_0_94V					(0x0AU << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_1_13V					(0x0BU << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_1_3V					(0x0CU << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_1_5V					(0x0DU << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_1_7V					(0x0EU << OCPC_VDS_LVL_Pos)
#define OCPC_VDS_LVL_1_88V					(0x0FU << OCPC_VDS_LVL_Pos)
#define OCPC_OCP_DEG_Pos 					4U
#define OCPC_OCP_DEG_Mask 					0x03U
#define OCPC_OCP_DEG_2us 					(0x00U << OCPC_OCP_DEG_Pos)
#define OCPC_OCP_DEG_4us 					(0x01U << OCPC_OCP_DEG_Pos)
#define OCPC_OCP_DEG_6us 					(0x02U << OCPC_OCP_DEG_Pos)
#define OCPC_OCP_DEG_8us 					(0x03U << OCPC_OCP_DEG_Pos)
#define OCPC_OCP_MODE_Pos 					6U
#define OCPC_OCP_MODE_Mask 					0x03U
#define OCPC_OCP_MODE_LatchedFault 			(0x00U << OCPC_OCP_MODE_Pos)
#define OCPC_OCP_MODE_RetryingFault 		(0x01U << OCPC_OCP_MODE_Pos)
#define OCPC_OCP_MODE_ReportOnlyNoAction 	(0x02U << OCPC_OCP_MODE_Pos)
#define OCPC_OCP_MODE_NoReportNoAction 		(0x03U << OCPC_OCP_MODE_Pos)
#define OCPC_DEAD_TIME_Pos 					8U
#define OCPC_DEAD_TIME_Mask					0x03U
#define OCPC_DEAD_TIME_50ns 				(0x00U << OCPC_DEAD_TIME_Pos)
#define OCPC_DEAD_TIME_100ns 				(0x01U << OCPC_DEAD_TIME_Pos)
#define OCPC_DEAD_TIME_200ns 				(0x02U << OCPC_DEAD_TIME_Pos)
#define OCPC_DEAD_TIME_400ns 				(0x03U << OCPC_DEAD_TIME_Pos)
#define OCPC_TRETRY_Pos 					10U
#define OCPC_TRETRY_4ms						(0x00U << OCPC_TRETRY_Pos)
#define OCPC_TRETRY_5us						(0x01U << OCPC_TRETRY_Pos)


#define DRV8323S_CSAC 						0x06
#define CSAC_SEN_LVL_Pos        			0U
#define CSAC_SEN_LVL_Mask       			0x04U
#define CSAC_SEN_LVL_0_25V					(0x00U << CSAC_SEN_LVL_Pos)
#define CSAC_SEN_LVL_0_5V					(0x01U << CSAC_SEN_LVL_Pos)
#define CSAC_SEN_LVL_0_75V					(0x02U << CSAC_SEN_LVL_Pos)
#define CSAC_SEN_LVL_1V						(0x03U << CSAC_SEN_LVL_Pos)
#define CSAC_CSA_CAL_C_Pos					2U
#define CSAC_CSA_CAL_C						(0x01U << CSAC_CSA_CAL_C_Pos)
#define CSAC_CSA_CAL_B_Pos					3U
#define CSAC_CSA_CAL_B						(0x01U << CSAC_CSA_CAL_B_Pos)
#define CSAC_CSA_CAL_A_Pos					4U
#define CSAC_CSA_CAL_A						(0x01U << CSAC_CSA_CAL_A_Pos)
#define CSAC_DIS_SEN_Pos					5U
#define CSAC_DIS_SEN_EN						(0x00U << CSAC_DIS_SEN_Pos)
#define CSAC_DIS_SEN_DIS					(0x01U << CSAC_DIS_SEN_Pos)
#define CSAC_CSA_GAIN_Pos					6U
#define CSAC_CSA_GAIN_Mask					0x03U
#define CSAC_CSA_GAIN_5V					(0x00 << CSAC_CSA_GAIN_Pos)
#define CSAC_CSA_GAIN_10V					(0x01 << CSAC_CSA_GAIN_Pos)
#define CSAC_CSA_GAIN_20V					(0x02 << CSAC_CSA_GAIN_Pos)
#define CSAC_CSA_GAIN_40V					(0x03 << CSAC_CSA_GAIN_Pos)
#define CSAC_LS_REF_Pos         			8U
#define CSAC_LS_REF_Mask					0x01U
#define CSAC_LS_REF_SHxToSPx				(0x00U << CSAC_LS_REF_Pos)
#define CSAC_LS_REF_SHxToSNx				(0x01U << CSAC_LS_REF_Pos)
#define CSAC_VREF_DIV_Pos					9U
#define CSAC_VREF_DIV_Mask					0x01U
#define CSAC_VREF_DIV_DIS					(0x00U << CSAC_VREF_DIV_Pos)
#define CSAC_VREF_DIV_EN					(0x01U << CSAC_VREF_DIV_Pos)
#define CSAC_FET_Pos						10U
#define CSAC_FET_Mask						0x01U
#define CSAC_FET_SPx						(0x00U << CSAC_FET_Pos)
#define CSAC_FET_SHx						(0x01U << CSAC_FET_Pos)




#define DRV832xS_CS_PIN					GPIO_DRV8323_CS_PIN
#define DRV832xS_CS_PORT				GPIO_DRV8323_CS_PORT
#define DRV832xS_ENABLE_PIN				GPIO_DRV8323_ENABLE_PIN
#define DRV832xS_ENABLE_PORT			GPIO_DRV8323_ENABLE_PORT
#define DRV832xS_FAUL_PIN				GPIO_DRV8323_FAULT_PIN
#define DRV832xS_FAUL_PORT				GPIO_DRV8323_FAULT_PORT

#define DRV832xS_SPI					hspi_in


#define DRV832xS_WRITE_CMD(addr, data)	((0x0000) | ((addr & 0x0F) << 11) | (data & 0x7FF))
#define DRV832xS_READ_CMD(addr)			((0x8000) | ((addr & 0x0F) << 11))

#define DRV832xS_CS_EN					(DRV832xS_CS_PORT->BSRR = DRV832xS_CS_PIN)
#define DRV832xS_CS_DIS					(DRV832xS_CS_PORT->BSRR = DRV832xS_CS_PIN << 16)
#define DRV832xS_EN						(DRV832xS_ENABLE_PORT->BSRR = DRV832xS_ENABLE_PIN)
#define DRV832xS_DIS					(DRV832xS_ENABLE_PORT->BSRR = DRV832xS_ENABLE_PIN << 16)



typedef struct{
    uint8_t spi_tx_buffer[2];
	uint8_t spi_rx_buffer[2];
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
    uint16_t r5;


    int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t);
    void (*spi_cs_en)(void);
    void (*spi_cs_dis)(void);
    void (*drv832xs_en)(void);
    void (*drv832xs_dis)(void);
}DRV832xS_t;





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








uint16_t DRV832xS_Read16(DRV832xS_t *drv, uint8_t addr);

uint16_t DRV832xS_Write16(DRV832xS_t *drv, uint8_t addr, uint16_t data);



void DRV832xS_Setup(DRV832xS_t *drv);



#endif
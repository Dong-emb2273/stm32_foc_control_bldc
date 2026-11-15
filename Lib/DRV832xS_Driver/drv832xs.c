#include "drv8323s.h"


//
void DRV832xS_Config(DRV832xS_t *drv, int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t),
	void (*spi_cs_en)(void),void (*spi_cs_dis)(void), 
	void (*drv832xs_en)(void), void (*drv832xs_dis)(void)){
	
	drv->spi_transfer = spi_transfer;
	drv->spi_cs_en = spi_cs_en;
	drv->spi_cs_dis = spi_cs_dis;
	drv->drv832xs_en = drv832xs_en;
	drv->drv832xs_dis = drv832xs_dis;

}

void DRV832xS_Init(DRV832xS_t *drv, uint16_t r1, uint16_t r2 ){


}


uint16_t DRV832xS_Write16(DRV832xS_t *drv, uint8_t addr, uint16_t data){

	uint16_t command = DRV8323_WRITE_CMD(addr, data);
	
	drv->spi_tx_buffer[0] = (command >> 8) & 0xFF; // Byte cao
	drv->spi_tx_buffer[1] = command & 0xFF;        // Byte thap

	drv->spi_cs_dis();
	drv->spi_transfer(drv->spi_tx_buffer, drv->spi_rx_buffer, 2);
	while(DRV832xS_SPI.State == HAL_SPI_STATE_BUSY ); 
	drv->spi_cs_en();

	return ((uint16_t)drv->spi_rx_buffer[0] << 8) | drv->spi_rx_buffer[1];
}

uint16_t DRV832xS_Read16(DRV832xS_t *drv, uint8_t addr){
	uint16_t command;
	uint16_t received_data;

	command = DRV8323_READ_CMD(addr); 

	drv->spi_tx_buffer[0] = (command >> 8) & 0xFF;
	drv->spi_tx_buffer[1] = command & 0xFF;

	drv->spi_cs_dis();
	drv->spi_transfer(drv->spi_tx_buffer, drv->spi_rx_buffer, 2);
	while(DRV832xS_SPI.State == HAL_SPI_STATE_BUSY ); 
	drv->spi_cs_en();

	received_data = ((uint16_t)drv->spi_rx_buffer[0] << 8) | drv->spi_rx_buffer[1];

	return (received_data & 0x07FF);
}

void DRV832xS_Check_Error(DRV832xS_t *drv){

	uint16_t val1 = DRV832xS_Read16(drv, FS1);
	uint16_t val2 = DRV832xS_Read16(drv, FS2);

	DRV832xS_Print_Fault(val1, val2);

}
	
void DRV832xS_Print_Fault(uint16_t val1, uint16_t val2)
{
    /* ================= FS1: Fault Status Register 1 ================= */

	if (val1 & FS1_VDS_LC)	printf("\n\r[FS1] VDS_LC: Overcurrent on Phase C Low-side MOSFET\r\n");
	if (val1 & FS1_VDS_HC)	printf("[FS1] VDS_HC: Overcurrent on Phase C High-side MOSFET\r\n");
	if (val1 & FS1_VDS_LB)	printf("[FS1] VDS_LB: Overcurrent on Phase B Low-side MOSFET\r\n");
	if (val1 & FS1_VDS_HB)	printf("[FS1] VDS_HB: Overcurrent on Phase B High-side MOSFET\r\n");
	if (val1 & FS1_VDS_LA)	printf("[FS1] VDS_LA: Overcurrent on Phase A Low-side MOSFET\r\n");
	if (val1 & FS1_VDS_HA)	printf("[FS1] VDS_HA: Overcurrent on Phase A High-side MOSFET\r\n");
	if (val1 & FS1_OTSD)	printf("[FS1] OTSD: Overtemperature Shutdown\r\n");
	if (val1 & FS1_UVLO)	printf("[FS1] UVLO: Undervoltage Lockout\r\n");
	if (val1 & FS1_GDF)		printf("[FS1] GDF: Gate Drive Fault\r\n");
	if (val1 & FS1_VDS_OCP)	printf("[FS1] VDS_OCP: VDS Monitor Overcurrent Fault\r\n");
	if (val1 & FS1_FAULT)	printf("[FS1] FAULT: Fault Status Active (nFAULT)\r\n");

    /* ================= FS2: Fault Status Register 2 ================= */

    if (val2 & FS2_VGS_LC)	printf("[FS2] VGS_LC: Gate Drive Fault - Phase C Low-side MOSFET\r\n");
    if (val2 & FS2_VGS_HC)	printf("[FS2] VGS_HC: Gate Drive Fault - Phase C High-side MOSFET\r\n");
    if (val2 & FS2_VGS_LB)	printf("[FS2] VGS_LB: Gate Drive Fault - Phase B Low-side MOSFET\r\n");
    if (val2 & FS2_VGS_HB)	printf("[FS2] VGS_HB: Gate Drive Fault - Phase B High-side MOSFET\r\n");
    if (val2 & FS2_VGS_LA)	printf("[FS2] VGS_LA: Gate Drive Fault - Phase A Low-side MOSFET\r\n");
    if (val2 & FS2_VGS_HA)	printf("[FS2] VGS_HA: Gate Drive Fault - Phase A High-side MOSFET\r\n");
    if (val2 & FS2_CPUV)	printf("[FS2] CPUV: Charge Pump Undervoltage Fault\r\n");
    if (val2 & FS2_OTW)		printf("[FS2] OTW: Overtemperature Warning\r\n");
    if (val2 & FS2_SC_OC)	printf("[FS2] SC_OC: Overcurrent on Phase C Sense Amplifier\r\n");
    if (val2 & FS2_SB_OC)	printf("[FS2] SB_OC: Overcurrent on Phase B Sense Amplifier\r\n");
    if (val2 & FS2_SA_OC)	printf("[FS2] SA_OC: Overcurrent on Phase A Sense Amplifier\r\n");


    /* ================= Check if no faults ================= */

    if ((val1 == 0U) && (val2 == 0U))
    {
        printf("[DRV8323] No faults detected\r\n");
    }
}

void DRV832xS_Setup(DRV832xS_t *drv){

	drv->spi_cs_en();
	drv->drv832xs_en();

	HAL_Delay(20);
	DRV832xS_Write16(drv, 0x02, 0x0001);
	HAL_Delay(20);
	DRV832xS_Write16(drv, 0x02, 0x0001);
	DRV832xS_Write16(drv, 0x03, 0x03AA);
	DRV832xS_Write16(drv, 0x04, 0x07AA);
	DRV832xS_Write16(drv, 0x05, 0x0159);

}
//













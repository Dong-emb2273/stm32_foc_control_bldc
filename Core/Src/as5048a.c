

#include "as5048a.h"
#include "spi.h"


static void AS5048A_Config_CS(AS5048A_t *encd){
	GPIO_InitTypeDef as5048a = {0};
	as5048a.Pin = encd->AS5048A_cs_pin;
	as5048a.Mode = GPIO_MODE_OUTPUT_PP;
	as5048a.Pull = GPIO_NOPULL;
	as5048a.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(encd->AS5048A_cs_port, &as5048a);
}
//

static uint8_t calc_even_parity(uint16_t value){
	value ^= value >> 8;
	value ^= value >> 4;
	value ^= value >> 2;
	value ^= value >> 1;
	return value & 1;
}
//

int AS5048A_Config(AS5048A_t *encd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
	if (encd == NULL || hspi == NULL || cs_port == NULL || cs_pin == 0) {
			return 0;
	}

	encd->AS5048A_spi = hspi;
	encd->AS5048A_cs_port = cs_port;
	encd->AS5048A_cs_pin = cs_pin;
	AS5048A_Config_CS(encd);
	AS5048A_cs_set(encd);
	
	return 1;
}
//

int AS5048A_Config_ALL(AS5048A_t *encd){
	if (encd == NULL) {
		return 0;
	}
	AS5048A_Config_CS(encd);
	AS5048A_cs_set(encd);
	
	return 1;
}
//

int AS5048A_CheckExist(AS5048A_t *encd) {

	uint16_t cmd = AS5048A_WRITE_CMD | AS5048A_REG_ANGLE;
//	uint8_t rx_buffer[2];
	cmd |= (calc_even_parity(cmd) << 15);

	AS5048A_cs_reset(encd);
	HAL_SPI_TransmitReceive(encd->AS5048A_spi, (uint8_t*)&cmd, encd->spi_rx_buffer, 2, 100);
	AS5048A_cs_set(encd);
	HAL_Delay(1);
	AS5048A_cs_reset(encd);
	HAL_SPI_TransmitReceive(encd->AS5048A_spi, (uint8_t*)&cmd, encd->spi_rx_buffer, 2, 100);
	AS5048A_cs_set(encd);

	const uint16_t raw_data = ((uint16_t)encd->spi_rx_buffer[0] << 8) | encd->spi_rx_buffer[1];
	
	if (raw_data == 0x0000 || raw_data == 0xFFFF) {
			return -1; // No Device
	}

	uint16_t data_15bit = raw_data & 0x7FFF;
	uint8_t expected_parity = calc_even_parity(data_15bit);
	uint8_t received_parity = (raw_data >> 15) & 0x1;

	if (expected_parity != received_parity) {
			return 0; // Have Device but erro 
	}
	// Error flag check (bit 14)
//	if ((raw_data >> 14) & 0x1) {
//		return 0;
//	}
	
	return 1; // Have Device 
}
//

int AS5048A_DetectExit(AS5048A_t *encd){
	int8_t Count = 0;
	for(uint8_t i = 0; i < 10; i++){
		Count += AS5048A_CheckExist(encd);
		HAL_Delay(1);
	}
	if (Count > 4) return 1; // c� deive
	else if (Count < -4) return 0;// Have Device but erro
	else  return 0;// No deive
	
}
//

int AS5048A_StartRead(void *handle) {
	
	AS5048A_t *self = (AS5048A_t *)handle;
	uint16_t cmd = AS5048A_WRITE_CMD | AS5048A_REG_ANGLE;
	cmd |= (calc_even_parity(cmd) << 15);  // bit 15 parity

	AS5048A_cs_reset(self);
	if (HAL_SPI_TransmitReceive_DMA(self->AS5048A_spi, (uint8_t*)&cmd, self->spi_rx_buffer, 2) != HAL_OK) {
		return 0;
	}

	return 1;
}
//

float AS5048A_ParseData(void *handle) {
	AS5048A_t *self = (AS5048A_t *)handle;
	AS5048A_cs_set(self);
	
	const uint16_t raw_data = ((uint16_t)self->spi_rx_buffer[0] << 8) | self->spi_rx_buffer[1];

	// Parity check (bit 15 = parity)
	const uint16_t data_15bit = raw_data & 0x7FFF;
	const uint8_t expected_parity = calc_even_parity(data_15bit);
	const uint8_t received_parity = (raw_data >> 15) & 0x1;
	if (expected_parity != received_parity) {
		return -1.0f;
	}

	// Error flag check (bit 14)
//	if ((raw_data >> 14) & 0x1) {
//		return -1.0f;
//	}
	
	return (float)(raw_data & 0x3FFF) * ANGLE_SCALE_FACTOR;
}
//









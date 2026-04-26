#ifndef AS5048A_DRIVER_INC_AS5048A_H_
#define AS5048A_DRIVER_INC_AS5048A_H_


#include "main.h"
#include <string.h>
#include <math.h>


//======================== Config PIN =============================
#define AS5048A_cs_set(encd) ((encd)->AS5048A_cs_port->BSRR = (encd)->AS5048A_cs_pin)
#define AS5048A_cs_reset(encd) ((encd)->AS5048A_cs_port->BSRR = (encd)->AS5048A_cs_pin<<16)

//======================== END ====================================


#define ANGLE_SCALE_FACTOR 0.021973997F  // Pre-calculate scale factor (360.0f / 16383.0f)
#define AS5048A_REG_ANGLE  0x3FFF
#define AS5048A_WRITE_CMD  0x4000


typedef struct {
	SPI_HandleTypeDef *AS5048A_spi;
	GPIO_TypeDef			*AS5048A_cs_port;  
	uint16_t 					AS5048A_cs_pin;        
	uint8_t 					spi_rx_buffer[2];
}AS5048A_t;
//


// static void AS5048A_Config_CS(AS5048A_t *encd);

// static uint8_t calc_even_parity(uint16_t value);

int AS5048A_Config(AS5048A_t *encd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

int AS5048A_Config_ALL(AS5048A_t *encd);

int AS5048A_CheckExist(AS5048A_t *encd);

int AS5048A_DetectExit(AS5048A_t *encd);

int AS5048A_StartRead(void *handle);

float AS5048A_ParseData(void *handle);


#endif /* AS5048A_DRIVER_INC_AS5048A_H_ */
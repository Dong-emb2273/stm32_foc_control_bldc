#ifndef AS5048A_DRIVER_INC_AS5048A_H_
#define AS5048A_DRIVER_INC_AS5048A_H_


#include "main.h"
#include "spi.h"
#include "encoder.h"
#include <string.h>
#include <math.h>


//======================== Config PIN =============================
#define AS5048A_cs_set(encd)   (((struct Encoder_t*)encd)->active_port->cs_port->BSRR = ((struct Encoder_t*)encd)->active_port->cs_pin)
#define AS5048A_cs_reset(encd) (((struct Encoder_t*)encd)->active_port->cs_port->BSRR = ((struct Encoder_t*)encd)->active_port->cs_pin << 16)
//======================== END ====================================


#define ANGLE_SCALE_FACTOR 0.021973997F  // Pre-calculate scale factor (360.0f / 16383.0f)
#define AS5048A_REG_ANGLE  0x3FFF
#define AS5048A_WRITE_CMD  0x4000



//


// static void AS5048A_Config_CS(AS5048A_t *encd);

// static uint8_t calc_even_parity(uint16_t value);

void AS5048A_Config_CS(void *handle);
int AS5048A_CheckExist(void *handle);
int AS5048A_DetectExit(void *handle);
int AS5048A_StartRead(void *handle);
float AS5048A_ParseData(void *handle);


#endif /* AS5048A_DRIVER_INC_AS5048A_H_ */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h" // Thu vi?n HAL STM32
#include <string.h>
#include <math.h>
#include "spi.h"
#include "as5048a.h"

#define ANGLE_FILTER_ALPHA    0.71539f  //0.71539f //   // Faktor filter
#define MAX_ANGLE_JUMP_DEG    50.0f     // Batas maksimal lonjakan sudut (derajat)
#define SPIKE_REJECT_COUNT    10        // Jumlah sampel untuk konfirmasi spike

// Configurations (tune these based on your system)
#define MIN_DT_US             500UL      // Minimum 1ms interval for valid RPM (avoid division by tiny numbers)
#define MAX_RPM_JUMP          20.0f      // Conservative RPM jump threshold
#define RPM_FILTER_ALPHA      0.71539f   //0.71539f      // Base filter coefficient (balanced response)
#define DEGREES_PER_REV       360.0f     // For 1:1 gear ratio
#define MICROS_TO_MINUTES     6e7f       // Conversion factor (�s to minutes)


#define ACTUAL_ANGLE_FILTER_ALPHA 0.54187f
//#define ACTUAL_ANGLE_FILTER_ALPHA 0.4187f
extern _Bool encd_get_val_flag;
#define ENCODER_GetFlag() (encd_get_val_flag == 1)
#define ENCODER_Set_Flag() (encd_get_val_flag = 1)
#define ENCODER_Reset_Flag() (encd_get_val_flag = 0)

/* Enum ph�n lo?i */
typedef enum {
	ENCODER_LOC_INTERNAL = 0,
	ENCODER_LOC_EXTERNAL = 1
} EncoderLocation_t;

typedef enum {
	ENCODER_TYPE_NONE = 0,
	ENCODER_TYPE_AS5600,
	ENCODER_TYPE_AS5048A,
	ENCODER_TYPE_MT6835
} EncoderType_t;



/* Struct c?u h�nh c?ng ph?n c?ng */
typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef      *cs_port;
	uint16_t          cs_pin;
	I2C_HandleTypeDef *hi2c;
} Encoder_Port_t;

/* Struct T?ng (L?p giao di?n) */

typedef struct {
	EncoderLocation_t location;
	EncoderType_t type;
	Encoder_Port_t enc_internal;
	Encoder_Port_t enc_external;

	float raw_angle;  

	float angle_filtered;
	float prev_angle_filtered;
	float prev_raw_angle;
	uint8_t spike_counter;
	
	float prev_angle;
	float filtered_rpm;
	float prev_rpm;
	float angle_accumulator;
	uint32_t time_accumulator;
    
	float output_prev_angle;
	float output_angle_ovf;
  	float output_angle_filtered;
	
	
	void *hw_encoder;  


	int   (*start_read)(void *handle);
	float (*parse_data)(void *handle);
}Encoder_t;

/* C�c bi?n to�n c?c qu?n l� FOC */
extern Encoder_t encoder;


/*  */
int ENCODER_Setup();
int ENCODER_AutoDetect();
float ENCODER_GetDegree(Encoder_t *encd);
float ENCODER_GetRPM(Encoder_t *encd, uint32_t dt_us);
float ENCODER_GetActualDegree(Encoder_t *encd);
void Encoder_Init_AutoDetect(Encoder_t *enc, EncoderLocation_t loc, Encoder_Port_t *port);

#endif /* ENCODER_H */
#include "encoder.h"

#include <string.h> // cho NULL


Encoder_t encoder;

AS5048A_t as5048a;
_Bool encd_get_val_flag;


int ENCODER_Setup(){
	encoder.enc_internal.hspi = &hspi1;
	encoder.enc_internal.cs_port = GPIOB;
	encoder.enc_internal.cs_pin = GPIO_PIN_9;
											
	encoder.enc_external.hspi = &hspi2;
	encoder.enc_external.cs_port = GPIOB;
	encoder.enc_external.cs_pin = GPIO_PIN_11;

	
	ENCODER_Reset_Flag();
	return 0;
}

int ENCODER_AutoDetect(){
	encoder.location = ENCODER_LOC_INTERNAL;
	encoder.type = ENCODER_TYPE_AS5048A;
	AS5048A_Config(&as5048a,encoder. enc_internal.hspi, encoder.enc_internal.cs_port, encoder.enc_internal.cs_pin);
	if(AS5048A_DetectExit(&as5048a)){

		encoder.hw_encoder = (void*)&as5048a;
		encoder.start_read = AS5048A_StartRead;
		encoder.parse_data = AS5048A_ParseData;
		ENCODER_Set_Flag();
		return 0;
	}
		
	encoder.location = ENCODER_LOC_EXTERNAL;
	encoder.type = ENCODER_TYPE_AS5048A;
	AS5048A_Config(&as5048a,encoder. enc_external.hspi, encoder.enc_external.cs_port, encoder.enc_external.cs_pin);
	if(AS5048A_DetectExit(&as5048a)){
		encoder.hw_encoder = (void*)&as5048a;
		encoder.start_read = AS5048A_StartRead;
		encoder.parse_data = AS5048A_ParseData;
		ENCODER_Set_Flag();
		return 0;
	}
	
	encoder.type = ENCODER_TYPE_NONE;
	return 1;
}

float ENCODER_GetDegree(Encoder_t *encd){
	encd->raw_angle = encd->parse_data(encd->hw_encoder);
	
	float angle_diff = encd->raw_angle - encd->prev_raw_angle;
	angle_diff -= 360.0f * floorf((angle_diff + 180.0f) / 360.0f);

	if (fabsf(angle_diff) > MAX_ANGLE_JUMP_DEG) {
		if (++encd->spike_counter < SPIKE_REJECT_COUNT) {
			return encd->angle_filtered;
		}
		encd->spike_counter = 0;
	} else {
		encd->spike_counter = 0;
	}

	encd->prev_raw_angle = encd->raw_angle;

	// Filter IIR dengan wrap-around
	float filtered_diff = encd->raw_angle - encd->angle_filtered;
	filtered_diff -= 360.0f * floorf((filtered_diff + 180.0f) / 360.0f);
	encd->angle_filtered += ANGLE_FILTER_ALPHA * filtered_diff;

	if (encd->angle_filtered >= 360.0f)
		encd->angle_filtered -= 360.0f;
	else if (encd->angle_filtered < 0.0f)
		encd->angle_filtered += 360.0f;

	return encd->angle_filtered;
	
};
//

float ENCODER_GetRPM(Encoder_t *encd, uint32_t dt_us) {
	// Handle angle wrap-around (optimized)
	float angle_diff = encd->angle_filtered - encd->prev_angle;
	angle_diff -= 360.0f * floorf((angle_diff + 180.0f) * (1.0f/360.0f));
	encd->prev_angle = encd->angle_filtered;
	if (dt_us == 0) {
        
        return encd->filtered_rpm; 
    }

#if 0
	// Accumulate angle and time for low-RPM precision
	encd->angle_accumulator += angle_diff;
	encd->time_accumulator += dt_us;

	// Only calculate RPM when sufficient data is collected
	if (encd->time_accumulator < MIN_DT_US && encd->filtered_rpm > 1.0f) {
			return encd->filtered_rpm;
	}

	// Calculate RPM (optimized floating point)
	float rpm_instant = (encd->angle_accumulator * MICROS_TO_MINUTES) /
											(encd->time_accumulator * DEGREES_PER_REV);

	// Reset accumulators
	encd->angle_accumulator = 0.0f;
	encd->time_accumulator = 0;
#else
	// Calculate RPM (optimized floating point)
	float rpm_instant = (angle_diff * MICROS_TO_MINUTES) / (dt_us * DEGREES_PER_REV);
#endif
	// Two-stage spike rejection
	float rpm_delta = rpm_instant - encd->prev_rpm;
	float abs_delta = fabsf(rpm_delta);

	if (abs_delta > MAX_RPM_JUMP) {
			// Gradual rejection with 50% of the delta, capped at MAX_RPM_JUMP
			float limited_delta = copysignf(fminf(abs_delta * 0.5f, MAX_RPM_JUMP), rpm_delta);
			rpm_instant = encd->prev_rpm + limited_delta;
	}

	// IIR Filter with dynamic weighting
	float filtered = encd->filtered_rpm * (1.0f - RPM_FILTER_ALPHA) + rpm_instant * RPM_FILTER_ALPHA;

	// Very low RPM clamping (0.1 RPM resolution)
	if (fabsf(filtered) < 0.1f) {
			filtered = 0.0f;
	}

	// Update state
	encd->prev_rpm = rpm_instant;
	encd->filtered_rpm = filtered;

	return encd->filtered_rpm;
}
//

float ENCODER_GetActualDegree(Encoder_t *encd) {
    const float m_current_angle = encd->angle_filtered;
	float angle_dif = (m_current_angle - encd->output_prev_angle);

	if (angle_dif< -180) {
		encd->output_angle_ovf++;
	}
	else if (angle_dif> 180) {
		encd->output_angle_ovf--;
	}
	float out_deg = (m_current_angle + encd->output_angle_ovf * 360.0);
  encd->output_angle_filtered = (1.0f - ACTUAL_ANGLE_FILTER_ALPHA) * encd->output_angle_filtered + ACTUAL_ANGLE_FILTER_ALPHA * out_deg;
	encd->output_prev_angle = m_current_angle;

	// return out_deg;
    return encd->output_angle_filtered;
}

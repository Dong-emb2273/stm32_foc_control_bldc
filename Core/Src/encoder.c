#include "encoder.h"
#include "flash.h"
#include "user_config.h"
#include <string.h> // cho NULL


Encoder_t encoder;


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
	// 1. Thử dò tìm ở cổng Internal
	encoder.location = ENCODER_LOC_INTERNAL;
	encoder.active_port = &encoder.enc_internal; // Gắn active_port vào internal
	encoder.type = ENCODER_TYPE_AS5048A;
	
	AS5048A_Config_CS(&encoder); 
	
	if(AS5048A_DetectExit(&encoder)){
		encoder.start_read = AS5048A_StartRead;
		encoder.parse_data = AS5048A_ParseData;
		ENCD_LOCATION = ENCODER_LOC_INTERNAL; 
		ENCD_TYPE = ENCODER_TYPE_AS5048A; 
		ENCODER_Set_Flag();
		encoder.is_connected = 1;
		return 0;
	}
		
	// 2. Thử dò tìm ở cổng External nếu Internal không có
	encoder.location = ENCODER_LOC_EXTERNAL;
	encoder.active_port = &encoder.enc_external; 
	encoder.type = ENCODER_TYPE_AS5048A;
	
	AS5048A_Config_CS(&encoder);
	
	if(AS5048A_DetectExit(&encoder)){
		encoder.start_read = AS5048A_StartRead;
		encoder.parse_data = AS5048A_ParseData;
		ENCD_LOCATION = ENCODER_LOC_EXTERNAL; 
		ENCD_TYPE = ENCODER_TYPE_AS5048A; 
		ENCODER_Set_Flag();
		encoder.is_connected = 1;
		return 0;
	}
	
	encoder.type = ENCODER_TYPE_NONE;
	encoder.is_connected = 0; 
	return 1;
}

void ENCODER_CHECK(){
	if(encoder.type == ENCODER_TYPE_NONE){
		printf("No Encoder Detected\r\n");
		return;
	}
	if(encoder.type == ENCODER_TYPE_AS5048A){
		printf("AS5048A Detected\r\n");
	}
	if(encoder.location == ENCODER_LOC_INTERNAL){
		printf("Encoder Location: INTERNAL\r\n");
	}
	else if(encoder.location == ENCODER_LOC_EXTERNAL){
		printf("Encoder Location: EXTERNAL\r\n");
	}
	static uint16_t first_check = 1;
	for (int i = 0; i < 1000; i++) {
		if(encoder.parse_data(&encoder) < 0.0f){
			first_check ++;
		}
		osDelay(1);
	}
	
	printf("Encoder Check: %d / 1000 failed reads\r\n", first_check);
	first_check = 0;
}

void ENCODER_Init_From_Config(void) {
    // 1. Phục hồi Vị trí cắm (Location) và gán cổng SPI tương ứng
    if (ENCD_LOCATION == ENCODER_LOC_INTERNAL) {
        encoder.location = ENCODER_LOC_INTERNAL;
        encoder.active_port = &encoder.enc_internal; 
    } else {
        encoder.location = ENCODER_LOC_EXTERNAL;
        encoder.active_port = &encoder.enc_external;
    }

    // 2. PHỤC HỒI CÁC CON TRỎ HÀM (BINDING)
    encoder.type = ENCD_TYPE;
    
    switch (encoder.type) {
        case ENCODER_TYPE_AS5048A:
            // Khởi tạo chân CS
            AS5048A_Config_CS(&encoder);
            
            
            encoder.start_read = AS5048A_StartRead;
            encoder.parse_data = AS5048A_ParseData;
            
            ENCODER_Set_Flag(); // 
            break;
            
        case ENCODER_TYPE_MT6835:
            // (Ví dụ sau này bạn thêm cảm biến khác)
            // MT6835_Config_CS(&encoder);
            // encoder.start_read = MT6835_StartRead;
            // encoder.parse_data = MT6835_ParseData;
            break;
            
        case ENCODER_TYPE_NONE:
        default:
            // Không có cảm biến, gán hàm rỗng (Dummy function) để chống lỗi chia cho 0 hoặc Null Pointer
            encoder.start_read = NULL; 
            encoder.parse_data = NULL;
            ENCODER_Reset_Flag();
            break;
    }
}

float ENCODER_GetDegree(Encoder_t *encd){
	if (encd->parse_data == NULL) {
		return 0.0f; 
	}
	encd->raw_angle = encd->parse_data(encd);

	if (encd->raw_angle < 0.0f) {
		encd->error_count++;
        if (encd->error_count > 30) {
            encd->is_connected = 0; 
        }
		return encd->angle_filtered; 
	}

	encd->error_count = 0;
    encd->is_connected = 1;

	// Spike rejection (optimized)
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

float ENCODER_GetRPM(Encoder_t *encd, float dt_us) {
	// Handle angle wrap-around (optimized)
	float angle_diff = encd->angle_filtered - encd->prev_angle;
	angle_diff -= 360.0f * floorf((angle_diff + 180.0f) * (1.0f/360.0f));
	encd->prev_angle = encd->angle_filtered;
	if (dt_us == 0) {
        
        return encd->filtered_rpm; 
    }
	// Calculate RPM (optimized floating point)
	float rpm_instant = (angle_diff * 60.0f) / (dt_us * DEGREES_PER_REV);

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

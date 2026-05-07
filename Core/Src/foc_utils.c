/*
 * FOC_utils.c
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */
 
#include <stdint.h>
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "foc_utils.h"
#include "user_config.h"
#include "flash.h"
#include "encoder.h"

extern motor_config_t m_config;

_Bool foc_ready = 0;


float error_temp[ERROR_LUT_SIZE] = {0};

/**
 * @brief Initialize PWM output pointers for FOC controller
 * @param hfoc Pointer to FOC controller structure
 * @param pwm_a Pointer to PWM channel A register (e.g., &TIM1->CCR1)
 * @param pwm_b Pointer to PWM channel B register (e.g., &TIM1->CCR2)
 * @param pwm_c Pointer to PWM channel C register (e.g., &TIM1->CCR3)
 */
void foc_pwm_init(foc_t *hfoc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c, uint32_t pwm_res) {
    // Validate pointers
    if(hfoc == NULL || pwm_a == NULL || pwm_b == NULL || pwm_c == NULL) {
        // Error handling (could be an assertion or error code)
        return;
    }

    hfoc->pwm_a = pwm_a;
    hfoc->pwm_b = pwm_b;
    hfoc->pwm_c = pwm_c;
    hfoc->pwm_res = pwm_res;

    // Optional: Initialize PWM values to 0
    *hfoc->pwm_a = 0;
    *hfoc->pwm_b = 0;
    *hfoc->pwm_c = 0;
}

//

void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv) {
	if (hfoc == NULL || pole_pairs == 0 || kv <= 0) {
		return;
	}

	// hfoc->pole_pairs = pole_pairs;
	// hfoc->kv = kv;
}

//

void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir) {
	if (hfoc == NULL) return;

	// hfoc->m_angle_offset = m_rad_offset;
	
}

//


//

void foc_set_limit_current(foc_t *hfoc, float i_limit) {
	if (hfoc == NULL) return;

	hfoc->max_current = i_limit;
}

//

void foc_MTPA(foc_t *hfoc, float Is, float *Id_ref, float *Iq_ref) {
    float L_diff = L_Q - L_D;
    if (L_diff < 0) {
        *Id_ref = 0.0f;
        *Iq_ref = Is;
        return;
    }
    float Is_square = Is * Is;
    float temp = sqrtf(hfoc->flux_linkage * hfoc->flux_linkage + 8.0f * (L_diff * L_diff) * Is_square);
    float id = (hfoc->flux_linkage - temp) / (4.0f * L_diff);
    float iq = sqrtf(Is_square - id * id);
    if (Is < 0) iq = -iq;

    *Id_ref = id;
    *Iq_ref = iq;
}

void foc_fw_set_vs_ref(foc_t *hfoc, float vs_ref) {
    hfoc->fw_vs_ref = vs_ref;
}

float foc_fw_update(foc_t *hfoc) {
    float vs = sqrtf(hfoc->vd * hfoc->vd + hfoc->vq * hfoc->vq);
    float error = hfoc->fw_vs_ref - vs;
    float mv = pi_control(&hfoc->fw_ctrl, error);
    return mv;
}

void foc_current_limit(float *id_ref, float *iq_ref, float max_current) {
    if (*id_ref > max_current) {
        *id_ref = max_current;
    }
    else if (*id_ref < -max_current) {
        *id_ref = -max_current;
    }
    float max_iq = sqrtf(max_current * max_current - *id_ref * *id_ref);
    if (*iq_ref > max_iq) {
        *iq_ref = max_iq;
    }
    else if (*iq_ref < -max_iq) {
        *iq_ref = -max_iq;
    }
}

//
int foc_torque_control_update(foc_t *hfoc ) {
    int ret = 0;
    static uint8_t event_speed_loop_count = 0;
    static float rpm_temp = 0.0f;
        
    foc_current_control_update(hfoc);
    

    if (event_speed_loop_count >= SPEED_CONTROL_CYCLE) {
        event_speed_loop_count = 0;
        
        hfoc->actual_rpm = ENCODER_GetRPM(&encoder, FOC_TS * SPEED_CONTROL_CYCLE);
        
        foc_set_flag();
        ret = 1;
    }

    event_speed_loop_count++;

    return ret;
}


void foc_current_control_update(foc_t *hfoc) {
	if (hfoc == NULL ) {
		hfoc->id_ctrl.integral = 0.0f;
		hfoc->id_ctrl.last_error = 0.0f;
		hfoc->iq_ctrl.integral = 0.0f;
		hfoc->iq_ctrl.last_error = 0.0f;
		return;
	}

    DRV8323_Get_Current(&hfoc->drv8323s, &hfoc->ia, &hfoc->ib,&hfoc->ic);

	// // Hard limit references
	// hfoc->id_ref = CONSTRAIN(hfoc->id_ref, -hfoc->max_current, hfoc->max_current);
	// hfoc->iq_ref = CONSTRAIN(hfoc->iq_ref, -hfoc->max_current, hfoc->max_current);

// 1. Giới hạn lực kéo (I_ref)
    float target_id = CONSTRAIN(hfoc->id_ref, -hfoc->max_current, hfoc->max_current);
    float target_iq = CONSTRAIN(hfoc->iq_ref, -hfoc->max_current, hfoc->max_current);

    // 2. LẬT PHA ĐIỆN TỪ Ở ĐÂY (Điều hướng hệ quy chiếu)
    // Nếu bị đấu ngược dây pha, ta lật ngược lệnh dòng điện Iq
    if (DIR_PHASE == REVERSE_DIR) {
        target_iq = -target_iq; 
        
    }



	// pre calculate sin & cos
	float sin_theta, cos_theta;
	pre_calc_sin_cos(hfoc->e_angle_rad_comp, &sin_theta, &cos_theta);

	// Get measured currents
	clarke_park_transform(hfoc->ia, hfoc->ib, sin_theta, cos_theta, &hfoc->id, &hfoc->iq);

	const float alpha_i_filt = 0.1f;
	hfoc->id_filtered = (1.0f - alpha_i_filt) * hfoc->id_filtered + alpha_i_filt * hfoc->id;
	hfoc->iq_filtered = (1.0f - alpha_i_filt) * hfoc->iq_filtered + alpha_i_filt * hfoc->iq;

	// Continue normal FOC
    hfoc->vd = pi_control(&hfoc->id_ctrl, target_id - hfoc->id_filtered);
    hfoc->vq = pi_control(&hfoc->iq_ctrl, target_iq - hfoc->iq_filtered);


	float valpha, vbeta;
	uint32_t da, db, dc;
	inverse_park_transform(hfoc->vd, hfoc->vq, sin_theta, cos_theta, &valpha, &vbeta);
	svpwm(valpha, vbeta, hfoc->v_bus, hfoc->pwm_res, &da, &db, &dc);

	// pwm limit
	*(hfoc->pwm_a) = CONSTRAIN(da, 0, hfoc->pwm_res);
	*(hfoc->pwm_b) = CONSTRAIN(db, 0, hfoc->pwm_res);
	*(hfoc->pwm_c) = CONSTRAIN(dc, 0, hfoc->pwm_res);
}
//

void foc_speed_control_update(foc_t *hfoc, float rpm_reference) {
	if (hfoc == NULL || (hfoc->control_mode != SPEED_CONTROL_MODE && hfoc->control_mode != POSITION_CONTROL_MODE)) {
		hfoc->speed_ctrl.integral = 0.0f;
		hfoc->speed_ctrl.last_error = 0.0f;
		return;
	}

    hfoc->id_ref = 0.0f;
    hfoc->iq_ref = pi_control(&hfoc->speed_ctrl, rpm_reference - hfoc->actual_rpm);
}
//

void foc_position_control_update(foc_t *hfoc, float deg_reference) {
	if (hfoc == NULL || hfoc->control_mode != POSITION_CONTROL_MODE) {
		hfoc->pos_ctrl.integral = 0.0f;
		hfoc->pos_ctrl.last_error = 0.0f;
		return;
	}
#if 1

//    const float min_speed = 3000.0f;
//    const float deadband_rpm = 0.5f;
	float error = deg_reference - hfoc->actual_angle;
    hfoc->rpm_ref = pid_control(&hfoc->pos_ctrl, error);

    foc_speed_control_update(hfoc, hfoc->rpm_ref);
#else
 
#endif
}
//

extern _Bool sensor_is_calibrated;

void foc_sensored_calc_electric_angle(foc_t *hfoc) {
	// Check for NULL pointer and invalid parameters
	if (hfoc == NULL || PPAIRS <= 0) {
			return;
	}

	float angle_deg = ENCODER_GetDegree(&encoder);

	// Normalize mechanical angle
	hfoc->m_angle_rad = DEG_TO_RAD(angle_deg) - ENCDER_OFFSET;
	norm_angle_rad(&hfoc->m_angle_rad);

	// Calculate raw electric angle
	float e_rad = hfoc->m_angle_rad * PPAIRS;
	
	// Handle sensor direction
	if (DIR_PHASE == REVERSE_DIR) {
			e_rad = TWO_PI - e_rad;
	}

	hfoc->e_angle_rad = e_rad;

	// Calculate LUT index with wrap-around
	float lut_idx_f = (hfoc->m_angle_rad / TWO_PI) * ERROR_LUT_SIZE;
	lut_idx_f = fmodf(lut_idx_f, ERROR_LUT_SIZE);
	if (lut_idx_f < 0) {
			lut_idx_f += ERROR_LUT_SIZE;
	}

	// Get neighboring indices with wrap-around
	int idx0 = (int)lut_idx_f % ERROR_LUT_SIZE;
	int idx1 = (idx0 + 1) % ERROR_LUT_SIZE;
	float frac = lut_idx_f - (float)idx0;

	// Linear interpolation
	float encoder_error = m_config.encd_error_comp[idx0] * (1.0f - frac) + m_config.encd_error_comp[idx1] * frac;
	e_rad += encoder_error;
	
	// Normalize final electric angle
	norm_angle_rad(&e_rad);

	hfoc->e_angle_rad_comp = e_rad;
	
	ENCODER_Set_Flag();
}
//

float foc_calc_mech_rpm_encoder(foc_t *hfoc, float encd_rpm) {
	// if (DIR_PHASE == REVERSE_DIR) {
	// 		hfoc->actual_rpm = -encd_rpm;
	// }
	// else {
	// 		hfoc->actual_rpm = encd_rpm;
	// }
    hfoc->actual_rpm = encd_rpm;
	return hfoc->actual_rpm;
}
//

float foc_calc_mech_pos_encoder(foc_t *hfoc, float encd_deg) {
	// if (DIR_PHASE == REVERSE_DIR) {
	// 	hfoc->actual_angle = -encd_deg;
	// }
	// else {
	// 	hfoc->actual_angle = encd_deg;
	// }
    hfoc->actual_angle = encd_deg;
	return hfoc->actual_angle;
}
//

void foc_cal_encoder_misalignment(foc_t *hfoc) {
    open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
    osDelay(500);
    float rad_offset = 0.0f;
    for (int i = 0; i < CAL_ITERATION; i++) {
        rad_offset += DEG_TO_RAD(*hfoc->angle_filtered);
        osDelay(1);
    }
    open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
    rad_offset = rad_offset / (float)CAL_ITERATION;
    //hfoc->m_angle_offset = rad_offset;
    ENCDER_OFFSET = rad_offset;
    printf("Encoder Mechanical Offset (rad): %f\r\n", ENCDER_OFFSET);

}
//

void foc_cal_encoder(foc_t *hfoc) {
    memset(error_temp, 0, sizeof(error_temp));

    printf("Starting Encoder Calibration Task...\r\n");
    foc_cal_encoder_misalignment(hfoc);

    for (int i = 0; i < ERROR_LUT_SIZE; i++) {
        float mech_deg = (float)i * (360.0f / (float)ERROR_LUT_SIZE);
        float elec_rad = DEG_TO_RAD(mech_deg * PPAIRS);
        open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, elec_rad);
        osDelay(5);

        float mech_rad = hfoc->m_angle_rad;
        float raw_delta = elec_rad - hfoc->e_angle_rad;
        float delta = elec_rad - hfoc->e_angle_rad_comp;
        
        raw_delta -= TWO_PI * floorf((raw_delta + PI) / TWO_PI);
        delta -= TWO_PI * floorf((delta + PI) / TWO_PI);
        
        float lut_pos = (mech_rad / TWO_PI) * ERROR_LUT_SIZE;
        int index = (int)(lut_pos);

        while (index < 0) {
        index += ERROR_LUT_SIZE;
        }
        index %= ERROR_LUT_SIZE;

        error_temp[index] = raw_delta;
    

    }
    
    for (int i = 0; i < ERROR_LUT_SIZE; i++) {
        if (error_temp[i] == 0) {
        int last_i = i - 1;
        int next_i = i + 1;
        if (last_i < 0) last_i += ERROR_LUT_SIZE;
        if (next_i > ERROR_LUT_SIZE) next_i -= ERROR_LUT_SIZE;
        error_temp[i] = (error_temp[last_i] + error_temp[next_i]) / 2.0f;
        }
    }

    memcpy(m_config.encd_error_comp, error_temp, sizeof(error_temp));

    open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);

    hfoc->done_cal_encoder = 1;
}
//


void foc_start_calibration(foc_t *hfoc) {
    hfoc->done_orderphase = 0;
    hfoc->done_cal_encoder = 0;
 
}





void foc_auto_calibration(foc_t *hfoc) {
    printf("Starting Order Phase Calibration Task...\r\n");

    const float SWEEP_CYCLES = 20.0f; 
    const float W_CAL = TWO_PI / 0.2f; 
    const float SWEEP_TIME_SEC = (SWEEP_CYCLES * TWO_PI) / W_CAL; 
    const uint32_t step_delay_ms = 5; 
    const uint32_t total_steps = (uint32_t)(SWEEP_TIME_SEC * 1000.0f / step_delay_ms);

    // ==========================================
   for(int i = 0; i < 50; i++) {
        ENCODER_GetActualDegree(&encoder);
        osDelay(1); 
    }

    open_loop_voltage_control(hfoc, VD_CAL, 0.0f, 0.0f);

    osDelay(1000); 
    
    // ==========================================
    encoder.output_angle_ovf = 0;
    encoder.output_prev_angle = encoder.angle_filtered; // Góc thô sau khi lọc nội bộ
    encoder.output_angle_filtered = encoder.angle_filtered; // Ép biến actual nhảy thẳng đến đây


    float actual_theta_start = ENCODER_GetActualDegree(&encoder);
    
    
    for (uint32_t i = 0; i < total_steps; i++) {
        float elapsed_sec = (float)i * step_delay_ms / 1000.0f; 
        float elec_angle = W_CAL * elapsed_sec;
        open_loop_voltage_control(hfoc, VD_CAL, 0.0f, elec_angle);
        ENCODER_GetActualDegree(&encoder); // Gọi liên tục để bù tràn
        osDelay(step_delay_ms); 
    }

  
    float actual_theta_end = ENCODER_GetActualDegree(&encoder);
    open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f); // Tắt áp
    

    float delta_mech = actual_theta_end - actual_theta_start;
    
    if (fabs(delta_mech) > 0.1f) {
        PPAIRS = round((SWEEP_CYCLES * 360.0f) / fabs(delta_mech));
        printf("Pole Pairs: %d\r\n", PPAIRS);
    }
    
   
    if (delta_mech > 0) {
        DIR_PHASE = NORMAL_DIR;
        printf("Phase order & Sensor match (NORMAL)\r\n");
    } else {
        DIR_PHASE = REVERSE_DIR;
        printf("Phase order & Sensor mismatched! Swapping (REVERSE)\r\n");
    }

    hfoc->done_orderphase = 1; 
    printf("Calibration Complete!\r\n");
}


//

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad) {
    float valpha, vbeta;
    uint32_t da, db, dc;
    const uint32_t pwm_res = hfoc->pwm_res;

    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &valpha, &vbeta);
    svpwm(valpha, vbeta, 12.0, pwm_res, &da, &db, &dc);

    *(hfoc->pwm_a) = CONSTRAIN(da, 0, pwm_res);
    *(hfoc->pwm_b) = CONSTRAIN(db, 0, pwm_res);
    *(hfoc->pwm_c) = CONSTRAIN(dc, 0, pwm_res);
}
//

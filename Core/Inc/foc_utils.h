/*
 * FOC_utils.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_UTILS_H_
#define FOC_INC_FOC_UTILS_H_

#include <stdint.h>
#include <string.h>
#include "drv8323s.h"
#include "foc_math.h"
#include "pid_utils.h"
#include "as5048a.h"


#define ERROR_LUT_SIZE (1024)

#define CAL_ITERATION 1000

#define VD_CAL 0.5f
#define VQ_CAL 0.0f

#define is_foc_ready() (foc_ready)
#define foc_reset_flag() (foc_ready = 0)
#define foc_set_flag() (foc_ready = 1)

#define MAX_I_SAMPLE 128

#define MAX_SAMPLE_BUFF 1024

#define DEBUG_HFI	0

/* extern variable */
extern _Bool foc_ready;
extern float Vd_buff[MAX_I_SAMPLE];
extern float Vq_buff[MAX_I_SAMPLE];
extern float Id_buff[MAX_I_SAMPLE];
extern float Iq_buff[MAX_I_SAMPLE];



typedef enum {
	TORQUE_CONTROL_MODE,
	SPEED_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	CALIBRATION_MODE1,
	AUDIO_MODE,
	TEST_MODE,
	POWER_UP_MODE,
}motor_mode_t;

typedef enum {
	NORMAL_DIR, REVERSE_DIR
}dir_mode_t;

typedef enum {
  RS, LD, LQ
}inject_taregt_t;

// state machine for HFI
typedef enum {
	MOTOR_STATE_HFI,
	MOTOR_STATE_SMO
}motor_state_t;

typedef enum {
	STARTUP_IDLE, STARTUP_ALIGN, STARTUP_OPEN_LOOP_RAMP
}startup_state_t;

typedef enum {
    CAL_IDLE = 0,
    CAL_ALIGNING,
    CAL_SWEEPING,
    CAL_DONE
} cal_state_t;

typedef struct {
	
//	AS5048A_t AS5048A;
	
	uint8_t pole_pairs;
	float kv;
	float Rs;
	float Ld;
	float Lq;
	float max_current;


	float m_angle_rad; 
	float e_angle_rad; 
	float e_angle_rad_comp; 
	float m_angle_offset;
	float e_rad;
	float last_e_rad;


	float vd, vq;
	float id, iq;
	float id_filtered, iq_filtered;
	float v_alpha, v_beta;
	float i_alpha, i_beta;
	float va, vb, vc;
	float ia, ib, ic;
	float v_bus;
	float i_bus;

	float rpm_temp;
	float actual_rpm;
	float actual_angle;
	int32_t m_angle_overflow_count;

	float I_ctrl_bandwidth;
	float id_ref, iq_ref;
	float rpm_ref;

    // uint8_t loop_count;

	volatile uint32_t *pwm_a;
	volatile uint32_t *pwm_b;
	volatile uint32_t *pwm_c;
	uint32_t pwm_res;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;
	PID_Controller_t pos_ctrl;
	
	float sp_pos, sp_vel, sp_iq, kp, kd;

	motor_mode_t control_mode;

	float gear_ratio;
	dir_mode_t sensor_dir;
	float *angle_filtered;



	//debug
	int sample_index;
	_Bool collect_sample_flag;
}foc_t;

void foc_pwm_init(foc_t *hfoc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c, uint32_t pwm_res);

void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv);

void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir);

void foc_gear_reducer_init(foc_t *hfoc, float ratio);

void foc_set_limit_current(foc_t *hfoc, float i_limit);

void foc_current_control_update(foc_t *hfoc);

void foc_speed_control_update(foc_t *hfoc, float rpm_reference);

void foc_position_control_update(foc_t *hfoc, float deg_reference);

float foc_calc_mech_rpm_encoder(foc_t *hfoc, float encd_rpm);

float foc_calc_mech_pos_encoder(foc_t *hfoc, float encd_deg);

void foc_sensored_calc_electric_angle(foc_t *hfoc);

void foc_start_calibration(void);

void foc_auto_calibration_update(foc_t *hfoc);

void foc_cal_encoder_misalignment(foc_t *hfoc);

void foc_cal_encoder(foc_t *hfoc);

void foc_set_torque_control_bandwidth(foc_t *hfoc, float bandwidth);

float foc_get_mech_degree(foc_t *hfoc);

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);

#endif /* FOC_INC_FOC_UTILS_H_ */
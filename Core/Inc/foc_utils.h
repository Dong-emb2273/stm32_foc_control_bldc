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
#include "as5048a.h"
#include "foc_math.h"
#include "pid_utils.h"
#include "joint_acc.h"
#include "lpf.h"



#define ERROR_LUT_SIZE (1024)

#define CAL_ITERATION 1000

#define VD_CAL I_CAL
#define VQ_CAL 0.0f

#define is_foc_ready() (foc_ready)
#define foc_reset_flag() (foc_ready = 0)
#define foc_set_flag() (foc_ready = 1)

#define MAX_I_SAMPLE 128

#define MAX_SAMPLE_BUFF 1024

/* LPF Cut-off frequency for Id and Iq */
#define HFI_ID_LPF_FC 200.0f
#define HFI_IQ_LPF_FC 200.0f

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
	IMPEDANCE_CONTROL_MODE,
	// CALIBRATION_MODE1,
	// AUDIO_MODE,
	// TEST_MODE,
	// POWER_UP_MODE,
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
	DRV8323_t drv8323s;

	
	float max_current;
	float flux_linkage;

	float meas_inj_freq;
	float meas_inj_amp;
	float meas_inj_omega;
	inject_taregt_t meas_inj_target;
	int meas_inj_n;
	_Bool meas_inj_start_flag;



	float m_angle_rad; 
	float e_angle_rad; 
	float e_angle_rad_comp; 

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


	float id_ref, iq_ref;
	float rpm_ref;

  

	volatile uint32_t *pwm_a;
	volatile uint32_t *pwm_b;
	volatile uint32_t *pwm_c;
	uint32_t pwm_res;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;
	PID_Controller_t pos_ctrl;

	//field weakening
	PID_Controller_t fw_ctrl;
	float fw_vs_ref;
	_Bool fw_enable;

	SecondOrderLPF id_lpf;
	SecondOrderLPF iq_lpf;

	
	float sPoint_Pos, sPoint_Vel, sPoint_Tor, kp, kd;

	motor_mode_t control_mode;


	float *angle_filtered;

	uint8_t done_orderphase;
	uint8_t done_cal_encoder;
	
	uint16_t counter;

}foc_t;

extern foc_t hfoc; 

void foc_pwm_init(foc_t *hfoc, volatile uint32_t *pwm_a, volatile uint32_t *pwm_b, volatile uint32_t *pwm_c, uint32_t pwm_res);

void foc_motor_init(foc_t *hfoc, uint8_t pole_pairs, float kv);

void foc_sensor_init(foc_t *hfoc, float m_rad_offset, dir_mode_t sensor_dir);

void foc_gear_reducer_init(foc_t *hfoc, float ratio);

void foc_set_limit_current(foc_t *hfoc, float i_limit);

void foc_MTPA(foc_t *hfoc, float Is, float *Id_ref, float *Iq_ref);

void foc_fw_set_vs_ref(foc_t *hfoc, float vs_ref);

float foc_fw_update(foc_t *hfoc);

void foc_current_limit(float *id_ref, float *iq_ref, float max_current);

int foc_get_power_voltage(foc_t *hfoc);

void foc_get_v_phase(foc_t *hfoc);

void foc_current_control_update(foc_t *hfoc);

int foc_torque_control_update(foc_t *hfoc);

void foc_speed_control_update(foc_t *hfoc, float rpm_reference);

void foc_position_control_update(foc_t *hfoc, float deg_reference);

void foc_impedance_control(JointCommand_t *cmd, foc_t *hfoc);

void foc_control_loop(foc_t *hfoc);

float foc_calc_mech_rpm_encoder(foc_t *hfoc, float encd_rpm);

float foc_calc_mech_pos_encoder(foc_t *hfoc, float encd_deg);

void foc_sensored_calc_electric_angle(foc_t *hfoc);

void foc_start_calibration(foc_t *hfoc);

void foc_auto_calibration(foc_t *hfoc);


void meas_inj_dq_process(foc_t *hfoc, float ts);
void estimate_resistance(foc_t *hfoc);
void estimate_inductance(foc_t *hfoc, float ts);
void start_measure(inject_taregt_t target);
int measure_R(float vdc);
int measure_L(float f, float amp);
void calibration_seq(void);


void foc_cal_encoder_misalignment(foc_t *hfoc);

void foc_cal_encoder(foc_t *hfoc);

void foc_set_torque_control_bandwidth(foc_t *hfoc, float bandwidth);

float foc_get_mech_degree(foc_t *hfoc);

void motor_turn_off(foc_t *hfoc);

void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);

#endif /* FOC_INC_FOC_UTILS_H_ */
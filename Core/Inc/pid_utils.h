#ifndef _PID_H_
#define _PID_H_

#include <stdint.h>


typedef struct {
    float kp;        // Proportional gain
    float ki;        // Integral gain
    float kd;        // Differential gain
    float integral;  // Integral accumulator
    float out_max;   // Output upper limit (clamp)
    float out_max_dynamic;
    float ts;
    float e_deadband;
    float last_error;
    float d_alpha_filter; // Derivative filter coefficient
    float d_filtered;
    float mv;
} PID_Controller_t;

float pi_control(PID_Controller_t *pi, float error);
float pd_control(PID_Controller_t *pd, float error);
float pid_control(PID_Controller_t *pid, float error);
void pid_reset_integrator(PID_Controller_t *pid);
void pid_init(PID_Controller_t *pid, float kp, float ki, float kd, float Ts, float out_max, float deadband);

#endif /* PID_H_ */
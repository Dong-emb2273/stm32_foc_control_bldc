#ifndef IMPEDANCE_CONTROL_H_
#define IMPEDANCE_CONTROL_H_

#include <stdint.h>

typedef struct
{
  float Kp;
  float Kd;
  float max_out;
}impedance_t;

float impedance_control_update(impedance_t *imp, float error, float error_dot, float cff);
void impedance_set_stiffness(impedance_t *imp, float kp);
float impedance_get_stiffness(impedance_t *imp);
void impedance_set_damping(impedance_t *imp, float kd);
float impedance_get_damping(impedance_t *imp);
void impedance_set_max_out(impedance_t *imp, float max);
float impedance_get_max_out(impedance_t *imp);

#endif /* IMPEDANCE_CONTROL_H_ */
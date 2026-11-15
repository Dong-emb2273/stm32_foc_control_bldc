#include "impedance_control.h"

float impedance_control_update(impedance_t *imp, float error, float error_dot, float cff) {
  float curr = imp->Kp * error + imp->Kd * error_dot + cff;
  if (curr > imp->max_out) curr = imp->max_out;
  else if (curr < -imp->max_out) curr = -imp->max_out;
  return curr;
}

void impedance_set_stiffness(impedance_t *imp, float kp) {
  imp->Kp = kp;
}

float impedance_get_stiffness(impedance_t *imp) {
  return imp->Kp;
}

void impedance_set_damping(impedance_t *imp, float kd) {
  imp->Kd = kd;
}

float impedance_get_damping(impedance_t *imp) {
  return imp->Kd;
}

void impedance_set_max_out(impedance_t *imp, float max) {
  imp->max_out = max;
}

float impedance_get_max_out(impedance_t *imp) {
  return imp->max_out;
}

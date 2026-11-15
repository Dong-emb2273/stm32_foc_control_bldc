#include "pll_filter.h"


void pll_filter_init(pll_filter_t *hf, float Ts, float bandwidth_hz, float damping_ratio) {
  hf->Ts = Ts;
  float omega = TWO_PI * bandwidth_hz;
  hf->kp = 2.0f * omega * damping_ratio;
  hf->ki = omega * omega;
}

void pll_filter_update(pll_filter_t *hf, float position_in) {
  // phase detector
  float error = position_in - hf->estimate_position;

  // predict position and velocity
  hf->estimate_position += (hf->estimate_velocity + hf->kp * error) * hf->Ts;
  hf->estimate_velocity += hf->ki * error * hf->Ts;
}

float pll_filter_get_estimate_position(pll_filter_t *hf) {
  return hf->estimate_position;
}

float pll_filter_get_estimate_velocity(pll_filter_t *hf) {
  return hf->estimate_velocity;
}


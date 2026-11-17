#ifndef PLL_FILTER_H_
#define PLL_FILTER_H_

#include <stdint.h>

#ifndef TWO_PI
#define TWO_PI 6.2831853f
#endif

typedef struct {
  float estimate_position;
  float estimate_velocity;
  float Ts;
  float kp;
  float ki;
}pll_filter_t;

void pll_filter_init(pll_filter_t *hf, float Ts, float bandwidth_hz, float damping_ratio);
void pll_filter_update(pll_filter_t *hf, float position_in);
float pll_filter_get_estimate_position(pll_filter_t *hf);
float pll_filter_get_estimate_velocity(pll_filter_t *hf);

#endif /* PLL_FILTER_H_ */
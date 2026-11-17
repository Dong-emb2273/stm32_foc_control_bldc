#ifndef MOVING_AVERAGE_H_
#define MOVING_AVERAGE_H_

#include <stdint.h>

#define CMA_MAX_RADIUS 32   /* max supported half-window; radius is clamped to this */

void circular_moving_average(float *data, int data_size, uint16_t radius, uint8_t passes);


#endif /* MOVING_AVERAGE_H_ */
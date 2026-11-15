#include "moving_average.h"


inline void circular_moving_average(float *data, int data_size, uint16_t radius, uint8_t passes) {
    const int32_t n = data_size;
    int32_t r = (int32_t)radius;

    if (r > (n - 1) / 2)    r = (n - 1) / 2;     /* window must not exceed table */
    if (r > CMA_MAX_RADIUS) r = CMA_MAX_RADIUS;
    if (r == 0 || passes == 0) return;

    float head[CMA_MAX_RADIUS + 1];  /* original data[0..r]: needed when window wraps past the end */
    float lag[CMA_MAX_RADIUS];       /* original data[i-r .. i-1]: already overwritten in data[] */

    const float inv_win = 1.0f / (float)(2 * r + 1);

    while (passes--) {
        /* Save the first r+1 originals before they get overwritten */
        for (int32_t k = 0; k <= r; k++) {
            head[k] = data[k];
        }

        /* Initial window sum for i = 0 (indices -r .. +r, wrapped) */
        float sum = 0.0f;
        for (int32_t k = -r; k <= r; k++) {
            sum += data[(k + n) % n];
        }

        int32_t slot = 0;
        for (int32_t i = 0; i < n; i++) {
            /* Sample leaving the window: x[i - r] */
            float drop = (i >= r) ? lag[slot]            /* already overwritten -> use saved copy */
                                  : data[i - r + n];     /* tail, still original */

            /* Sample entering the window: x[i + r + 1] */
            float add  = (i + r + 1 < n) ? data[i + r + 1]        /* not yet overwritten */
                                         : head[i + r + 1 - n];   /* wrapped -> saved head */

            lag[slot] = data[i];          /* keep original x[i] for use r steps later */
            data[i]   = sum * inv_win;    /* write smoothed value in place */
            sum      += add - drop;

            if (++slot == r) slot = 0;
        }
    }
}

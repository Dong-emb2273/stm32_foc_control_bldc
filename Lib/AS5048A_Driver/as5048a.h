
#ifndef AS5048A_H_
#define AS5048A_H_

#include <stdint.h>
#include "driver.h"
#include "pll_filter.h"

typedef enum {
    SENSOR_DIR_NORMAL,
    SENSOR_DIR_REVERSE
}sensor_dir_t;

typedef struct {
    uint8_t spi_tx_buffer[2];
    uint8_t spi_rx_buffer[2];
    
    int32_t count_raw;
    int32_t last_count_raw;
    int32_t countinuous_count;
    pll_filter_t count_filt;

    float continuous_deg;

    sensor_dir_t dir;
    float count_to_deg_scale;

    _Bool spi_transfer_flag;
    _Bool spi_transfer_done_flag;

    int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t);
    void (*spi_cs_en)(void);
    void (*spi_cs_dis)(void);
}AS5048A_t;

void AS5048A_Config(AS5048A_t *encd, int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t), void (*spi_cs_en)(void), void (*spi_cs_dis)(void));
void AS5048A_Init(AS5048A_t *encd, sensor_dir_t dir, float scale);
void AS5048A_set_pll_filter(AS5048A_t *encd, float bw_hz, float damping_ratio, float Ts);
void AS5048A_pll_filter_update(AS5048A_t *encd);
void AS5048A_update(AS5048A_t *encd);
void AS5048A_set_spi_transfer_done(AS5048A_t *encd);
float AS5048A_get_degree(AS5048A_t *encd);
float AS5048A_get_rpm(AS5048A_t *encd);
float AS5048A_get_continuous_degree(AS5048A_t *encd);

#endif /* AS5048A_H_ */
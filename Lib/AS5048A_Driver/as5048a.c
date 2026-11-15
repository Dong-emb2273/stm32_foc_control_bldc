#include "as5048a.h"
#include <string.h>
#include <math.h>

#ifndef TWO_PI
#define TWO_PI 6.2831853f
#endif

#define AS5048A_REG_ANGLE  0x3FFF
#define AS5048A_READ_CMD   0x4000

static uint8_t calc_even_parity(uint16_t value) {
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return value & 1;
}

void AS5048A_Config(AS5048A_t *encd, int (*spi_transfer)(uint8_t*, uint8_t*, uint16_t), void (*spi_cs_en)(void), void (*spi_cs_dis)(void)) {
    encd->spi_transfer  = spi_transfer;
    encd->spi_cs_en  = spi_cs_en;
    encd->spi_cs_dis = spi_cs_dis;
    encd->spi_cs_en();
}

void AS5048A_Init(AS5048A_t *encd, sensor_dir_t dir, float scale) {
    encd->dir = dir;
    encd->count_to_deg_scale = scale;
    encd->last_count_raw = 0;
    encd->countinuous_count = 0;
}

void AS5048A_set_pll_filter(AS5048A_t *encd, float bw_hz, float damping_ratio, float Ts) {
    pll_filter_init(&encd->count_filt, Ts, bw_hz, damping_ratio);
}

static int AS5048A_start(AS5048A_t *encd) {
    if (encd->spi_transfer_flag) return 0;
    uint16_t cmd = AS5048A_READ_CMD | AS5048A_REG_ANGLE;
    cmd |= (calc_even_parity(cmd) << 15);  // bit 15 parity

    encd->spi_tx_buffer[0] = (uint8_t)(cmd >> 8);
    encd->spi_tx_buffer[1] = (uint8_t)(cmd & 0xFF);

	encd->spi_cs_dis();
    if (encd->spi_transfer(encd->spi_tx_buffer, encd->spi_rx_buffer, 2) != 0) {
        encd->spi_cs_en();
        return -1;
    }

    encd->spi_transfer_flag = 1;
	return 0;
}

static void AS5048A_calc_degree(AS5048A_t *encd) {
    if (!encd->spi_transfer_flag) return;
    encd->spi_transfer_flag = 0;
    encd->spi_cs_en();

    const uint16_t raw_data = ((uint16_t)encd->spi_rx_buffer[0] << 8) | encd->spi_rx_buffer[1];

    // Parity check (bit 15 = parity)
    const uint16_t data_15bit = raw_data & 0x7FFF;
    const uint8_t expected_parity = calc_even_parity(data_15bit);
    const uint8_t received_parity = (raw_data >> 15) & 0x1;
    if (expected_parity != received_parity) {
        return;
    }

    // Error flag check (bit 14)
    // is this needed?
    // if ((raw_data >> 14) & 0x1) {
    //     return;
    // }

    uint16_t pos = raw_data & 0x3FFF;
    encd->count_raw = (encd->dir == SENSOR_DIR_NORMAL) ? pos : (uint16_t)((0x4000 - pos) & 0x3FFF);

    // The raw angle wraps every 16384 counts. Convert the difference to the
    // shortest signed movement so crossing zero does not look like a full turn.
    int32_t delta = encd->count_raw - encd->last_count_raw;
    if (delta > 8192) {
        delta -= 16384;
    } else if (delta < -8192) {
        delta += 16384;
    }
    encd->countinuous_count += delta;
    encd->last_count_raw = encd->count_raw;
}

void AS5048A_pll_filter_update(AS5048A_t *encd) {
    pll_filter_update(&encd->count_filt, (float)encd->countinuous_count);
}

void AS5048A_update(AS5048A_t *encd) {
    AS5048A_start(encd);
    if (encd->spi_transfer_done_flag) {
        encd->spi_transfer_done_flag = 0;
        AS5048A_calc_degree(encd);
    }
}

void AS5048A_set_spi_transfer_done(AS5048A_t *encd) {
    encd->spi_transfer_done_flag = 1;
}

float AS5048A_get_degree(AS5048A_t *encd) {
    float deg = encd->count_raw * encd->count_to_deg_scale;
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg;
}

float AS5048A_get_rpm(AS5048A_t *encd) {
    float deg_s = encd->count_filt.estimate_velocity * encd->count_to_deg_scale;
    return (deg_s / 6.0f);
}

float AS5048A_get_continuous_degree(AS5048A_t *encd) {
    return encd->countinuous_count * encd->count_to_deg_scale;
}
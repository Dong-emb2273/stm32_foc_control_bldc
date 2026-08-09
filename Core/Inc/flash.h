/*
 * flash.h
 *
 *  Created on: August 8, 2025
 *      Author: munir
 */

#ifndef FLASH_H_
#define FLASH_H_

#include "stm32f4xx_hal.h"
#include "foc_utils.h"

#define FLASH_SECTOR_ADDR  ((uint32_t)0x080E0000)
#define FLASH_SECTOR_NUM   FLASH_SECTOR_11

#define SOF_FLAG 0xAA
#define EOF_FLAG 0x55

typedef struct {
    uint8_t valid_SOF;

    float id_kp;
    float id_ki;
    float id_out_max;
    float id_e_deadband;

    float iq_kp;
    float iq_ki;
    float iq_out_max;
    float iq_e_deadband;
	
    float I_ctrl_bandwidth;
    float i_max_current;
    float v_max_voltage;
    
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float speed_out_max;
    float speed_e_deadband;
    
    float pos_kp;
    float pos_ki;
    float pos_kd;
    float pos_out_max;
    float pos_e_deadband;

    float spoint_pos;
    float spoint_vel;
    float spoint_tor;

    float zero_angle;

    motor_mode_t control_mode;
    uint8_t state;
    uint8_t next_state;

    // flag bit 
    // uint8_t power_flag :1;
    // uint8_t save_flag :1;
    // uint8_t motor_running :2;
    // uint8_t cur_view :1;
    // uint8_t vel_view :1;
    // uint8_t pos_view :1;
    // uint8_t reserved :1;

    uint32_t  status_flags;

    float voffset_a;
    float voffset_b;
    float voffset_c;

    EncoderLocation_t encd_location;
    EncoderType_t encd_type;
    float encd_offset;
    float encd_error_comp[ERROR_LUT_SIZE];

    uint32_t freq;
    float i_cal;
    uint8_t pole_pairs;
    dir_mode_t dir;
    float kt;
	float gear_ratio;

    float Rs;
    float Ld;
    float Lq;

    int32_t can_id;
    int32_t can_master;
    int32_t can_timeout;

    float pos_min;
    float pos_max;
    float vel_min;
    float vel_max;
    float tor_min;
    float tor_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;



    uint8_t valid_EOF;
}motor_config_t;

HAL_StatusTypeDef flash_save_config(motor_config_t *data);
HAL_StatusTypeDef flash_erase_ready(void);
HAL_StatusTypeDef flash_save_emergency(motor_config_t *data);

void flash_read_config(motor_config_t *data);
void flash_default_config(motor_config_t *data);
void flash_auto_tuning_torque_control(motor_config_t *data);

#endif
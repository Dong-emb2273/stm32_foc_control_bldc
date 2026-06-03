/// Values stored in flash, which are modified by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif
#include "flash.h"

extern motor_config_t m_config;


#define I_BW                (m_config.I_ctrl_bandwidth)     // Current loop bandwidth
#define KT			        (m_config.kt)                   // Torque Constant (N-m/A)
#define GR			        (m_config.gear_ratio)           // Gear ratio
#define PPAIRS		        (m_config.pole_pairs)           // Number of motor pole-pairs
#define DIR_PHASE           (m_config.dir)                  // Phase swapping during calibration
#define I_CAL		        (m_config.i_cal)                // Calibration current (A)
#define ENCDER_OFFSET       (m_config.encd_offset)          // Encoder zero position offset (rad)
#define ENCD_LOCATION       (m_config.encd_location)        // Encoder location (internal or external)
#define ENCD_TYPE           (m_config.encd_type)            // Encoder type (e.g., AS5048A, MT6835)

#define L_D			        (m_config.Ld)					// D-axis inductance
#define L_Q			        (m_config.Lq)					// Q-axis inductance
#define R_S			        (m_config.Rs)					// Single phase resistance
#define V_OFFSET_A          (m_config.voffset_a)            // ADC voltage offset for phase A
#define V_OFFSET_B          (m_config.voffset_b)            // ADC voltage offset for phase B
#define V_OFFSET_C          (m_config.voffset_c)            // ADC voltage offset for
#define M_ZERO              (m_config.zero_angle)           // Mechanical zero angle (degrees)

#define CONTROL_MODE        (m_config.control_mode)         // Control mode (0: torque, 1: speed, 2: position)
#define SPOINT_POS          (m_config.spoint_pos)           // Position setpoint for step response testing
#define SPOINT_VEL          (m_config.spoint_vel)           // Velocity setpoint for step response testing
#define SPOINT_TOR          (m_config.spoint_tor)           // Torque setpoint for step response testing

#define ID_KP               (m_config.id_kp)               // D-axis current controller proportional gain
#define ID_KI               (m_config.id_ki)               // D-axis current controller integral gain
#define ID_OUT_MAX          (m_config.id_out_max)          // D-axis current controller output limit
#define ID_E_DEADBAND       (m_config.id_e_deadband)       // D-axis current controller error deadband

#define IQ_KP               (m_config.iq_kp)               // Q-axis current controller proportional gain
#define IQ_KI               (m_config.iq_ki)               // Q-axis current controller integral gain
#define IQ_OUT_MAX          (m_config.iq_out_max)          // Q-axis current controller output limit
#define IQ_E_DEADBAND       (m_config.iq_e_deadband)       // Q-axis current controller error deadband

#define SPEED_KP            (m_config.speed_kp)            // Speed controller proportional gain
#define SPEED_KI            (m_config.speed_ki)            // Speed controller integral gain
#define SPEED_KD            (m_config.speed_kd)            // Speed controller derivative gain
#define SPEED_OUT_MAX       (m_config.speed_out_max)       // Speed controller output limit
#define SPEED_E_DEADBAND    (m_config.speed_e_deadband)    // Speed controller  error deadband

#define POS_KP              (m_config.pos_kp)              // Position controller proportional gain
#define POS_KI              (m_config.pos_ki)              // Position controller integral gain
#define POS_KD              (m_config.pos_kd)              // Position controller derivative gain
#define POS_OUT_MAX         (m_config.pos_out_max)         // Position controller output limit
#define POS_E_DEADBAND      (m_config.pos_e_deadband)      // Position controller error deadband

#define CAN_ID              (m_config.can_id)               // CAN bus ID
#define CAN_MASTER          (m_config.can_master)           // CAN bus "master" ID
#define CAN_TIMEOUT         (m_config.can_timeout)          // CAN bus timeout period

#define STATE               (m_config.state)                // Current state of the motor (0: idle, 1: running, 2: error)
#define NEXT_STATE          (m_config.next_state)           // Next state of the motor after current operation completes
#define POWER_FLAG          (m_config.power_flag)           // Flag indicating whether motor is powered on (1) or off (0)

#define P_MIN		        (m_config.pos_min)              // Position setpoint lower limit (rad)
#define P_MAX		        (m_config.pos_max)              // Position setupoint upper bound (rad)

#define V_MIN		        (m_config.vel_min)              // Velocity setpoint lower bound (rad/s)
#define V_MAX		        (m_config.vel_max)              // Velocity setpoint upper bound (rad/s)

#define T_MIN 		        (m_config.tor_min)	
#define T_MAX 		        (m_config.tor_max)	

#define KP_MAX		        (m_config.kp_max)               // Max position gain (N-m/rad)
#define KP_MIN		        (m_config.kp_min)               // Min position gain (N-m/rad)

#define KD_MAX		        (m_config.kd_max)               // Max velocity gain (N-m/rad/s)
#define KD_MIN		        (m_config.kd_min)               // Min velocity gain (N-m/rad/s)


#define I_MAX               (m_config.iq_out_max)           // Current limit


#define THETA_MIN               1.0f                // Minimum position setpoint
#define THETA_MAX               1.0f                // Maximum position setpoint
#define I_FW_MAX                1.0f                // Maximum field weakening current
#define R_NOMINAL               1.0f                // Nominal motor resistance, set during calibration
#define TEMP_MAX                1.0f                // Temperature safety lmit
#define I_MAX_CONT              1.0f                // Continuous max current
#define R_TH					1.0f				// Thermal resistance (C/W)
#define C_TH					1.0f				// Thermal mass (C/J)

#define VB_MIN					1.0f				
#define VB_MAX					1.0f				



#define E_ZERO					1
#define ENCODER_LUT             1                // Encoder offset LUT - 128 elements long






#ifdef __cplusplus
}
#endif

#endif	 /* USER_CONFIG_H_ */
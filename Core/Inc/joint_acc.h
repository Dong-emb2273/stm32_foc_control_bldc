
#ifndef __JOINT_H__
#define __JOINT_H__

#include <stdint.h>

// Định nghĩa số lượng khớp - dễ dàng thay đổi khi cần nâng cấp robot
#define NUM_JOINTS 10 
#define MAX_CAN_ID 0x00F

typedef struct {
	float 		p_des;  // (rad)
	float 		v_des;  // (rad/s)
	float 		kp;     // (N.m/rad)
	float 		kd;     // (N.m.s/rad)
	float 		t_ff;   // (N.m)
	uint32_t	can_id;
} JointCommand_t;

typedef struct {
	float 		p_act;    // (rad)
	float 		v_act;    // (rad/s)
	float 		t_act;    // (N.m)
	float 		v_batt;   // (Volts)
} JointState_t;

typedef struct {
	const char*	name; 		// (name URDF on ROS 2)
	double 		position; // (rad)
	double 		velocity; // (rad/s)
	double 		effort;   // (N.m)
} JointROS_t;


typedef struct {
	JointCommand_t	cmd;      
	JointState_t    state;    
	JointROS_t 		ros; 
} JointRobot_t;

void init_robot_joints();


#endif

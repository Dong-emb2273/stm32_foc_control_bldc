#include <stdint.h>
#include <string.h>

#include "joint_acc.h"
#include "structs.h"

void init_robot_joints(void) {
    memset(robot_joints, 0, sizeof(robot_joints));
    memset(joint_map, 0, sizeof(joint_map));

    // Right Leg
    robot_joints[0].ros.name   = "right_hip_yaw";
    robot_joints[0].cmd.can_id = 0x010;
    robot_joints[0].cmd.kp     = 50.0f;
    robot_joints[0].cmd.kd     = 1.5f;
    joint_map[(robot_joints[0].cmd.can_id >> 4) - 1] = &robot_joints[0];

    robot_joints[1].ros.name   = "right_hip_roll";
    robot_joints[1].cmd.can_id = 0x020;
    robot_joints[1].cmd.kp     = 50.0f;
    robot_joints[1].cmd.kd     = 1.5f;
    joint_map[(robot_joints[1].cmd.can_id >> 4) - 1] = &robot_joints[1];

    robot_joints[2].ros.name   = "right_hip_pitch";
    robot_joints[2].cmd.can_id = 0x030;
    robot_joints[2].cmd.kp     = 50.0f;
    robot_joints[2].cmd.kd     = 1.5f;
    joint_map[(robot_joints[2].cmd.can_id >> 4) - 1] = &robot_joints[2];

    robot_joints[3].ros.name   = "right_knee";
    robot_joints[3].cmd.can_id = 0x040;
    robot_joints[3].cmd.kp     = 50.0f;
    robot_joints[3].cmd.kd     = 1.5f;
    joint_map[(robot_joints[3].cmd.can_id >> 4) - 1] = &robot_joints[3];

    robot_joints[4].ros.name   = "right_ankle";
    robot_joints[4].cmd.can_id = 0x050;
    robot_joints[4].cmd.kp     = 50.0f;
    robot_joints[4].cmd.kd     = 1.5f;
    joint_map[(robot_joints[4].cmd.can_id >> 4) - 1] = &robot_joints[4];

    // Left Leg
    robot_joints[5].ros.name   = "left_hip_yaw";
    robot_joints[5].cmd.can_id = 0x060;
    robot_joints[5].cmd.kp     = 50.0f;
    robot_joints[5].cmd.kd     = 1.5f;
    joint_map[(robot_joints[5].cmd.can_id >> 4) - 1] = &robot_joints[5];

    robot_joints[6].ros.name   = "left_hip_roll";
    robot_joints[6].cmd.can_id = 0x070;
    robot_joints[6].cmd.kp     = 50.0f;
    robot_joints[6].cmd.kd     = 1.5f;
    joint_map[(robot_joints[6].cmd.can_id >> 4) - 1] = &robot_joints[6];

    robot_joints[7].ros.name   = "left_hip_pitch";
    robot_joints[7].cmd.can_id = 0x080;
    robot_joints[7].cmd.kp     = 50.0f;
    robot_joints[7].cmd.kd     = 1.5f;
    joint_map[(robot_joints[7].cmd.can_id >> 4)] = &robot_joints[7];

	robot_joints[8].ros.name   = "right_knee";
    robot_joints[8].cmd.can_id = 0x090;
    robot_joints[8].cmd.kp     = 50.0f;
    robot_joints[8].cmd.kd     = 1.5f;
    joint_map[(robot_joints[8].cmd.can_id >> 4)] = &robot_joints[8];

    robot_joints[9].ros.name   = "right_ankle";
    robot_joints[9].cmd.can_id = 0x0A0;
    robot_joints[9].cmd.kp     = 50.0f;
    robot_joints[9].cmd.kd     = 1.5f;
    joint_map[(robot_joints[4].cmd.can_id >> 4)] = &robot_joints[9];

}
//

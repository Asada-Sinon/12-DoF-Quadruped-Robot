/* robot_params.c */
#include "robot_params.h"
#include <stdio.h>  // 如果需要文件读写功能


/* 全局参数实例 */
RobotParams robot_params = {
    /* 步态参数 */
    .trot_gait = {
        .phase = {0.0, 0.5, 0.5, 0.0}, 
        .T = 0.5, //0.38
        .stance_ratio = 0.52,
        .swing_ratio = 1- 0.52,
        .step_length = 0,
        .swing_height = 0.07,
        .stance_depth = 0,//0.008
        .stand_height = DEFAULT_STAND_HEIGHT
    },
    /* 电机参数 */
    .motor_param = {
        .motor_control_params = {
            //FL
            {70, 0.5}, 
            {80, 0.5}, 
            {90, 0.5}, 
            //FR
            {70, 0.5}, 
            {80, 0.5}, 
            {90, 0.5}, 
            //HL
            {50, 0.5}, 
            {50, 0.5}, 
            {50, 0.5},  
            //HR
            {50, 0.5}, 
            {50, 0.5}, 
            {50, 0.5},
            //FL
//            {0, 0}, 
//            {0, 0}, 
//            {0, 0}, 
//            //FR
//            {0, 0}, 
//            {0, 0}, 
//            {0, 0}, 
//            //HL
//            {0, 0}, 
//            {0, 0}, 
//            {0, 0}, 
//            //HR
//            {0, 0}, 
//            {0, 0}, 
//            {0, 0}
        },
        .motor_id = {
            {MOTOR_FL_HIP_ID, MOTOR_FL_THIGH_ID, MOTOR_FL_CALF_ID},
            {MOTOR_FR_HIP_ID, MOTOR_FR_THIGH_ID, MOTOR_FR_CALF_ID},
            {MOTOR_HL_HIP_ID, MOTOR_HL_THIGH_ID, MOTOR_HL_CALF_ID},
            {MOTOR_HR_HIP_ID, MOTOR_HR_THIGH_ID, MOTOR_HR_CALF_ID}
        },
        /* 电机角度限制 */
        .motor_limits = {
            [LEG_FL] = {
                .max_hip_angle = MOTOR_FL_HIP_MAX,
                .min_hip_angle = MOTOR_FL_HIP_MIN,
                .max_thigh_angle = MOTOR_FL_THIGH_MAX,
                .min_thigh_angle = MOTOR_FL_THIGH_MIN,
                .max_calf_angle = MOTOR_FL_CALF_MAX,
                .min_calf_angle = MOTOR_FL_CALF_MIN
            },
            [LEG_FR] = {
                .max_hip_angle = MOTOR_FR_HIP_MAX,
                .min_hip_angle = MOTOR_FR_HIP_MIN,
                .max_thigh_angle = MOTOR_FR_THIGH_MAX,
                .min_thigh_angle = MOTOR_FR_THIGH_MIN,
                .max_calf_angle = MOTOR_FR_CALF_MAX,
                .min_calf_angle = MOTOR_FR_CALF_MIN
            },
            [LEG_HL] = {
                .max_hip_angle = MOTOR_HL_HIP_MAX,
                .min_hip_angle = MOTOR_HL_HIP_MIN,
                .max_thigh_angle = MOTOR_HL_THIGH_MAX,
                .min_thigh_angle = MOTOR_HL_THIGH_MIN,
                .max_calf_angle = MOTOR_HL_CALF_MAX,
                .min_calf_angle = MOTOR_HL_CALF_MIN
            },
            [LEG_HR] = {
                .max_hip_angle = MOTOR_HR_HIP_MAX,
                .min_hip_angle = MOTOR_HR_HIP_MIN,
                .max_thigh_angle = MOTOR_HR_THIGH_MAX,
                .min_thigh_angle = MOTOR_HR_THIGH_MIN,
                .max_calf_angle = MOTOR_HR_CALF_MAX,
                .min_calf_angle = MOTOR_HR_CALF_MIN
            }
        },
        /* 电机方向 */
        .motor_dir = {
            [LEG_FL] = {
                .hip_dir = MOTOR_FL_HIP_DIRECTION,
                .thigh_dir = MOTOR_FL_THIGH_DIRECTION,
                .calf_dir = MOTOR_FL_CALF_DIRECTION
            },
            [LEG_FR] = {
                .hip_dir = MOTOR_FR_HIP_DIRECTION,
                .thigh_dir = MOTOR_FR_THIGH_DIRECTION,
                .calf_dir = MOTOR_FR_CALF_DIRECTION
            },
            [LEG_HL] = {
                .hip_dir = MOTOR_HL_HIP_DIRECTION,
                .thigh_dir = MOTOR_HL_THIGH_DIRECTION,
                .calf_dir = MOTOR_HL_CALF_DIRECTION
            },
            [LEG_HR] = {
                .hip_dir = MOTOR_HR_HIP_DIRECTION,
                .thigh_dir = MOTOR_HR_THIGH_DIRECTION,
                .calf_dir = MOTOR_HR_CALF_DIRECTION
            }
        },
        /* 电机零位 */
        .motor_zero_pos = {
            [LEG_FL] = {
                .hip_zero_pos = MOTOR_FL_HIP_ANGLE_ZERO_POS,
                .thigh_zero_pos = MOTOR_FL_THIGH_ANGLE_ZERO_POS,
                .calf_zero_pos = MOTOR_FL_CALF_ANGLE_ZERO_POS
            },
            [LEG_FR] = {
                .hip_zero_pos = MOTOR_FR_HIP_ANGLE_ZERO_POS,
                .thigh_zero_pos = MOTOR_FR_THIGH_ANGLE_ZERO_POS,
                .calf_zero_pos = MOTOR_FR_CALF_ANGLE_ZERO_POS
            },
            [LEG_HL] = {
                .hip_zero_pos = MOTOR_HL_HIP_ANGLE_ZERO_POS,
                .thigh_zero_pos = MOTOR_HL_THIGH_ANGLE_ZERO_POS,
                .calf_zero_pos = MOTOR_HL_CALF_ANGLE_ZERO_POS
            },
            [LEG_HR] = {
                .hip_zero_pos = MOTOR_HR_HIP_ANGLE_ZERO_POS,
                .thigh_zero_pos = MOTOR_HR_THIGH_ANGLE_ZERO_POS,
                .calf_zero_pos = MOTOR_HR_CALF_ANGLE_ZERO_POS
            }
        }
    },
    /* 姿态参数 */
    .posture = {
        .stand_height = DEFAULT_STAND_HEIGHT,
        .center_of_gravity = {
            .translation = {DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_X, DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_Y, DEFAULT_CENTER_OF_GRAVITY_TRANSLATION_Z},
            .rotation = {DEFAULT_CENTER_OF_GRAVITY_ROTATION_PITCH, DEFAULT_CENTER_OF_GRAVITY_ROTATION_YAW, DEFAULT_CENTER_OF_GRAVITY_ROTATION_ROLL},
            .velocity = {DEFAULT_CENTER_OF_GRAVITY_VX, DEFAULT_CENTER_OF_GRAVITY_VY, DEFAULT_CENTER_OF_GRAVITY_VW},
            .foot_offset = {DEFAULT_CENTER_OF_GRAVITY_FOOT_OFFEST_X, DEFAULT_CENTER_OF_GRAVITY_FOOT_OFFEST_Y},
            .stand_cog_offset = {DEFAULT_CENTER_OF_GRAVITY_STAND_COG_OFFSET_X, DEFAULT_CENTER_OF_GRAVITY_STAND_COG_OFFSET_Y},
            .trot_cog_forward_offset = {DEFAULT_CENTER_OF_GRAVITY_TROT_COG_FORWARD_OFFSET_X, 0},
            .trot_cog_backward_offset = {DEFAULT_CENTER_OF_GRAVITY_TROT_COG_BACKWARD_OFFSET_X, 0}
        },
        .center_of_gravity_limit = {
            .cog_x_max = DEFAULT_CENTER_OF_GRAVITY_MAX_X,
            .cog_x_min = DEFAULT_CENTER_OF_GRAVITY_MIN_X,
            .cog_y_max = DEFAULT_CENTER_OF_GRAVITY_MAX_Y,
            .cog_y_min = DEFAULT_CENTER_OF_GRAVITY_MIN_Y,
            .cog_z_max = DEFAULT_CENTER_OF_GRAVITY_MAX_Z,
            .cog_z_min = DEFAULT_CENTER_OF_GRAVITY_MIN_Z
        },
        .default_joint_pos = {
            {DEFAULT_JOINT_ANGLE_HIP, DEFAULT_JOINT_ANGLE_THIGH, DEFAULT_JOINT_ANGLE_CALF},
            {DEFAULT_JOINT_ANGLE_HIP, DEFAULT_JOINT_ANGLE_THIGH, DEFAULT_JOINT_ANGLE_CALF},
            {DEFAULT_JOINT_ANGLE_HIP, DEFAULT_JOINT_ANGLE_THIGH, DEFAULT_JOINT_ANGLE_CALF},
            {DEFAULT_JOINT_ANGLE_HIP, DEFAULT_JOINT_ANGLE_THIGH, DEFAULT_JOINT_ANGLE_CALF}
        },
        .contact = {1, 1, 1, 1},
        .phase = {0.5, 0.5, 0.5, 0.5}
    },

    /* 连杆参数 */
    .leg_links = {
        .hip_length = LINK_LENGTH_HIP,
        .thigh_length = LINK_LENGTH_THIGH,
        .calf_length = LINK_LENGTH_CALF
    },
    
    /* 关节限制 */
    .joint_limits = {
        .max_hip_angle = MOTOR_FL_HIP_MAX,   // 使用FL的值作为通用限制
        .min_hip_angle = MOTOR_FL_HIP_MIN,
        .max_thigh_angle = MOTOR_FL_THIGH_MAX,
        .min_thigh_angle = MOTOR_FL_THIGH_MIN,
        .max_calf_angle = MOTOR_FL_CALF_MAX,
        .min_calf_angle = MOTOR_FL_CALF_MIN
    }   
};

/* 获取机器人参数 */
RobotParams* get_robot_params(void) {
    return &robot_params;
}

"""Configuration for Unitree robots.

The following configurations are available:

* :obj:`DIGIT_CFG`: Agility robotics Digit_v3

Reference: https://github.com/unitreerobotics/unitree_ros
"""

from isaac_digit_v3.assets import ISAAC_ASSETS_DIR

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import DCMotorCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
##
# Configuration
##

DIGIT_V3_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_ASSETS_DIR}/Robots/huamnoid/humandoid_urdf_limit.usd",
        # activate_contact_sensors=False,
        activate_contact_sensors=True,
        
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
            
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, 
            solver_position_iteration_count=4, 
            solver_velocity_iteration_count=1,
            fix_root_link=False,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.01),
        joint_pos={
            "left_leg_hip_roll_joint": 0.3654,
            "left_leg_hip_yaw_joint": -0.0054,
            "left_leg_hip_pitch_joint": 0.2996,
            "left_leg_knee_joint": 0.342677,
            "left_leg_toe_a_joint": -0.10821,
            "left_leg_toe_b_joint":  0.119,
            "left_leg_shin_joint": 0.0,
            "left_leg_heel_spring_joint": 0.0,

            
            "right_leg_hip_roll_joint": -0.3654,
            "right_leg_hip_yaw_joint": 0.0054,
            "right_leg_hip_pitch_joint": -0.2996,
            "right_leg_knee_joint": -0.342677,
            "right_leg_toe_a_joint": 0.10821,
            "right_leg_toe_b_joint": -0.119,
            "right_leg_shin_joint": 0.0,
            "right_leg_heel_spring_joint": 0.0,
            
            "left_arm_shoulder_roll_joint": -0.1543,
            "left_arm_shoulder_pitch_joint": 1.0918,
            "left_arm_shoulder_yaw_joint": 0.0060, 
            "left_arm_elbow_joint": 0.0, 
            
            "right_arm_shoulder_roll_joint": 0.1543,
            "right_arm_shoulder_pitch_joint": -1.0918,
            "right_arm_shoulder_yaw_joint": -0.0060, 
            "right_arm_elbow_joint": 0.0, 
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.95,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_leg_hip_roll_joint", 
                ".*_leg_hip_yaw_joint", 
                ".*_leg_hip_pitch_joint", 
                ".*_leg_knee_joint",
                ".*_leg_shin_joint",
                ".*_leg_heel_spring_joint",
                ],
            # effort_limit=300.0,
            # velocity_limit=100.0,
            stiffness={
                ".*_leg_hip_roll_joint": 100.0,
                ".*_leg_hip_yaw_joint": 100.0,
                ".*_leg_hip_pitch_joint": 100.0,
                ".*_leg_knee_joint": 150.0,
                ".*_leg_shin_joint": 6000.0,
                ".*_leg_heel_spring_joint": 4375.0,
                },
            damping={
                ".*_leg_hip_roll_joint": 10.0,
                ".*_leg_hip_yaw_joint": 10.0,
                ".*_leg_hip_pitch_joint": 10.0,
                ".*_leg_knee_joint": 10.0, 
                ".*_leg_shin_joint": 0.0,
                ".*_leg_heel_spring_joint": 0.0,
                },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[ ".*_leg_toe_a_joint", ".*_leg_toe_b_joint"],
            # effort_limit=100,
            # velocity_limit=100.0,
            stiffness={
                ".*_leg_toe_a_joint": 100.0,
                ".*_leg_toe_b_joint": 100.0
                },
            damping={
                ".*_leg_toe_a_joint": 10.0,
                ".*_leg_toe_b_joint": 10.0
                },
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[".*_arm_shoulder_roll_joint",  ".*_arm_shoulder_pitch_joint", ".*_arm_shoulder_yaw_joint", ".*_arm_elbow_joint"],
            # effort_limit=300,
            # velocity_limit=100.0,
            stiffness={
                ".*_arm_shoulder_roll_joint": 100.0,
                 ".*_arm_shoulder_pitch_joint": 100.0,
                ".*_arm_shoulder_yaw_joint": 100.0,
                ".*_arm_elbow_joint": 100.0,
            },
            damping={
                ".*_arm_shoulder_roll_joint": 10.0,
                ".*_arm_shoulder_pitch_joint": 10.0,
                ".*_arm_shoulder_yaw_joint": 10.0,
                ".*_arm_elbow_joint": 10.0,
            },
        ),
    },
)
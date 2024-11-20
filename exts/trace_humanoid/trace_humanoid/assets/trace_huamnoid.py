"""Configuration for Unitree robots.

The following configurations are available:

* :obj:`DIGIT_CFG`: Agility robotics Digit_v3

Reference: https://github.com/unitreerobotics/unitree_ros
"""

from trace_humanoid.assets import ISAAC_ASSETS_DIR

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import DCMotorCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.utils.assets import ISAAC_NUCLEUS_DIR
##
# Configuration
##

TRACE_HUMANOID_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_ASSETS_DIR}/Robots/humanoid/humandoid_urdf_limit.usd",
        
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
        pos=(0.0, 0.0, 0.39),
        joint_pos={
            "left_hip_roll_joint": 0.0,
            "left_hip_yaw_joint": 0.0,
            "left_hip_pitch_joint": 0.0,
            "left_knee_joint": 0.0,
            "left_ankle_joint": 0.0,
            
            "right_hip_roll_joint": 0.0,
            "right_hip_yaw_joint": 0.0,
            "right_hip_pitch_joint": 0.0,
            "right_knee_joint": 0.0,
            "right_ankle_joint": 0.0,

        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.95,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_roll_joint", 
                ".*_hip_yaw_joint", 
                ".*_hip_pitch_joint", 
                ".*_knee_joint",
                ],
            # effort_limit=300.0,
            # velocity_limit=100.0,
            stiffness={
                ".*_hip_roll_joint": 100.0,
                ".*_hip_yaw_joint": 100.0,
                ".*_hip_pitch_joint": 100.0,
                ".*_knee_joint": 300.0,

                },
            damping={
                ".*_hip_roll_joint": 1.0,
                ".*_hip_yaw_joint": 1.0,
                ".*_hip_pitch_joint": 1.0,
                ".*_knee_joint": 1.0, 
                },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[ ".*_ankle_joint"],
            # effort_limit=100,
            # velocity_limit=100.0,
            stiffness={
                ".*_ankle_joint": 300.0,
                },
            damping={
                ".*_ankle_joint": 1.0,
                },
        ),
    },
)
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg

import isaac_digit_v3.tasks.locomotion.velocity.mdp as mdp
from isaac_digit_v3.tasks.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)
    

##
# Pre-defined configs
## 
from isaac_digit_v3.assets.digit_v3_updated import DIGIT_V3_CFG

@configclass 
class Digit_V3_RewardCfg(RewardsCfg):
    """Reward terms for the MDP."""
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-200.0)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time_positive_biped,
        weight= 2.5,
        params={
            "command_name": "base_velocity",
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names= ".*_leg_toe_roll"),
            "threshold": 0.35,
        },
    )
    feet_slide = RewTerm(
        func=mdp.feet_slide,
        weight=-0.25,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names= ".*_leg_toe_roll"),
            "asset_cfg": SceneEntityCfg("robot", body_names= ".*_leg_toe_roll"),
        },
    )
    # Penalize ankle joint limits
    # dof_pos_limits = RewTerm(
    #     func=mdp.joint_pos_limits,
    #     weight=-1.0,
    #     params={
    #         "asset_cfg": SceneEntityCfg(
    #             "robot", 
    #             joint_names=[
    #                 ".*_leg_toe_pitch_joint", 
    #                 ".*_leg_toe_roll_joint",
    #                 ".*_leg_toe_a_joint",
    #                 ".*_leg_toe_b_joint",
    #                 ".*_leg_knee_joint",
    #             ],
    #         )
    #     },
    # )
    
    base_height_l2 = RewTerm(
        func=mdp.base_height_square,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", body_names=["torso_base"]), "target_height": 1.0},
    )

    # Penalize deviation from default of the joints that are not essential for locomotion
    joint_deviation_hip_ry = RewTerm(
        func=mdp.joint_deviation_l2,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", 
                joint_names=[
                    ".*_leg_hip_roll_joint", 
                    ".*_leg_hip_yaw_joint",
                ],
            )
        },
    )

    joint_deviation_hip_arm_pitch = RewTerm(
        func=mdp.joint_deviation_l2,
        weight=-0.5,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot", 
                joint_names=[
                    ".*_arm_shoulder_pitch_joint",
                    ".*_leg_hip_pitch_joint",
                ],
            )
        },
    )
    
    joint_deviation_arms_ry = RewTerm(
        func=mdp.joint_deviation_l2,
        weight=-2.0,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_arm_shoulder_roll_joint",
                    ".*_arm_shoulder_yaw_joint",
                ],
            )
        },
    )

    joint_deviation_arms_elbow = RewTerm(
        func=mdp.joint_deviation_l2,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    ".*_arm_elbow_joint",                    
                ],
            )
        },
    )



@configclass
class DigitRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Digit rought environment configuration. """
    rewards: Digit_V3_RewardCfg = Digit_V3_RewardCfg()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # scene
        self.scene.robot = DIGIT_V3_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # self.scene.height_scanner = None    
        # self.observations.policy.height_scan = None
        # actions
        self.actions.joint_pos.scale = 0.5
        # randomizations
        self.randomization.push_robot = None
        self.randomization.add_base_mass = None
        # self.randomization.add_all_joint_default_pos = None
        # self.randomization.randomize_actuator_gains = None
        # self.randomization.reset_robot_joints.params["position_range"] = (1.0, 1.0)
        self.randomization.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        }
        # terminations


        # rewards
        # self.rewards.undesired_contacts = None
        self.rewards.undesired_contacts.weight = -0.5
        self.rewards.flat_orientation_l2.weight = -0.5
        self.rewards.joint_torques_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", 
            joint_names=[
                ".*_hip_.*", 
                ".*_leg_knee_joint", 
                ".*_leg_toe_a_joint",
                ".*_leg_toe_b_joint",
            ]
        ) 
        self.rewards.action_rate_l2.weight = -0.01
        self.rewards.joint_acc_l2.weight = -1.25e-7
        self.rewards.joint_acc_l2.params["asset_cfg"] = SceneEntityCfg(
            "robot", 
            joint_names=[
                ".*_hip_.*", 
                ".*_leg_knee_joint", 
                ".*_leg_toe_a_joint",
                ".*_leg_toe_b_joint",
            ]
        )
        self.rewards.dof_pos_limits.weight = -1.0
        self.rewards.dof_pos_limits.params["asset_cfg"] = SceneEntityCfg(
            "robot", 
            joint_names=[
                ".*_hip_.*", 
                ".*_leg_knee_joint", 
                ".*_leg_toe_a_joint",
                ".*_leg_toe_b_joint",
                ".*_arm_shoulder_roll_joint",
                ".*_arm_shoulder_pitch_joint",
                ".*_arm_shoulder_yaw_joint",
                ".*_arm_elbow_joint",
            ]
        )
        
        # Commands
        self.commands.base_velocity.ranges.lin_vel_x = (0.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)




        
@configclass
class DigitRoughEnvCfg_Play(DigitRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.episode_length_s = 40.0
        # spawn the robot randomly in the grid (instead of their terrain levels)
        self.scene.terrain.max_init_terrain_level = None
        # reduce the number of terrains to save memory
        if self.scene.terrain.terrain_generator is not None:
            self.scene.terrain.terrain_generator.num_rows = 5
            self.scene.terrain.terrain_generator.num_cols = 5
            self.scene.terrain.terrain_generator.curriculum = False
        
        self.commands.base_velocity.ranges.lin_vel_x = (1.0, 1.0)
        self.commands.base_velocity.ranges.lin_vel_y = (0.0, 0.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.2, 0.2)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.randomization.base_external_force_torque = None
        self.randomization.push_robot = None
        
        
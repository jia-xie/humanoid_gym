from omni.isaac.lab.utils import configclass
from omni.isaac.lab.managers import RewardTermCfg as RewTerm
from omni.isaac.lab.managers import SceneEntityCfg

import trace_humanoid.tasks.locomotion.velocity.mdp as mdp
from trace_humanoid.tasks.locomotion.velocity.velocity_env_cfg import (
    LocomotionVelocityRoughEnvCfg,
    RewardsCfg,
)
    

##
# Pre-defined configs
## 
from trace_humanoid.assets.trace_huamnoid import TRACE_HUMANOID_CFG

@configclass 
class Digit_V3_RewardCfg(RewardsCfg):
    """Reward terms for the MDP."""
    termination_penalty = RewTerm(func=mdp.is_terminated, weight=-4000.0)
    # base_height_l2 = RewTerm(
    #     func=mdp.base_height_square,
    #     weight=-0.5,
    #     params={"asset_cfg": SceneEntityCfg("robot", body_names=["base_link"]), "target_height": 0.20},
    # )


@configclass
class DigitRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    """Digit rought environment configuration. """
    rewards: Digit_V3_RewardCfg = Digit_V3_RewardCfg()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # scene
        self.scene.robot = TRACE_HUMANOID_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # actions
        self.actions.joint_pos.scale = 0.5
        # randomizations
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # override rewards

        # no terrain curriculum
        self.curriculum.terrain_levels = None
        self.events.push_robot = None
        self.events.add_base_mass = None
        self.events.reset_base.params = {
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
        self.rewards.undesired_contacts = None
        # self.rewards.undesired_contacts.weight = -0.5
        self.rewards.flat_orientation_l2.weight = -0.5
        self.rewards.action_rate_l2.weight = -0.01
        self.rewards.dof_pos_limits.weight = -1.0
        
        # Commands
        self.commands.base_velocity.ranges.lin_vel_x = (-0.4, 0.4)
        self.commands.base_velocity.ranges.lin_vel_y = (-1.0, 1.0)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.2, 0.2)




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
        
        self.commands.base_velocity.ranges.lin_vel_x = (0.05, 0.05)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.25, 0.25)
        self.commands.base_velocity.ranges.ang_vel_z = (-0.2, 0.2)
        self.commands.base_velocity.ranges.heading = (0.0, 0.0)
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.base_external_force_torque = None
        self.events.push_robot = None
        
        
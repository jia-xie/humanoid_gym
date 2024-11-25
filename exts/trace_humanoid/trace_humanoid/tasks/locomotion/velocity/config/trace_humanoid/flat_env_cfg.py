from omni.isaac.lab.utils import configclass

from .rough_env_cfg import DigitRoughEnvCfg

@configclass
class DigitFlatEnvCfg(DigitRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # override rewards
        self.rewards.feet_air_time.weight = 1.5
        self.rewards.feet_air_time.params["threshold"] = 0.3
        # no terrain curriculum
        self.curriculum.terrain_levels = None


class DigitFlatEnvCfg_PLAY(DigitRoughEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.base_external_force_torque = None
        self.events.push_robot = None

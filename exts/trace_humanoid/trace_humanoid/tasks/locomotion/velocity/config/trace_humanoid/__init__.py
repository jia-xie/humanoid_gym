import gymnasium as gym

from . import agents, flat_env_cfg, rough_env_cfg

##
# Register Gym environment
##


gym.register(
    id="Velocity-Flat-Trace_Humanoid-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.DigitFlatEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_cfg:DigitFlatPPORunnerCfg",
    },
)

gym.register(
    id="Velocity-Flat-Trace_Humanoid-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": flat_env_cfg.DigitFlatEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_cfg:DigitFlatPPORunnerCfg",
    },
)

gym.register(
    id="Velocity-Rough-Trace_Humanoid-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.DigitRoughEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_cfg:DigitRoughPPORunnerCfg",
    },
)

gym.register(
    id="Velocity-Rough-Trace_Humanoid-Play-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": rough_env_cfg.DigitRoughEnvCfg_Play,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_cfg:DigitRoughPPORunnerCfg",
    },
)

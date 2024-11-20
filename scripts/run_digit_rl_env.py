import argparse

from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="Running the Digit_v3 RL environment.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
from omni.isaac.lab.envs import ManagerBasedRLEnv

from base_rl_env import CartpoleEnvCfg
# from isaac_digit_v3.tasks.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg
from isaac_digit_v3.tasks.locomotion.velocity.config.digit_v3.rough_env_cfg import DigitRoughEnvCfg
from isaac_digit_v3.tasks.locomotion.velocity.config.digit_v3.flat_env_cfg import DigitFlatEnvCfg

def main():
    """Main function."""
    # create environment configuration
    env_cfg = DigitFlatEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs

    # setup RL environment
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # simulation physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random actions
            joint_efforts = torch.randn_like(env.action_manager.action)
            # step the environment
            obs, rew, terminated, truncated, info = env.step(joint_efforts)
            # print current orientation of pole
            # print("[Env 0]: base joint: ", obs["policy"][0][1].item())
            # update counter
            count += 1
    # close the environment
    env.close()
    
if __name__ == "__main__":
    main()
    simulation_app.close()
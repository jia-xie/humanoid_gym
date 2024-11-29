# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to simulate bipedal robots.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p source/standalone/demos/bipeds.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse
import torch

from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser(description="This script demonstrates how to simulate bipedal robots.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import Articulation
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.sim import PhysxCfg, SimulationCfg
from omni.isaac.lab.sim.spawners.materials.physics_materials_cfg import RigidBodyMaterialCfg

from omni.isaac.lab_assets.cassie import CASSIE_CFG  # isort:skip
from omni.isaac.lab_assets import H1_CFG  # isort:skip
from omni.isaac.lab_assets import G1_CFG  # isort:skip
from isaac_digit_v3.assets.digit import DIGIT_CFG
from isaac_digit_v3.assets.digit_v3_updated import DIGIT_V3_CFG
from trace_humanoid.assets.trace_huamnoid import TRACE_HUMANOID_CFG

def main():
    """Main function."""
    sim = SimulationContext(
        sim_utils.SimulationCfg(
            dt=1/480, 
            device="cuda", 
            gravity=(0.0, 0.0, -9.81),
            physx=PhysxCfg(
                solver_type=0,
                max_position_iteration_count=4,
                max_velocity_iteration_count=1,
                min_position_iteration_count=4,
                min_velocity_iteration_count=1,
                bounce_threshold_velocity=0.2,
            ),
        )
    )
    sim.set_camera_view(eye=[6.0, 2.0, 2.25], target=[0.0, 2.0, 1.0])
    
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)
    
    origins = torch.tensor([
        [0.0, 0.0, 0.0],
        [0.0, 2.0, 0.0],
        [0.0, 4.0, 0.0],
    ]).to(device=sim.device)

    # Robots
    # cassie = Articulation(CASSIE_CFG.replace(prim_path="/World/Cassie"))
    # h1 = Articulation(H1_CFG.replace(prim_path="/World/H1"))
    # g1 = Articulation(G1_CFG.replace(prim_path="/World/G1"))
    # digit = Articulation(DIGIT_V3_CFG.copy().replace(prim_path="/World/Digit"))
    trace = Articulation(TRACE_HUMANOID_CFG.copy().replace(prim_path="/World/trace"))
    # digit_urdf = Articulation(DIGIT_URDF_CFG.copy().replace(prim_path="/World/Digit"))
    # digit_v4 = Articulation(DIGITV4_CFG.replace(prim_path="/World/DigitV4"))
    # robots = [cassie, h1, g1]
    # robots = [ digit, digit_v4, h1]
    robots = [trace]


    # Play the simulator
    sim.reset()

    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count % 300 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            for index, robot in enumerate(robots):
                # reset dof state
                joint_pos, joint_vel = robot.data.default_joint_pos, robot.data.default_joint_vel
                joint_stiffness, joint_damping = robot.data.default_joint_stiffness, robot.data.default_joint_damping
                root_state = robot.data.default_root_state.clone()
                root_state[:, :3] += origins[index]
                # print(robot.joint_names)
                robot.write_root_state_to_sim(root_state)
                robot.reset()
            # reset command
            print(">>>>>>>> Reset!")
        # # apply action to the robot
        for robot in robots:
            # joint_pos_target = robot.data.default_joint_pos + torch.randn_like(robot.data.joint_pos) * 0.1
            joint_pos_target = robot.data.default_joint_pos 
            robot.set_joint_position_target(joint_pos_target)
            robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        for robot in robots:
            robot.update(sim_dt)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

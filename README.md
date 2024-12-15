# Humanoid Robot

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.2.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)

## Overview

This repository shows the RL training code for the cutomized humanoid robot, featuring close-loop chain mechanism in Isaac Sim simulation. 


## Installation

- Install Isaac Lab by following the [installation guide](https://isaac-sim.github.io/IsaacLab/source/setup/installation/index.html). We recommend using the conda installation as it simplifies calling Python scripts from the terminal.

- Using a python interpreter that has Isaac Lab installed, install the library

```bash
python -m pip install -e exts/trace_humanoid
```


## Run the code
```bash
# Train
python scripts/rsl_rl/train.py --task Velocity-Flat-Trace_Humanoid-v0 --num_env 4096 --headless --logger tensorboard
# Play
python scripts/rsl_rl/play.py --task Velocity-Flat-Trace_Humanoid-v0 --num_env 50
```
**Note**

* Resume training from folder or checkpoint, add `--resume --load_run run_folder_name --checkpoint model.pt`
* Record video of a trained agent (requires installing `ffmpeg`), add `--video --video_length 200`
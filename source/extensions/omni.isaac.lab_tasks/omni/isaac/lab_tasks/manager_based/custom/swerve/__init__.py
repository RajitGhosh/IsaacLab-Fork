# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Cart locomotion environment
"""

import gymnasium as gym

from . import agents, swerve_env_cfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Swerve-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=False,
    kwargs={
        "env_cfg_entry_point": swerve_env_cfg.SwerveEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
)

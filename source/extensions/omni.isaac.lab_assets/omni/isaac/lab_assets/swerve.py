# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a Cart robot."""


import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets import ArticulationCfg

##
# Configuration
##

SWERVE_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/workspace/isaaclab/source/extensions/omni.isaac.lab_assets/data/Robots/swerve.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
            enable_gyroscopic_forces=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5), joint_pos={".*": 0.0},
    ),
    actuators={
        "drive": ImplicitActuatorCfg(
        joint_names_expr=["wheel1_drive", "wheel2_drive", "wheel3_drive", "wheel4_drive"],
        effort_limit=400.0,
        velocity_limit=100.0,
        stiffness=0.0,
        damping=10.0,
        ),
        "steer": ImplicitActuatorCfg(
        joint_names_expr=["wheel1_steer", "wheel2_steer", "wheel3_steer", "wheel4_steer"],
        effort_limit=40.0,
        velocity_limit=10.0,
        stiffness=0.0,
        damping=10.0,
        ),
    },
)
"""Configuration for a Cart robot."""

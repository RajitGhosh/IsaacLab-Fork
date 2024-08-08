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

CART_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"/workspace/isaaclab/source/extensions/omni.isaac.lab_assets/data/Robots/cart.usd"
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 2.0), joint_pos={"wheel1_joint": 0.0, "wheel2_joint": 0.0, "wheel3_joint": 0.0, "wheel4_joint": 0.0}
    ),
    actuators={
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=0.0,
   ),
},
)
"""Configuration for a Cart robot."""

# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the ANYbotics robots.

The following configuration parameters are available:

* :obj:`ANYMAL_B_CFG`: The ANYmal-B robot with ANYdrives 3.0
* :obj:`ANYMAL_C_CFG`: The ANYmal-C robot with ANYdrives 3.0
* :obj:`ANYMAL_D_CFG`: The ANYmal-D robot with ANYdrives 3.0

Reference:

* https://github.com/ANYbotics/anymal_b_simple_description
* https://github.com/ANYbotics/anymal_c_simple_description
* https://github.com/ANYbotics/anymal_d_simple_description

"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ActuatorNetLSTMCfg, DCMotorCfg, IdealPDActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab.sensors import RayCasterCfg
from omni.isaac.lab_assets import ISAACLAB_ASSETS_DATA_DIR

#from .velodyne import VELODYNE_VLP_16_RAYCASTER_CFG

##
# Configuration - Actuators.
##

LUNA_SIMPLE_ACTUATOR_CFG = IdealPDActuatorCfg(
    joint_names_expr=[".*joint"],
    effort_limit=10.0,
    stiffness={".*": 40.0},
    velocity_limit=1000.0,
    damping={".*": 10.0}

)
"""Configuration for ANYdrive 3.x with DC actuator model."""


##
# Configuration - Articulation.
##

LUNA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/kamaro/luna/luna.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=0, fix_root_link=True
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.02, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.55),
        joint_pos={
            ".*_shoulder_joint": 0.0,  # all HAA
            ".*_hip_joint": -0.4,  # both front HFE
            ".*_knee_joint": 0.8,  # both hind HFE
        },
    ),
    actuators={"legs": LUNA_SIMPLE_ACTUATOR_CFG},
    soft_joint_pos_limit_factor=0.75,
)
"""Configuration of LUNA quadruped using simple DC actuator models."""

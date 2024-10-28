from omni.isaac.core.articulations import Articulation, ArticulationSubset
from omni.isaac.core.utils.types import ArticulationAction
import numpy as np

# Load robot
robot = Articulation("/UR10")
robot.initialize()
print("DOF names:", robot.dof_names)

# Set position for all joints
robot.apply_action(ArticulationAction(np.array([1.0, 0.0,2.0,3.0,0.0,0.0])))

# Print position
position = robot.get_joint_positions()
print("position:", position)
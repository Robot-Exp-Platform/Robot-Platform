import math
import pybullet as p
from robot_pybullet.robot import Robot, Joint

FRAC_PI_4 = math.pi / 4
FRAC_PI_2 = math.pi / 2

PANDA_DOF = 7
PANDA_URDF = "franka_panda/panda.urdf"

PANDA_DEFAULT_JOINT = [0.0, -FRAC_PI_4, 0.0, -2.3562, 0.0, FRAC_PI_2, FRAC_PI_4]


class Panda(Robot):
    """franka panda"""

    def __init__(self, base_pose=None):
        super().__init__(base_pose)
        self.robot_id = p.loadURDF(
            PANDA_URDF, useFixedBase=True, basePosition=base_pose
        )
        self.njoint = PANDA_DOF
        self.default_q = PANDA_DEFAULT_JOINT
        for i in range(self.njoint):
            self.control_joint.append(Joint(self.robot_id, i))
        self.reset()

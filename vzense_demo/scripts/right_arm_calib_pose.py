#!/usr/bin/env python3
import numpy as np
from skrobot.viewers import TrimeshSceneViewer
from skrobot.viewers import PyrenderViewer
from skrobot.model import Axis
from vzense_demo.k_arm import KARM
from vzense_demo.k_arm_interface import KARMROSRobotInterface

robot_model = KARM()

for j in robot_model.joint_list:
    if j.max_joint_velocity == 0.0:
        j.max_joint_velocity = np.deg2rad(5)

viewer = PyrenderViewer()
viewer.add(robot_model)
viewer.show()
ri = KARMROSRobotInterface(robot_model)
robot_model.angle_vector(ri.angle_vector())


calib_pose_angles = np.array([ 4.9470838e-02, -1.5792319e+00,  1.2072419e+00, -1.7295620e+00,
                               -1.1977290e-04, -3.0139906e-06,  3.9547164e-04, -1.5707964e+00,
                               0.0000000e+00,  0.0000000e+00,  0.0000000e+00], dtype=np.float32)
robot_model.angle_vector(calib_pose_angles)

yes_or_no = input('Move robot? [Y/n]').lower()
if yes_or_no == 'y':
    ri.angle_vector(robot_model.angle_vector(), 10)
    ri.wait_interpolation()

import time
import numpy as np
import rospy
from skrobot.coordinates import Coordinates
from skrobot.coordinates import CascadedCoords

from vzense_demo.k_arm import KARM
from vzense_demo.k_arm_interface import KARMROSRobotInterface
from skrobot.viewers import TrimeshSceneViewer
from skrobot.viewers import PyrenderViewer
from skrobot.model import Axis
from skrobot.model import Link
from skrobot.model import FixedJoint
from skrobot.model import RotationalJoint
from vzense_demo.subscribers.bounding_box_array_subscriber import BoundingBoxArraySubscriber


robot_model = KARM()

# 関節速度を制限する設定。早く動かしたい場合は上限を緩和する
for j in robot_model.joint_list:
    if j.max_joint_velocity == 0.0:
        j.max_joint_velocity = np.deg2rad(5)

robot_model.init_pose()
robot_model.rotate(- np.pi / 2.0, 'y')
viewer = PyrenderViewer()  # viewer
viewer.add(robot_model)
viewer.show()

rarm_end_coords_axis = Axis.from_coords(robot_model.rarm_end_coords)
viewer.add(rarm_end_coords_axis)

rarm_elbow_end_coords_axis = Axis.from_coords(robot_model.rarm_elbow_end_coords)
viewer.add(rarm_elbow_end_coords_axis)

larm_end_coords_axis = Axis.from_coords(robot_model.larm_end_coords)
viewer.add(larm_end_coords_axis)

larm_elbow_end_coords_axis = Axis.from_coords(robot_model.larm_elbow_end_coords)
viewer.add(larm_elbow_end_coords_axis)

rarm_target_coords_axis = Axis()
viewer.add(rarm_target_coords_axis)

larm_target_coords_axis = Axis()
viewer.add(larm_target_coords_axis)


use_ri = True
send_time = 3
if use_ri:
    rospy.init_node('k_arm_demo')
    ri = KARMROSRobotInterface(robot_model)  # 実機との接続処理
    boxes_sub = BoundingBoxArraySubscriber('/box_fitting_node/output/boxes',
                                           start=False)
    robot_model.angle_vector(ri.angle_vector())  # 実機の関節角度をrobot_model (viewerにうつっているもの)に反映させる。


def ri2ir():
    robot_model.angle_vector(ri.angle_vector())


def calibrated_angle_vector(target_angles, time=10.0):
    target_angles = np.array(target_angles)
    robot_model.angle_vector(target_angles)
    ri.angle_vector(robot_model.angle_vector(), time)
    ri.wait_interpolation()
    current_angles = ri.angle_vector()
    target_angles += target_angles - current_angles
    ri.angle_vector(target_angles, time)  # robot_modelの姿勢を関節角度指令として実機に送る。
    ri.wait_interpolation()  # ロボットの関節指令の補間が終わるまで待つ。


viewer.redraw()  # viewerを更新する（viewerをクリックしても更新される)


robot_model.rarm.angle_vector(np.array(
    [0.6, -0.0316071, 1.4665161, -1.9041522], dtype=np.float32))  # 右腕のrobot_modelの関節値を4つ分更新
# robot_model.RARM_JOINT0.joint_angle(0.6)  # 個別に設定する場合はこのように関節Jointクラス(RARM_JOINT0)にjoint_angleというメソッドがあるのでこれで送る。
# 今の値を確認する場合は引数を与えずに関数を呼ぶrobot_model.RARM_JOINT0.joint_angle()
robot_model.larm.angle_vector(np.array(
    [0.6, -0.0316071, -1.4665161, -1.9041522], dtype=np.float32))
viewer.redraw()

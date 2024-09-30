#!/usr/bin/env python3

import numpy as np
import datetime
import skrobot
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import xyzw2wxyz
import rospy
from vzense_demo.k_arm import KARM
from vzense_demo.k_arm_interface import KARMROSRobotInterface
from skrobot.viewers import TrimeshSceneViewer

import cv2
import tf
import tf2_ros
from pybsc import load_yaml


launch_file_template = """<launch>

  <arg name="{from_frame}_frame_id" default="/{from_frame_id}" />
  <arg name="{to_frame}_frame_id" default="/{to_frame_id}" />
  <node name="{from_frame}_to_{to_frame}_static_transform_publisher"
        pkg="tf" type="static_transform_publisher"
        args="{x} {y} {z}
              {q_x} {q_y} {q_z} {q_w}
              $(arg {from_frame}_frame_id) $(arg {to_frame}_frame_id) 100" />

</launch>
"""


def write_launch_from_pose(
        R, t, from_frame, to_frame, from_frame_id, to_frame_id, filename):
    """

    Write launch file from pose

    Parameters
    ----------
    pose : np.ndarray or list
        [x, y, z, q_x, q_y, q_z, q_w]
    """
    coords = Coordinates(pos=t.reshape(-1), rot=R)
    s = launch_file_template.format(from_frame=from_frame,
                                    to_frame=to_frame,
                                    from_frame_id=from_frame_id,
                                    to_frame_id=to_frame_id,
                                    x=coords.translation[0],
                                    y=coords.translation[1],
                                    z=coords.translation[2],
                                    q_x=coords.quaternion[1],
                                    q_y=coords.quaternion[2],
                                    q_z=coords.quaternion[3],
                                    q_w=coords.quaternion[0])
    with open(filename, 'w') as f:
        f.write(s)


def handeye_calib(yaml_filepath, eye_in_hand=True):
    data = load_yaml(yaml_filepath)

    # create random pose
    R_gripper2base = []
    R_marker2camera = []
    t_gripper2base = []
    t_marker2camera = []
    for d in data.values():
        base_to_ee = d['base_to_ee_transform']
        coords = Coordinates(pos=base_to_ee[:3],
                             rot=xyzw2wxyz(base_to_ee[3:]))
        if eye_in_hand:
            gripper2base = coords
        else:
            gripper2base = coords.inverse_transformation()
        t_gripper2base.append(gripper2base.translation)
        R_gripper2base.append(gripper2base.rotation)

        camera_to_tag = d['camera_to_tag_transform']
        coords = Coordinates(pos=camera_to_tag[:3],
                             rot=xyzw2wxyz(camera_to_tag[3:]))
        if eye_in_hand:
            marker2camera = coords
        else:
            marker2camera = coords.inverse_transformation()
        R_marker2camera.append(marker2camera.worldrot())
        t_marker2camera.append(marker2camera.worldpos())
    R_base2marker, t_base2marker = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_marker2camera,
        t_marker2camera,
        method=cv2.CALIB_HAND_EYE_TSAI)
    return R_base2marker, t_base2marker



class DataCollector(object):

    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self.filedata_cnt = 0
        self.filedata = {}

        self.base_frame_id = rospy.get_param('~base_frame_id', 'BODY')
        self.ee_frame_id = rospy.get_param('~ee_frame_id', 'RARM_LINK3')
        self.camera_frame_id = rospy.get_param('~camera_frame_id',
                                               'rarm_hand_camera_link')
        self.marker_frame_id = rospy.get_param('~marker_frame_id', '6x5chessboard')

    def lookup_transform(self, from_frame_id, to_frame_id, stamp=None,
                         timeout=4.0):
        if stamp is None:
            stamp = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(
                from_frame_id, to_frame_id, stamp,
                rospy.Duration(timeout))
            (trans, rot) = self.tf_listener.lookupTransform(
                from_frame_id, to_frame_id, stamp)
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
                tf2_ros.TransformException,
                rospy.exceptions.ROSTimeMovedBackwardsException):
            return None
        return (trans, rot)

    def save_data(self, filename='/tmp/calib.yaml'):
        import yaml
        with open(filename, 'w') as f:
            f.write(yaml.dump(self.filedata))

    def store_data(self):
        stamp = rospy.Time.now()
        fdata = {}
        fdata['stamp'] = float(rospy.Time.now().to_sec())
        camera_to_marker_transform = self.lookup_transform(
            self.camera_frame_id, self.marker_frame_id, stamp,
            timeout=10.0)
        base_to_ee_transform = self.lookup_transform(
            self.base_frame_id, self.ee_frame_id, stamp)
        if camera_to_marker_transform is None or \
           base_to_ee_transform is None:
            return False
        fdata['camera_to_tag_transform'] = \
            np.append(camera_to_marker_transform[0],
                      camera_to_marker_transform[1]).tolist()
        fdata['base_to_ee_transform'] = \
            np.append(base_to_ee_transform[0],
                      base_to_ee_transform[1]).tolist()
        self.filedata[self.filedata_cnt] = fdata
        self.filedata_cnt += 1


r = KARM()
for j in r.joint_list:
    if j.max_joint_velocity == 0.0:
        j.max_joint_velocity = np.deg2rad(5)
ri = KARMROSRobotInterface(r)
viewer = TrimeshSceneViewer()
viewer.add(r)

r.angle_vector(ri.angle_vector())
ri.angle_vector(r.reset_pose(), 5.0)
ri.wait_interpolation()

dc = DataCollector()


RARM_JOINT0_MIN = 0
RARM_JOINT0_MAX = 0
RARM_JOINT1_MIN = 0
RARM_JOINT1_MAX = 0
RARM_JOINT2_MIN = 0
RARM_JOINT2_MAX = 0
RARM_JOINT3_MIN = 0
RARM_JOINT3_MAX = 0


# n_divide = 3
# n_pose = n_divide ** 4
# cnt = 0
# for angle0 in np.linspace(RARM_JOINT0_MIN, RARM_JOINT0_MAX):
#     r.RARM_JOINT0.joint_angle(angle0)
#     for angle1 in np.linspace(RARM_JOINT1_MIN, RARM_JOINT1_MAX):
#         r.RARM_JOINT1.joint_angle(angle1)
#         for angle2 in np.linspace(RARM_JOINT2_MIN, RARM_JOINT2_MAX):
#             r.RARM_JOINT2.joint_angle(angle2)
#             for angle3 in np.linspace(RARM_JOINT3_MIN, RARM_JOINT3_MAX):
#                 r.RARM_JOINT3.joint_angle(angle3)
#                 ri.angle_vector(r.angle_vector(), 5.0)
#                 ri.wait_interpolation()
#                 rospy.sleep(2.0)
#                 dc.store_data()
#                 estimated_finish_time = datetime.datetime.now() + \
#                     datetime.timedelta(seconds=(n_pose - cnt - 1) * 8)
#                 print("estimated finish time {}".format(estimated_finish_time))
#                 cnt += 1
# dc.save_data()

dc.save_data()

R_base2marker, t_base2marker = handeye_calib(
    '/tmp/calib.yaml')

write_launch_from_pose(R_base2marker, t_base2marker,
                       'r_wrist',
                       'rarm_camera',
                       dc.ee_frame_id,
                       dc.camera_frame_id,
                       '/tmp/hoge.launch')

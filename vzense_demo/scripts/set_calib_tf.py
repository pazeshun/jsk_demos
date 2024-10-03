#!/usr/bin/env python3

import argparse

import tf
import tf2_ros
import cv2
import sys
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import xyzw2wxyz
from skrobot.coordinates.math import wxyz2xyzw
import rospkg
from pathlib import Path
import rospy
from dynamic_tf_publisher.srv import SetDynamicTF
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion


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
        coords, from_frame_id, to_frame_id, filename):
    """

    Write launch file from pose

    Parameters
    ----------
    pose : np.ndarray or list
        [x, y, z, q_x, q_y, q_z, q_w]
    """
    s = launch_file_template.format(from_frame=from_frame_id,
                                    to_frame=to_frame_id,
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



def set_tf(pos, q, parent, child, freq):
    rospy.wait_for_service('/set_dynamic_tf')
    try:
        client = rospy.ServiceProxy('/set_dynamic_tf', SetDynamicTF)
        pose = TransformStamped()
        pose.header.frame_id = parent
        pose.child_frame_id = child
        pose.transform.translation = Vector3(*pos)
        pose.transform.rotation = Quaternion(*q)
        res = client(freq, pose)
        return
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--results-path', default='calib_results')
    parser.add_argument('--to-frame-id', default='right_vzense_camera_frame')
    parser.add_argument('--for-robot', action='store_true')
    args = parser.parse_args()

    rospy.init_node('set_calib_tf')
    rospack = rospkg.RosPack()
    package_path = Path(rospack.get_path('vzense_demo'))
    file_path = package_path / f'{args.results_path}/Results/calibrated_cameras_data.yml'
    if not file_path.exists():
        print(f'{file_path} not found')
        sys.exit(1)
    fs = cv2.FileStorage(str(file_path), cv2.FILE_STORAGE_READ)

    camera_1_pose_matrix = fs.getNode('camera_1').getNode(
        'camera_pose_matrix').mat()
    print("Camera 1 pose matrix:")
    print(camera_1_pose_matrix)
    coords = Coordinates(camera_1_pose_matrix)
    if args.for_robot is not True:
        set_tf(coords.translation,
               list(coords.quaternion[1:]) + [coords.quaternion[0]],
               'left_vzense_camera_frame',
               args.to_frame_id,
               100)
        write_launch_from_pose(
            coords,
            'left_vzense_camera_frame',
            args.to_frame_id,
            package_path / 'launch' / 'config.launch'
        )
    else:
        tf_listener = tf.TransformListener()
        rospy.sleep(2.0)

        def lookup_transform(from_frame_id, to_frame_id, stamp=None,
                             timeout=10.0):
            if stamp is None:
                stamp = rospy.Time.now()
            try:
                tf_listener.waitForTransform(
                    from_frame_id, to_frame_id, stamp,
                    rospy.Duration(timeout))
                (translation, quaternion_xyzw) = tf_listener.lookupTransform(
                    from_frame_id, to_frame_id, stamp)
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.TransformException,
                    rospy.exceptions.ROSTimeMovedBackwardsException):
                return None, None
            return (translation, quaternion_xyzw)

        print('set tf: left_vzense_camera_frame -> BODY')
        translation, quaternion_xyzw = lookup_transform('BODY', args.to_frame_id)
        body_to_hand_camera = Coordinates(pos=translation, rot=xyzw2wxyz(quaternion_xyzw))
        left_camera_to_body = coords.copy_worldcoords().transform(body_to_hand_camera.inverse_transformation())
        set_tf(left_camera_to_body.translation,
               wxyz2xyzw(left_camera_to_body.quaternion),
               'left_vzense_camera_frame',
               'BODY',
               100)
        write_launch_from_pose(
            left_camera_to_body,
            'left_vzense_camera_frame',
            'BODY',
            package_path / 'launch' / 'right_roobt_config.launch'
        )

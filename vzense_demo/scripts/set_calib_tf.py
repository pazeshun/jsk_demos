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
    file_path = Path(rospack.get_path('vzense_demo')) / f'{args.results_path}/Results/calibrated_cameras_data.yml'
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
        translation, quaternion_xyzw = lookup_transform(args.to_frame_id, 'BODY')
        hand_camera_to_body = Coordinates(pos=translation, rot=xyzw2wxyz(quaternion_xyzw))
        left_camera_to_body = coords.copy_worldcoords().transform(hand_camera_to_body)
        set_tf(coords.translation,
               wxyz2xyzw(left_camera_to_body.quaternion),
               'left_vzense_camera_frame',
               'BODY',
               100)

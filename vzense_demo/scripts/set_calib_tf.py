#!/usr/bin/env python3

import cv2
import sys
from skrobot.coordinates import Coordinates
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
    rospy.init_node('set_calib_tf')
    rospack = rospkg.RosPack()
    file_path = Path(rospack.get_path('vzense_demo')) / 'calib_results/Results/calibrated_cameras_data.yml'
    if not file_path.exists():
        print(f'{file_path} not found')
        sys.exit(1)
    fs = cv2.FileStorage(str(file_path), cv2.FILE_STORAGE_READ)

    camera_1_pose_matrix = fs.getNode('camera_1').getNode(
        'camera_pose_matrix').mat()
    print("Camera 1 pose matrix:")
    print(camera_1_pose_matrix)
    coords = Coordinates(camera_1_pose_matrix)
    set_tf(coords.translation,
           list(coords.quaternion[1:]) + [coords.quaternion[0]],
           'left_vzense_camera_frame',
           'right_vzense_camera_frame',
           10)

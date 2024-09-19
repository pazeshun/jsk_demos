#!/usr/bin/env python3

import cv2
import numpy as np
import cv2.aruco as aruco
from jsk_topic_tools import ConnectionBasedTransport
import rospy
import cv_bridge
from sensor_msgs.msg import Image


def detect_charuco_board(image):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
    squares_x = 5
    squares_y = 5
    square_length = 0.04
    marker_length = 0.03

    parameters = aruco.DetectorParameters()
    charuco_board = aruco.CharucoBoard(
        size=[squares_x, squares_y],
        squareLength=square_length,
        markerLength=marker_length,
        dictionary=aruco_dict)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_img_points = aruco.detectMarkers(
        gray, aruco_dict,
        parameters=parameters)
    if ids is not None:
        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, charuco_board)
        if retval > 0:
            aruco.drawDetectedMarkers(image, corners, ids)
            aruco.drawDetectedCornersCharuco(
                image, charuco_corners, charuco_ids)
    return image


class ArucoMarkerDetector(ConnectionBasedTransport):

    def __init__(self):
        super(ArucoMarkerDetector, self).__init__()
        self.bridge = cv_bridge.CvBridge()
        self.pub_img = self.advertise(
            '~image', Image, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber(
            '~image',
            Image, self.callback,
            queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def callback(self, img_msg):
        bridge = self.bridge
        img = bridge.imgmsg_to_cv2(img_msg)
        img = np.array(img)
        out_img = detect_charuco_board(img)
        out_img_msg = bridge.cv2_to_imgmsg(out_img, encoding=img_msg.encoding)
        out_img_msg.header = img_msg.header
        self.pub_img.publish(out_img_msg)


if __name__ == '__main__':
    rospy.init_node('aruco_marker_detector')
    node = ArucoMarkerDetector()
    rospy.spin()

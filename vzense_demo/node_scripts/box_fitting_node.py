#!/usr/bin/env python

import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import DepthImageFilterConfig as Config
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import cv2
from cameramodels import PinholeCameraModel
from skrobot.coordinates.math import rotation_matrix_from_axis
from skrobot.coordinates.math import wxyz2xyzw
from skrobot.coordinates.math import matrix2quaternion


class BoxFittingNode(ConnectionBasedTransport):

    def __init__(self):
        super(BoxFittingNode, self).__init__()

        rospy.loginfo('Waiting for camera info...')
        msg = rospy.wait_for_message('/virtual_camera_mono/camera_info', CameraInfo)
        rospy.loginfo('Camera info received.')
        self.cm = PinholeCameraModel.from_camera_info(msg)

        self.bridge = cv_bridge.CvBridge()
        self.srv = Server(Config, self.config_callback)
        self.pub_pose = self.advertise('~box_pose', PoseStamped, queue_size=1)

    def subscribe(self):
        self.sub = rospy.Subscriber('/depth_image_creator/output', Image, self.callback, queue_size=1, buff_size=2**24)

    def unsubscribe(self):
        self.sub.unregister()

    def config_callback(self, config, level):
        self.depth_threshold = config.threshold
        self.negative = config.negative
        return config

    def callback(self, depth_img_msg):
        bridge = self.bridge

        supported_encodings = {'16UC1', '32FC1', '64FC1'}
        if depth_img_msg.encoding not in supported_encodings:
            rospy.logwarn('Unsupported depth image encoding: {0}'.format(depth_img_msg.encoding))

        depth = bridge.imgmsg_to_cv2(depth_img_msg)
        if depth_img_msg.encoding == '16UC1':
            depth = depth / 1000.0  # convert metric: mm -> m

        mask = np.zeros(depth.shape, dtype=np.uint8)
        mask_idx = np.logical_and(0 < depth, depth < self.depth_threshold)
        if self.negative is True:
            mask_idx = np.logical_not(mask_idx)
        mask[mask_idx] = 255
        mask = mask.astype(np.uint8)

        # モルフォロジー変換でノイズを除去する
        kernel = np.ones((3, 3), np.uint8)
        binary_cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # ノイズ除去後の画像の非ゼロ（点がある）部分の座標を取得
        points = np.column_stack(np.where(binary_cleaned > 0))

        if len(points) == 0:
            rospy.logwarn("No valid points found in the depth image.")
            return

        # 点群に最小外接矩形をフィッティングする
        rect = cv2.minAreaRect(points)

        # rectの出力は ((center_y, center_x), (幅, 高さ), 角度) の形式
        (center_y, center_x), (width, height), angle = rect
        angle = - angle

        # 深度画像から中心点の深度を取得
        z = np.mean(depth[binary_cleaned > 0])

        # 2Dのピクセル座標を3D座標に変換 (カメラモデルを使用)
        point_3d = self.cm.project_pixel_to_3d_ray((center_x, center_y))
        point_3d = [p * z for p in point_3d]  # スケールに深度をかけて3D空間の座標にする

        # 角度方向に進んだ座標を計算
        length = 10  # 適当に進む距離を指定
        offset_x = length * np.cos(np.deg2rad(angle))
        offset_y = length * np.sin(np.deg2rad(angle))

        # 新しい座標を計算
        new_pixel_x = center_x + offset_x
        new_pixel_y = center_y + offset_y

        # 進んだ先の座標も3Dに投影
        new_point_3d = self.cm.project_pixel_to_3d_ray((new_pixel_x, new_pixel_y))
        new_point_3d = [p * z for p in new_point_3d]  # スケールに深度をかけて3D空間にする

        # ベクトルを計算
        direction_vector = np.array(new_point_3d) - np.array(point_3d)

        # Z方向を定義
        z_axis = np.array([0, 0, 1])

        # 回転行列を求める
        rotation_matrix = rotation_matrix_from_axis(first_axis=direction_vector, second_axis=z_axis, axes='xz')

        # 回転行列をクオータニオンに変換する
        quaternion = wxyz2xyzw(matrix2quaternion(rotation_matrix))

        # PoseStampedメッセージを作成
        pose_msg = PoseStamped()
        pose_msg.header = depth_img_msg.header  # タイムスタンプとフレームIDを一致させる

        # 位置 (3D座標)
        pose_msg.pose.position.x = point_3d[0]
        pose_msg.pose.position.y = point_3d[1]
        pose_msg.pose.position.z = point_3d[2]

        # 姿勢 (クオータニオン)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        # PoseStampedをパブリッシュ
        self.pub_pose.publish(pose_msg)

        # デバッグ用
        rospy.loginfo(f'Center 3D: {point_3d}, Angle: {angle} degrees')


if __name__ == '__main__':
    rospy.init_node('box_fitting_node')
    node = BoxFittingNode()
    rospy.spin()

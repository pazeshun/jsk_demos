#!/usr/bin/env python

from scipy import stats
import cv_bridge
from dynamic_reconfigure.server import Server
from jsk_perception.cfg import DepthImageFilterConfig as Config
from jsk_topic_tools import ConnectionBasedTransport
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import BoundingBox
import cv2
from cameramodels import PinholeCameraModel
from skrobot.coordinates.math import rotation_matrix_from_axis
from skrobot.coordinates.math import wxyz2xyzw
from skrobot.coordinates.math import matrix2quaternion
from skrobot.coordinates.base import Coordinates


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
        self.pub_mask = self.advertise(
            '~output/mask', Image, queue_size=1)
        self.pub_boxes = self.advertise('~output/boxes', BoundingBoxArray,
                                        queue_size=1)

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

        if self.pub_mask.get_num_connections() > 0:
            mask_msg = bridge.cv2_to_imgmsg(binary_cleaned, encoding='mono8')
            mask_msg.header = depth_img_msg.header
            self.pub_mask.publish(mask_msg)

        if self.pub_pose.get_num_connections() == 0 and self.pub_boxes.get_num_connections() == 0:
            return

        # ノイズ除去後の画像の非ゼロ（点がある）部分の座標を取得
        points = np.column_stack(np.where(binary_cleaned > 0))

        if len(points) == 0:
            rospy.logwarn("No valid points found in the depth image.")
            return

        # 外れ値を除去する（Zスコアで外れ値を検出）
        z_scores = stats.zscore(points, axis=0)
        filtered_points = points[(np.abs(z_scores) < 2).all(axis=1)]

        if len(filtered_points) == 0:
            rospy.logwarn("No valid points after outlier removal.")
            return

        # 点群に最小外接矩形をフィッティングする
        rect = cv2.minAreaRect(filtered_points)

        # rectの出力は ((center_y, center_x), (幅, 高さ), 角度) の形式
        (center_y, center_x), (width, height), angle = rect

        # 有効な深度値を取得（空間に対応する大きな深度を除外）
        valid_depths = depth[binary_cleaned > 0]
        valid_depths = valid_depths[valid_depths < self.depth_threshold]  # 空間の深度を除去

        if len(valid_depths) > 0:
            # ヒストグラムを生成（深度の分布を把握）
            hist, bin_edges = np.histogram(valid_depths, bins=50, range=(np.min(valid_depths), np.max(valid_depths)))

            # 一定の頻度以上の深度を採用（例: ヒストグラム内の点数が100以上の範囲を選択）
            significant_bins = bin_edges[:-1][hist > 100]  # 100個以上の点がある深度を採用
            if len(significant_bins) > 0:
                # 最も近い深度範囲の下限を取得
                z_nearest = significant_bins[0]
            else:
                rospy.logwarn("No significant depth values found.")
                return
        else:
            rospy.logwarn("No valid depths found.")
            return

        # 最も近い深度のピクセル位置を特定する
        nearest_pixel_idx = np.where(np.isclose(depth, z_nearest, atol=0.01))  # 最も近い深度に近いピクセルを取得
        z = z_nearest

        box_x = (width / self.cm.fx) * z  # 実際の奥行き（z）でスケーリング
        box_y = (height / self.cm.fy) * z  # 実際の奥行き（z）でスケーリング

        # 長い方をx軸に、短い方をy軸に設定するため、widthとheightを比較
        if box_x > box_y:
            angle += 90
        else:
            box_x, box_y = box_y, box_x
        rospy.loginfo(f'Box width: {box_x} height: {box_y}')
        angle = - angle

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

        real_box_x = 0.5
        real_box_y = 0.3
        real_box_z = 0.3

        if (box_x - real_box_x) > 0.1 or (box_y - real_box_y) > 0.1:
            rospy.logwarn('Estimated box is invalid.')
            return

        coords = Coordinates(pos=point_3d, rot=rotation_matrix)
        coords.translate((0, 0, real_box_z / 2.0), 'local')
        rotation_matrix = coords.rotation
        point_3d = coords.translation

        quaternion = wxyz2xyzw(matrix2quaternion(rotation_matrix))

        if self.pub_boxes.get_num_connections() > 0:
            boxes_msg = BoundingBoxArray(header=depth_img_msg.header)
            box_msg = BoundingBox(header=depth_img_msg.header)
            box_msg.pose.position.x = point_3d[0]
            box_msg.pose.position.y = point_3d[1]
            box_msg.pose.position.z = point_3d[2]
            box_msg.pose.orientation.x = quaternion[0]
            box_msg.pose.orientation.y = quaternion[1]
            box_msg.pose.orientation.z = quaternion[2]
            box_msg.pose.orientation.w = quaternion[3]
            box_msg.dimensions.x = real_box_x
            box_msg.dimensions.y = real_box_y
            box_msg.dimensions.z = real_box_z
            boxes_msg.boxes.append(box_msg)
            self.pub_boxes.publish(boxes_msg)

        # PoseStampedをパブリッシュ
        if self.pub_pose.get_num_connections() > 0:
            # PoseStampedメッセージを作成
            pose_msg = PoseStamped(header=depth_img_msg.header)

            # 位置 (3D座標)
            pose_msg.pose.position.x = point_3d[0]
            pose_msg.pose.position.y = point_3d[1]
            pose_msg.pose.position.z = point_3d[2]

            # 姿勢 (クオータニオン)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]
            self.pub_pose.publish(pose_msg)

        # デバッグ用
        rospy.loginfo(f'Center 3D: {point_3d}, Angle: {angle} degrees')


if __name__ == '__main__':
    rospy.init_node('box_fitting_node')
    node = BoxFittingNode()
    rospy.spin()

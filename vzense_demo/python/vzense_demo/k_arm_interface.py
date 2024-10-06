import actionlib
import control_msgs.msg
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolRequest

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
from vzense_demo.k_arm import KARM


class HandInterface(object):

    def __init__(self, hand='rhand'):
        self.init_hand(hand)

    def init_hand(self, hand):
        self.hand = hand
        self.jnt_names = []
        for finger in ['thumb', 'index']:
            self.jnt_names.append(
                '{}_{}_base_joint'.format(self.hand, finger))
            self.jnt_names.append(
                '{}_{}_joint'.format(self.hand, finger))
        self.jnt_pos = None
        self.rarm_traj_pub = rospy.Publisher(
            '{}/joint_trajectory'.format(self.hand),
            JointTrajectory,
            queue_size=1,
        )
        self.enable_int_pubs = []
        self.enable_tof_pubs = []
        for finger in ['thumb', 'index']:
            for pos in ['tip', 'root']:
                self.enable_int_pubs.append(
                    rospy.Publisher(
                        '{}/{}/distal/{}/enable_intensity'.format(
                            self.hand, finger, pos),
                        Bool,
                        queue_size=1,
                        latch=True,
                    )
                )
                self.enable_tof_pubs.append(
                    rospy.Publisher(
                        '{}/{}/distal/{}/enable_tof'.format(
                            self.hand, finger, pos),
                        Bool,
                        queue_size=1,
                        latch=True,
                    )
                )
        set_init_i_srv_names = []
        for finger in ['thumb', 'index']:
            for pos in ['tip', 'root']:
                set_init_i_srv_names.append(
                    '{}/{}/distal/{}/intensity_model_acquisition/set_init_intensity'.format(
                        self.hand, finger, pos)
                )
        self.set_init_i_srvs = []
        for srv_name in set_init_i_srv_names:
            rospy.wait_for_service(srv_name)
            self.set_init_i_srvs.append(
                rospy.ServiceProxy(srv_name, Trigger))

    def move_hand(self, target_pos, time=1, wait_time=0):
        rospy.loginfo('Current joint position: {}'.format(self.jnt_pos))
        rospy.loginfo('Target joint position: {}'.format(target_pos))
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time(0)  # Immediate execution
        traj_msg.joint_names = self.jnt_names
        traj_msg.points.append(
            JointTrajectoryPoint(
                positions=target_pos,
                time_from_start=rospy.Duration(time),
            )
        )
        self.rarm_traj_pub.publish(traj_msg)
        if wait_time > 0:
            rospy.sleep(wait_time)

    def start_grasp(self, time=1, wait_time=0,
                    angle=90):
        angle = max(min(angle, 130), 0)
        self.move_hand([-np.pi/2, np.deg2rad(angle), 0, np.deg2rad(angle)],
                       time=time, wait_time=wait_time)

    def stop_grasp(self, time=1, wait_time=0):
        self.move_hand([-np.pi/2, 0, 0, 0],
                       time=time, wait_time=wait_time)

    def grasp_box_pose(self):
        self.move_hand([0, 0, -np.deg2rad(30), 0], wait_time=5)
        # self.move_hand([0, np.deg2rad(150), -np.deg2rad(30), np.deg2rad(0)], wait_time=4)
        self.move_hand([0, np.deg2rad(150), -np.deg2rad(30), np.deg2rad(130)], wait_time=0)

    def servo_on(self):
        service_name = "{}/servo_state_manager/servo".format(self.hand)
        rospy.loginfo('{} servo on called. waiting service {}'.format(self.hand, service_name))
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, SetBool)
        ret = service(SetBoolRequest(data=True))
        rospy.loginfo('{} servo on successfully called'.format(self.hand))
        return ret

    def servo_off(self):
        service_name = "{}/servo_state_manager/servo".format(self.hand)
        rospy.loginfo('{} servo off called. waiting service {}'.format(self.hand, service_name))
        rospy.wait_for_service(service_name)
        service = rospy.ServiceProxy(service_name, SetBool)
        ret = service(SetBoolRequest(data=False))
        rospy.loginfo('{} servo off successfully called'.format(self.hand))
        return ret


class KARMROSRobotInterface(ROSRobotInterfaceBase):

    def __init__(self, *args, **kwargs):
        super(KARMROSRobotInterface, self).__init__(*args, **kwargs)
        self.rhand = HandInterface('rhand')

    @property
    def rarm_controller(self):
        return dict(
            controller_type='rarm_controller',
            controller_action='rarm_controller/follow_joint_trajectory_action',  # NOQA
            controller_state='rarm_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=[
                "RARM_JOINT0",
                "RARM_JOINT1",
                "RARM_JOINT2",
                "RARM_JOINT3",
                "RARM_JOINT4",
                "RARM_JOINT5",
                "RARM_JOINT6",
            ],
        )


    @property
    def larm_controller(self):
        return dict(
            controller_type='larm_controller',
            controller_action='larm_controller/follow_joint_trajectory_action',  # NOQA
            controller_state='larm_controller/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=[
                "LARM_JOINT0",
                "LARM_JOINT1",
                "LARM_JOINT2",
                "LARM_JOINT3",
                "LARM_JOINT4",
                "LARM_JOINT5",
                "LARM_JOINT6",
            ],
        )

    def default_controller(self):
        return [self.rarm_controller,
                self.larm_controller]

    def start_grasp(self):
        self.move_hand([-np.pi/2, np.pi/2, 0, np.pi/2], 1, 5)

    def stop_grasp(self):
        self.move_hand([-np.pi/2, 0, 0, 0], 1, 5)


if __name__ == '__main__':
    from skrobot.viewers import TrimeshSceneViewer
    from skrobot.viewers import PyrenderViewer
    from skrobot.model import Axis
    robot_model = KARM()

    for j in robot_model.joint_list:
        if j.max_joint_velocity == 0.0:
            j.max_joint_velocity = np.deg2rad(5)

    robot_model.init_pose()
    robot_model.rotate(- np.pi / 2.0, 'y')
    viewer = PyrenderViewer()
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

    rarm_target_coords = Axis()
    viewer.add(rarm_target_coords)

    ri = KARMROSRobotInterface(robot_model)
    robot_model.angle_vector(ri.angle_vector())
    viewer.redraw()

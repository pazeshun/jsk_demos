import os
import sys

from cached_property import cached_property
import numpy as np

from skrobot.coordinates import CascadedCoords
from skrobot.model import RobotModel

from skrobot.models.urdf import RobotModelFromURDF


def convert_to_str(x):
    if isinstance(x, str):
        pass
    elif isinstance(x, bytes):
        x = x.decode('utf-8')
    else:
        raise ValueError(
            'Invalid input x type: {}'
            .format(type(x)))
    return x


class KARM(RobotModelFromURDF):

    def __init__(self, *args, **kwargs):
        if not ('urdf_file' in kwargs or 'urdf' in kwargs):
            kwargs['urdf'] = self._urdf()
        super(KARM, self).__init__(*args, **kwargs)
        self.name = 'karm'

        self.RARM_JOINT0.min_angle = -1.0
        self.RARM_JOINT0.max_angle = 1.0
        self.RARM_JOINT1.min_angle = -1.1
        self.RARM_JOINT1.max_angle = 0.2

        # self.rarm_end_coords = CascadedCoords(
        #     parent=self.RARM_hand_base,
        #     name='rarm_end_coords',
        #     pos=[0.015, -0.04, 0.2])
        self.rarm_end_coords = CascadedCoords(
            parent=self.RARM_LINK3,
            name='rarm_end_coords',
            pos=[-0.015, -0.04, -0.52]).rotate(
                np.pi / 2.0, 'y').rotate(np.pi / 2.0, 'x')

        self.larm_end_coords = CascadedCoords(
            parent=self.LARM_LINK3,
            name='larm_end_coords',
            pos=[-0.015, -0.04, -0.52]).rotate(
                np.pi / 2.0, 'y').rotate(np.pi / 2.0, 'x')

        self.rarm_elbow_end_coords = CascadedCoords(
            parent=self.RARM_LINK4,
            name='rarm_elbow_end_coords', pos=(0, -0.08, -0.02)).rotate(-np.pi / 2.0, 'z').rotate(np.pi / 2.0, 'x')

        self.larm_elbow_end_coords = CascadedCoords(
            parent=self.LARM_LINK4,
            name='larm_elbow_end_coords', pos=(0, -0.08, -0.02)).rotate(-np.pi / 2.0, 'z').rotate(np.pi / 2.0, 'x')

    @cached_property
    def rarm(self):
        rarm_links = [
            self.BODY,
            self.RARM_LINK0,
            self.RARM_LINK1,
            self.RARM_LINK2,
            self.RARM_LINK3,
            # self.RARM_LINK4,
            # self.RARM_LINK5,
            # self.RARM_LINK6,
        ]
        rarm_joints = []
        for link in rarm_links:
            if hasattr(link, 'joint') and link.joint is not None:
                rarm_joints.append(link.joint)
        r = RobotModel(link_list=rarm_links, joint_list=rarm_joints)
        r.end_coords = self.rarm_end_coords
        return r

    @cached_property
    def larm(self):
        larm_links = [
            self.BODY,
            self.LARM_LINK0,
            self.LARM_LINK1,
            self.LARM_LINK2,
            self.LARM_LINK3,
            # self.LARM_LINK4,
            # self.LARM_LINK5,
            # self.LARM_LINK6,
        ]
        larm_joints = []
        for link in larm_links:
            if hasattr(link, 'joint') and link.joint is not None:
                larm_joints.append(link.joint)
        r = RobotModel(link_list=larm_links, joint_list=larm_joints)
        r.end_coords = self.larm_end_coords
        return r

    def reset_pose(self):
        self.RARM_JOINT0.joint_angle(0.8)
        self.RARM_JOINT1.joint_angle(-0.5)
        self.RARM_JOINT3.joint_angle(-1.3)

    def init_pose(self):
        self.angle_vector(np.zeros_like(self.angle_vector()))
        self.RARM_hand_thumb_base_joint.joint_angle(-np.pi / 2.0)
        return self.angle_vector()

    def _urdf(self):
        import rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('vzense_demo')
        self.resolve_filepath = os.path.join(
            package_path, 'model', 'robot_model.urdf')
        if not os.path.exists(self.resolve_filepath):
            print('You should run "roscd vzense_demo/model && xacro robot_model.xacro > robot_model.urdf"')
            sys.exit(1)
        with open(self.resolve_filepath, 'rb') as f:
            urdf_text = f.read()
        urdf_text = convert_to_str(urdf_text)
        return urdf_text


if __name__ == '__main__':
    from skrobot.viewers import PyrenderViewer
    from skrobot.model import Axis
    robot_model = KARM()
    robot_model.init_pose()
    robot_model.rotate(- np.pi / 2.0, 'y')
    # robot_model.rotate(np.pi / 2.0, 'x', 'world')
    robot_model.rotate(-np.pi / 2.0, 'x', 'world')
    viewer = PyrenderViewer()
    viewer.add(robot_model)
    viewer.show()
    rarm_end_coords_axis = Axis.from_coords(robot_model.rarm_end_coords)
    viewer.add(rarm_end_coords_axis)

    rarm_elbow_end_coords_axis = Axis.from_coords(robot_model.rarm_elbow_end_coords)
    viewer.add(rarm_elbow_end_coords_axis)

    larm_end_coords_axis = Axis.from_coords(robot_model.larm_end_coords)
    viewer.add(larm_end_coords_axis)

    rarm_target_coords = Axis()
    viewer.add(rarm_target_coords)
    # rarm_target_coords.newcoords(coords.copy_worldcoords().translate((-0.2, 0.1, 0.1)))

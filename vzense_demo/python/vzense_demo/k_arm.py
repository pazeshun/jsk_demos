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

        self.rarm_end_coords = CascadedCoords(
            parent=self.RARM_hand_base,
            name='rarm_end_coords',
            pos=[0.015, -0.04, 0.2])

    @cached_property
    def rarm(self):
        rarm_links = [
            self.BODY,
            self.RARM_LINK0,
            self.RARM_LINK1,
            self.RARM_LINK2,
            self.RARM_LINK3,
            self.RARM_LINK4,
            self.RARM_LINK5,
            self.RARM_LINK6,
        ]
        rarm_joints = []
        for link in rarm_links:
            if hasattr(link, 'joint') and link.joint is not None:
                rarm_joints.append(link.joint)
        r = RobotModel(link_list=rarm_links, joint_list=rarm_joints)
        r.end_coords = self.rarm_end_coords
        return r

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
    from skrobot.viewers import TrimeshSceneViewer
    from skrobot.model import Axis
    robot_model = KARM()
    robot_model.init_pose()
    viewer = TrimeshSceneViewer()
    viewer.add(robot_model)
    viewer.show()
    rarm_end_coords_axis = Axis.from_coords(robot_model.rarm_end_coords)
    viewer.add(rarm_end_coords_axis)

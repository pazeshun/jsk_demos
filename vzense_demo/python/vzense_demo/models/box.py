from enum import IntEnum

import numpy as np
import skrobot


class BoundingBoxVertexType(IntEnum):
    FrontTopRight = 0
    FrontTopLeft = 1
    FrontBottomLeft = 2
    FrontBottomRight = 3
    RearTopRight = 4
    RearTopLeft = 5
    RearBottomLeft = 6
    RearBottomRight = 7
    Center = 8
    TotalCornerVertexCount = 8
    # Corner vertexes doesn't include the center point
    TotalVertexCount = 9


class BoundingBox(skrobot.coordinates.CascadedCoords):

    def __init__(self, *args, **kwargs):
        self.dimensions = kwargs.pop('dimensions', [1.0, 1.0, 1.0])
        super(BoundingBox, self).__init__(*args, **kwargs)

    @property
    def dimensions(self):
        return self._dimensions

    @dimensions.setter
    def dimensions(self, dim):
        self._dimensions = np.array(dim, 'f')

    @property
    def vertices_coords(self):
        center_coords = self.copy_worldcoords()
        dx, dy, dz = self.dimensions / 2.0
        add_y = 1.0
        vertices_coords = [
            # front top right
            center_coords.copy_worldcoords().translate((dx, dy, dz)),
            # front top left
            center_coords.copy_worldcoords().translate((dx, -dy - add_y, dz)),
            # front bottom left
            center_coords.copy_worldcoords().translate((dx, -dy - add_y, -dz)),
            # front bottom right
            center_coords.copy_worldcoords().translate((dx, dy, -dz)),
            # rear top right
            center_coords.copy_worldcoords().translate((-dx, dy,  dz)),
            # rear top left
            center_coords.copy_worldcoords().translate((-dx, -dy - add_y, dz)),
            # rear bottom left
            center_coords.copy_worldcoords().translate(
                (-dx, -dy - add_y, -dz)),
            # rear bottom right
            center_coords.copy_worldcoords().translate((-dx, dy, -dz)),
            # center
            center_coords.copy_worldcoords(),
        ]
        return vertices_coords

    @property
    def vertices(self):
        return np.array([c.worldpos() for c in self.vertices_coords],
                        dtype=np.float64)

    @staticmethod
    def from_ros_message(box_msg):
        trans = (box_msg.pose.position.x,
                 box_msg.pose.position.y,
                 box_msg.pose.position.z)
        q = (box_msg.pose.orientation.w,
             box_msg.pose.orientation.x,
             box_msg.pose.orientation.y,
             box_msg.pose.orientation.z)
        if box_msg.pose.orientation.w == 0 \
           and box_msg.pose.orientation.x == 0 \
           and box_msg.pose.orientation.y == 0 \
           and box_msg.pose.orientation.z == 0:
            box_msg.pose.orientation.w = 1.0

        box = BoundingBox(
            pos=trans, rot=q,
            dimensions=(box_msg.dimensions.x,
                        box_msg.dimensions.y,
                        box_msg.dimensions.z))
        return box

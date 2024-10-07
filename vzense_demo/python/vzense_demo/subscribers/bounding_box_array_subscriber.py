from enum import IntEnum

import jsk_recognition_msgs.msg
import numpy as np
import skrobot

from vzense_demo.subscriber import TopicSubscriber
from vzense_demo.models.box import BoundingBox


class BoundingBoxArraySubscriber(TopicSubscriber):

    def __init__(self,
                 topic_name,
                 one_shot=False,
                 start=True,
                 wait=False):
        super(BoundingBoxArraySubscriber, self).__init__(
            topic_name,
            jsk_recognition_msgs.msg.BoundingBoxArray,
            one_shot=one_shot,
            start=start,
            wait=wait)

    def to_coords(self, timeout=10.0):
        msg = self.wait_new_message(timeout=timeout)
        if msg is None:
            return None
        return [BoundingBox.from_ros_message(box)
                for box in self.msg.boxes]

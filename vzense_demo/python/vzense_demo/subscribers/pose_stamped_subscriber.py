import geometry_msgs.msg
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords

from vzense_demo.subscriber import TopicSubscriber


class PoseStampedSubscriber(TopicSubscriber):

    def __init__(self,
                 topic_name,
                 one_shot=False,
                 start=True,
                 wait=False):
        super(PoseStampedSubscriber, self).__init__(
            topic_name,
            geometry_msgs.msg.PoseStamped,
            one_shot=one_shot,
            start=start,
            wait=wait)

    def to_coords(self, wait=True):
        if wait:
            self.wait_new_message()
        return geometry_pose_to_coords(self.msg)

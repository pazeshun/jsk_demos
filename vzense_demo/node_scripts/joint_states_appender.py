#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState


class JointStatesMerger:
    def __init__(self):
        self.joint_state_1_sub = rospy.Subscriber('/joint_states',
                                                  JointState, self.joint_state_1_callback)
        self.joint_state_2_sub = rospy.Subscriber('/joint_states_hand',
                                                  JointState, self.joint_state_2_callback)
        self.joint_states_pub = rospy.Publisher('/concat_joint_states',
                                                JointState, queue_size=10)

        self.joint_state_1 = None
        self.joint_state_2 = JointState()
        self.joint_state_2.position = [- np.pi / 2.0, 0, 0, 0]
        self.joint_state_2.name = ['RARM_hand_thumb_base_joint',
                                   'RARM_hand_thumb_joint',
                                   'RARM_hand_index_base_joint',
                                   'RARM_hand_index_joint',
                                   ]

    def joint_state_1_callback(self, msg):
        self.joint_state_1 = msg
        self.publish_merged_joint_states()

    def joint_state_2_callback(self, msg):
        self.joint_state_2 = msg
        self.publish_merged_joint_states()

    def publish_merged_joint_states(self):
        if self.joint_state_1 is None or self.joint_state_2 is None:
            return

        merged_joint_state = JointState()
        merged_joint_state.header.stamp = rospy.Time.now()
        merged_joint_state.name = self.joint_state_1.name + self.joint_state_2.name
        merged_joint_state.position = list(self.joint_state_1.position) + list(self.joint_state_2.position)
        merged_joint_state.velocity = list(self.joint_state_1.velocity) + list(self.joint_state_2.velocity)
        merged_joint_state.effort = list(self.joint_state_1.effort) + list(self.joint_state_2.effort)
        self.joint_states_pub.publish(merged_joint_state)


if __name__ == '__main__':
    rospy.init_node('joint_states_merger')
    merger = JointStatesMerger()
    rospy.spin()

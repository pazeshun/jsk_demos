#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import PoseStamped
from pr2_controllers_msgs.msg import Pr2GripperCommandActionGoal


class GripperMsgConverter():
    LR = ["l", "r"]
    sgns = {"l": 1, "r": -1}

    def __init__(self):
        rospy.init_node("GripperMsgConverter")
        self.g_sub = {}
        self.g_pub = {}
        for lr in self.LR:
            self.g_sub[lr] = rospy.Subscriber(
                "/master_"+lr+"hand_pose", PoseStamped, self.cb, lr)
            self.g_pub[lr] = rospy.Publisher(
                '/'+lr+'_gripper_controller/gripper_action/goal',
                Pr2GripperCommandActionGoal, queue_size=1)
        self.cnt = 0
        rospy.loginfo("start main loop")
        rospy.spin()

    def cb(self, msg, lr):
        # 0 ~ 1.0
        val = msg.pose.position.y
        gripper_cmd = Pr2GripperCommandActionGoal()
        gripper_cmd.goal.command.position = (1 - val) * 0.1
        gripper_cmd.goal.command.max_effort = 75

        self.cnt += 1
        if self.cnt % 10 == 0:
            self.g_pub[lr].publish(gripper_cmd)


if __name__ == '__main__':
    inst = GripperMsgConverter()

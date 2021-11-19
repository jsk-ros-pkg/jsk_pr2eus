#!/usr/bin/env python


import math
import rospy
import actionlib
from control_msgs.msg import *


class DummyJTA(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryActionFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer('dummy_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.logwarn("Started Dummy Joint Trajectory Action Server")
        rospy.logwarn("If joint < 0, set aborted")
        rospy.logwarn("If joint >= 100, set preempted")

    def execute_cb(self, goal):
        success = True
        if len(goal.trajectory.points) > 0 and len(goal.trajectory.points[0].positions) > 0:
            position = goal.trajectory.points[0].positions[0] * 180 / math.pi
            rospy.loginfo("Received {}".format(position))
            if position < 0:
                rospy.logwarn("Set aborted")
                self._as.set_aborted()
            if position >= 100:
                rospy.logwarn("Set preempted")
                self._as.set_preempted()
                success = False
        self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        if success:
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('dummy_jta')
    server = DummyJTA()
    rospy.spin()

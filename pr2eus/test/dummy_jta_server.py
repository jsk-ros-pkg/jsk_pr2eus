#!/usr/bin/env python


import math
import rospy
import actionlib
from control_msgs.msg import *


class DummyJTA(object):
    # create messages that are used to publish feedback/result
    _feedback = FollowJointTrajectoryFeedback()
    _result = FollowJointTrajectoryResult()

    def __init__(self, robot='pr2'):
        self._as = actionlib.SimpleActionServer('dummy_controller/follow_joint_trajectory', FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.logwarn("Started Dummy Joint Trajectory Action Server")
        rospy.logwarn("If joint < 0, set aborted")
        rospy.logwarn("If joint >= 100, set preempted")

    # Return result based on real robot jta server.
    # https://github.com/jsk-ros-pkg/jsk_pr2eus/pull/460#issuecomment-975418370
    def execute_cb(self, goal):
        if len(goal.trajectory.points) > 0 and len(goal.trajectory.points[0].positions) > 0:
            position = goal.trajectory.points[0].positions[0] * 180 / math.pi
            rospy.loginfo("Received {}".format(position))
            self._as.publish_feedback(self._feedback)
            if robot == 'pr2':
                if position < 0:
                    rospy.logwarn("Set aborted")
                    self._result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                    self._as.set_aborted(result=self._result)
                elif position >= 100:
                    rospy.logwarn("Set preempted")
                    self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    self._as.set_preempted(result=self._result)
                else:
                    self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    self._as.set_succeeded(result=self._result)
            if robot == 'fetch':
                if position < 0:
                    rospy.logwarn("Set aborted")
                    self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    self._as.set_aborted(
                        result=self._result,
                        text='Controller manager forced preemption.')
                elif position >= 100:
                    rospy.logwarn("Set preempted")
                    self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    self._as.set_preempted(
                        result=self._result,
                        text='Trajectory preempted')
                else:
                    self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    self._as.set_succeeded(
                        result=self._result,
                        text='Trajectory succeeded.')
            if robot == 'kinova':
                if position < 0:
                    rospy.logwarn("Set aborted")
                    self._result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    self._result.error_string = 'After validation, trajectory execution failed in the arm with sub error code SUB_ERROR_NONE'
                    self._as.set_aborted(result=self._result)
                elif position >= 100:
                    rospy.logwarn("Set preempted")
                    self._result.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    self._result.error_string = 'Trajectory execution failed in the arm with sub error code 55\nThe speed while executing\\n\ the trajectory was too damn high and caused the robot to stop.\n'
                    # NOTE: this line is not typo but kinova driver's behavior
                    self._as.set_aborted(result=self._result)
                else:
                    self._result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                    self._as.set_succeeded(result=self._result)

if __name__ == '__main__':
    rospy.init_node('dummy_jta')
    robot = rospy.get_param('~robot', 'pr2')
    server = DummyJTA(robot)
    rospy.spin()

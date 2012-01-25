#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2eus_openrave')
import rospy,roslaunch
import os,time,unittest
import geometry_msgs.msg
import std_msgs.msg

class TestPr2EusOpenrave(unittest.TestCase):
    def test_ray_coords(self):

        rospy.init_node('pr2_openrave_simulation_node')
        rospy.wait_for_service('/MoveToHandPosition')
        pub1 = rospy.Publisher('ray_coords', geometry_msgs.msg.PoseStamped)
        pub2 = rospy.Publisher('change_pose_pr2', std_msgs.msg.String)

        target_poses = [
            geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.600, -0.100,1.000),
                                   orientation=geometry_msgs.msg.Quaternion(1.0 ,0.000,0.000,0.000)),

            geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.591, 0.079,0.985),
                                   orientation=geometry_msgs.msg.Quaternion(0.0 ,0.707 ,0.0, 0.707))
                        ]
        time.sleep(10)
        msg = std_msgs.msg.String()
        msg.data = "reset-pose"
        pub2.publish(msg)

        try:
            for pose in target_poses:
                msg = geometry_msgs.msg.PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/base_link"
                msg.pose = pose
                print "sending...",msg
                pub1.publish(msg)
                print "wait for 20 secs"
                time.sleep(20)

                msg = std_msgs.msg.String()
                msg.data = "reset-pose"
                pub2.publish(msg)
                print "resetting pose."

        finally:
            rospy.loginfo('shutting down')

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2eus_openrave', 'test_pr2eus_openrave', TestPr2EusOpenrave)



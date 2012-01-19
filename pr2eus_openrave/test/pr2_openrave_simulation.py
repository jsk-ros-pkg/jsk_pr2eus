#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2eus_openrave')
import rospy, roslaunch
import os,time,unittest
import geometry_msgs.msg

class TestPr2EusOpenrave(unittest.TestCase):
    def test_ray_coords(self):

        rospy.init_node('pr2_openrave_simulation_node')
        rospy.wait_for_service('/MoveToHandPosition')
        pub = rospy.Publisher('ray_coords', geometry_msgs.msg.PoseStamped)

        target_poses = [geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.591, 0.079,0.985),
                                               orientation=geometry_msgs.msg.Quaternion(0.0 ,0.707 ,0.0, 0.707))]
        time.sleep(10) # ???

        try:
            for pose in target_poses:
                msg = geometry_msgs.msg.PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/base_link"
                msg.pose = pose
                print "sending...",msg
                pub.publish(msg)
                print "wait for 10 secs"
                time.sleep(10) # wait at most 10 sec
                print "done"
        finally:
            rospy.loginfo('shutting down')

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2eus_openrave', 'test_pr2eus_openrave', TestPr2EusOpenrave)



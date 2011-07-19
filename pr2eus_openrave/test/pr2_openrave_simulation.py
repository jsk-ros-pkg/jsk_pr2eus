#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2eus_openrave')
import rospy, roslaunch
import os,time,unittest
import geometry_msgs.msg

class TestPr2EusOpenrave(unittest.TestCase):
    def test_ray_coords(self):
        run_id = roslaunch.core.generate_run_id()
        launch = roslaunch.parent.ROSLaunchParent(run_id,[roslib.packages.get_pkg_dir("pr2eus_openrave")+"/launch/pr2_openrave_simulation.launch"])
        launch.start()

        rospy.init_node('pr2_openrave_simulation_node')
        rospy.wait_for_service('/MoveToHandPosition')
        pub = rospy.Publisher('ray_coords', geometry_msgs.msg.PoseStamped)

        target_poses = [geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.591, 0.079,0.985),
                                               orientation=geometry_msgs.msg.Quaternion(0.0 ,0.707 ,0.0, 0.707))]
        time.sleep(10) # ???

        try:
            print launch.pm.get_active_names()
            controlname = [name for name in launch.pm.get_active_names() if name.find('pr2eus_openrave_eus')>=0]
            for pose in target_poses:
                controlproc = launch.pm.get_process(controlname[0])
                print "checkcontrolproc",controlproc
                if controlproc is None or not controlproc.is_alive():
                    break
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
            if controlproc is None or not controlproc.is_alive():
                rospy.loginfo('test failed')
                self.fail(pose+"does not solved")
            rospy.loginfo('shutting down')
            launch.shutdown()

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2eus_openrave', 'test_pr2eus_openrave', TestPr2EusOpenrave)



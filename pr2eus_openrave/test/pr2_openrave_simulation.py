#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2eus_openrave')
import rospy,roslaunch,tf
import os,time,unittest,numpy
import geometry_msgs.msg
import std_msgs.msg

class TestPr2EusOpenrave(unittest.TestCase):
    def test_ray_coords(self):
        rospy.init_node('pr2_openrave_simulation_node')
        self.tf = tf.TransformListener();

        rospy.wait_for_service('/MoveToHandPosition')

        pub1 = rospy.Publisher('ray_coords', geometry_msgs.msg.PoseStamped)
        pub2 = rospy.Publisher('change_pose_pr2', std_msgs.msg.String)
 
        while pub1.get_num_connections() < 1 :
            print "waiting"
            time.sleep(1)

        target_poses = [
            geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.600, -0.100,1.000),
                                   orientation=geometry_msgs.msg.Quaternion(1.0 ,0.000,0.000,0.000)),

            geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(0.591, 0.079,0.985),
                                   orientation=geometry_msgs.msg.Quaternion(0.0 ,0.707 ,0.0, 0.707))
                        ]
        time.sleep(5)
        try:
            for pose in target_poses:
                msg = geometry_msgs.msg.PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/base_link"
                msg.pose = pose
                pub1.publish(msg)
                time.sleep(30) # need to wait until /ray_coords finished

                # check if reached to the goal
                #tm = self.tf.getLatestCommonTime("/base_link","/r_gripper_tool_frame")
                #(trans,rot) = self.tf.lookupTransform("/base_link","/r_gripper_tool_frame",rospy.Time(0))
                #
                #print "original goal",pose
                #print "openrave goal",trans,rot
                #diff_pose = numpy.linalg.norm(numpy.array(trans)-numpy.array((pose.position.x,pose.position.y,pose.position.z)))
                #diff_rot = numpy.linalg.norm(numpy.array(rot)-numpy.array((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)))
                #print "diff pose",diff_pose
                #print "diff rot",diff_rot
                #assert  diff_pose < 1.0, "check openrave goal pose"+str(pose)+","+str(trans)
                #assert diff_rot < 1.5, "check openrave goal rot"+str(pose)+","+str(rot)

                stopmsg = std_msgs.msg.String()
                stopmsg.data = "stop-visualize"
                pub2.publish(stopmsg)
                print "removing visualization."
                time.sleep(1)

                tuckmsg = std_msgs.msg.String()
                tuckmsg.data = "tuck-arm-l"
                pub2.publish(tuckmsg)
                print "resetting pose."
                time.sleep(4)

        finally:
            rospy.loginfo('shutting down')

if __name__ == '__main__':
    import rostest
    rostest.unitrun('pr2eus_openrave', 'test_pr2eus_openrave', TestPr2EusOpenrave)



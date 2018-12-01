#!/usr/bin/env python

from __future__ import print_function

import unittest

import rospy
from sound_play.msg import SoundRequest

PKG = 'pr2eus'
NAME = 'test_speack'

class TestSpeak(unittest.TestCase):

    speak_msg = None
    speak_msg_jp = None

    def cb(self, msg):
        rospy.logwarn("received speak_msg")
        self.speak_msg = msg

    def cb_jp(self, msg):
        rospy.logwarn("received speak_msg_jp")        
        self.speak_msg_jp = msg

    def setUp(self):
        rospy.init_node(NAME)
        rospy.Subscriber('/robotsound', SoundRequest, self.cb)
        rospy.Subscriber('/robotsound_jp', SoundRequest, self.cb_jp)

    def test_speak(self):
        rospy.sleep(10)
        rospy.logwarn(self.speak_msg)
        rospy.logwarn(self.speak_msg_jp)
        while self.speak_msg == None:
            rospy.logwarn("waiting for speak_msg")
            rospy.sleep(1)
        rospy.logwarn("check speak_msg")
        self.assertTrue(self.speak_msg.command == SoundRequest.PLAY_ONCE)
        self.assertTrue(self.speak_msg.volume > 0)
        self.assertTrue(len(self.speak_msg.arg) > 0)
        
        while self.speak_msg_jp == None:
            rospy.logwarn("waiting for speak_msg_jp")
            rospy.sleep(1)
        rospy.logwarn("check speak_msg")
        self.assertTrue(self.speak_msg_jp.command == SoundRequest.PLAY_ONCE)
        self.assertTrue(self.speak_msg_jp.volume > 0)
        self.assertTrue(len(self.speak_msg_jp.arg) > 0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestSpeak)

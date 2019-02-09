#!/usr/bin/env python

from __future__ import print_function

import unittest

import rospy
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestResult

import actionlib

PKG = 'pr2eus'
NAME = 'test_speack_action'

class TestSpeakAction(unittest.TestCase):
    _result = SoundRequestResult()

    speak_msg = None
    speak_msg_jp = None

    def cb(self, msg):
        rospy.logwarn("received speak_msg")
        self.speak_msg = msg.sound_request
        self._as_en.set_succeeded(self._result)

    def cb_jp(self, msg):
        rospy.logwarn("received speak_msg_jp")        
        self.speak_msg_jp = msg.sound_request
        self._as_jp.set_succeeded(self._result)

    def setUp(self):
        rospy.init_node(NAME)
        self._as_en = actionlib.SimpleActionServer('/robotsound', SoundRequestAction, execute_cb=self.cb, auto_start=False)
        self._as_jp = actionlib.SimpleActionServer('/robotsound_jp', SoundRequestAction, execute_cb=self.cb_jp, auto_start=False)
        self._as_en.start()
        self._as_jp.start()

    def test_speak(self):
        rospy.sleep(10)
        rospy.logwarn(self.speak_msg)
        rospy.logwarn(self.speak_msg_jp)
        while self.speak_msg == None:
            rospy.logwarn("waiting for speak_msg")
            rospy.sleep(1)
        rospy.logwarn("check speak_msg")
        self.assertTrue(self.speak_msg.command == SoundRequest.PLAY_ONCE)
        if hasattr(SoundRequest, 'volume'): # volume is added from 0.3.0 https://github.com/ros-drivers/audio_common/commit/da9623414f381642e52f59701c09928c72a54be7#diff-fe2d85580f1ccfed4e23a608df44a7f7
            self.assertTrue(self.speak_msg.volume > 0)
        self.assertTrue(len(self.speak_msg.arg) > 0)
        
        while self.speak_msg_jp == None:
            rospy.logwarn("waiting for speak_msg_jp")
            rospy.sleep(1)
        rospy.logwarn("check speak_msg")
        self.assertTrue(self.speak_msg_jp.command == SoundRequest.PLAY_ONCE)
        if hasattr(SoundRequest, 'volume'):
            self.assertTrue(self.speak_msg_jp.volume > 0)
        self.assertTrue(len(self.speak_msg_jp.arg) > 0)

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestSpeakAction)

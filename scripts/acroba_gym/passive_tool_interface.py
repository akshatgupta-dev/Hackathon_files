#!/usr/bin/env python3
import rospy
import os

from std_msgs.msg import Bool

class PassiveToolInterface(object):
    def __init__(self,set_state_topic="passive_tool"):
        self._pubstate=rospy.Publisher(set_state_topic,Bool,queue_size=1)
        self._state=False
        
    def setState(self,state):
        self._state=state
        msg=Bool()
        msg.data=state
        self._pubstate.publish(msg)
#!/usr/bin/env python3
import rospy
import os

from std_msgs.msg import Float64
from acroba_unity_msgs.msg import GripperControlCommand
from sensor_msgs.msg import JointState
import numpy as np
import copy
import sys

class ParallelYawControl(object):
    def __init__(self,state_topic:str="tool_joint_states",move_topic:str="move_tool_joints"):
        self._pub1=rospy.Publisher(move_topic,GripperControlCommand,queue_size=5)
        self.joint_names=["base_finger_1","base_finger_2"]
        self._state_topic=state_topic
        self._sub1=rospy.Subscriber(self._state_topic,JointState,self.state_callback)
        self._joints=None
        self.connected=False
        self.max_spanning_width = 0.11
        
    def check_connections(self):
        """check the connection, await till it is connected
        """        
        while self._joints is None and not rospy.is_shutdown():
            try:
                state_msg = rospy.wait_for_message(self._state_topic, JointState, timeout=5.0)
                rospy.logdebug("Current "+self._state_topic+" READY=>")
                self.connected=True
            except Exception as e: 
                print(e)
                rospy.logerr("Current "+self._state_topic+" not ready yet, retrying...")
    
    def open(self,speed:float=1):
        """ open the gripper to the max spanning width with speed

        Args:
            speed (float, optional): the ratio of max speed when moving the gripper. Defaults to 1.
        """        
        self.move([self.max_spanning_width],speed)
        
    def close(self,speed=1):
        """ close the gripper 

        Args:
            speed (int, optional): the ratio of max speed when moving the gripper. Defaults to 1.
        """        
        self.move([0],speed)
        
    def move(self,positions:list,speed:float=1.0,effort:float=0.0):
        """move the gripper to specific spanning width with speed and effort

        Args:
            positions (list): target spanning width in a list, ex. [0.05]
            speed (int, optional): the ratio of max speed when moving the gripper. Defaults to 1.
            effort (int, optional): param not used. Defaults to 0.
        """        
        msg=GripperControlCommand()
        msg.positions=positions
        msg.names=self._joints.name
        msg.speed=speed
        msg.effort=effort
        self._pub1.publish(msg)
        
    def get_state(self)->list:
        """obtain joint state of the gripper

        Returns:
            list: list of spanning width, ex. [0.05]
        """               
        if self._joints is None:
            return None
        return list(copy.deepcopy(self._joints.position))
    
    def state_callback(self,msg):
        """call back function for the state subscriber
        """        
        self.connected=True
        self._joints=copy.deepcopy(msg)
            
if __name__=='__main__':
    rospy.init_node('gripper_control')
    parallel_yaw=ParallelYawControl()
    rospy.sleep(1)
    if parallel_yaw.connected==False:
        rospy.logerr("Gripper not connected")
        exit

    rospy.loginfo("close")
    parallel_yaw.close()
    rospy.sleep(1)
    parallel_yaw.open()
    rospy.sleep(1)
        
        
        
        
    
    
    
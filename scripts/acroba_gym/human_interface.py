#!/usr/bin/env python3
from os import name
import rospy
from std_msgs.msg import Empty
import actionlib
import acroba_unity_msgs.msg 
from acroba_unity_msgs.msg import HumanCommand
from typing import Type

class HumanInterface(object):
    
    def __init__(self):
        
        self._pub=rospy.Publisher("/human_command",HumanCommand,queue_size=0)
        rospy.sleep(0.4)
        
    def publish(self,msg):#:Type[acroba_unity_msgs.msg.HumanCommand]):
        self._pub.publish(msg)
        
if __name__ == '__main__':
    rospy.init_node('human_interface', anonymous=True)
    human_interface=HumanInterface()
    command_msg=HumanCommand()
    command_msg.human_name="human_operator"
    command_msg.pose.x=1
    command_msg.pose.y=1
    command_msg.command=HumanCommand.CREATE
    human_interface.publish(command_msg)
    rospy.sleep(2)
    
    
    command_msg.pose.x=-2
    command_msg.pose.y=3
    command_msg.command=HumanCommand.MOVE
    human_interface.publish(command_msg)
    print("Human moving")
    rospy.sleep(5)  
    
    command_msg.pose.x=-1.5
    command_msg.pose.y=-3
    command_msg.command=HumanCommand.MOVE
    human_interface.publish(command_msg)
    print("Human moving")
    rospy.sleep(5)  
    

    command_msg.pose.x=2
    command_msg.pose.y=-3
    command_msg.command=HumanCommand.MOVE
    human_interface.publish(command_msg)
    print("Human moving")
    rospy.sleep(5)  
    
    command_msg.command=HumanCommand.DESTROY
    var=input()
    human_interface.publish(command_msg)
    print("Human moving")
    rospy.sleep(0.4)
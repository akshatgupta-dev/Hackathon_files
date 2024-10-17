#!/usr/bin/env python3
from os import name
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import copy
from acroba_unity_msgs.srv import SetControllers,GetControllers
from acroba_unity_msgs.msg import JointControllers,JointController 
import math

class ControllerManager(object):
    """ Class that controls the modificacion of the controllers in unity"""
    def __init__(self,setter_topic="controller_setter_service",getter_topic="controller_getter_service"):
        self.controller_setter=rospy.ServiceProxy(setter_topic,SetControllers)
        self.controller_getter=rospy.ServiceProxy(getter_topic,GetControllers)
    def set_controllers(self,controllers):
        if not isinstance(controllers,JointControllers):
            rospy.logerr("set_controller with incorrect type of input")
            return
        try:
            res=self.controller_setter(controllers)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            res=None
        return res
    
    def get_controllers(self):
        try:
            res=self.controller_getter()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            res=None
        return res
    
def test_service():
    rospy.init_node("controller_manager")
    controller_manager=ControllerManager()
    controller_msg=JointControllers()
    controller_msg.controller_ids=[0,1,2,5]
    
    #Tool and robot velocities
    controller_msg.robot_vel=0.54
    controller_msg.tool_vel=0.1
    #controller for 0,1,2 joints
    joint_cont1=JointController()
    joint_cont1.damping=333
    joint_cont1.stiffness=666
    joint_cont1.force_limit=1000
    controller_msg.joint_controllers.extend([joint_cont1,joint_cont1,joint_cont1])
    joint_cont2=JointController()
    joint_cont2.damping=1098
    joint_cont2.stiffness=19234
    joint_cont2.force_limit=2000
    joint_cont2.body_angular_damping=0.345
    joint_cont2.body_linear_damping=0.213
    joint_cont2.joint_friction=0.1
    controller_msg.joint_controllers.append(joint_cont2)
    
    rospy.loginfo("Set controllers")
    rospy.sleep(1)
    #res=controller_manager.set_controllers(controller_msg)
    #print(res)
    rospy.sleep(1)
    #Receive the controllers
    res=controller_manager.get_controllers()
    print(res)
    print("Correctly received",abs(res.controllers.robot_vel-controller_msg.robot_vel)<0.0001)
       
if __name__ == "__main__":
    test_service()
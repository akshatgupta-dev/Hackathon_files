#!/usr/bin/env python3
from os import name
from threading import current_thread
from typing import Set
import rospy
from std_msgs.msg import String,ColorRGBA
from acroba_unity_msgs.msg import UnityLight
from geometry_msgs.msg import Pose
import copy
from acroba_unity_msgs.srv import SetLights,GetLights
from tf.transformations import *
import math

class LightManager(object):
    """ Class that manage of the lights of unity scene"""
    def __init__(self,setter_topic="set_lights",getter_topic="get_lights"):
        self.is_connected=False
        self.set_light_service=setter_topic
        self.get_light_service=getter_topic
        self.light_setter=rospy.ServiceProxy(self.set_light_service,SetLights)
        self.light_getter=rospy.ServiceProxy(self.get_light_service,GetLights)
        rospy.sleep(0.1)
        #self.check_connections()
        
    def check_connections(self):
        while self.is_connected==False and not rospy.is_shutdown():
            try:
                rospy.wait_for_service(self.get_light_service,5)
                rospy.wait_for_service(self.set_light_service,5)
                self.is_connected=True
                rospy.loginfo("Connected")
            except Exception as e: 
                print(e)
                rospy.logerr("Services are not ready yet, retrying...")
        
    def set_lights(self,lights):
        """
        Parameters
        ----------
        name : lights
            list of UnityLight: directional and spot light point to the z in Unity 
            which corresponds to the X in Ros
        """
        for luz in lights:
            if not isinstance(luz,UnityLight):
                rospy.logerr("set_light with incorrect type of input")
                return
        try:
            res=self.light_setter(lights)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            res=None
        return res
    
    def get_lights(self):
        try:
            res=self.light_getter()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            res=None
        return res
    
    
def  test_service():
    rospy.init_node("light_manager")
    light_manager=LightManager()
    rospy.sleep(1)
    current_lights=light_manager.get_lights()
    print(current_lights)
    rospy.sleep(1)
    
    print("Add a model service")
    input_service=[]
    light_to_add=UnityLight()
    light_to_add.name="added light"
    light_to_add.type=UnityLight.SPOT
    light_to_add.status=UnityLight.ADD
    light_to_add.angle=90
    light_to_add.range=2
    light_to_add.color.r=0
    light_to_add.color.g=1
    light_to_add.color.b=0
    light_to_add.color.a=0
    light_to_add.pose.position.x=0.2
    light_to_add.pose.position.y=0.2
    light_to_add.pose.position.z=1.5
    light_to_add.intensity=8
    q_1=quaternion_from_euler(0,math.pi/2,0)
    light_to_add.pose.orientation.x=q_1[0]
    light_to_add.pose.orientation.y=q_1[1]
    light_to_add.pose.orientation.z=q_1[2]
    light_to_add.pose.orientation.w=q_1[3]
    input_service.append(light_to_add)
    
    ## remove 
    light_to_remove=UnityLight()
    light_to_remove.status=UnityLight.REMOVE
    light_to_remove.name="Directional Light"
    input_service.append(light_to_remove)
    
    ## Change
    light_to_change=UnityLight()
    #Get the previous light to change to avoid writting al parameters
    for light in current_lights.lights:
        if light.name=="Spot Light":
            light_to_change=light
            break
    light_to_change.status=UnityLight.CHANGED
    light_to_change.color.r=0
    light_to_change.color.b=1
    light_to_change.color.g=0
    light_to_change.color.a=0
    light_to_change.intensity=20
    
    input_service.append(light_to_change)
    
    res=light_manager.set_lights(input_service)
    print("responde for service",res)
    
    
    rospy.sleep(1)
    
    
    
    
if __name__ == "__main__":
    test_service()
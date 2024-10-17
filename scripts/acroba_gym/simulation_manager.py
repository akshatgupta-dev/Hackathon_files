#!/usr/bin/env python3
from os import name
import rospy
from std_msgs.msg import Empty
import actionlib
import acroba_unity_msgs.msg
from acroba_unity_msgs.srv import ResetScene

class SimulationManager(object):
    
    def __init__(self):
        self._reset_sim = rospy.ServiceProxy('reset_sim_srv', ResetScene)
        self._pause_pub=rospy.Publisher("/pause_physics",Empty,queue_size=1)
        self._unpause_pub=rospy.Publisher("/unpause_physics",Empty,queue_size=1)

    def reset_sub_callback(self, msg):
        print("finished")
        self.reset_finished = True

    def reset_sim(self):
        self._reset_sim()
        
    def pause_physics(self):
        empty_msg=Empty()
        self._pause_pub.publish(empty_msg)
    
    def unpause_physics(self):
        empty_msg=Empty()
        self._unpause_pub.publish(empty_msg)
        
if __name__ == '__main__':
    rospy.init_node('simulation_manager_sample', anonymous=True, log_level=rospy.DEBUG)
    
    simulation_manager=SimulationManager()
    rospy.sleep(3)
    simulation_manager.reset_sim()
    rospy.sleep(3)
    
    
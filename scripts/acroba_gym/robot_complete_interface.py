#!/usr/bin/env python3
import rospy

from rospy.core import logerr
from geometry_msgs.msg import Pose,PoseStamped
from acroba_utils import geom_utils
from trac_ik_python.trac_ik import IK
from acroba_gym.robot_simple_interface import RobotJointControl 
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from std_msgs.msg import Bool
import copy
import time
import math
import numpy as np
from typing import Union
from copy import deepcopy

class RobotCompleteControl(RobotJointControl):
    """Robot Complete Control is an extension of the simple interface with IK solver. The urdf information is required via /robot_description
    """    
    def __init__(self,namespace='',base_name="/base_robot",end_name="/end_tool"):
        #Constructor of the base class
        super().__init__(namespace)
        self.namespace=namespace
        self.current_pose=None
        self.base_link=rospy.get_param(self.namespace+base_name,"world")
        self.end_link=rospy.get_param(self.namespace+end_name,"end_tool")
        #get urdf info from /robot_description
        self.urdf_string=rospy.get_param(self.namespace+"/robot_description")
        rospy.loginfo("Initialize IK solver with base [%s] and [%s]",self.base_link,self.end_link)
        #setup the IK solver
        self.ik_solver=IK(base_link=self.base_link,tip_link=self.end_link,urdf_string=self.urdf_string,
        solve_type="Distance",timeout=1)
        # cartesian pose for the end tool 
        self.pose_sub=rospy.Subscriber(self.namespace+"/robot_pose", PoseStamped, self.pose_callback,queue_size=1)
        self.check_complete_connections()
        
    def check_complete_connections(self):
        """await for the connection
        """        
        start_time=time.time()
        while self.pose_sub.get_num_connections()==0 and (time.time()-start_time)<5:
            rospy.sleep(0.5)
        if self.pose_sub.get_num_connections()>0:
            self.is_connected=True
        else:
            self.is_connected=False
        return
    
    def poseToJoint(self,seed:list,geom_pose:Pose)-> Union[list,None]:
        """convert Cartesian pose to joint pose 

        Args:
            seed (list): list of joint values as the seed for finding the optimal IK solution, usually the current joint pose
            geom_pose (Pose): target Cartesian pose

        Returns:
            list|None: joint values if a solution found, otherwise None
        """        
        position=self.ik_solver.get_ik(seed,geom_pose.position.x,geom_pose.position.y,
                                        geom_pose.position.z,geom_pose.orientation.x,
                                        geom_pose.orientation.y,geom_pose.orientation.z,
                                        geom_pose.orientation.w)
        return position   

    def pose_callback(self,msg):
        """call back function for obtaining the end_link Cartesian pose
        """        
        self.current_pose=copy.deepcopy(msg.pose)
        
    def get_current_pose(self)-> Union[list,None]:
        """obtain the current Cartesian pose of the end_link

        Returns:
            Pose|None: end_link Cartesian pose, otherwise None
        """        
        if self.current_pose is not None:
            return self.current_pose
        else:
            return None
        
    def movePose(self,pose:Pose,time:int=2,verbose:bool=False):
        """move to a target Cartesain pose

        Args:
            pose (Pose): target Cartesian pose
            time (int, optional): time from start for the target pose. Defaults to 2 sec.
            verbose (bool, optional): verbose option. Defaults to False.

        Returns:
            _type_: _description_
        """        
        seed=self.get_current_joints()
        
        ik_sol=self.poseToJoint(seed,pose)
        if ik_sol is None:
            rospy.logerr("No IK solution found")
            return False
        if verbose:
            print("ik sol:",ik_sol)
        ## send ik solution as  1 point trajectory in the unity_control topic
        trajectory_msgs=JointTrajectory()
        pt_ik=JointTrajectoryPoint()
        pt_ik.positions=copy.deepcopy(ik_sol)
        pt_ik.time_from_start=rospy.Duration(time)
        trajectory_msgs.points.append(pt_ik)
        #topic trajectory followgin
        self.execute_trajectory_topic(trajectory_msgs)
        return True
        #using joint_state works better than with action
        #ik_sol_list=list(ik_sol)
        #self.move_joints(ik_sol_list)

    def cartesianMove(self,pre_pose:Pose,post_pose:Pose)->bool:
        """Cartesian path from pre_pose to post_pose maintaining the orientation of the pre_pose

        Args:
            pre_pose (Pose): starting Cartesian pose
            post_pose (Pose): target Cartesian pose

        Returns:
            bool: if the Cartesian path can be executed
        """
        current_seed=self.get_current_joints()
        ik_sol1=self.poseToJoint(current_seed,pre_pose)
        if ik_sol1 is None:
            rospy.logerr("Cartesian path not posible")
            return False
        current_seed=ik_sol1
        step=0.005 #5 mm between points
        diff_x=post_pose.position.x-pre_pose.position.x
        diff_y=post_pose.position.y-pre_pose.position.y
        diff_z=post_pose.position.z-pre_pose.position.z
        distance=math.sqrt(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z)
        
        num_inter=distance/step
        
        num_inter=math.floor(distance/step)
        
        num_inter=max(num_inter,1)
        cartesian_trajectory_msgs=JointTrajectory()
        pose_mat=geom_utils.geompose2mat(pre_pose)
        mat_to_add=geom_utils.matFromPosEul(diff_x/num_inter,diff_y/num_inter,diff_z/num_inter,0,0,0)
        time_from_start=0.1
        for i in range(num_inter):
            pose_mat=np.matmul(mat_to_add,pose_mat)
            
            inter_pose=geom_utils.mat2geompose(pose_mat)
            ik_sol_inter=self.pose_to_joint(current_seed,inter_pose)
            current_seed=ik_sol_inter
            if ik_sol_inter is None:
                rospy.logerr("Cartesian path not posible")
                return False
            
            pt_cart=JointTrajectoryPoint()
            pt_cart.positions=copy.deep.copy(ik_sol_inter)
            pt_cart.time_from_start=rospy.Duration(time_from_start)
            time_from_start=time_from_start+0.1
            cartesian_trajectory_msgs.points.append(pt_cart)
            
        #post_pose.orientation=copy.deepcopy(pre_pose.orientation)
        ik_sol_last=self.pose_to_joint(current_seed,post_pose)
        if ik_sol_last is None:
            rospy.logerr("Cartesian path not posible")
            return False
        pt_cart=JointTrajectoryPoint()
        pt_cart.positions=copy.deep.copy(ik_sol_last)
        pt_cart.time_from_start=rospy.Duration(time_from_start)
        cartesian_trajectory_msgs.points.append(pt_cart)
        
        self.execute_trajectory_topic(cartesian_trajectory_msgs)
        return True
        
if __name__=='__main__':
    rospy.init_node("robot_complete_interface")
    robot_complete_interface=RobotCompleteControl()
    rospy.loginfo("Is connected %i",robot_complete_interface.is_connected)
    joints=robot_complete_interface.get_current_joints()
    print("Current joints [",joints,"]")
    curr_pose=robot_complete_interface.get_current_pose()
    print("Current Pose[",curr_pose,"]")
    ##forward kinematic but instead use a know pose
    
    base_pose=Pose()
    base_pose.position.x=0.451
    base_pose.position.y=-0.833
    base_pose.position.z=0.895
    
    pose1=Pose()
    pose1=deepcopy(curr_pose)

    pose1.position.z = pose1.position.z - 0.005
    print("pose to get inverse",pose1)
    
    ik_solution=robot_complete_interface.poseToJoint(joints,curr_pose)
    print("ik solution",ik_solution,"and tpye",type(ik_solution))
    print("moving joints via trayectory")
    robot_complete_interface.movePose(pose1,1,True)
    
    

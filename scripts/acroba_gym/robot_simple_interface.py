#!/usr/bin/env python3
import rospy
import os
from rospy.core import logerr
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64,Header
from actionlib_msgs.msg import GoalID
import actionlib
from acroba_unity_msgs.msg import ExecuteTrajectoryAction,ExecuteTrajectoryGoal
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint,JointTrajectory
from acroba_unity_msgs.msg import UnityErrorCode, CollisionState
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryGoal,JointTolerance,FollowJointTrajectoryResult,FollowJointTrajectoryActionGoal
from std_msgs.msg import Bool
import copy
import time
import math
import numpy as np
import sys
import time

class RobotJointControl(object):
    """[summary]
    Class that send and received joint states on a robot
    anlges are received via /joint_states
    angles are send via /move_joints: for this topci only the position field is taking
    into accout
    """
    def __init__(self,namespace=''):
        self.current_joints=None
        self.namespace=namespace
        self._joint_topic=self.namespace+'/joint_states'
        # Simple Robot Control Interface
        # joint states
        self.sub=rospy.Subscriber(self._joint_topic, JointState, self.joint_callback,queue_size=1)
        # move robot to a joint pose
        self.pub=rospy.Publisher(self.namespace+'/move_joints',JointState,queue_size=1,latch=False)
        # move robot to follow a trajectory via action
        self.action_client=actionlib.SimpleActionClient(self.namespace+'/unity_robot/follow_joint_trajectory',FollowJointTrajectoryAction)
        # move robot to follow a trajectory via topic
        self.trajectory_topic_pub=rospy.Publisher(self.namespace+'/unity_robot/command',JointTrajectory,queue_size=1,latch=False)
        self.in_collision=None
        # force robot to a joint pose
        self.pub_set=rospy.Publisher(self.namespace+'/set_joints',JointState,queue_size=1)
        # robot collision status
        self.sub_col=rospy.Subscriber(self.namespace+'/robot_collision', CollisionState, self.collision_callback,queue_size=None)
        self.is_connected=False
        rospy.sleep(4)
        self.check_connection()
        self.target_joint=None
    
    def check_connection(self):
        """await the robot interface to be connected
        """        
        self.action_started=self.action_client.wait_for_server(rospy.Duration(secs=3))
        if self.action_started==False:
            rospy.logwarn("Not possible to connect to action server")
        start_time=time.time()
        while self.sub.get_num_connections()==0 and (time.time()-start_time)<10:
            rospy.sleep(0.5)
        if self.sub.get_num_connections()>0:
            self.is_connected=True
        else:
            self.is_connected=False
        
    def joint_callback(self,msg):
        """call back function to obtain the joint states
        """        
        self.current_joints=copy.deepcopy(msg)
        
    def collision_callback(self,msg):
        """call back function to obtain the robot collision status
        """        
        m = copy.deepcopy(msg)
        self.in_collision = len(m.model_in_collision1) != 0
        
    def set_joint_pose(self,positions:list):
        """ Set joint set the joints while deactivating the colliders so it can go through walls to get is collision   

        Args:
            positions (list): list of joint values in radian
        """        
        joint_set_msg=JointState()
        joint_set_msg.position=positions   
        print("publishing this message",joint_set_msg) 
        self.pub_set.publish(joint_set_msg)        
        
    def get_current_joints(self)->list:
        """get the current joint pose 

        Returns:
            list: list of joint values in radian
        """        
        if self.current_joints is not None:
            return copy.deepcopy(list(self.current_joints.position))
        else:
            return None
        
    def execute_trajectory_action(self,goal_trajectory:FollowJointTrajectoryGoal):
        """execute a joint trajectory goal

        Args:
            goal_trajectory (FollowJointTrajectoryGoal): action goal of joint trajectory
        """        
        if self.action_started==False:
            #Try to connect again
            rospy.logwarn("Reconnecting to action server")
            self.action_started=self.action_client.wait_for_server(rospy.Duration(secs=1))
            if self.action_started==False:
                rospy.logerr("Conection not posible")
                #return False
        
        self.action_client.send_goal(goal_trajectory,feedback_cb=self.feedback_callback)
        return
    
    def wait_for_action_results(self, secs:int)->FollowJointTrajectoryResult:
        """wait secs for the action goal to be finished 

        Args:
            secs (int): number of seconds to wait for the result  

        Returns:
            FollowJointTrajectoryResult: result of the action goal
        """        
        self.action_client.wait_for_result(rospy.Duration(secs=secs))
        res=self.action_client.get_result()
        return res
    
    def feedback_callback(self,feedback):
        """call back function for receiving feedbacks from the action server during the execution of an action goal
        """        
        rospy.loginfo("Error feedback [%s]"% str(feedback.error.positions))
       
    def execute_trajectory_topic(self,trajectory:JointTrajectory):
        """execute the input joint trajectory via the ros topic

        Args:
            trajectory (JointTrajectory): joint trajectory
        """        
        if len(trajectory.points)>0:
            self.trajectory_topic_pub.publish(trajectory)
        else:
            rospy.logwarn("No point to send in the trajectory")

def trajectory_topic(robot_interface):
    trajectory_msgs=JointTrajectory()
    pt1=JointTrajectoryPoint()
    pt1.positions=[0,-1.57,0,0,0,0,0]
    pt1.time_from_start=rospy.Duration(2)
    pt2=JointTrajectoryPoint()
    pt2.positions=[0,0,0,0,0,0,0]
    pt2.time_from_start=pt1.time_from_start+rospy.Duration(10)
    trajectory_msgs.points.extend([pt1,pt2])
    rospy.loginfo("Execute trajectory")
    robot_interface.execute_trajectory_topic(trajectory_msgs)
    
    #Trajectory replacement
    trajectory_msgs2=JointTrajectory()
    pt1_2=JointTrajectoryPoint()
    pt1_2.positions=[0,-2.3,0,0,0,0,0]
    pt1_2.time_from_start=rospy.Duration(2)
    pt2_2=JointTrajectoryPoint()
    pt2_2.positions=[0.3,0,0,0,0,0,0]
    pt2_2.time_from_start=pt1.time_from_start+rospy.Duration(2)
    trajectory_msgs2.points.extend([pt1_2,pt2_2])
    rospy.sleep(3)
    rospy.loginfo("Sending trajectory replacement")
    robot_interface.execute_trajectory_topic(trajectory_msgs2)
    
def trajectory_action(robot_interface):
    action_goal=FollowJointTrajectoryGoal()
    
    pt1=JointTrajectoryPoint()
    pt1.positions=[0,-0.76,0,0,0,0,0]
    pt1.time_from_start=rospy.Duration(2)
    pt2=JointTrajectoryPoint()
    pt2.positions=[0,-1.57,-0.76,0,0,0,0]
    pt2.time_from_start=pt1.time_from_start+rospy.Duration(5)
    pt3=JointTrajectoryPoint()
    pt3.positions=[0,0,0,0,0,0]
    pt3.time_from_start=pt2.time_from_start+rospy.Duration(1)
    action_goal.trajectory.points.extend([pt1,pt2,pt3])
    tolerance1=JointTolerance()
    tolerance1.position=-1
    tolerance1.velocity=-1
    tolerance1.acceleration=-1
    action_goal.path_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
    action_goal.goal_tolerance.extend([tolerance1,tolerance1,tolerance1,tolerance1,tolerance1,tolerance1])
    action_goal.goal_time_tolerance=rospy.Duration(30)
    rospy.loginfo("Sending trajectory")
    robot_interface.execute_trajectory_action(action_goal)
    #result=robot_interface.action_client.wait_for_result(rospy.Duration(10))
    #print(result)
    
    #Trajectory without tolerances
    rospy.sleep(4)
    action_goal2=FollowJointTrajectoryGoal()
    pt1_2=JointTrajectoryPoint()
    pt1_2.positions=[0,-2.3,-1.57,0.5,0.5,0.5]
    pt1_2.time_from_start=rospy.Duration(2)
    pt2_2=JointTrajectoryPoint()
    pt2_2.positions=[0.3,0,0,0,0,0,0]
    pt2_2.time_from_start=pt1.time_from_start+rospy.Duration(2)
    action_goal2.trajectory.points.extend([pt1_2,pt2_2])
    tolerance2=JointTolerance()
    tolerance2.position=2.5
    tolerance2.velocity=100
    tolerance2.acceleration=100
    tolerance_goal=JointTolerance()
    tolerance_goal.position=0.05
    tolerance_goal.velocity=100
    tolerance_goal.acceleration=100
    action_goal2.path_tolerance.extend([tolerance2,tolerance2,tolerance2,tolerance2,tolerance2,tolerance2])
    action_goal2.goal_tolerance.extend([tolerance2,tolerance2,tolerance2,tolerance2,tolerance2,tolerance2])
    action_goal2.goal_time_tolerance=rospy.Duration(2,100)
    rospy.loginfo("Sending trajectory replacement with strict tolerances")
    
    robot_interface.execute_trajectory_action(action_goal2)
    robot_interface.action_client.wait_for_result(rospy.Duration(secs=10))
    res=robot_interface.action_client.get_result()
    print("action result",res)
    #rospy.sleep(0.2)
    #robot_interface.execute_trajectory_action(action_goal)


import numpy as np
from scipy.spatial.transform import Rotation as R

# Assuming there's an existing RobotInterface class
class RobotInterface:
    # ... existing methods ...

    def generate_grasp_pose(self, object_pose, object_dimensions):
        """
        Generate a grasping pose given an object pose and dimensions.
        
        :param object_pose: List or array [x, y, z, qx, qy, qz, qw]
        :param object_dimensions: List or array [length, width, height]
        :return: Dictionary containing position, orientation, and grasp width
        """
        # Extract object position and orientation
        object_position = np.array(object_pose[:3])
        object_orientation = R.from_quat(object_pose[3:])

        # Calculate grasp position (e.g., center of the object)
        grasp_position = object_position + object_orientation.apply([0, 0, object_dimensions[2]/2])

        # Calculate approach direction (e.g., top-down approach)
        approach_direction = [0, 0, -1]  # Negative z-axis for top-down

        # Calculate gripper orientation
        gripper_z = -approach_direction
        gripper_y = np.cross([1, 0, 0], gripper_z)
        if np.linalg.norm(gripper_y) < 1e-6:
            gripper_y = np.cross([0, 1, 0], gripper_z)
        gripper_y /= np.linalg.norm(gripper_y)
        gripper_x = np.cross(gripper_y, gripper_z)
        
        gripper_rotation = R.from_matrix(np.column_stack((gripper_x, gripper_y, gripper_z)))

        # Calculate grasp width based on object dimensions
        grasp_width = min(object_dimensions[0], object_dimensions[1])  # Assuming a parallel jaw gripper

        return {
            'position': grasp_position,
            'orientation': gripper_rotation.as_quat(),
            'grasp_width': grasp_width
        }

    # You might want to add a method to execute the grasp
    def execute_grasp(self, grasp_pose):
        # Implement the logic to move the robot to the grasp pose and close the gripper
        pass


        
if __name__=='__main__':
    rospy.init_node("robot_interface")
    robot_interface=RobotJointControl()
    rospy.loginfo("Is connected %i",robot_interface.is_connected)
    joints=robot_interface.get_current_joints()
    print(type(joints))
    print(joints)
    rospy.sleep(0.5)
    trajectory_action(robot_interface)
    robot_interface.set_joint_pose([0,0,0,0,0,0])
    trajectory_topic(robot_interface)


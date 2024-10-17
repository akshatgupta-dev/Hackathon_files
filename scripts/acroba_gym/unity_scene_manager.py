#!/usr/bin/env python3
from os import name
import rospy
from std_msgs.msg import String
from acroba_unity_msgs.msg import ModelStates
from tf.transformations import *
import math
from geometry_msgs.msg import Pose,Point
import copy


class UnitySceneManager(object):
    """Class that communicate with the scene and interact with it, ex. get the monitored model pose, set model pose
    """    
    def __init__(self, unity_model_list=None):
        """[summary]

        Args:
            unity_model_list ([string list], optional): [description]. Models to search for, if None use all models.
        """
        self._model_pose_dict = {}
        self._unity_model_list = unity_model_list
        self.model_state_topic="/model_states"
        self._set_models_states=ModelStates()
        # We now start the suscriber once we have the indexes of each model
        self._sub=rospy.Subscriber(self.model_state_topic, ModelStates, self.callback,queue_size=1)
        self._model_pub=rospy.Publisher("/set_model_states", ModelStates, queue_size=2)
        self._add_model_pub=rospy.Publisher("/add_model",ModelStates,queue_size=1)
        self._add_frame_pub=rospy.Publisher("/add_frame",ModelStates,queue_size=1)
        self._remove_model_pub=rospy.Publisher("/remove_model",ModelStates,queue_size=1)
        self.is_connected=False
        self.raw_model_data=None
        self.check_connections()
    
    def check_connections(self):
        """await for the connection
        """        
        while not bool(self._model_pose_dict) and not rospy.is_shutdown() and not self.is_connected:
            try:
                state_msg = rospy.wait_for_message(self.model_state_topic, ModelStates, timeout=5.0)
                rospy.logdebug("Current "+self.model_state_topic+" READY=>")
                self.is_connected=True
            except Exception as e: 
                print(e)
                rospy.logerr("Current "+self.model_state_topic+" not ready yet, retrying...")
    
        
    def find_element_in_list(self,element, list_element):
        try:
            index_element = list_element.index(element)
            return index_element
        except ValueError:
            return -1
    
    def set_models_states(self,name_list:list,pose_list:list):
        """set up the poses of a set of monitored objects

        Args:
            name_list (list): list of the object names
            pose_list (list): list of the target object poses
        """        
        state_msg=ModelStates()
        state_msg.name=name_list
        state_msg.pose=pose_list
        self._model_pub.publish(state_msg)
        
    def remove_models(self,name_list):
        """remove a set of monitored objects from the scene

        Args:
            name_list (list): a list of object names to be removed
        """        
        state_msg=ModelStates()
        state_msg.name=name_list
        self._remove_model_pub.publish(state_msg)

    def callback(self, data):
        """call back function to obtain the current status of the monitored objects in the scene
        """        
        self.raw_model_data=data
        self._model_pose_dict.clear()
        self.is_connected=True
        if self._unity_model_list is not None:
            for model_name in self._unity_model_list:
                # Retrieve the corresponding index
                model_name_found = self.find_element_in_list(model_name,data.name)
                if model_name_found>=0:
                    self._model_pose_dict[model_name]=data.pose[model_name_found]
        else:
            for i  in range(len(data.name)):
                self._model_pose_dict[data.name[i]]=data.pose[i]
                
    def get_all_models_dict(self):
        if bool(self._model_pose_dict):
            return copy.deepcopy(self._model_pose_dict)
        else:
            None
            
    def get_model_pose(self, model_name:str)->Pose:
        """get the pose of one object 

        Args:
            model_name (str): the name of the object

        Returns:
            Pose: the pose of the object
        """        
        pose_now = None

        try:
            pose_now = self._model_pose_dict[model_name]
        except Exception as e:
            s = str(e)
            rospy.loginfo("Error, The _robots_models_dict is not ready = " + s)

        return pose_now

    #Add models relative to the prefab folder of the unity project
    def add_models(self,names_list,pose_list,scale_list=None):
        """Add mesh models that are relative to the prefab folder

        Args:
            names_list ([string]): list of relative name from the prefab folder
            pose_list ([type]): list of poses
        """
        state_msg=ModelStates()
        state_msg.name=names_list
        state_msg.pose=pose_list
        if scale_list is not None:
            state_msg.scale=scale_list
        self._add_model_pub.publish(state_msg)

    def add_frame(self,name,pose,ref_frame_list=""):
        """ add a frame to an existing frame. Experimental usage.

        Args:
            name (_type_): name of the frame to be added
            pose (_type_): pose of the frame to be added
            ref_frame_list (str, optional): parent frame that the frame is added to. Defaults to "".
        """        
        state_msg=ModelStates()
        state_msg.name=[name]
        state_msg.pose=[pose]
        state_msg.header.frame_id = ref_frame_list
        self._add_frame_pub.publish(state_msg)
        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node("ros_unity_scene_manager")
    scene_manager = UnitySceneManager()
    pose_msg=Pose()
    pose_msg.position.x=5.196000099182129
    pose_msg.position.y=1.8860000371932983
    pose_msg.position.z=1.1190000295639038
    q_1=quaternion_from_euler(0,0,0)
    pose_msg.orientation.x=q_1[0]
    pose_msg.orientation.y=q_1[1]
    pose_msg.orientation.z=q_1[2]
    pose_msg.orientation.w=q_1[3]
    scale=Point()
    scale.x=0.001
    scale.y=0.001
    scale.z=0.001
    scene_manager.add_models(['pda6-50_10lvtin'],[pose_msg],[scale])
    rospy.sleep(1)
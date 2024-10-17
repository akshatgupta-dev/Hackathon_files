import sys
import os
import math
import rospy
from geometry_msgs.msg import Pose  # Import geometry messages for Pose
from acroba_unity_msgs.msg import ModelStates  # Import the ModelStates message

def calculate_distance(robot_pose, object_pose):
    """Calculate Euclidean distance between robot and object."""
    return math.sqrt((object_pose.position.x - robot_pose.position.x) ** 2 +
                     (object_pose.position.y - robot_pose.position.y) ** 2 +
                     (object_pose.position.z - robot_pose.position.z) ** 2)

def rank_objects(robot_pose, object_poses):
    """Rank objects based on distance to the robot."""
    ranked_objects = []
    for obj_id, obj_pose in object_poses.items():
        distance = calculate_distance(robot_pose, obj_pose)
        ranked_objects.append((obj_id, distance))

    # Sort objects based on distance (ascending)
    ranked_objects.sort(key=lambda x: x[1])
    return ranked_objects

def callback(data):
    # Obtain robot pose (replace with actual method to get robot's pose)
    robot_pose = Pose()  # Initialize with actual robot's pose
    robot_pose.position.x = 0  # Replace with actual robot's current position
    robot_pose.position.y = 0
    robot_pose.position.z = 0

    object_poses = {}
    
    # Iterate through all models in the ModelStates message
    for i in range(len(data.name)):
        obj_id = data.name[i]
        obj_pose = data.pose[i]
        object_poses[obj_id] = obj_pose  # Store the Pose object directly
    
    ranked_objects = rank_objects(robot_pose, object_poses)
    rospy.loginfo("Ranked Grasping Order: %s", ranked_objects)

def listener():
    rospy.init_node('object_ranking', anonymous=True)
    rospy.Subscriber('/model_states', ModelStates, callback)  # Subscribe to the model states topic
    rospy.spin()

if __name__ == '__main__':
    listener()

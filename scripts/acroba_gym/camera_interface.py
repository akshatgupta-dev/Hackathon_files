#!/usr/bin/env python3

import rospy
import actionlib
import cv2
import numpy as np
from acroba_unity_msgs.msg import RGBDSensorAction, RGBDSensorResult, RGBDSensorGoal
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

class CameraControl(object):
    connected = False

    def __init__(self, global_namespace="", local_namespace='/unity_camera'):
        self._unityclient = actionlib.SimpleActionClient(global_namespace + local_namespace, RGBDSensorAction)
        self._subinfo = rospy.Subscriber(global_namespace + local_namespace + '/camera_info', CameraInfo, self.info_callback)
        self.connected = self._unityclient.wait_for_server(rospy.Duration(5))
        self.intrinsic_matrix = None
        self.camera_width = None
        self.camera_height = None
        self.lut = None

        if not self.connected:
            rospy.logerr("Not possible to connect to the action server")
        else:
            rospy.loginfo("Connected to Unity Action Server")

    def capture(self) -> RGBDSensorResult:
        if not self.connected:
            rospy.logwarn("Reconnecting to action server")
            self.connected = self._unityclient.wait_for_server(rospy.Duration(1))
            if not self.connected:
                rospy.logerr("Connection not possible")
                return None  # Return None if not connected

        goal = RGBDSensorGoal()
        self._unityclient.send_goal(goal)
        self._unityclient.wait_for_result()
        return self._unityclient.get_result()

    def info_callback(self, camera_info: CameraInfo):
        if camera_info is not None:
            self.intrinsic_matrix = np.eye(3)
            self.intrinsic_matrix[0, 0] = camera_info.K[0]
            self.intrinsic_matrix[0, 2] = camera_info.K[2]
            self.intrinsic_matrix[1, 1] = camera_info.K[4]
            self.intrinsic_matrix[1, 2] = camera_info.K[5]

            self.camera_height = camera_info.height
            self.camera_width = camera_info.width

    def depthmap_to_pointlist(self, depth_map: np.ndarray) -> list:
        if self.lut is None and self.intrinsic_matrix is not None:
            self.lut = np.zeros((self.camera_height, self.camera_width, 2), dtype=np.float32)
            for row in range(depth_map.shape[0]):
                for col in range(depth_map.shape[1]):
                    self.lut[row, col, 0] = (col - self.intrinsic_matrix[0, 2]) / self.intrinsic_matrix[0, 0]
                    self.lut[row, col, 1] = (row - self.intrinsic_matrix[1, 2]) / self.intrinsic_matrix[1, 1]

        point_list = []
        x = depth_map * self.lut[:, :, 0]
        y = depth_map * self.lut[:, :, 1]
        xyz = np.stack((x, y, depth_map), axis=-1)
        return xyz.reshape(-1, 3).tolist()

    def localize_pieces(self, depth_image):
        # Convert depth map to point cloud
        point_cloud_array = self.depthmap_to_pointlist(depth_image)
        point_cloud_array = np.array(point_cloud_array)

        # Filter out invalid points (depth = 0)
        valid_points = point_cloud_array[point_cloud_array[:, 2] > 0]

        # Implement clustering (for simplicity, using KMeans clustering)
        from sklearn.cluster import KMeans

        num_clusters = 5  # Adjust based on your scenario
        kmeans = KMeans(n_clusters=num_clusters).fit(valid_points[:, :3])
        labels = kmeans.labels_

        for i in range(num_clusters):
            cluster_points = valid_points[labels == i]
            if len(cluster_points) > 0:
                print(f"Cluster {i}: {len(cluster_points)} points")

if __name__ == '__main__':
    rospy.init_node('camera_interface')
    camera_interface = CameraControl()
    rospy.loginfo("Is connected %i", camera_interface.connected)

    action_result = camera_interface.capture()
    if action_result is not None:  # Ensure action_result is valid
        bridge_object = CvBridge()
        cv_rgb_image = bridge_object.imgmsg_to_cv2(action_result.image, desired_encoding="bgra8")
        cv_depth_image = bridge_object.imgmsg_to_cv2(action_result.depth_map, desired_encoding="32FC1")

        print("Intrinsic Matrix\n", camera_interface.intrinsic_matrix)

        # Resize the RGB image
        desired_width = 640  # Desired width
        desired_height = 480  # Desired height
        resized_rgb_image = cv2.resize(cv_rgb_image, (desired_width, desired_height))

        # Display the resized RGB image
        cv2.imshow("Resized RGB Image", resized_rgb_image)
        cv2.waitKey(0)  # Wait for a key press to close the window
        cv2.destroyAllWindows()

        # Localize pieces in the captured depth image
        camera_interface.localize_pieces(cv_depth_image)
    else:
        rospy.logerr("Failed to capture images.")

import rospy
import numpy as np

from sensor_msgs.msg import JointState
import sys
import roslib.packages as rp
from tf_tools.srv import *
from tf_tools.msg import ROSMap

class TFClient:
    def __init__(self):
        rospy.loginfo("[TFClient] initializing...")
        self.got_state=False
        self.loop_rate=rospy.Rate(100.0)
        self.srv_name = "visualize"
        rospy.loginfo("[TFClient] initialized")


    def update_points_markers(self, points, frames):
        req = DataRequest()
        req.control_mode = 1
        req.index_points = points[0].ravel('F') # 'F' denotes column vectors 
        req.middle_points = points[1].ravel('F')
        req.ring_points = points[2].ravel('F')
        req.thumb_points = points[3].ravel('F')
        req.string_data = frames

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_object_normals_markers(self, normals_tails, normals_tips, frame):
        self.update_normals_markers(normals_tails, normals_tips, frame, "OBJECT")

    def update_fingertips_normals_markers(self, normals_tails, normals_tips, frame):
        self.update_normals_markers(normals_tails, normals_tips, frame, "FINGERTIP")

    def update_normals_markers(self, normals_tails, normals_tips, frame, mode):
        req = DataRequest()
        req.control_mode = 6
        req.string_data = [frame, mode]

        req.normal_0_tail = normals_tails[0]
        req.normal_1_tail = normals_tails[1]
        req.normal_2_tail = normals_tails[2]
        req.normal_3_tail = normals_tails[3]

        req.normal_0_tip = normals_tips[0]
        req.normal_1_tip = normals_tips[1]
        req.normal_2_tip = normals_tips[2]
        req.normal_3_tip = normals_tips[3]

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_planes(self, planes, frames):
        req = DataRequest()
        req.control_mode = 2
        req.data = extract_data_from_planes(planes)
        req.string_data = frames

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_closest_points(self, points):
        points_for_visualization = []
        for i in range(points.shape[1]):
            point = points[:,i]
            points_for_visualization += point.tolist()
        req = DataRequest()
        req.control_mode = 3
        req.data = points_for_visualization

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_marker_mesh(self, mesh_resource_path, frame):
        req = DataRequest()
        req.control_mode = 10
        req.string_data = [mesh_resource_path, frame]

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_box_mesh(self, frame, width, height, depth):
        req = DataRequest()
        req.control_mode = 11
        req.string_data = [frame, str(width), str(height), str(depth)]

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def update_object_pose(self, position, rotation, obj_frame, parent_frame="world"):
        '''
        @param position Point
        @param rotation Quaternion
        '''
        req = DataRequest()
        req.control_mode = 4
        req.data = [position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w]
        req.string_data = [obj_frame, parent_frame]

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_mesh_pose(self, position, rotation, obj_frame, parent_frame="world"):
        '''
        position and rotation are arrays
        '''
        req = DataRequest()
        req.control_mode = 7
        try:
            req.data = [position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3]]
        except TypeError:
            req.data = [position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w]
        req.string_data = [obj_frame, parent_frame]

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def add_frame(self, position, rotation, obj_frame, parent_frame="world"):
        '''
        position and rotation are arrays
        '''
        req = DataRequest()
        req.control_mode = 8
        req.data = [position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3]]
        req.string_data = [obj_frame, parent_frame]

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def delete_all_frames(self):
        '''
         Removes frames added via 'add_frame' method.
        '''
        req = DataRequest()
        req.control_mode = 9

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_collision_spheres(self, spheres_poses, spheres_diameters):
        req = DataRequest()
        req.control_mode = 12
        spheres_ros = []
        for sphere in spheres_poses:
            ros_map = ROSMap(sphere,spheres_poses[sphere], spheres_diameters[sphere])
            spheres_ros.append(ros_map)
        req.spheres = spheres_ros

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_projected_points(self, points, frames):
        req = DataRequest()
        print("points: ", points)
        req.control_mode = 5
        req.index_points = points[0].ravel('F') # 'F' denotes column vectors 
        req.middle_points = points[1].ravel('F')
        req.ring_points = points[2].ravel('F')
        req.thumb_points = points[3].ravel('F')
        req.string_data = frames
        print("req: ", req)

        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def call_service(self, req):
        rospy.wait_for_service(self.srv_name)
        try:
            service = rospy.ServiceProxy(self.srv_name, Data)
            resp = service(req)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


def extract_data_from_planes(planes):
    data = []
    for plane in planes:
        (x,y,z), origin = plane
        data += x.ravel().tolist()
        data += y.ravel().tolist()
        data += z.ravel().tolist()
        data += origin.ravel().tolist()
    return data

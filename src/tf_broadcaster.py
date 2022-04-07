#!/usr/bin/env python
import roslib
import rospy
import tf
import threading
import numpy as np
from math import sqrt as sqrt
from tf_tools.srv import Data
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R

ARM_NAME = "lbr4"
class TFBroadcaster():
    def __init__(self):
        rospy.init_node("fixed_tf_broadcater")# for debugging
        self.points = []
        self.planes_markers = []
        self.points_markers = []
        self.object_normals_markers = []
        self.ftips_normals_markers = []
        self.contacts_pub = rospy.Publisher("contact_points", Marker, queue_size=1)
        self.planes_pub = rospy.Publisher("planes", Marker, queue_size=1)
        self.projected_pub = rospy.Publisher("projected_points", Marker, queue_size=1)
        self.object_normals_pub = rospy.Publisher("object_normals", Marker, queue_size=1)
        self.ftips_normals_pub = rospy.Publisher("ftips_normals", Marker, queue_size=1)
        self.mesh_marker_pub = rospy.Publisher("mesh_marker", Marker, queue_size=1)
        self.box_marker_pub = rospy.Publisher("box_marker", Marker, queue_size=1)
        self.collision_spheres_pub = rospy.Publisher("collision_spheres", Marker, queue_size=1)
        self.update_collision_spheres()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(100)
        self.thread_started = False
        self.object_position = None
        self.object_orientation = None
        self.mesh_position = None
        self.mesh_orientation = None
        self.projected_points = []
        self.frames_position = []
        self.frames_orientation = []
        self.frames_names = []
        self.frames_parent_frame = []
        self.mesh_marker = None
        self.box_marker = None

        s = rospy.Service("visualize", Data, self.callback)

    def callback(self, req):
        control_mode = req.control_mode

        if control_mode == 1: # points as markers
            points = []
            points.append(parse_points(req.index_points))
            points.append(parse_points(req.middle_points))
            points.append(parse_points(req.ring_points))
            points.append(parse_points(req.thumb_points))
            frames = req.string_data
            self.update_points_markers(points, frames)
            return True
        elif control_mode == 2: # planes visualization
            planes = parse_planes(req.data)
            frames = req.string_data
            self.update_planes(planes, frames)
            return True
        elif control_mode == 3: # closest points visualization
            self.points = []
            points = parse_points(req.data)
            for i in range(points.shape[1]):
                point = points[:,i]
                self.points.append(point)
        elif control_mode == 4: # estimated object pose TF
            self.update_object_pose(req)
        elif control_mode == 5: # projected point
            points = []
            points.append(parse_points(req.index_points))
            points.append(parse_points(req.middle_points))
            points.append(parse_points(req.ring_points))
            points.append(parse_points(req.thumb_points))
            frames = req.string_data
            self.update_projected_points(points, frames)
        elif control_mode == 6: # visualize normals
            frame = req.string_data[0]
            mode = req.string_data[1]
            normals_tails = [req.normal_0_tail, req.normal_1_tail, req.normal_2_tail, req.normal_3_tail]
            normals_tips = [req.normal_0_tip, req.normal_1_tip, req.normal_2_tip, req.normal_3_tip]
            self.update_normals_markers(normals_tails, normals_tips, frame, mode)
        elif control_mode == 7: # object mesh pose TF
            self.update_mesh_pose(req)
        elif control_mode == 8: # add another TF to publish
            self.add_frame(req)
        elif control_mode == 9: # delete frames
             self.frames_position = []
             self.frames_orientation = []
             self.frames_names = []
             self.frames_parent_frame = []
        elif control_mode == 10:
             mesh_resource = req.string_data[0]
             frame_name = req.string_data[1]
             self.update_mesh_marker(mesh_resource, frame_name)
        elif control_mode == 11:
             frame = req.string_data[0]
             x = float(req.string_data[1])
             y = float(req.string_data[2])
             z = float(req.string_data[3])
             self.update_box_marker(frame, x,y,z)
        else:
            raise Exception("mode not supported yet")
         
        return False
        
    def update_object_pose(self, req):
        '''
        Estimated object pose, center of the object.
        '''
        self.object_position = [req.data[0], req.data[1], req.data[2]]
        self.object_orientation = [req.data[3], req.data[4], req.data[5], req.data[6]]
        self.object_frame_name = req.string_data[0]
        self.object_parent_frame = req.string_data[1]

    def update_mesh_pose(self, req):
        '''
        Mesh pose, frame located as in .stl file
        '''
        self.mesh_position = [req.data[0], req.data[1], req.data[2]]
        self.mesh_orientation = [req.data[3], req.data[4], req.data[5], req.data[6]]
        self.mesh_frame_name = req.string_data[0]
        self.mesh_parent_frame = req.string_data[1]

    def add_frame(self, req):
        self.frames_position.append([req.data[0], req.data[1], req.data[2]])
        self.frames_orientation.append([req.data[3], req.data[4], req.data[5], req.data[6]])
        self.frames_names.append(req.string_data[0])
        self.frames_parent_frame.append(req.string_data[1])

    def update_collision_spheres(self):
        link_diameter= {'2': 0.195, '3': 0.195, '4': 0.195, '5': 0.195, '6': 0.195, '7': 0.10500000000000001, 'palm': 0.005, 'allegro_mount': 0.10500000000000001, 'virtual_palm_center': 0.14500000000000002, 'index_link_0': 0.0, 'index_link_1': 0.04, 'index_link_2': 0.04, 'index_link_3': 0.04, 'middle_link_0': 0.0, 'middle_link_1': 0.04, 'middle_link_2': 0.04, 'middle_link_3': 0.04, 'ring_link_0': 0.0, 'ring_link_1': 0.04, 'ring_link_2': 0.04, 'ring_link_3': 0.04, 'thumb_link_0': 0.0, 'thumb_link_1': 0.04, 'thumb_link_2': 0.04, 'thumb_link_3': 0.04, 'index_biotac_origin': 0.02, 'middle_biotac_origin': 0.02, 'ring_biotac_origin': 0.02, 'thumb_biotac_origin': 0.02, 'index_virtual_sphere_1': 0.04, 'index_virtual_sphere_2': 0.04, 'index_virtual_sphere_3': 0.030000000000000002, 'index_biotac_virtual_sphere_1': 0.02, 'index_biotac_virtual_sphere_2': 0.02, 'middle_virtual_sphere_1': 0.04, 'middle_virtual_sphere_2': 0.04, 'middle_virtual_sphere_3': 0.030000000000000002, 'middle_biotac_virtual_sphere_1': 0.02, 'middle_biotac_virtual_sphere_2': 0.02, 'ring_virtual_sphere_1': 0.04, 'ring_virtual_sphere_2': 0.04, 'ring_virtual_sphere_3': 0.030000000000000002, 'ring_biotac_virtual_sphere_1': 0.02, 'ring_biotac_virtual_sphere_2': 0.02, 'thumb_virtual_sphere_2': 0.04, 'thumb_virtual_sphere_3': 0.030000000000000002, 'thumb_biotac_virtual_sphere_1': 0.02, 'thumb_biotac_virtual_sphere_2': 0.02}

        self.collision_spheres_markers = []
        
        if ARM_NAME == "lbr4":
            frames = ["lbr4_" + str(i) + "_link" for i in range(2,8)]
        else:
            frames = ["iiwa_link_" + str(i) for i in range(2,8)]
        for i in range(len(frames)):
            if "iiwa" in frames[i]:
                sphere_id = frames[i].split("_")[-1]
            else:
                sphere_id = frames[i].split("_")[1]
            marker = Marker()
            marker.header.frame_id = frames[i]
            marker.ns = "frame_sphere=" + str(frames[i])
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = link_diameter[sphere_id]
            marker.scale.y = marker.scale.x
            marker.scale.z = marker.scale.x
            marker = set_color(marker, 2, 0.5)
            marker.pose.orientation.w = 1.0
            self.collision_spheres_markers.append(marker)

        for frame in link_diameter:
            if frame in set([str(i) for i in range(2,8)]):
                continue
            marker = Marker()
            marker.header.frame_id = frame
            marker.ns = "frame_sphere=" + frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = link_diameter[frame]
            marker.scale.y = marker.scale.x
            marker.scale.z = marker.scale.x
            marker = set_color(marker, 2, 0.5)
            marker.pose.orientation.w = 1.0
            self.collision_spheres_markers.append(marker)

    def update_points_markers(self, points, frames):
        self.points_markers = []
        for i, finger in enumerate(points):
            marker = Marker()
            marker.header.frame_id = frames[i]
            marker.ns = "points=" + str(i)
            marker.type = marker.SPHERE_LIST
            marker.action = marker.ADD
            marker.scale.x = .01
            marker.scale.y = .01
            marker.scale.z = .01
            marker = set_color(marker, i)
            marker.pose.orientation.w = 1.0
            for j in range(finger.shape[1]):
                point_raw = finger[:,j]
                point = Point()
                point.x = point_raw[0]
                point.y = point_raw[1]
                point.z = point_raw[2]
                marker.points.append(point)
            self.points_markers.append(marker)

    def update_normals_markers(self, normals_tails, normals_tips, frame, mode):
        if mode == "FINGERTIP":
            self.ftips_normals_markers = []
        elif mode == "OBJECT":
            self.object_normals_markers = []
        else:
            raise Exception("mode not supported")

        for i, normal in enumerate(normals_tails):
            marker = Marker()
            marker.header.frame_id = frame
            marker.ns = "normal=" + str(i)
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = .003
            marker.scale.y = .003
            marker.scale.z = .003
            if mode == "OBJECT":
                marker = set_color(marker, i)
            else:
                marker = set_color(marker, 4)
            marker.pose.orientation.w = 1.0

            tail_array = normals_tails[i]
            tail = Point(tail_array[0], tail_array[1], tail_array[2])

            tip_array = normals_tips[i]
            tip = Point(tip_array[0], tip_array[1], tip_array[2])

            # nicer visualization
            scale = 0.05
            tip_array_short = [normals_tails[i][0] + (normals_tips[i][0] - normals_tails[i][0]) * scale,
                               normals_tails[i][1] + (normals_tips[i][1] - normals_tails[i][1]) * scale,
                               normals_tails[i][2] + (normals_tips[i][2] - normals_tails[i][2]) * scale]
            tip_short  = Point(tip_array_short[0], tip_array_short[1], tip_array_short[2])

            marker.points = [tail, tip_short]
            #marker.points = [tail, tip]
            if mode == "FINGERTIP":
                self.ftips_normals_markers.append(marker)
            elif mode == "OBJECT": 
                self.object_normals_markers.append(marker)

    def update_projected_points(self, points, frames):
        self.projected_points = []
        for i, finger in enumerate(points):
            marker = Marker()
            marker.header.frame_id = frames[i]
            marker.ns = "points=" + str(i)
            marker.type = marker.SPHERE_LIST
            marker.action = marker.ADD
            marker.scale.x = .003
            marker.scale.y = .003
            marker.scale.z = .003
            marker = set_color(marker, i)
            marker.pose.orientation.w = 1.0
            for j in range(finger.shape[1]):
                point_raw = finger[:,j]
                point = Point()
                point.x = point_raw[0]
                point.y = point_raw[1]
                point.z = point_raw[2]
                marker.points.append(point)
            self.projected_points.append(marker)

    def update_planes(self, planes, frames):
        self.planes_markers = []
        for i, plane in enumerate(planes):
            axis, origin = plane
            x,y,z = axis

            marker = Marker()
            marker.header.frame_id = frames[i]
            marker.ns = "ns=" + str(i)
            marker.id = i

            marker.type = marker.CUBE
            marker.action = marker.ADD

            marker.pose.position.x = origin[0]
            marker.pose.position.y = origin[1]
            marker.pose.position.z = origin[2]

            # orientation
            rot_matrix = np.matrix([[x[0,0], y[0,0], z[0,0]],
                                    [x[1,0], y[1,0], z[1,0]],
                                    [x[2,0], y[2,0], z[2,0]]])
            quat_R = R.from_dcm(rot_matrix).as_quat()

            marker.pose.orientation.x = quat_R[0]
            marker.pose.orientation.y = quat_R[1]
            marker.pose.orientation.z = quat_R[2]
            marker.pose.orientation.w = quat_R[3]

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.001 # to look like a small plane

            #r,g,b,y depending on i
            marker = set_color(marker, i)
            self.planes_markers.append(marker)

    def update_box_marker(self, frame, x, y, z):
        self.box_marker = None

        marker = Marker()
        marker.header.frame_id = frame
        marker.type = Marker.CUBE
        marker.id = 2
        marker.action = Marker.MODIFY

        position = [0,0,0]
        orientation = [0,0,0,1]
        scale = [x,y,z]
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.a = 0.6
        marker.color.b = 1.0

        self.box_marker = marker

    def update_mesh_marker(self, mesh_resource, frame):
        self.mesh_marker = None
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = mesh_resource
        marker.id = 1
        marker.action = Marker.MODIFY

        position = [0,0,0]
        orientation = [0,0,0,1]
        scale = [1,1,1]
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.a = 0.6
        marker.color.g = 1.0

        self.mesh_marker = marker

    def publish(self):
        while not rospy.is_shutdown():
            # points on the object
            for i, point in enumerate(self.points):
                 self.br.sendTransform(point,
                                  (1,0,0,0),
                                  rospy.Time.now(),
                                  "point_" + str(i), # child
                                  "palm_link") # parent
            
            # object frame
            if self.object_position is not None:
                self.br.sendTransform(self.object_position,
                                      self.object_orientation,
                                      rospy.Time.now(),
                                      self.object_frame_name,
                                      self.object_parent_frame)

            if self.mesh_position is not None:
                self.br.sendTransform(self.mesh_position,
                                      self.mesh_orientation,
                                      rospy.Time.now(),
                                      self.mesh_frame_name,
                                      self.mesh_parent_frame)

            for i, _ in enumerate(self.frames_position):
                self.br.sendTransform(self.frames_position[i],
                                      self.frames_orientation[i],
                                      rospy.Time.now(),
                                      self.frames_names[i],
                                      self.frames_parent_frame[i])

            # frames are set in headers
            for marker in self.planes_markers:
                self.planes_pub.publish(marker)

            for marker in self.points_markers:
                self.contacts_pub.publish(marker)

            for marker in self.ftips_normals_markers:
                self.ftips_normals_pub.publish(marker)

            for marker in self.object_normals_markers:
                self.object_normals_pub.publish(marker)

            for marker in self.collision_spheres_markers:
                self.collision_spheres_pub.publish(marker)

            for marker in self.projected_points:
                self.projected_pub.publish(marker)

            if self.mesh_marker is not None:
                self.mesh_marker_pub.publish(self.mesh_marker)
            if self.box_marker is not None:
                self.box_marker_pub.publish(self.box_marker)

            self.rate.sleep()

def set_color(marker,i, alpha=1):
    marker.color.a = alpha # transparency 
    if i == 0: # index
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
    elif i == 1: # middle
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
    elif i == 2: # ring
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
    elif i == 3: # thumb
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 0
    else:
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 1
    return marker

def parse_points(points):
    nr_points = int(len(points)/3)
    np_points = np.zeros((3,nr_points))
    for i in range(nr_points):
        np_points[:,i] = points[i*3:(i+1)*3]
    return np_points

def parse_plane(data):
    x = np.matrix(data[0:3]).T
    y = np.matrix(data[3:6]).T
    z = np.matrix(data[6:9]).T
    origin = np.matrix(data[9:12]).T
    plane = [(x,y,z), origin]
    return plane

def parse_planes(data):
    planes = []
    for i in range(4):
        planes.append(parse_plane(data[i*12:(i+1)*12]))

    return planes

def main():
    broadcaster = TFBroadcaster()
    broadcaster.publish()

if __name__ == "__main__":
    main()

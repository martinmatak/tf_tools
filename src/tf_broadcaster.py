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

class TFBroadcaster():
    def __init__(self):
        rospy.init_node("fixed_tf_broadcater")# for debugging
        self.points = []
        self.planes_markers = []
        self.points_markers = []
        self.contacts_pub = rospy.Publisher("contact_points", Marker, queue_size=1)
        self.planes_pub = rospy.Publisher("planes", Marker, queue_size=1)
        self.projected_pub = rospy.Publisher("projected_points", Marker, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(1)
        self.thread_started = False
        self.object_position = None
        self.object_orientation = None
        self.projected_point = None
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
        elif control_mode == 4: # object pose TF
            self.update_object_pose(req)
        elif control_mode == 5: # projected point
            point = np.zeros((3,1))
            data = req.data
            point[0,0] = data[0]
            point[1,0] = data[1]
            point[2,0] = data[2]
            frame = req.string_data[0]
            self.update_projected_point(point, frame)
        else:
            raise Exception("mode not supported yet")
         
        return False
        
    def update_object_pose(self, req):
        self.object_position = [req.data[0], req.data[1], req.data[2]]
        self.object_orientation = [req.data[3], req.data[4], req.data[5], req.data[5]]
        self.object_frame_name = req.string_data[0]
        self.object_parent_frame = req.string_data[1]

    def update_points_markers(self, points, frames):
        self.points_markers = []
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
            self.points_markers.append(marker)

    def update_projected_point(self, point, frame):
        print("updated projected point")
        self.projected_point = []
        marker = Marker()
        marker.header.frame_id = frame
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = .003
        marker.scale.y = .003
        marker.scale.z = .003

        marker.color.a = 1
        marker.color.r = 255
        marker.color.g = 140
        marker.color.b = 0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]

        self.projected_point = marker

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
           
            for marker in self.planes_markers:
                self.planes_pub.publish(marker)

            for marker in self.points_markers:
                self.contacts_pub.publish(marker)

            if self.projected_point is not None:
                self.projected_pub.publish(self.projected_point)

            self.rate.sleep()

def set_color(marker,i):
    marker.color.a = 1 # transparency 
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
    else: # thumb
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 0

    return marker

def parse_points(points):
    nr_points = len(points) / 3
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

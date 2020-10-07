#!/usr/bin/env python
import roslib
import rospy
import tf
import threading
import numpy as np
from math import sqrt as sqrt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R

class TFBroadcaster():
    def __init__(self):
        #rospy.init_node("fixed_tf_broadcater")# for debugging
        self.points = []
        self.planes_markers = []
        self.points_markers = []
        self.marker_pub = rospy.Publisher("planes", Marker, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)
        self.thread_started = False
        self.object_position = None
        self.object_orientation = None

    def update_object_pose(self, object_position, object_orientation, frame_name, parent_frame="world"):
        self.object_position = [object_position.x, object_position.y, object_position.z]
        self.object_orientation = [object_orientation.x, object_orientation.y, object_orientation.z, object_orientation.w]
        self.object_frame_name = frame_name
        self.object_parent_frame = parent_frame
        self.start_publishing()

    def update_points(self, points):
        assert points is not None, "Points must not be None"
        self.points = points
        self.start_publishing()

    def update_points_markers(self, points, frames):
        for i, finger in enumerate(points):
            for j in range(finger.shape[1]):
                marker = Marker()
                marker.header.frame_id = frames[i]
                marker.ns = "points=" + str(i)
                marker.id = j

                marker.type = marker.SPHERE
                marker.action = marker.ADD

                marker.scale.x = .003
                marker.scale.y = .003
                marker.scale.z = .003

                marker = set_color(marker, i)

                marker.pose.orientation.w = 1.0

                point = finger[:,j]
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = point[2]
                self.points_markers.append(marker)

        self.start_publishing()

    def update_planes(self, planes, frames):
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

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.001 # to look like a small plane

            #r,g,b,y depending on i
            marker = set_color(marker, i)
            self.planes_markers.append(marker)

        self.start_publishing()

    def start_publishing(self):
        if not self.thread_started:
            thread = threading.Thread(target=self.publish, args=(), kwargs={})
            thread.setDaemon(True)
            thread.start()
            self.thread_started = True

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
                marker.header.stamp = rospy.Time.now()
                self.marker_pub.publish(marker)

            for marker in self.points_markers:
                marker.header.stamp = rospy.Time.now()
                self.marker_pub.publish(marker)

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

'''
def main():
    broadcaster = TFBroadcaster()
    index = np.random.rand(3,3)
    middle = np.random.rand(3,3)
    ring = np.random.rand(3,3)
    thumb = np.random.rand(3,3)
   
    frames = ["map", "map", "map", "map"]
    points = [index, middle, ring, thumb]
    broadcaster.update_points_markers(points, frames)
    broadcaster.publish()

if __name__ == "__main__":
    main()
'''

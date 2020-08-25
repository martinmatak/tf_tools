#!/usr/bin/env python
import roslib
import rospy
import tf
import threading

class TFBroadcaster():
    def __init__(self):
        rospy.init_node("fixed_tf_broadcater")
        self.points = []
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)
        self.thread_started = False
        self.object_position = None
        self.object_orientation = None

    def update_object_pose(self, object_position, object_orientation, frame_name, parent_frame="world"):
        self.object_position = object_position
        self.object_orientation = object_orientation
        self.object_frame_name = frame_name
        self.object_parent_frame = parent_frame
        self.start_publishing()

    def update_points(self, points):
        assert points is not None, "Points must not be None"
        self.points = points
          
    def start_publishing(self):
        if not self.thread_started:
            thread = threading.Thread(target=self.publish_points, args=(), kwargs={})
            thread.setDaemon(True)
            thread.start()
            self.thread_started = True

    def publish_points(self):
        while not rospy.is_shutdown():
            # points on the object
            for i, point in enumerate(self.points):
                 self.br.sendTransform(point,
                                  (1,0,0,0),
                                  rospy.Time.now(),
                                  "point_" + str(i), # child
                                  "palm_link") # parent
            # object frame
            self.br.sendTransform(self.object_position,
                                  self.object_orientation,
                                  rospy.Time.now(),
                                  self.object_frame_name,
                                  self.object_parent_frame)
            self.rate.sleep()

#!/usr/bin/env python
import roslib
import rospy
import tf
import threading

class TFBroadcaster():
    def __init__(self):
        self.points = []
        #rospy.init_node('tf_broadcaster_helper')
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)
        self.thread_started = False

    def update_points(self, points):
        assert points is not None, "Points must not be None"
        self.points = points

        if not self.thread_started:
            thread = threading.Thread(target=self.publish_points, args=(), kwargs={})
            thread.setDaemon(True)
            thread.start()
            print("thread started")
            self.thread_started = True

    def publish_points(self):
        while not rospy.is_shutdown():
            for i, point in enumerate(self.points):
                 self.br.sendTransform(point,
                                  (1,0,0,0),
                                  rospy.Time.now(),
                                  "point_" + str(i), # child
                                  "palm_link") # parent
            self.rate.sleep()

import rospy
import numpy as np

from sensor_msgs.msg import JointState
import sys
import roslib.packages as rp
from tf_tools.srv import *

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

    def update_planes(self, planes, frames):
        req = DataRequest()
        req.control_mode = 2
        req.data = extract_data_from_planes(planes)
        req.string_data = frames
        print("request: ", req)
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
    print("data: ", data)
    return data

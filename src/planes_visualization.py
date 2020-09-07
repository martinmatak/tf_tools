#!/usr/bin/env python
import roslib
import rospy
import threading
import sys
from matplotlib import colors
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class PlaneVisualization():
    
    def __init__(self, topic):
        rospy.init_node("plane_visualization") 
        #self.robot_state_pub = rospy.Publisher(topic, DisplayRobotState, queue_size=1)
        self.marker_publisher = rospy.Publisher(topic, Marker, queue_size=1)
        color = get_color('deepskyblue')
        #update_state_service = rospy.Service("update_robot_state", UpdateRobotState, self.update_robot_state)

    #def update_robot_state(self, request):
    #    arm_msg = rospy.wait_for_message("lbr4/joint_states", JointState)
    #    arm_joints = arm_msg.position
    #    hand_joints = request.joint_state
    #    self.robot_state.state.joint_state.position = arm_joints + hand_joints
    #    return UpdateRobotStateResponse(success=True)

def get_color(color_name):
    """
    color can be any name from this page:
    http://matplotlib.org/mpl_examples/color/named_colors.hires.png
    """
    converter = colors.ColorConverter()
    c = converter.to_rgba(colors.cnames[color_name])
    return ColorRGBA(*c)

if __name__ == '__main__':
    plane_vis = PlaneVisualization("marker_topic")
    rate = rospy.Rate(10.0)
    marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=0,
            pose=Pose(Point(1, 1, 1), Quaternion(0, 0, 0, 1)),
            scale=Vector3(3.36, 3.36, 3.96),
            header=Header(frame_id='world'),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
            text="hello world")
    while not rospy.is_shutdown():
        plane_vis.marker_publisher.publish(marker)
        rate.sleep()

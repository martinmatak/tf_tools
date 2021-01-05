#!/usr/bin/env python
import roslib
import rospy
import threading
import sys
from sensor_msgs.msg import JointState
from matplotlib import colors
from moveit_msgs.msg import RobotState, DisplayRobotState, ObjectColor
from std_msgs.msg import ColorRGBA
sys.path.append(roslib.packages.get_pkg_dir('grasp_pipeline'))
from grasp_pipeline.srv import *

class RobotStatePublisher():
    def __init__(self, topic, jvalue):
        rospy.init_node("robot_state_visualization") 
        self.robot_state_pub = rospy.Publisher(topic, DisplayRobotState, queue_size=1)
        self.robot_state = DisplayRobotState()
        #arm_joints = ["lbr4_j0", "lbr4_j1", "lbr4_j2", "lbr4_j3", "lbr4_j4", "lbr4_j5", "lbr4_j6"]
        hand_joints = ["index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
                       "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
                       "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
                       "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3"]
        #all_joints = arm_joints + hand_joints
        all_joints = hand_joints

        links = ["index_link_0", "index_link_1", "index_link_2", "index_link_3",
                 "middle_link_0", "middle_link_1", "middle_link_2", "middle_link_3",
                 "ring_link_0", "ring_link_1", "ring_link_2", "ring_link_3",
                 "thumb_link_0", "thumb_link_1", "thumb_link_2", "thumb_link_3"]

        self.robot_state.state.joint_state.name =  all_joints
        #self.robot_state.state.joint_state.position = [jvalue for i in range(len(all_joints))]
        self.robot_state.state.joint_state.position = [
         0.8774624,  1.9454455,   0.16636288,  1.5155147,   2.6493096,  -0.5224206,
 -1.4907938,   0.07906261,  0.5507617,   0.24776387, -2.169597,    1.0809791,
  1.3541403,  -2.8308346,   1.9292004,  -2.0491912        
        ]
        self.robot_state.state.joint_state.position = [
                -0.32902139845484335, 0.25842531985086836, -0.031838223531752696, -0.07480663652901409, -0.02899749818495323, -0.07829753524648275, 0.00020138101391786734, 0.0021933261367976797, 0.08626520755942302, -0.12569796558433133, 0.036194250465382716, 0.03316555749083516, 1.4398069372375693, 0.07860907941452844, 0.029524747362523307, 0.31397873869704274]
        color = get_color('deepskyblue')
        self.robot_state.highlight_links = [ObjectColor(id=l, color=color) for l in links]
        update_state_service = rospy.Service("update_robot_state", UpdateRobotState, self.update_robot_state)

    def update_robot_state(self, request):
        arm_msg = rospy.wait_for_message("lbr4/joint_states", JointState)
        arm_joints = arm_msg.position
        hand_joints = request.joint_state
        self.robot_state.state.joint_state.position = arm_joints + hand_joints
        return UpdateRobotStateResponse(success=True)

def get_color(color_name):
    """
    color can be any name from this page:
    http://matplotlib.org/mpl_examples/color/named_colors.hires.png
    """
    converter = colors.ColorConverter()
    c = converter.to_rgba(colors.cnames[color_name])
    return ColorRGBA(*c)

if __name__ == '__main__':
    rsp_grad = RobotStatePublisher("floating_allegro2", 1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        rsp_grad.robot_state_pub.publish(rsp_grad.robot_state)
        rate.sleep()

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

class JointStatePublisher():
    def __init__(self, topic, jvalue):
        rospy.init_node("allegro_tf_kinematics") 
        self.robot_state_pub = rospy.Publisher(topic, JointState, queue_size=1)
        self.robot_state = JointState()
        hand_joints = ["index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
                       "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
                       "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
                       "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3"]
        self.robot_state.name = hand_joints
        self.robot_state.position = [jvalue for i in range(len(hand_joints))]
        rospy.Subscriber("/allegro_hand_right/joint_cmd", JointState, self.update_robot_state)
 

    def update_robot_state(self, request):
        #arm_msg = rospy.wait_for_message("lbr4/joint_states", JointState)
        #arm_joints = arm_msg.position
        #self.robot_state.state.joint_state.position = arm_joints + hand_joints
        self.robot_state.state.joint_state.position = request.joint_state
        #return UpdateRobotStateResponse(success=True)

if __name__ == '__main__':
    js_publisher = JointStatePublisher("allegro_hand_right/joint_states", 1)
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        js_publisher.robot_state.header.stamp = rospy.Time.now()
        js_publisher.robot_state_pub.publish(js_publisher.robot_state)
        rate.sleep()

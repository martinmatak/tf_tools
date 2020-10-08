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
        req.index_points = points[0].ravel('F')
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

    '''
    def send_pos_cmd(self,q_des, contact_sensing = True):
        #print("waiting for service", self.srv_name)
        des_js=JointState()
        des_js.position=q_des
            resp=req(0,[0],[],des_js,JointTrajectory(),0,False,[],[],[],[], contact_sensing)
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def send_jtraj(self,jtraj,stop_at_contact=False):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(1,[0,1,2,3],[],JointState(),jtraj,0,stop_at_contact,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def reset_tare(self,f_idx=[0,1,2,3]):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(7,f_idx,[],JointState(),JointTrajectory(),0,True,[],[],[],[], False)
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def increase_stiffness(self,j_idx=[0,3, 4,7, 8,11, 12,15],des_js=JointState()):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(4,[],j_idx,des_js,JointTrajectory(),0,True,[],[],[],[], False)
            return resp.reached,resp.last_des_cmd
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def disable_stiffness(self,f_idx=[0,3]):
        print "currently unavailable"
        exit()
        
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(4,f_idx,[],JointState(),JointTrajectory(),0,True,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def get_smooth_traj(self,jtraj):
        rospy.wait_for_service('allegro/get_smooth_trajectory')
        traj_call=rospy.ServiceProxy('allegro/get_smooth_trajectory',GetSmoothTraj)
        max_acc=np.ones(16)*0.25
        max_vel=np.ones(16)*0.4

        resp=traj_call(jtraj,max_acc,max_vel,0.2,0.001)
        smooth_traj=resp.smooth_traj
        return smooth_traj
        
    def grasp_object(self,j_idx=[x for x in range(16)]):
        rospy.loginfo("Waiting for service: " + self.srv_name)
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(2,[],j_idx,JointState(),JointTrajectory(),0,True,[],[],[],[], True)
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def approach_contact(self,j_init,f_idx, app_vec, desired_position=[]):
        des_js=JointState()
        # des_js.position= j_init
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(3,f_idx,[],des_js,JointTrajectory(),0,True,app_vec+desired_position,[],[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def task_position_control(self, f_idx, desired_positions):
        flat_list = []
        for position in desired_positions:
           for coordinate in position:
               flat_list.append(coordinate)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(10, f_idx, [], JointState(), JointTrajectory(), 0, True, flat_list, [], [], [])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def force_servo(self,f_idx,app_vec,f_mag):
	# data = [f1_x, f1_y, f1_z, ..., f4_x, f4_y, f4_z, f_mag_f1, f_mag_f2, f_mag_f3, f_mag_f4]
        data=app_vec+f_mag # concatenation
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(8,f_idx,[],JointState(),JointTrajectory(),0,True,data,[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def enable_external_control(self,external_cmd_topic):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(5,[],[],JointState(),JointTrajectory(),0,False,[],[external_cmd_topic])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    def disable_external_control(self):
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            resp=req(6,[],[],JointState(),JointTrajectory(),0,False,[],[])
            return resp.reached
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def slide(self,j_idx=[1]):
        print("sliding..")
        rospy.loginfo("Waiting for service: " + self.srv_name)
        rospy.wait_for_service(self.srv_name)
        try:
            req=rospy.ServiceProxy(self.srv_name,SendHandCommand)
            positions = [0.034798 , 0.273659 , 0.345295, 0.241133 , 0.212866 , 0.245579, 0.239687 , -0.278327 , 0.241913, 0.229326 , 0.298814 , -0.275324]
            forces = [0, 0, 5, 0, 0, 5, 0, 0, 5, 0, 0, 5]
            resp=req(9,[],j_idx,JointState(),JointTrajectory(),0,True,[],[], positions, forces)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return resp.reached

    def get_jacobians(self):
        srv_name = "allegro/get_jacobians"
        rospy.wait_for_service(srv_name)
        try:
            req = rospy.ServiceProxy(srv_name, JacobiansMsg)
            response = req()
            return extract_jacobian(np.asarray(response.index)),\
                   extract_jacobian(np.asarray(response.middle)),\
                   extract_jacobian(np.asarray(response.ring)),\
                   extract_jacobian(np.asarray(response.thumb))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return None

    def get_biotac_contact_readings(self):
        srv_name = "allegro/fingertips_in_contact"
        rospy.wait_for_service(srv_name)
        try:
            req = rospy.ServiceProxy(srv_name, FingertipsInContact)
            response = req()
            return response.index_in_contact,\
                    response.middle_in_contact,\
                    response.ring_in_contact,\
                    response.thumb_in_contact
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return None


def extract_jacobian(jacobian_raw):
    rows = 6
    cols = 4
    jacobian = np.zeros((rows,cols))
    for col_id in range(cols):
        jacobian[:,col_id] = jacobian_raw[col_id*6:col_id*6 + 6]
    return jacobian
    '''

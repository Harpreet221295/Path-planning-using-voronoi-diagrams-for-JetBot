#!/usr/bin/env python

import rospy
import cv2
import apriltag
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np
import time
import math
from math import sin, cos, radians
from itertools import chain
import scipy
from scipy import linalg
import pickle
from std_msgs.msg import Float32MultiArray, Bool
import os
#from scipy.spatial.transform import Rotation as R

robot_wheel_radius = 0.031
axle_length = 0.12

#pose_pub = rospy.Publisher('/current_pose', Pose, queue_size=2)

pose_pub = rospy.Publisher('/pose_robot', Float32MultiArray, queue_size=2)

num_frames = 0

detection_flag = True

def save_to_compute_R(pos_robo_wrt_origin_meas_marker, pos_marker_wrt_origin_meas_robot):
    global pos_robo_wrt_origin_meas_marker_list
    global pos_marker_wrt_origin_meas_robot_list
    global pos_robo_wrt_orgin_meas_marker_filename
    global pos_marker_wrt_origin_meas_robot_filename
    global num_frames
    global save_freq
    pos_robo_wrt_origin_meas_marker_list.append(pos_robo_wrt_origin_meas_marker)
    pos_marker_wrt_origin_meas_robot_list.append(pos_marker_wrt_origin_meas_robot)
    
    if num_frames % save_freq == 0:
        pos_robo_wrt_origin_meas_marker_list_np = np.stack(pos_robo_wrt_origin_meas_marker_list, axis = 0)
        np.save(open(pos_robo_wrt_orgin_meas_marker_filename, "wb" ), pos_robo_wrt_origin_meas_marker_list_np)
        #pickle.dump(pos_robo_wrt_origin_meas_marker_list, open(pos_robo_wrt_orgin_meas_marker_filename, "wb" ))
        
        pos_marker_wrt_origin_meas_robot_list_np = np.stack(pos_marker_wrt_origin_meas_robot_list, axis = 0)
        np.save(open(pos_marker_wrt_origin_meas_robot_filename, "wb" ), pos_marker_wrt_origin_meas_robot_list_np)
        #pickle.dump(pos_marker_wrt_origin_meas_robot_list, open(pos_marker_wrt_origin_meas_robot_filename, "wb" ))
        
        print("Frames Saved...Total : {0}".format(num_frames))

def rotationMatrixToEulerAngles(R) :
    #assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x*57.2598, y*57.2598, z*57.2598])
    #return np.array([x, y, z])




class Marker4(object):
    num_markers = 0
    def __init__(self, id, state_pos):
        self.__class__.num_markers += 1
        self.id = id
        self.cnt = self.__class__.num_markers
        self.state_pos = state_pos
        self.matrix_m_to_w = self.determine_matrix_m_to_w()
       
    def __getstate__(self):
        return self.__dict__
    
    def __setstate__(self, d):
        self.__dict__ = d
    
    def determine_matrix_m_to_w(self):
        x = self.state_pos[0]
        z = self.state_pos[2]
        
        if abs(x) > abs(z):
            if x > 0:
                return np.array([[0,0,-1], [0,1,0], [1,0,0]])
            else:
                return np.array([[0,0,1], [0,1,0], [-1,0,0]])
        else:
            if z < 0:
                return np.array([[1,0,0], [0,1,0], [0,0,1]])
            else:
                return np.array([[-1,0,0], [0,1,0], [0,0,-1]])
                
        return None
    
    
markers4_dict = {
    5 : Marker4(5, np.array([1,0,-0.5])),
    6 : Marker4(6, np.array([1,0,0])),
    7 : Marker4(7, np.array([1,0,0.5])),
    2 : Marker4(2, np.array([0.5,0,1])),
    4 : Marker4(4, np.array([0,0,1])),
    1 : Marker4(1, np.array([-0.5,0,1])),
    8 : Marker4(8, np.array([-1,0,0.5])),
    9 : Marker4(9, np.array([-1,0,0])),
    3 : Marker4(3, np.array([-1,0,-0.5])),
    12 : Marker4(12, np.array([-0.5,0,-1])),
    11 : Marker4(11, np.array([0,0,-1])),
    10 : Marker4(10, np.array([0.5,0,-1]))
}    
    
class Marker(object):
    num_markers = 0
    def __init__(self, id, state_pos):
        self.__class__.num_markers += 1
        self.id = id
        self.cnt = self.num_markers
        self.state_pos = state_pos
        self.R = np.array([[0.0016, 0.000018], [0.000027, 0.0016]])  # estimated from experimental data   
        self.cov = self.determine_cov()
        self.matrix_m_to_w = self.determine_matrix_m_to_w()
       
    def __getstate__(self):
        return self.__dict__
    
    def __setstate__(self, d):
        self.__dict__ = d
    
    def determine_cov(self):
        return np.array([[0.0064,    0.000189], [0.00038, 0.0076]])

    
    def determine_matrix_m_to_w(self):
        x = self.state_pos[0]
        z = self.state_pos[2]
        
        if abs(x) > abs(z):
            if x > 0:
                return np.array([[0,0,-1], [0,1,0], [1,0,0]])
            else:
                return np.array([[0,0,1], [0,1,0], [-1,0,0]])
        else:
            if z < 0:
                return np.array([[1,0,0], [0,1,0], [0,0,1]])
            else:
                return np.array([[-1,0,0], [0,1,0], [0,0,-1]])
                
        return None


class Robot:
    def __init__(self, state_pos):
        self.state_pos = state_pos
        self.Q = np.array([[0.0002, 0, 0], [0, 0.0004, 0], [0, 0, 9]])
        self.R = np.array([[0.0009, 0, 0], [0, 0.0025, 0], [0, 0, 0.25]])
        self.cov = 4 * self.R
        self.G = None



def redfine_coords_of_cam_wrt_marker(pos):
    z = pos[2]
    x = pos[1]
    
    return np.array([x,0,z])

def find_angle_wrt_x(marker_coord, euler_angle):
    x,z = marker_coord
    if(abs(x) > abs(z)):
        if(x > 0):
            if euler_angle < 0:
                return abs(euler_angle)
            else:
                return -1 * euler_angle
        else:
            if euler_angle < 0:
                return -1 * (180 - abs(euler_angle))
            else:
                return 180 - euler_angle
    else:
        if(z > 0):
            if euler_angle < 0:
                return abs(euler_angle) + 90
            else:
                return 90 - euler_angle
        else:
            if euler_angle < 0:
                return -1 * (90 - abs(euler_angle))
            else:
                return -1 * (90 + abs(euler_angle))

def get_matrix(pos):
    return np.array([[0,0,-1], [0,1,0], [1,0,0]])

def localize_marker_using_robot(marker_pos_wrt_robot):
    global robot
    theta = robot.state_pos[2]
    xr = robot.state_pos[0]
    zr = robot.state_pos[1]
    x = marker_pos_wrt_robot[0]
    z = marker_pos_wrt_robot[2]
    
    return np.array([xr + z * cos(radians(theta)) + x * sin(radians(theta)), 0, zr + z*sin(radians(theta))-x*cos(radians(theta))])

def localize_robot_using_marker(rotation_matrix, translation_matrix, marker_obj):
    pose_c_t2 = np.matmul(rotation_matrix.T, translation_matrix)
    mat_tag_t = marker_obj.state_pos
    transformed_coords = redfine_coords_of_cam_wrt_marker(pose_c_t2)
    marker_to_w_matrix = marker_obj.matrix_m_to_w
    
    r = rotationMatrixToEulerAngles(rotation_matrix)
    theta = find_angle_wrt_x((marker_obj.state_pos[0], marker_obj.state_pos[2]), r[0])
    
    value =  np.matmul(marker_to_w_matrix, transformed_coords) + mat_tag_t
    print("robo pose: {0}".format([value[0], value[2], theta]))
    return np.array([value[0], value[2], theta])
    
def compute_mahalanobis_distance(tag_pos, marker_state_pos, marker_cov):
    tag_pos = np.array([tag_pos[0], tag_pos[2]])
    marker_state_pos = np.array([marker_state_pos[0], marker_state_pos[2]])
    dist = tag_pos - marker_state_pos
    inv_cov = linalg.inv(marker_cov)
    return np.dot(np.dot(dist, inv_cov), dist.T)
    
def is_marker_present(pos):
    global detected_markers_dict
    marker_pos_wrt_w = localize_marker_using_robot(pos)
    smallest_marker_mahalanobis_dist = np.inf
    smallest_marker_mahalanobis_id = -1
    for key, marker_obj in detected_markers_dict.items():
        dist = compute_mahalanobis_distance(marker_pos_wrt_w, marker_obj.state_pos, marker_obj.cov)
        print("mahalanobis dist: {0}".format(dist))
        if dist < smallest_marker_mahalanobis_dist:
            smallest_marker_mahalanobis_dist = dist
            smallest_marker_mahalanobis_id = key
    
    if smallest_marker_mahalanobis_id == -1:
        return -1
    if smallest_marker_mahalanobis_dist > 6.634:
        return -1
    
    return smallest_marker_mahalanobis_id
    
def initialize_new_marker(marker_pos_wrt_robot, id):
    global detected_markers_dict
    marker_state_pos = localize_marker_using_robot(marker_pos_wrt_robot)
    new_marker = Marker(id, marker_state_pos)
    detected_markers_dict[Marker.num_markers] = new_marker
    print("New marker intialized")
    print("Number of detected markers: {0}".format(len(detected_markers_dict)))
    return new_marker


def apply_kalman_filter_to_robot(detected_marker_obj, translation_matrix, rotation_matrix):
    global robot
    #prediction done in different function
    # KF update is done in this function
    
    print("Applying KF to Robot")
    print("current robot pos: {0}".format(robot.state_pos))
    print("current robot cov: {0}".format(robot.cov))
    
    measurement_robot_pos = localize_robot_using_marker(rotation_matrix, translation_matrix, detected_marker_obj)
    I = np.array([[1,0,0], [0,1,0], [0,0,1]])
    H = I
    S = np.matmul(H, np.matmul(robot.cov, H.T)) + robot.R
    K = np.matmul(np.matmul(robot.cov, H.T), linalg.inv(S))
    
    robot.state_pos = robot.state_pos + np.matmul(K, (measurement_robot_pos - np.matmul(H, robot.state_pos)))
    robot.cov = np.matmul((I - np.matmul(K, H)), robot.cov)
    
    print("updated robot pos: {0}".format(robot.state_pos))
    print("updated robot cov: {0}".format(robot.cov))
    
    
def apply_kalman_filter_to_marker(marker_pos_wrt_robot, dict_id):
    global robot
    global detected_markers_dict
    
    measurement= localize_marker_using_robot(marker_pos_wrt_robot)
    measurement = np.array(measurement[0], measurement[2])
    marker_obj = detected_markers_dict[dict_id]
    
    print("Applying KF to marker")
    print("current marker pos: {0}".format(marker_obj.state_pos))
    print("current marker cov: {0}".format(marker_obj.cov))
    
    
    I = np.array([[1,0], [0,1]])
    H = I
    
    S = np.matmul(np.matmul(H, marker_obj.cov), H.T) + marker_obj.R
    
    K = np.matmul(np.matmul(marker_obj.cov, H.T), S)
    
    pos = np.array([marker_obj.state_pos[0], marker_obj.state_pos[1]])
    
    pos = pos + np.matmul(K, (measurement - np.matmul(H, pos)))
    
    marker_obj.state_pos = np.array([pos[0], 0, pos[1]])
    
    detected_markers_dict[dict_id] = marker_obj
    
    print("updated marker pos: {0}".format(marker_obj.state_pos))
    print("updated marker cov: {0}".format(marker_obj.cov))
    
    pickle.dump(detected_markers_dict, open("detected_markers_dict.pkl", "wb"))
    
def process_detected_tag2(detection, id):
    global num_frames
    global robot
    num_frames += 1
        
    R11, R12, R13, t1,R21, R22, R23, t2, R31, R32, R33, t3 = detection[:12]
        
    rotation_matrix = np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
    translation_matrix = np.array([t1, t2, t3+0.07])
        
    print("tag id: {0}".format(id))
    print(translation_matrix)
        
    r = rotationMatrixToEulerAngles(rotation_matrix)
        
    print("Frame Number: {0}".format(num_frames))
        
    #robot.state_pos = np.array([1,0.5,8.31])
    pos_marker_wrt_origin_meas_robot = localize_marker_using_robot(translation_matrix)
    detected_marker_obj_id = is_marker_present(translation_matrix)
    detected_marker_obj = None
    if detected_marker_obj_id != -1:
        detected_marker_obj = detected_markers_dict[detected_marker_obj_id]
    
    if detected_marker_obj is not None:
        print("Detected marker object pos: {0}".format(detected_marker_obj.state_pos))
        
        apply_kalman_filter_to_robot(detected_marker_obj, translation_matrix, rotation_matrix)
        apply_kalman_filter_to_marker(translation_matrix, detected_marker_obj_id)
    else:
        print("Not found.....so initializing")
        initialize_new_marker(translation_matrix, id)
        #pickle.dump(detected_markers_dict, open("detected_markers_dict.pkl", "wb"))

        
def process_detected_tag3(detection, id):
    global num_frames
    global robot
    num_frames += 1
        
    R11, R12, R13, t1,R21, R22, R23, t2, R31, R32, R33, t3 = detection[:12]
        
    rotation_matrix = np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
    translation_matrix = np.array([t1, t2, t3+0.07])
        
    print("tag id: {0}".format(id))
    print(translation_matrix)
        
    marker_obj = markers4_dict[id]    
    
    return localize_robot_using_marker(rotation_matrix, translation_matrix, marker_obj)
           
def robo_motion_callback(msg):
    global robot
    global robot_wheel_radius
    global axle_length
    
    print(msg)
    
    vl = msg.data[0]
    vr = msg.data[1]
    delta_t = msg.data[2]
    
    #Kalman Filter Prediction First Step for Robot position
    R = robot_wheel_radius
    L = axle_length
    vel_and_ang_conv = np.array([[R/2, R/2], [R/L, -R/L]])
    theta = radians(robot.state_pos[2])
    world_coord_matrix = np.array([[cos(theta), 0, 0],[0, sin(theta), 0],[0, 0, 1]])
    change_dim_matrix = np.array([[1, 0], [1, 0], [0, 1]])
                                   
    G = np.matmul(np.matmul(world_coord_matrix, change_dim_matrix), vel_and_ang_conv)
    
    robot.state_pos = robot.state_pos + np.matmul(G, delta_t * np.array([vr, vl]).T)
    
    print("updated robo position: {0}".format(robot.state_pos))
    robot.cov = robot.cov + robot.Q
    
            

def tag_callback3(msg):
    pose_msg = Pose()
    pose_msg.header.stamp = msg.header.stamp
    
    if(len(msg.ids) > 0) and detection_flag:
        num_detected_tags = len(msg.ids)
        
        pose_matrices = []
        
        for detection, id in zip(msg.detections, msg.ids):
            pose_matrices.append(process_detected_tag3(detection.matrix, id))
        
        if len(pose_matrices) > 1:
            robot_pose = np.mean(pose_matrices, axis = 1)
        else:
            robot_pose = pose_matrices[0]
        print("updated: {0}".format(robot_pose))
        robot_pose_msg = Float32MultiArray()
        robot_pose_msg.data = robot_pose.tolist()
        pose_pub.publish(robot_pose_msg)
                
def detection_toggle_callback(msg):
    global detection_flag
    detection_flag = msg
    
    print("Detection flag: {0}".format(detection_flag))

        

if __name__ == "__main__":
    rospy.init_node('localization_node')
    rospy.Subscriber("/tag_poses", AprilDetections, tag_callback3)
    rospy.Subscriber("/toggle_detection", Bool, detection_toggle_callback)
    rospy.spin()

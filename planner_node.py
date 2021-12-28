#!/usr/bin/env python

import rospy
import cv2
import apriltag
from std_msgs.msg import Float32MultiArray, Bool
from navigation_dev.msg import AprilDetections
from navigation_dev.msg import Pose 
import numpy as np
import time
import pickle
import networkx as nx
import math
from numpy import arccos, array
from numpy.linalg import norm

ctrl_pub = rospy.Publisher('/ctrl_cmd', Float32MultiArray, queue_size=1)
#vel_pub = rospy.Publisher('/vel', Float32MultiArray, queue_size=2)
toggle_detection_pub = rospy.Publisher('/toggle_detection', Bool, queue_size=1)

class Robot:
    def __init__(self, state_pos):
        self.pose = np.array(state_pos)
          
    def update_pose(self, new_pose):
        self.pose = np.array(new_pose)
        

        
robot = Robot([0,0,0])

#movements_in_circle = None

def turn_90_clockwise():
    move = 0
    stop = 1
    
    # First leg
    
    speed_l = -0.85
    speed_r = 0.9
    
    start_detect_msg = False
    toggle_detection_pub.publish(start_detect_msg)
        
    for i in range(1):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        #time.sleep(0.2)
            
    start_detect_msg = True
    toggle_detection_pub.publish(start_detect_msg)
    vel_msg = Float32MultiArray()
    dist = 0
    angle = 90
    vel_msg.data = [dist/0.6, angle/0.6,0.6]
    vel_pub.publish(vel_msg)
    time.sleep(3)
    
    
def calibrate_with_vel():
    move = 0
    stop = 1
    
    speed_l = -0.78
    speed_r = -0.9
    
    start_detect_msg = False
    toggle_detection_pub.publish(start_detect_msg)
        
        
    for i in range(3):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.2)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        #time.sleep(0.2)
            
    start_detect_msg = True
    toggle_detection_pub.publish(start_detect_msg)
    vel_msg = Float32MultiArray()
    dist = 0
    angle = 0
    vel_msg.data = [dist/0.6, angle/0.6,0.6]
    vel_pub.publish(vel_msg)

def correct_orientation(current_orientation, target_orientation):
    angular_diff = target_orientation - current_orientation
    pass
    
def set_robot_pose_callback(msg):
    global robot
    robot.update_pose(msg.data)
    print("robot's pose updated")

def theta(v, w): 
    return arccos(v.dot(w)/(norm(v)*norm(w)))

def cover_waypoints(waypoints):
    global robot
    num_waypoint = len(waypoints)
    
    print("robot: {0}".format(robot.pose))
    for waypt in waypoints:
        movement_vector = np.array(waypt) - robot.pose[:2]
        movement_vector = movement_vector.astype('float')
        target_distance = np.linalg.norm(movement_vector)
        x_axis_vector = np.array([1,0]).astype('float')
        target_orientation = math.degrees(theta(movement_vector, x_axis_vector))
        
        if np.cross(x_axis_vector,movement_vector)/np.linalg.norm(movement_vector)  < 0:
            target_orientation = -1 * target_orientation
        
        angle = target_orientation - robot.pose[2]
        angle = get_shorter_angle(angle)
        angle = round(angle, 2)
        print("{0} -> {1}  dist: {2}    angle: {3}".format(robot.pose[:2].round(2), waypt, np.linalg.norm(movement_vector).round(2), round(angle,2)))
        
        dist = np.linalg.norm(movement_vector).round(2)
        rotate_robot(angle)
        translate_robot(dist)
        
        robot.update_pose([waypt[0],waypt[1], robot.pose[2]+angle])
        
        
        

def translate():
    move = 0
    stop = 1
    # + 30 - 40 degrees
    speed_l = -0.95
    speed_r = -0.95
    
    for i in range(2):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        #msg = Float32MultiArray()
        #msg.data = [move, speed_l, speed_r]
        #ctrl_pub.publish(msg)
        #time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(2)
        
def rotate():
    move = 0
    stop = 1
    # + 30 - 40 degrees
    speed_l = 0.75
    speed_r = -0.75
    
    for i in range(1):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        msg = Float32MultiArray()
        #msg.data = [move, speed_l, speed_r]
        #ctrl_pub.publish(msg)
        #time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.5)
    '''
    for i in range(1):
        msg = Float32MultiArray()
        msg.data = [move, speed_l, speed_r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        msg = Float32MultiArray()
        msg.data = [stop, speed_l, speed_r]
        ctrl_pub.publish(msg)
        #time.sleep(0.2)
    '''        
    #start_detect_msg = True
    #toggle_detection_pub.publish(start_detect_msg)

def move_robot_util(l,r,iters):
    #Utility function to move a robot using given left and right motor speeds for 0.3 seconds and then give 0.6 seconds localization window
    move = 0
    stop = 1
    for i in range(iters):
        start_detect_msg = False
        toggle_detection_pub.publish(start_detect_msg)
        msg = Float32MultiArray()
        msg.data = [move, l, r]
        ctrl_pub.publish(msg)
        time.sleep(0.3)
        #msg = Float32MultiArray()
        #msg.data = [move, speed_l, speed_r]
        #ctrl_pub.publish(msg)
        #time.sleep(0.3)
        
        msg = Float32MultiArray()
        msg.data = [stop, l, r]
        start_detect_msg = True
        toggle_detection_pub.publish(start_detect_msg)
        ctrl_pub.publish(msg)
        time.sleep(0.6)
    
    
def get_shorter_angle(angle):
    if abs(angle) > 180:
        if angle > 0:
            angle = 360 - angle
            angle = -1 * angle
        else:
            angle = 360 - abs(angle)
            #angle = -1 * angle
    return angle

def rotate_robot(angle):
    
    angle = get_shorter_angle(angle)
    
    if angle > 0:
        #current = angle
        if angle > 10 and angle < 30:
            move_robot_util(0.65, -0.65, 1)
        if angle > 30 and angle < 40:
            move_robot_util(0.75, -0.75, 1)
        if angle > 40 and angle<= 50:
            move_robot_util(0.75, -0.75, 1)
        if angle > 60 and angle < 70:
            move_robot_util(0.75, -0.75, 2)
    
    else:
        angle = abs(angle)
        if angle > 10 and angle < 30:
            move_robot_util(-0.72, 0.72, 1)
        if angle > 30 and angle < 40:
            move_robot_util(-0.75, 0.75, 1)
        if angle > 40 and angle<= 50:
            move_robot_util(-0.75, 0.75, 1)
        if angle > 60 and angle < 70:
            move_robot_util(0.73, -0.73, 2)
        if angle > 70:
            iters = angle / 30
            move_robot_util(-0.73, 0.73, iters)

            

def translate_robot(dist):
    
    if dist < 0.27:
        move_robot_util(-0.95, -0.95, 2)
    else:
        move_robot_util(-0.95, -0.95, 2)
        move_robot_util(-0.75, -0.75, 1)
            
def compute_eucledian_distance(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
    
if __name__ == "__main__":
    rospy.init_node('planner_node')
    rospy.Subscriber("/pose_robot", Float32MultiArray, set_robot_pose_callback)
    
    '''
        Generated from spatial_voronoi.ipynb 
    '''
    Nodes_dict = {1: (0.53, -0.53),
 2: (0.64, -0.25),
 3: (0.58, 0),
 4: (0.64, 0.25),
 5: (-0.25, 0.64),
 6: (0.25, 0.64),
 7: (0, 0.58),
 8: (0.53, 0.53),
 9: (-0.64, -0.25),
 10: (-0.64, 0.25),
 11: (-0.58, 0),
 12: (-0.53, 0.53),
 13: (-0.53, -0.53),
 14: (0, -0.58),
 15: (-0.25, -0.64),
 16: (0.25, -0.64),
 17: (1, -0.25),
 18: (1, 0.25),
 19: (1, 1),
 20: (-1, 1),
 21: (-0.25, 1),
 22: (0.25, 1),
 23: (-1, -1),
 24: (-1, -0.25),
 25: (-1, 0.25),
 26: (-0.25, -1),
 27: (0.25, -1),
 28: (1, -1)}
    Edges_dict = {1: (1, 2),
 2: (2, 3),
 3: (3, 4),
 4: (4, 8),
 5: (5, 12),
 6: (6, 7),
 7: (7, 5),
 8: (8, 6),
 9: (9, 13),
 10: (10, 11),
 11: (11, 9),
 12: (12, 10),
 13: (13, 15),
 14: (14, 16),
 15: (15, 14),
 16: (16, 1),
 17: (17, 2),
 18: (18, 4),
 19: (19, 8),
 20: (20, 12),
 21: (21, 5),
 22: (22, 6),
 23: (23, 13),
 24: (24, 9),
 25: (25, 10),
 26: (26, 15),
 27: (27, 16),
 28: (28, 1)}
    
    G = nx.Graph()
    
    for key in Edges_dict.keys():
        G.add_edge(Edges_dict[key][0], Edges_dict[key][1], weight = compute_eucledian_distance(Nodes_dict[Edges_dict[key][0]], Nodes_dict[Edges_dict[key][1]]))
     
    start_node = 1
    inter_node = 7
    end_node = 11
    
    path = nx.dijkstra_path(G, 1, 7)
    waypoints = [Nodes_dict[pt] for pt in path]
    
    
    print("First path: {0}".format(path))
    robot.update_pose([waypoints[0][0],waypoints[0][1], 0])
    cover_waypoints(waypoints[1:])
    
    
    
    print("robo pose: {0}".format(robot.pose))
    
    time.sleep(5)
    
    path = nx.dijkstra_path(G, 7 , 11)
    waypoints = [Nodes_dict[pt] for pt in path]
    print("Second path: {0}".format(path))
    print("Corresponding waypoints: {0}".format(waypoints))
    cover_waypoints(waypoints[1:])
    
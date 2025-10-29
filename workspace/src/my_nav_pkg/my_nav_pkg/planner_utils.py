#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Time
from my_nav_interfaces.srv import PlanPath

import random
import copy
import heapq
import math

def get_distance_from_positions(pose1: PoseStamped, pose2: PoseStamped):
    return math.sqrt((pose1.pose.position.x-pose2.pose.position.x)**2 + (pose1.pose.position.y-pose2.pose.position.y)**2)

def create_path_zigzag(start:PoseStamped, goal:PoseStamped):
    path = Path()
    # path.header.frame_id = "odom"
    # path.header.stamp = self.get_clock().now().to_msg()

    # Create a simple line path (x=0→4)
    for i in range(5):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(i)
        pose.pose.position.y = 1.0 * (i%2)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    
    if random.randint(0,1) % 2 == 0:
        path.poses.reverse()
        print("reverse")
    else:
        print("true")
    path.poses.append(goal)

    return path

def create_path_linear(start:PoseStamped, goal:PoseStamped, log):
    path = Path()
    # path.header.frame_id = "odom"
    # path.header.stamp = self.get_clock().now().to_msg()

    path.poses.append(copy.deepcopy(start))

    while get_distance_from_positions(path.poses[-1], goal) > 0.1:
        distance = get_distance_from_positions(path.poses[-1], goal)
        step_size = 0.2
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = path.poses[-1].pose.position.x + (goal.pose.position.x-path.poses[-1].pose.position.x) * min(distance, step_size) / distance
        pose.pose.position.y = path.poses[-1].pose.position.y + (goal.pose.position.y-path.poses[-1].pose.position.y) * min(distance, step_size) / distance
        pose.pose.position.x += (random.random() * 2 - 1.0) * 0.05
        pose.pose.position.y += (random.random() * 2 - 1.0) * 0.05
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

        log.info(f"{pose.pose.position=} {distance=}")


    path.poses.append(copy.deepcopy(goal))

    return path

def create_path_a(start:PoseStamped, goal:PoseStamped):

    # A*
    def GetCost(point1: PoseStamped):
        dist = math.sqrt((goal.pose.position.x-point1.pose.position.x)**2 + (goal.pose.position.y-point1.pose.position.y)**2)
        return dist

    # heapq returns value for minimum key 
    points = []
    heapq.heappush(points, (GetCost(start), start))

    visited_points = []
    
    path = Path()
    # path.header.frame_id = "odom"
    # path.header.stamp = self.get_clock().now().to_msg()

    step_size = 0.2


    # Create a simple line path (x=0→4)
    for i in range(5):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(i)
        pose.pose.position.y = 1.0 * (i%2)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    
    if random.randint(0,1) % 2 == 0:
        path.poses.reverse()
        print("reverse")
    else:
        print("true")
    path.poses.append(goal)

    return path

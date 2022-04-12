#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
import rrt
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        self.starting_pose = None
        self.goal_pose = None

        self.rrt = None 

    def init_pose(self, msg):
        self.pose = msg.pose.pose

    def map_cb(self, msg):
        self.map = msg

    def odom_cb(self, msg):
        self.curr_odom = msg.pose.pose

    def goal_cb(self, msg):
        """
        Callback for when a goal is received
        """
        start_point = self.convertToPixel(self.pose.position)
        end_point = self.convertToPixel(msg.pose.position)

        self.rrt = rrt.RRT(
                 start_point,
                 end_point,
                 None,
                 (msg.info.width, msg.info.height),
                 msg.data,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=5000000,
                 play_area = None)

        self.plan_path(start_point, end_point, self.map)

    def plan_path(self, start_point, end_point, map):
        path = self.rrt.planning()
        path.reverse()
        for point in path:
            self.trajectory.addPoint(self.convertToPoint(point))
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()
    
    def convertToPixel(self, point):
        """
        Converts a point in the map to a pixel in the image
        """
        return (int(point.x * self.map.info.resolution + self.map.info.origin.position.x),
                int(point.y * self.map.info.resolution + self.map.info.origin.position.y))

    def convertToPoint(self, pixel):
        """
        Converts a pixel in the map to a point in the map by rotating the pixel and dividing by resolution
        """
        return self.rotate_point(Point(pixel[0] / self.map.info.resolution - self.map.info.origin.position.x,
                                 pixel[1] / self.map.info.resolution - self.map.info.origin.position.y)), self.map.info.origin.orientation)

    def rotate_point(self, point, quat):
        """
        Rotates a point by quaternion
        """
        rotation_matrix=quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        rotated_point=np.dot(rotation_matrix, [point.x, point.y, 0, 1])
        return Point(rotated_point[0], rotated_point[1])

    def calc_headings(self, path):
        """
        Given a series of waypoints, calculates headings that lead to a smooth path
        """
        headings=[]
        for i in range(len(path) - 1):
            start_point=path[i]
            end_point=path[i + 1]
            heading=np.arctan2(
                end_point[1] - start_point[1], end_point[0] - start_point[0])
            headings.append(heading)
        return headings

    def is_valid_point(self, point):
        """
        Checks if the point is valid
        """
        if point[0] < 0 or point[0] >= self.map.info.width:
            return False
        if point[1] < 0 or point[1] >= self.map.info.height:
            return False
        if self.map.data[self.get_index(point)] == self.obstacle_threshold:
            return False
        return True

    def get_possible_moves(self, point):
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                new_point=(point[0] + i, point[1] + j)
                if self.is_valid_point(new_point):
                    yield new_point

    def get_index(self, point):
        """
        Returns the index of the point in the map
        """
        return int(point[0] + point[1] * self.map.info.width)

    def euclidean_distance(self, start_point, end_point, map):
        """
        Cost based off euclidean distance
        """
        return np.sqrt((start_point[0] - end_point[0])**2 + (start_point[1] - end_point[1])**2)

    def manhattan_distance(self, start_point, end_point):
        """
        Current heuristic uses manhattan distance
        """
        return abs(start_point[0] - end_point[0]) + abs(start_point[1] - end_point[1])



if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

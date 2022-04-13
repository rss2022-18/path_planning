#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import *
import rospkg
import time
import os
from utils import LineTrajectory
from a_star import AStarPlanner
from Queue import PriorityQueue


class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        self.obstacle_threshold = 100
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher(
            "/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_cb)
        self.pose_sub = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.init_pose)
        self.pose = None
        self.map = None

    def init_pose(self, msg):
        self.pose = msg.pose.pose

    def map_cb(self, msg):
        if self.map is None:
            print("Width", msg.info.width)
            print("Height", msg.info.height)
            print("Resolution", msg.info.resolution)
            self.planner = AStarPlanner(
                msg.data, msg.info.width, msg.info.height)
            with open('./dilated_map.npy', 'wb') as f:
                np.save(f, msg.data)
        self.map = msg

    def odom_cb(self, msg):
        self.curr_odom = msg.pose.pose

    def goal_cb(self, msg):
        """
        Callback for when a goal is received
        """
        start_point = self.convertToPixel(self.pose.position)
        end_point = self.convertToPixel(msg.pose.position)
        self.plan_path(start_point, end_point, self.map)

    def plan_path(self, start_point, end_point, map):
        """
        A* path planning algorithm using manhattan distance as heuristic
        """

        path = self.planner.find_path(start_point, end_point)
        # print(path)
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
        point = self.rotate_point(point, self.map.info.origin.orientation, -1)
        return (int(point.x * self.map.info.resolution + self.map.info.origin.position.x),
                int(point.y * self.map.info.resolution + self.map.info.origin.position.y))

    def convertToPoint(self, pixel):
        """
        Converts a pixel in the map to a point in the map by rotating the pixel and dividing by resolution
        """
        return self.rotate_point(Point(pixel[0] / self.map.info.resolution - self.map.info.origin.position.x,
                                 pixel[1] / self.map.info.resolution - self.map.info.origin.position.y, 0), self.map.info.origin.orientation)

    def rotate_point(self, point, quat, dir=1):
        """
        Rotates a point by quaternion
        """
        rotation_matrix = quaternion_matrix(
            [quat.x, quat.y, quat.z, dir*quat.w])
        rotated_point = np.dot(rotation_matrix, [point.x, point.y, point.z, 1])
        return Point(rotated_point[0], rotated_point[1], rotated_point[2])

    def calc_headings(self, path):
        """
        Given a series of waypoints, calculates headings that lead to a smooth path
        """
        headings = []
        for i in range(len(path) - 1):
            start_point = path[i]
            end_point = path[i + 1]
            heading = np.arctan2(
                end_point[1] - start_point[1], end_point[0] - start_point[0])
            headings.append(heading)
        return headings


if __name__ == "__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

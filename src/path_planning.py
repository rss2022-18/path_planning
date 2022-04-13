#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import *
import rospkg
from scipy.spatial.transform import Rotation as R
from utils import LineTrajectory
from planners import *
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
            grid = np.reshape(np.array(msg.data),
                              (msg.info.height, msg.info.width))
            # self.planner = RRTPlanner(
            #     grid, msg.info.width, msg.info.height, 10, path_resolution=10)
            self.planner = AStarPlanner(
                grid, msg.info.width, msg.info.height, 10)
        self.map = msg
        quat = self.map.info.origin.orientation
        transform = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        pos_vec = np.array(
            [[self.map.info.origin.position.x, self.map.info.origin.position.y, self.map.info.origin.position.z]]).T
        rotation_matrix = transform.as_dcm()
        self.transformation_matrix = np.vstack(
            [np.hstack([rotation_matrix, pos_vec]), np.array([0, 0, 0, 1])])

    def odom_cb(self, msg):
        self.curr_odom = msg.pose.pose

    def goal_cb(self, msg):
        """
        Callback for when a goal is received
        """
        start_point = self.convertToPixel(self.pose.position)
        end_point = self.convertToPixel(msg.pose.position)
        print("Start Point", start_point)
        print("End Point", end_point)
        self.plan_path(start_point, end_point, self.map)

    def plan_path(self, start_point, end_point, map):
        """
        A* path planning algorithm using manhattan distance as heuristic
        """
        self.trajectory.clear()
        print("Searching for path")
        path = self.planner.find_path(start_point, end_point)
        if path is not None:
            print("Found path!")
        else:
            print("Did not find path :(")
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
        point = np.array([point.x, point.y, 0, 1])
        point = np.dot(np.linalg.inv(self.transformation_matrix), point)
        return (int(point[0]/self.map.info.resolution), int(point[1]/self.map.info.resolution))

    def convertToPoint(self, pixel):
        """
        Converts a pixel in the map to a point in the map by rotating the pixel and dividing by resolution
        """
        point = np.array([pixel[0]*self.map.info.resolution,
                         (pixel[1])*self.map.info.resolution, 0, 1])
        point = np.dot(self.transformation_matrix, point)
        return Point(point[0], point[1], 0)

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

#!/usr/bin/env python

from gettext import translation
import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import Point, PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 2.0
        self.speed            = 1.0# FILL IN #
        #self.wrap             = 5# 5FILL IN # UNECCESSARY
        self.wheelbase_length = 0.35# FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

        #Subscribe to my localizer to get my x,y, position
        self.localize_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.odometry_cb, queue_size=10)
        rospy.loginfo('hi')

        self.state = [0,0,0] # originally set to origin. fix on first cb

        self.visualize_pub = rospy.Publisher("/goal_point", Marker, queue_size = 1)
        self.visualize_goal =VisualizeGoal()

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        print(self.trajectory.points)


    def follow_which_point(self, waypoints):
        #find the distances to all waypoints:
        # find waypoint closest to my car. 
        d_waypoints = np.linalg.norm(waypoints - np.tile(self.state[0:2], (len(self.trajectory.distances),1) ))
        idx = np.argmin(d_waypoints)
        closest_waypoint = waypoints[idx]

        dist_to_waypoint = self.trajectory.distance_along_trajectory(idx)
        Q = self.state[0:2] # only first two indeces

        print("State" + str(Q))
        r = self.lookahead

        goal_point = []
        intersect_pts = []

        # find a goal waypoint(along line)

        # look from the closest waypoint to the end of the trajectory.
        for i in range(idx, len(waypoints) -1):
            P1 = waypoints[i]
            V  = waypoints[i+1]

            a = np.dot(V,V)
            b = 2*np.dot(V, P1-Q)
            c = np.dot(P1, P1) + np.dot(Q,Q) - 2*np.dot(P1,Q) - r**2

            disc = b**2 - 4*a*c

            print("Discriminant Value " + str(disc))

            if disc<0:
                continue

            sqrt_disc = np.sqrt(disc)
            t1 = (-b + sqrt_disc) / (2.0 * a)
            t2 = (-b - sqrt_disc) / (2.0 * a)

            if t1<1 and t1>0:
                intersect_pts.append(P1 + t1*(V-P1))

            elif t2<1 and t2>0:
                intersect_pts.append(P1 + t2*(V-P1))

            # if not(0 <= t1 <= 1 or 0<=t2<=1):
            #     continue
            # else:
            #     intersect_pts.append(P1 + t1*(V-P1))
            #     intersect_pts.append(P1 + t2*(V-P1))

        
        if len(intersect_pts) == 0:
            return []
        
        else:
            return intersect_pts[-1] # return the most recently added one


        print('wtf point')

    def controller_cb(self, goal_pt):
        # Goal point is given in global coordinates.
        # Need the goal in relative coordinates. 
        angle_to_goal= np.arctan2(goal_pt[1], goal_pt[0])
        L1 = np.linalg.norm(goal_pt[0:1])
        numerator = 2*L1*np.sin(angle_to_goal).real
        denominator = self.lookahead

        delta = (np.arctan2(numerator, denominator)).real

        #TODO: Add more heuristics here


        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = delta
        drive_cmd.drive.speed = self.speed

        self.drive_pub.publish(drive_cmd)

        
        print("hiiiiii how are you")

    def odometry_cb(self, odometry_data):
        my_x = odometry_data.pose.pose.position.x
        my_y = odometry_data.pose.pose.position.y
        my_th= self.quat_to_yaw(odometry_data.pose.pose.orientation)

        rotation_matrix = tf.transformations.quaternion_matrix((odometry_data.pose.pose.orientation.x, odometry_data.pose.pose.orientation.y, odometry_data.pose.pose.orientation.z, odometry_data.pose.pose.orientation.w))
        translation_matrix = tf.transformations.translation_matrix((odometry_data.pose.pose.position.x, odometry_data.pose.pose.position.y, odometry_data.pose.pose.position.z))

        transform_matrix = np.linalg.inv(np.matmul(translation_matrix, rotation_matrix))



        self.state = np.array([my_x, my_y, my_th])

        waypoints = np.array(self.trajectory.points)

        if len(waypoints) < 3: #Three points is not enough for a trajectory!
            print("No Trajectory!")
            return


        goal_pt = self.follow_which_point(waypoints)

        if len(goal_pt) ==0:
            print("empty!")
            return
        else:
            print("Follow this point!")
            print(goal_pt)

            self.visualize_goal.draw(goal_pt)
            self.visualize_pub.publish(self.visualize_goal.line)


        relative_goal = np.matmul(transform_matrix, np.transpose( np.array([goal_pt[0], goal_pt[1],0,1]) )  )

        self.controller_cb(relative_goal) # goal pt is given in global coordiantes




        #print(self.state)
        print(str([my_x, my_y, my_th]))

        

    def quat_to_yaw(self, quaternion):
        
        my_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        rpy = euler_from_quaternion(my_quat)
        roll = rpy[0]
        pitch= rpy[1]
        yaw  = rpy[2]

        return yaw


class VisualizeGoal:
    def __init__(self):

        # Set parameters for the published message. 
        self.line = Marker()
        self.line.type = Marker.POINTS
        self.line.header.frame_id = "/map"
        self.line.scale.x = 3
        self.line.scale.y = 3
        self.line.color.a = 1.
        self.line.color.r = 1
        self.line.color.g = 0
        self.line.color.b = 1

    def draw(self, goal):
        self.line.points = []

        x = goal[0]
        y = goal[1]

        point = Point()
        point.x = x
        point.y = y
        self.line.points.append(point)

    


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()

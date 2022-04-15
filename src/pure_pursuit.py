#!/usr/bin/env python

from gettext import translation
#import queue
#import queue

from matplotlib.pyplot import close
import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import Point, PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from lab6.msg import error_msg

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 2.5
        self.speed          = [0.0,2.5,5.0]#2.5 #1.0# FILL IN #
        #self.wrap             = 5# 5FILL IN # UNECCESSARY
        self.wheelbase_length = 0.35# FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)

        self.wall_follow_ang = rospy.Subscriber("/wall_follow_ang", Float32,self.wall_follow_cb, queue_size=1)
        self.wall_ang_cmd = 0.0
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)


        self.error_pub = rospy.Publisher("/waypoint_error", error_msg,queue_size=10)
        
        self.waypoint_dist= rospy.Publisher("/nearest_waypoint", Float32, queue_size=10)
        self.distance_to_line = rospy.Publisher("/distance_to_path", Float32, queue_size=10)
        #Subscribe to my localizer to get my x,y, position
        self.localize_sub = rospy.Subscriber("/pf/pose/odom", Odometry, self.odometry_cb, queue_size=10)
        

        self.state = [0,0,0] # originally set to origin. fix on first cb

        self.visualize_pub = rospy.Publisher("/goal_point", Marker, queue_size = 1)
        self.visualize_goal =VisualizeGoal()

        self.visualize_waypoint_pub = rospy.Publisher("/follower_waypoint", Marker, queue_size = 1)
        self.visualize_waypoint = VisualizeGoal()

        self.transform_matrix = []

        self.distance_to_closest = 0.0
        self.gear = 0

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        print(self.trajectory.points)


    def wall_follow_cb(self, ang_data):
        self.wall_ang_cmd = ang_data.data



    def determine_follow_point(self, waypoints):
        car_x = self.state[0]
        car_y = self.state[1]
        car_pos= np.array([car_x, car_y])

        d_waypoints = np.ones(len(waypoints-1))*9e9


        # find closest distances to all waypoints SEGMENTS
        for i in range(len(waypoints)-1):
            V = waypoints[i]
            W = waypoints[i+1]

            l2 = np.linalg.norm(V-W)**2
            t  = ( (car_pos[0]-V[0])*(W[0]-V[0]) + (car_pos[1]-V[1])*(W[1]-V[1]) )/l2
            t= np.max((0, np.min((1,t))))
            closest_point = np.array([V[0]+t*(W[0]-V[0]), V[1]+t*(W[1]-V[1])])
            d_waypoints[i] = np.linalg.norm(car_pos - closest_point)

        # find closest waypoint
        min_idx = np.argmin(d_waypoints)

        minimum_distance = d_waypoints[min_idx]
        self.distance_to_line.publish(minimum_distance)
        min_waypoint = waypoints[min_idx]
        min_waypoint_dist = self.trajectory.distance_along_trajectory(min_idx)

        if min_idx + 1 < len(waypoints):
            fwd_waypoint = waypoints[min_idx+1]
            print(fwd_waypoint)
            print([car_x, car_y])
            self.distance_to_closest = ((fwd_waypoint[0]-car_x)**2 + (fwd_waypoint[1]-car_y)**2)**0.5
            print(self.distance_to_closest)
            self.waypoint_dist.publish(self.distance_to_closest)

        

        self.visualize_waypoint.draw(min_waypoint)
        self.visualize_waypoint_pub.publish(self.visualize_waypoint.line)

        # Find the goal point
        intersections = []
        Q = [car_x, car_y]
        r = max(0.5, self.speed[self.gear]*0.5) #self.lookahead # max(0.5, self.speed*0.5) # a dynamic self lookahaed distance

        for i in range(min_idx, len(waypoints)-1):
            P1 = waypoints[i]
            V  = waypoints[i+1] - P1

            a = np.dot(V,V)
            b = 2*np.dot(V, P1-Q)
            c = np.dot(P1, P1) + np.dot(Q,Q) -2*np.dot(P1, Q) - r**2

            discriminant = b**2 - 4*a*c

            if discriminant < 0:
                continue

            sol1 = (-b + np.sqrt(discriminant)) / (2.0*a)
            sol2 = (-b - np.sqrt(discriminant)) / (2.0*a)

            if sol1<1 and sol1>0:
                intersections.append(P1+sol1*V)
            elif sol2<1 and sol2>0:
                intersections.append(P1 + sol2*V)


        if len(intersections) == 0:
            goal_pt = []
        else:
            goal_pt = intersections[-1] # this is the furthest along line waypoint

        return goal_pt


    def controller_cb(self, goal_pt):
        # Goal point is given in global coordinates.
        # Need the goal in relative coordinates. 
        angle_to_goal= np.arctan2(goal_pt[1], goal_pt[0])
        L1 = np.linalg.norm(goal_pt[0:1])
        numerator = 2*L1*np.sin(angle_to_goal).real
        denominator = self.lookahead

        delta = (np.arctan2(numerator, denominator)).real

        drive_cmd = AckermannDriveStamped()

        #TODO: Add more heuristics here

        # DRS Heuristic:
        # if self.distance_to_closest > 6.0:
        #     self.gear = 2
        #     drive_cmd.drive.speed = self.speed[self.gear] #5.0
        #     drive_cmd.drive.steering_angle = self.wall_ang_cmd
        # else:
        #     self.gear = 1
        #     drive_cmd.drive.speed = self.speed[self.gear]
        #     drive_cmd.drive.steering_angle = delta

        self.gear = 1
        drive_cmd.drive.speed = self.speed[self.gear] #5.0
        drive_cmd.drive.steering_angle = delta
        
        

        self.drive_pub.publish(drive_cmd)

        #publish the error
        err_msg = error_msg()
        err_msg.x_error = goal_pt[1]
        err_msg.y_error = goal_pt[0]
        err_msg.distance_error = (goal_pt[1]**2 + goal_pt[0]**2)**0.5

        self.error_pub.publish(err_msg)

    

        
        

    def odometry_cb(self, odometry_data):
        my_x = odometry_data.pose.pose.position.x
        my_y = odometry_data.pose.pose.position.y
        my_th= self.quat_to_yaw(odometry_data.pose.pose.orientation)

        rotation_matrix = tf.transformations.quaternion_matrix((odometry_data.pose.pose.orientation.x, odometry_data.pose.pose.orientation.y, 
                                                                odometry_data.pose.pose.orientation.z, odometry_data.pose.pose.orientation.w))
        translation_matrix = tf.transformations.translation_matrix((odometry_data.pose.pose.position.x, odometry_data.pose.pose.position.y, 
                                                                    odometry_data.pose.pose.position.z))

        self.transform_matrix = np.linalg.inv(np.matmul(translation_matrix, rotation_matrix))



        self.state = np.array([my_x, my_y, my_th])

        waypoints = np.array(self.trajectory.points)

        if len(waypoints) < 3: #Three points is not enough for a trajectory!
            print("No Trajectory!")
            return


        #goal_pt = self.follow_which_point(waypoints)
        goal_pt = self.determine_follow_point(waypoints)

        if len(goal_pt) ==0:
            #print("empty!")
            return
        else:
            #print("Follow this point!")
            #print(goal_pt)

            self.visualize_goal.draw(goal_pt)
            self.visualize_pub.publish(self.visualize_goal.line)


        relative_goal = np.matmul(self.transform_matrix, np.transpose( np.array([goal_pt[0], goal_pt[1],0,1]) )  )

        #print("Relative Goal" + str(relative_goal))

        self.controller_cb(relative_goal) # goal pt is given in global coordiantes




        #print(self.state)
        #print(str([my_x, my_y, my_th]))

        

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
        self.line.scale.x = 1
        self.line.scale.y = 1
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



       # def distance_to_waypoint(waypoints, car):
    #     print('Distance to waypoint!')

    # def distance_to_segment(self, p1, p2, car):
    #     x1, y1 = p1
    #     x2, y2 = p2
    #     xp = car[0]
    #     yp = car[1]

    #     dx = x2 - x1
    #     dy = y2 - y1

    #     norm = dx**2 + dy**2

    #     u = ((xp - x1)*dx + (yp - y1)*dy )/ float(norm)

    #     if u> 1:
    #         u = 1
    #     elif u<0:
    #         u = 0

    #     x = x1+ u*dx
    #     y = y1+ u*dy

    #     dist_x = x-xp
    #     dist_y = y-yp

    #     dist = (dist_x**2 + dist_y**2)**0.5

    #     return dist


    # def candidate_intersection(self, P1,V,Q,r):
    #     (x1,y1) = ((P1[0] - Q[0]),(P1[1]- Q[1]))
    #     (x2,y2) = ((V[0] - Q[0]),(V[1]- Q[1]))
    #     dx, dy  = float(x2-x1), float(y2 - y1)

    #     dr = (dx**2 + dy**2)**(0.5)
    #     capD = x1*y2 - x2*y1
    #     disc = r**2*dr**2 - capD**2

    #     if disc <0:
    #         return None
    #     else:
    #         intersections = [( Q[0] + (capD*dy + sign*(-1 if dy<0 else 1)*dx*disc**0.5)/(dr**2), 
    #                             Q[1] + (-capD*dx+ sign*abs(dy)*disc**0.5)/(dr**2) ) 
    #                             for sign in ( (1,-1) if dy<0 else(-1,1) ) ] # collaborated w another team for this
                
    #         frac_along_segment = [(xi - P1[0])/dx if abs(dx)>abs(dy) else (yi - P1[1])/dy for xi,yi in intersections] # collaborated w another team for this
    #         intersect_pts = [(pt,frac) for pt, frac in zip(intersections, frac_along_segment) if 0<= frac <= 1]
    #         return intersect_pts


    # def follow_which_point(self, waypoints):
    #     #find the distances to all waypoints:
    #     # find waypoint closest to my car. 
    #     #d_waypoints = np.linalg.norm(waypoints - np.tile(self.state[0:2], (len(self.trajectory.distances),1) ))
    #     d_waypoints = np.zeros(len(self.trajectory.points)-1)

    #     i = 0
    #     for p1, p2 in zip(self.trajectory.points[:-1], np.roll(self.trajectory.points,-1)[:-1]):
    #         d_waypoints[i] = self.distance_to_segment(p1, p2, self.state)
    #         i += 1
    #     #print("Distances: ")
    #     #print(d_waypoints)
    #     #d_waypoints = np.linalg.norm(waypoints - np.tile)
    #     idx = np.argmin(d_waypoints)


    #     # closest_waypoint = waypoints[idx]
    #     # closest_waypoint_relative_coords = np.matmul(self.transform_matrix, np.transpose( np.array([closest_waypoint[0], closest_waypoint[1],0,1]) )  )

    #     # self.distance_to_closest = (closest_waypoint_relative_coords[0]**2 + closest_waypoint_relative_coords[1]**2)**0.5

    #     # #dist_to_waypoint = self.trajectory.distance_along_trajectory(closest_waypoint)

    #     # print("Closest Waypoint " + str(closest_waypoint_relative_coords))
    #     # print("Closest Waypoint Distance" + str(self.distance_to_closest))

    #     # self.visualize_waypoint.draw(closest_waypoint)
    #     # self.visualize_waypoint_pub.publish(self.visualize_waypoint.line)

    #     Q = self.state[0:2] # only first two indeces

    #     #print("State" + str(Q))
    #     r = self.lookahead

    #     goal_point = []
    #     intersect_pts = []

    #     # find a goal waypoint(along line)

    #     # look from the closest waypoint to the end of the trajectory.
    #     for i in range(idx, len(d_waypoints)):
    #         P1 = self.trajectory.points[i]
    #         V  = self.trajectory.points[i+1] 

    #         intersections = self.candidate_intersection(P1,V,Q,r)
    #         if intersections is not None and len(intersections) :
    #             candidate_goal = []
    #             j = 0

    #             while len(candidate_goal) < 2:
    #                 if intersections is not None and len(intersections) != 0:
    #                     for point in intersections:
    #                         point = (point[0], point[1]+j)
    #                         candidate_goal.append(point)
    #                 j += 1
    #                 if i+j <= len(d_waypoints) -1:
    #                     intersections = self.candidate_intersection(self.trajectory.points[i+j], self.trajectory.points[i+j+1], Q, r)
    #                 else:
    #                     break
                
    #             goal_point = max(candidate_goal, key=lambda x: x[1])[0]
    #             break
        
    #         else:
    #             i = idx
    #             x1,y1 = self.trajectory.points[i]
    #             x2,y2 = self.trajectory.points[i+1]
    #             x3,y3 = Q[0], Q[1]

    #             dx, dy = x2-x1, y2-y1
    #             det= dx*dx + dy*dy
    #             a = (dy*(y3-y1) + dx*(x3-x1)) / float(det)
    #             x = [x1+a*dx, y1+a*dy]
    #             goal_point = x
            

    #     return goal_point

        #     if len(intersect_pts) == 0:
        #         continue
        #     else:
        #         break
        
            
        # print("Intersection Points")
        # print(intersect_pts)

        # if len(intersect_pts) == 0:
        #     return []
        # else:    
        #     if len(intersect_pts) == 1:
        #         return intersect_pts[0]
        #     elif len(intersect_pts) >= 2:  
        #         return intersect_pts[1] #max(intersect_pts, key=lambda x: x[1])[0]
        # #print('wtf point')

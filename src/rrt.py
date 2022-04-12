import math
import random

import matplotlib.pyplot as plt
import numpy as np


class RRT:
    """
    RRT planning Class!
    """
    class Node:
        """
        Node Class...
        """
        def __init__(self,x,y):
            self.x = x 
            self.y = y
            self.parent = None

    class AreaBounds:

        def __init__(self,area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 occupancy_grid,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=5000000,
                 play_area = None
                 ):
        """
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        randArea could be the whole map!
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        """
        
        self.start = self.Node(start[0],start[1])
        self.end = self.Node(goal[0],goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.node_list = [] 
        self.max_iter = 500
        self.occupancy_grid = occupancy_grid
        self.goal_sample_rate = goal_sample_rate
        self.path_resolution = path_resolution

    def get_random_node(self):
        if random.randint(0,100) > self.goal_sample_rate:
            rnd = self.Node(
                math.floor(random.uniform(self.min_rand, self.max_rand)),
                math.floor(random.uniform(self.min_rand, self.max_rand)))
        else:
            rnd = self.Node(self.end.x, self.end.y)

        return rnd
            
    def get_nearest_node(self,node_list,rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind


    def check_collisions_improved(self,start,end):
        if (start.x,start.y) == (end.x,end.y):
            return True
        step_size_movement = 1  #This is how far we move in x and y on every step
        path_list = []
        d,theta = self.calc_distance_and_angle(start, end)
        print("Theta between",(start.x,start.y),"and ",(end.x,end.y),"is",theta)
        conti = True
        x_start = start.x
        y_start = start.y
        # print(x_start,y_start)
        while conti:
            step_x = step_size_movement*math.cos(theta)
            step_y = step_size_movement*math.sin(theta)
            print("Steps in x and y", step_x,step_y)
            print("Theta",theta)
            # print([int(x_start+step_x),int(y_start+step_y)])
            if self.occupancy_grid[int(x_start+step_x),int(y_start+step_y)] == 1:  ## CHANGE 1 TO OCCUPANCY GRID PERCENT!
                return False
            else:
                print("Current pos",[int(x_start+step_x),int(y_start+step_y)] ,"End: ",[end.x,end.y])
                og_shape = self.occupancy_grid.shape
                if int(x_start+step_x)==int(end.x) and int(y_start+step_y) == int(end.y):
                    conti = False
                x_start += step_x
                y_start += step_y
                
        return True


    def calc_distance_and_angle(self,from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
    
    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def planning(self):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            print("Random Generated Node",rnd_node.x,rnd_node.y)
            nearest_ind = self.get_nearest_node(self.node_list, rnd_node) # Changed get_nearest_node_index for get_nearest_node
            
            nearest_node = self.node_list[nearest_ind]
            print('Nearest Node',nearest_node.x,nearest_node.y)

            new_node = self.steer(nearest_node, rnd_node)
            
            if self.check_collisions_improved(nearest_node, new_node): # Deleted self.check_if_outside_play_area(new_node, self.play_area)
                self.node_list.append(new_node)

                if new_node.x == self.end.x and new_node.y == self.end.y:
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path

    def steer(self, from_node, to_node):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        print("Distance d:",d)
        if self.path_resolution < d:
            # ratio = self.path_resolution/d
            new_node.x += math.floor(self.path_resolution * math.cos(theta)) ## DELETED MATH FLOOR. SIZES <1 GET CONVERTED INTO 0
            new_node.y += math.floor(self.path_resolution * math.sin(theta))
            print("X and Y:",new_node.x,new_node.y)
        else:
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        return new_node

occupancy_grid_ex = np.array([
    [0,0,0,0,0,1,0,0,0,0],
    [0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,1,0,0],
    [0,0,1,0,0,0,0,1,0,0]
    ])
occupancy_grid_ex_2 = np.array([
    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    [0,0,1,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0],
    ])
obstacle_l = [(5, 0), (5, 1), (2, 2), (5, 2), (2, 3), (5, 3), (2, 4), (5, 4), (2, 5), 
    (2, 6), (2, 7), (2, 8), (7, 8), (2, 9), (7, 9)]
start = [0,0]

goal =  [8,8]
iter_counter = 0

rrt = RRT(start=[0, 0],  goal=[8, 8 ], rand_area = [0,9],occupancy_grid = occupancy_grid_ex, obstacle_list = obstacle_l,path_resolution = 1)
# rrt = RRT(start=[0, 0],  goal=[7, 22 ], rand_area = [0,24],occupancy_grid = occupancy_grid_ex_2, obstacle_list = obstacle_l,path_resolution = 1)

path = rrt.planning()
print(path)

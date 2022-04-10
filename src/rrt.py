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
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
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
        self.directions = [[-1,-1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1,1]]
        self.start = self.Node(start[0],start[1])
        self.end = self.Node(goal[0],goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[0]
        self.node_list = [] 
        self.max_iter = 500

    def get_random_node(self):
        
        if random.randint(0,100) > self.get_random_node:
            #Used to be random.uniform!
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = self.Node(self.end.x, self.end.y)

        return rnd
            
    def get_nearest_node(node_list,rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))


        return minind

    #TODO
    def detect_collisions (node,obstacleList):
        pass

    def calc_distance_and_angle(from_node, to_node):
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

#TODO    
    def planning(self):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
                self.check_collision(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            # check if the node we are at is the goal node
            # if self.calc_dist_to_goal(self.node_list[-1].x,
            #                             self.node_list[-1].y) <= self.expand_dis:
            if self.node_list[-1].x == self.end.x and self.node_list[-1].y == self.end.y:
                return self.generate_final_course(len(self.node_list) - 1)

            # if animation and i % 5:
            #     self.draw_graph(rnd_node)
        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d 

        if self.path_resolution < d:
            ratio = self.path_resolution/d
        else:
            ratio = d/self.path_resolution

        new_node.x += d * ratio * math.cos(theta)
        new_node.y += d * ratio * math.sin(theta)

        # for _ in range(n_expand):
        #     new_node.x += self.path_resolution * math.cos(theta)
        #     new_node.y += self.path_resolution * math.sin(theta)
        #     new_node.path_x.append(new_node.x)
        #     new_node.path_y.append(new_node.y)

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

start = [0,0]

goal =  [8,8]
iter_counter = 0

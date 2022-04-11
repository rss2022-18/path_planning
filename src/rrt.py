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
        self.occupancy_grid = occupancy_grid

    def get_random_node(self):
        if random.randint(0,100) > self.get_random_node:
            rnd = self.Node(
                math.floor(random.uniform(self.min_rand, self.max_rand)),
                math.floor(random.uniform(self.min_rand, self.max_rand)))
        else:
            rnd = self.Node(self.end.x, self.end.y)

        return rnd
            
    def get_nearest_node(node_list,rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def sign(n):
        return (n > 0) - (n < 0)

    def check_collision(self, start, end ):
    #ray tracing taken from here 
    #https://stackoverflow.com/questions/35807686/find-cells-in-array-that-are-crossed-by-a-given-line-segment
        (xA, yA) = start.x, start.y
        (xB, yB) = end.x, end.y 
        (dx, dy) = (xB - xA, yB - yA)
        (sx, sy) = (self.sign(dx), self.sign(dy))

        grid_A = (math.floor(xA), math.floor(yA))
        grid_B = (math.floor(xB), math.floor(yB))
        (x, y) = grid_A
        traversed=[grid_A]

        tIx = dy * (x + sx - xA) if dx != 0 else float("+inf")
        tIy = dx * (y + sy - yA) if dy != 0 else float("+inf")

        while (x,y) != grid_B:
            # NB if tIx == tIy we increment both x and y
            (movx, movy) = (tIx <= tIy, tIy <= tIx)

            if movx:
                # intersection is at (x + sx, yA + tIx / dx^2)
                x += sx
                tIx = dy * (x + sx - xA)

            if movy:
                # intersection is at (xA + tIy / dy^2, y + sy)
                y += sy
                tIy = dx * (y + sy - yA)

            traversed.append( (x,y) )

        return traversed

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

    def planning(self):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
                self.check_collision(nearest_node, new_node):
                self.node_list.append(new_node)

                if new_node.x == self.end.x and new_node.y == self.end.y:
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path

    def steer(self, from_node, to_node):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        if self.path_resolution < d:
            ratio = self.path_resolution/d
            new_node.x += math.floor(d * ratio * math.cos(theta))
            new_node.y += math.floor(d * ratio * math.sin(theta))
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

start = [0,0]

goal =  [8,8]
iter_counter = 0

from Queue import PriorityQueue
import numpy as np
import math
import random

import matplotlib.pyplot as plt


class Planner(object):
    def __init__(self, map, height, width, obstacle_threshold):
        self.map = map
        self.height = height
        self.width = width
        self.obstacle_threshold = obstacle_threshold


class AStarPlanner(Planner):
    def __init__(self, map, height, width, obstacle_threshold):
        super(AStarPlanner, self).__init__(
            map, height, width, obstacle_threshold)

    def find_path(self, start_point, end_point):
        """
        A* path planning algorithm using manhattan distance as heuristic
        """
        open_paths = PriorityQueue()
        open_paths.put(start_point, 0)
        came_from = {}
        cost_so_far = {}
        cost_so_far[start_point] = 0
        while not open_paths.empty():
            current_point = open_paths.get()
            if current_point == end_point:
                break
            for next in self.get_possible_moves(current_point):
                new_cost = cost_so_far[current_point] + \
                    self.euclidean_distance(current_point, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + \
                        self.euclidean_distance(next, end_point)
                    open_paths.put(next, priority)
                    came_from[next] = current_point
        path = [current_point]
        print(current_point)
        while path[-1] != start_point:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    def is_valid_point(self, point):
        """
        Checks if the point is valid
        """
        if point[0] < 0 or point[0] >= self.width:
            return False
        if point[1] < 0 or point[1] >= self.height:
            return False
        if self.map[self.get_index(point)] >= self.obstacle_threshold:
            return False
        return True

    def get_possible_moves(self, point):
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                new_point = (point[0] + i, point[1] + j)
                if self.is_valid_point(new_point):
                    yield new_point

    def get_index(self, point):
        """
        Returns the index of the point in the map
        """
        return int(point[0] + point[1] * self.width)

    def euclidean_distance(self, start_point, end_point):
        """
        Cost based off euclidean distance
        """
        return np.sqrt((start_point[0] - end_point[0])**2 + (start_point[1] - end_point[1])**2)

    def manhattan_distance(self, start_point, end_point):
        """
        Current heuristic uses manhattan distance
        """
        return abs(start_point[0] - end_point[0]) + abs(start_point[1] - end_point[1])


class RRTPlanner(Planner):
    """
    RRT planning Class!
    """
    class Node:
        """
        Node Class...
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self, map, height, width, obstacle_threshold, path_resolution=10, goal_sample_rate=5):
        """
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        """
        super(RRTPlanner, self).__init__(
            map, height, width, obstacle_threshold)
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.node_list = []
        self.max_iter = 10000
        self.occupancy_grid = map

    def get_index(self, point):
        return int(point[0] + point[1] * self.width)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            not_a_collision = True
            rnd = self.Node(
                math.floor(random.uniform(0, self.height)),
                math.floor(random.uniform(0, self.width)))
        else:
            rnd = self.Node(self.end.x, self.end.y)

        return rnd

    def get_nearest_node(self, node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def check_collisions_improved(self, start, end):
        if (start.x, start.y) == (end.x, end.y):
            return True
        step_size_movement = 10  # This is how far we move in x and y on every step
        _, theta = self.calc_distance_and_angle(start, end)
        conti = True
        x_start = start.x
        y_start = start.y
        while conti:
            step_x = step_size_movement*math.cos(theta)
            step_y = step_size_movement*math.sin(theta)
            ind = self.get_index((int(x_start+step_x), int(y_start+step_y)))
            if self.occupancy_grid[ind] > 0:  # CHANGE 1 TO OCCUPANCY GRID PERCENT!
                return False
            if abs(int(x_start+step_x) - int(end.x)) < 10 and abs(int(y_start+step_y) - int(end.y)) < 10:
                return True
            x_start += step_x if int(x_start+step_x) != int(end.x) else 0
            y_start += step_y if int(y_start+step_y) != int(end.y) else 0
        return True

    def calc_distance_and_angle(self, from_node, to_node):
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

    def find_path(self, start_point, goal):
        self.start = self.Node(start_point[0], start_point[1])
        self.end = self.Node(goal[0], goal[1])
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            # Changed get_nearest_node_index for get_nearest_node
            nearest_ind = self.get_nearest_node(self.node_list, rnd_node)

            nearest_node = self.node_list[nearest_ind]
            # print('Nearest Node',nearest_node.x,nearest_node.y)
            new_node = self.steer(nearest_node, rnd_node)
            # print("New next target node",(new_node.x,new_node.y))
            # Deleted self.check_if_outside_play_area(new_node, self.play_area)
            if self.check_collisions_improved(nearest_node, new_node):
                self.node_list.append(new_node)

                if new_node.x == self.end.x and new_node.y == self.end.y:
                    print("Found goal state!")
                    return self.generate_final_course(len(self.node_list) - 1)
        return None  # cannot find path

    def reached_goal(self, node):
        return np.sqrt((node.x - self.end.x)**2 + (node.y - self.end.y)**2) < 1

    def steer(self, from_node, to_node):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        if self.path_resolution < d:
            # DELETED MATH FLOOR. SIZES <1 GET CONVERTED INTO 0
            new_node.x += math.floor(self.path_resolution * math.cos(theta))
            new_node.y += math.floor(self.path_resolution * math.sin(theta))
        else:
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        return new_node

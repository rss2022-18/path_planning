from Queue import PriorityQueue
import heapq
import numpy as np
import math
import random

import matplotlib.pyplot as plt


class Planner(object):
    def __init__(self, map, height, width, obstacle_threshold, path_resolution=1):
        self.map = map
        self.height = height
        self.width = width
        self.obstacle_threshold = obstacle_threshold
        self.path_resolution = path_resolution

    def get_index(self, point):
        return int(point[1]), int(point[0])

    def is_valid_point(self, point):
        """
        Checks if the point is valid
        """
        if point[0] < 0 or point[0] >= self.height:
            return False
        if point[1] < 0 or point[1] >= self.width:
            return False
        if self.map[self.get_index(point)] > 0:
            return False
        return True


class AStarPlanner(Planner):
    class Node:
        def __init__(self, parent=None, position=None):
            self.parent = parent
            self.position = position
            self.g = 0
            self.h = 0
            self.f = 0

        def __eq__(self, other):
            return self.position == other.position

        def __lt__(self, other):
            return self.f < other.f

        def __gt__(self, other):
            return self.f > other.f

    def __init__(self, map, height, width, obstacle_threshold, path_resolution=1):
        super(AStarPlanner, self).__init__(
            map, height, width, obstacle_threshold, path_resolution)

    def find_path(self, start_point, end_point):
        """
        A* path planning algorithm using manhattan distance as heuristic
        """
        start_node = self.Node(None, start_point)
        open_paths = [start_node]
        costs = {start_point: 0}
        closed_paths = set()
        while len(open_paths) > 0:
            current_node = heapq.heappop(open_paths)
            if current_node.position == end_point:
                print("Break!")
                break
            if current_node.position in closed_paths:
                continue
            closed_paths.add(current_node.position)
            for next_point in self.get_possible_moves(current_node.position):
                next_node = self.Node(current_node, next_point)
                next_node.g = current_node.g + \
                    self.euclidean_distance(current_node.position, next_point)
                next_node.h = self.euclidean_distance(next_point, end_point)
                next_node.f = next_node.g + next_node.h

                if next_point not in closed_paths and (next_node not in open_paths or costs[next_point] > next_node.f):
                    heapq.heappush(open_paths, next_node)
                    costs[next_point] = next_node.f

        if current_node.position != end_point:
            print("Failed to find path!")
            print("Last node evaluated:", current_node.position)
            return [end_point, start_point]
        path = [current_node.position]
        while path[-1] != start_point:
            path.append(current_node.parent.position)
            current_node = current_node.parent
        path.reverse()
        return path

    def get_possible_moves(self, point):
        moves = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                new_point = (point[0] + i, point[1] + j)
                if self.is_valid_point(new_point):
                    moves.append(new_point)
        return moves

    def is_valid_path(self, point, i, j):
        if i == 0:
            i = self.path_resolution*[0]
        else:
            if i > 0:
                i = list(range(i))
            else:
                i = list(range(i, 0))
        if j == 0:
            j = self.path_resolution*[0]
        else:
            if j > 0:
                j = list(range(j))
            else:
                j = list(range(j, 0))
        for x, y in zip(i, j):
            if not self.is_valid_point((point[0] + x, point[1] + y)):
                return False
        return True

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

    def __init__(self, map, height, width, obstacle_threshold, path_resolution=5, goal_sample_rate=5):
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

        # return int(point[0] + point[1] * self.width)

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
        # This is how far we move in x and y on every step
        step_size_movement = self.path_resolution
        _, theta = self.calc_distance_and_angle(start, end)
        conti = True
        x_start = start.x
        y_start = start.y
        while conti:
            step_x = step_size_movement*math.cos(theta)
            step_y = step_size_movement*math.sin(theta)
            ind = self.get_index((int(x_start+step_x), int(y_start+step_y)))
            # if not self.is_valid_point(ind):
            #     return False
            # CHANGE 1 TO OCCUPANCY GRID PERCENT!
            if abs(self.occupancy_grid[ind]) > 0:
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

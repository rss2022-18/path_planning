from Queue import PriorityQueue
import numpy as np


class AStarPlanner:
    def __init__(self, map, height, width, obstacle_threshold):
        self.map = map
        self.map_height = height
        self.map_width = width
        self.obstacle_threshold = obstacle_threshold

    def find_path(self, start_point, end_point):
        """
        A* path planning algorithm using manhattan distance as heuristic
        """
        open_paths = PriorityQueue()
        open_paths.put(start_point, 0)
        came_from = {}
        cost_so_far = {}
        cost_so_far[start_point] = 0
        loop_count = 0
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
                        self.manhattan_distance(next, end_point)
                    open_paths.put(next, priority)
                    came_from[next] = current_point
        path = [end_point]
        print(current_point)
        while path[-1] != start_point:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    def is_valid_point(self, point):
        """
        Checks if the point is valid
        """
        if point[0] < 0 or point[0] >= self.map_width:
            return False
        if point[1] < 0 or point[1] >= self.map_height:
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
        return int(point[0] + point[1] * self.map_width)

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

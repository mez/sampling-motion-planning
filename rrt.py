from pathplanning.types import Node, Point
import numpy as np


# the RRT class will be responsible for generating the random tree and finding a path from the start to the goal
# the goal is to expand the tree until we reach the goal or we run out of iterations
class RRT:
    def __init__(
        self,
        start: Point,
        goal: Point,
        num_iterations: int,
        config_space: np.ndarray,
        step_size: float,
    ):
        self.random_tree = Node(start)
        self.goal = Node(goal)
        self.neartest_node = None
        self.iterations = min(
            num_iterations, 200
        )  # limit the number of iterations to 200
        self.config_space = config_space
        self.step_size = (
            step_size  # the maximum distance between two nodes in the tree aka rho
        )
        self.path_distance = (
            0  # starts at 0 and will be updated as we add nodes to the tree
        )
        self.nearest_distance = 10000  # updated as you find the nearest node to the random sample, initialized to a large number
        self.num_waypoints = 0  # number of waypoints in the path from start to goal
        self.waypoints = []  # list of waypoints in the path from start to goal

    # add the point to the nearest node and add goal when reached
    def add_child(self, location: Point):
        if location.x == self.goal.location.x and location.y == self.goal.location.y:
            # add the goal node to the children of the nearest node.
            pass
        else:
            temp_node = Node(location)
            # add temp node to children of nearest node.

    # samples a random point in the configuration space and returns it as a tuple (x, y)
    def sample_a_point(self):
        # note np array indexing is row, column so we need to use shape[1] for x and shape[0] for y
        x = np.random.randint(1, self.config_space.shape[1])
        y = np.random.randint(1, self.config_space.shape[0])
        return np.array([x, y])

    # steer a distance step_size from location_start to location_end and return the new location as a tuple (x, y)
    def steer_to_point(self, location_start: Point, location_end: Point) -> Point:
        offset = self.step_size * self.find_unit_vector(location_start, location_end)
        coord = (
            np.array([location_start.x, location_start.y]) + offset
        )
        coord[0] = min(coord[0], self.config_space.shape[1] - 1)
        coord[1] = min(coord[1], self.config_space.shape[0] - 1)
        return Point(coord[0], coord[1])

    # checks if the line segment between location_start and location_end intersects with any obstacles in the configuration space
    def is_point_in_obstacle(self, location_start: Point, location_end: Point):
        unit_vector = self.find_unit_vector(location_start, location_end)
        for t in np.linspace(0, self.step_size, int(self.step_size) * 2):
            x = int(location_start.x + t * unit_vector[0])
            y = int(location_start.y + t * unit_vector[1])
            if self.config_space[y, x] == 1:
                return True
        return False

    # find unit vector between a node and an end point and return it as a tuple (x, y)
    def find_unit_vector(self, location_start: Point, location_end: Point):
        vector = np.array(
            [
                location_end.x - location_start.x,
                location_end.y - location_start.y,
            ]
        )
        norm = np.linalg.norm(vector)
        if norm == 0:
            return vector
        return vector / norm

    # find the nearest node from a given unconnected point (Euclidean distance) and return the node
    def find_nearest_node(self, root: Node, point: Point) -> Node:
        # return condition if the root node is None
        if root is None:
            return None
        # find the distance between the root node and the point
        distance = self.find_distance(root.location, point)
        # if the distance is less than the nearest distance, update the nearest node and nearest distance
        if distance < self.nearest_distance:
            self.nearest_distance = distance
            self.nearest_node = root
        # recursively check the children of the root node
        for child in root.children:
            self.find_nearest_node(child, point)

    # find euclidean distance between a node and a point and return the distance
    def find_distance(self, node_point: Point, point: Point):
        distance = np.sqrt(
            (node_point.x - point.x) ** 2 + (node_point.y - point.y) ** 2
        )
        return distance

    # check if the goal has been reached within step size and return True or False
    def is_goal_reached(self, node: Node):
        pass

    # reset nearest node and nearest distance to initial values
    def reset_nearest_node(self):
        self.nearest_distance = 10000
        self.nearest_node = None

    # trace the path from goal to start
    def trace_rrt_path(self, goal_node: Node):
        pass


# # Rapidly-exploring Random Tree March
# class RRTMarch:
#     NotImplementedError


# # Rapidly-exploring Random Tree Star
# class RRTStar:
#     NotImplementedError


# # Probabilistic Road Map
# class PRM:
#     NotImplementedError

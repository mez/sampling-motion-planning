from pp_types import Node, Point
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
            num_iterations, 5000
        )  # limit the number of iterations to 5000
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

    # add the point to the nearest node
    def add_child(self, location: Point):
        if self.nearest_node is None:
            raise ValueError("nearest_node is None — call find_nearest_node first")
        new_node = Node(location)
        self.nearest_node.children.append(new_node)
        new_node.parent = self.nearest_node

    # samples a random point in the configuration space and returns it as a tuple (x, y)
    def sample_a_point(self):
        # note np array indexing is row, column so we need to use shape[1] for x and shape[0] for y
        x = np.random.randint(1, self.config_space.shape[1])
        y = np.random.randint(1, self.config_space.shape[0])
        return Point(x, y)

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
            x = round(location_start.x + t * unit_vector[0])
            y = round(location_start.y + t * unit_vector[1])
            if x < 0 or y < 0 or x >= self.config_space.shape[1] or y >= self.config_space.shape[0]:
                continue
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
    def is_goal_reached(self, point: Point):
        distance = self.find_distance(point, self.goal.location)
        return distance <= self.step_size

    # reset nearest node and nearest distance to initial values
    def reset_nearest_values(self):
        self.nearest_distance = 10000
        self.nearest_node = None

    # trace the path from goal to start
    def trace_rrt_path(self, goal_node: Node):
        # recursive trace back to start node and add each node to the waypoints list
        if goal_node is None:
            # we are done now reverse the waypoints list so that it is in the correct order from start to goal
            self.waypoints.reverse()
            return
        
        waypoint = Point(goal_node.location.x, goal_node.location.y)
        self.waypoints.append(waypoint)
        self.num_waypoints += 1
        self.path_distance += self.step_size #this is a rough estimate of the path distance, we can improve this later by calculating the actual distance between nodes
        self.trace_rrt_path(goal_node.parent)

    def run(self):
        for i in range(self.iterations):
            self.reset_nearest_values()
            print(f"Iteration {i+1}/{self.iterations}")

            sampled_point = self.sample_a_point()
            self.find_nearest_node(self.random_tree, sampled_point)

            # we move from nearest_node towards the sampled_point by a distance of step_size and create a new node at that location
            steered_point = self.steer_to_point(self.nearest_node.location, sampled_point)
            if not self.is_point_in_obstacle(self.nearest_node.location, steered_point):
                self.add_child(steered_point)

                if self.is_goal_reached(steered_point):
                    print("Goal reached!")
                    self.trace_rrt_path(self.nearest_node.children[-1])  # trace back from the last added child
                    return self.waypoints

# # Rapidly-exploring Random Tree March
# class RRTMarch:
#     NotImplementedError


# # Rapidly-exploring Random Tree Star
# class RRTStar:
#     NotImplementedError


# # Probabilistic Road Map
# class PRM:
#     NotImplementedError

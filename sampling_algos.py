import numpy as np

class TreeNode:
    def __init__(self, location_x: int, location_y: int):
        # location tells us where the node is in the configuration space
        self.location_x = location_x
        self.location_y = location_y
        self.parent = None
        self.children = []
  
# the RRT class will be responsible for generating the random tree and finding a path from the start to the goal
# the goal is to expand the tree until we reach the goal or we run out of iterations
class RRT:
    def __init__(self, start: tuple, goal: tuple, num_iterations: int, config_space: np.ndarray, step_size: int):
        self.random_tree = TreeNode(start[0], start[1])
        self.goal = TreeNode(goal[0], goal[1])
        self.neartest_node = None
        self.iterations = min(num_iterations, 200) # limit the number of iterations to 200
        self.config_space = config_space
        self.step_size = step_size #the maximum distance between two nodes in the tree aka rho
        self.path_distance = 0 #starts at 0 and will be updated as we add nodes to the tree
        self.nearest_distance = 10000 # updated as you find the nearest node to the random sample, initialized to a large number
        self.num_waypoints = 0 # number of waypoints in the path from start to goal
        self.waypoints = [] # list of waypoints in the path from start to goal

    # add the point to the nearest node and add goal when reached
    def add_child(self, location_x: int, location_y: int):
       pass

    # samples a random point in the configuration space and returns it as a tuple (x, y)
    def sample_a_point(self):
        pass

    # steer a distance step_size from location_start to location_end and return the new location as a tuple (x, y)
    def steer_to_point(self, location_start: tuple, location_end: tuple):
        pass

    #checks if the line segment between location_start and location_end intersects with any obstacles in the configuration space
    def is_point_in_obstacle(self, location_start: tuple, location_end: tuple):
        pass

# Rapidly-exploring Random Tree March
class RRTMarch:
    NotImplementedError

# Rapidly-exploring Random Tree Star
class RRTStar:
    NotImplementedError

# Probabilistic Road Map
class PRM:
    NotImplementedError
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rrt import RRT
from pp_types import Point

# We need a configuration space, a 2D space where 0 is free space and 1 is occupied space. We will create a simple configuration space with a few obstacles.
config_space = np.zeros((100, 100))
# Add some obstacles to the configuration space
config_space[20:40, 20:40] = 1  # Square obstacle
config_space[60:80, 60:80] = 1  # Another square


# lets initialize the RRT class with a start and goal location, number of iterations, configuration space, and step size
start = Point(10, 10)
goal = Point(90, 90)
num_iterations = 2000
step_size = 5.0
goal_region = plt.Circle((goal.x, goal.y), step_size, color='g', alpha=0.5)

def plot_tree(node, ax):
    for child in node.children:
        ax.plot([node.location.x, child.location.x], [node.location.y, child.location.y], color='#aaaaaa', linewidth=0.5)
        plot_tree(child, ax)

rrt = RRT(start, goal, num_iterations, config_space, step_size)
waypoints = rrt.run()

# view the configuration space with the start and goal locations
fig = plt.figure("RRT Path Planning")
plt.imshow(config_space, cmap='binary', origin='lower')
ax = plt.gca()

plot_tree(rrt.random_tree, ax)

if waypoints:
    xs = [p.x for p in waypoints]
    ys = [p.y for p in waypoints]
    ax.plot(xs, ys, 'b--', linewidth=2, label='Path')

plt.plot(start.x, start.y, 'ro', label='Start')
plt.plot(goal.x, goal.y, 'go', label='Goal')
ax.add_patch(goal_region)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
plt.legend()
plt.show()

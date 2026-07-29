import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# We need a configuration space, a 2D space where 0 is free space and 1 is occupied space. We will create a simple configuration space with a few obstacles.
config_space = np.zeros((100, 100))
# Add some obstacles to the configuration space
config_space[20:40, 20:40] = 1  # Square obstacle
config_space[60:80, 60:80] = 1  # Another square


# lets initialize the RRT class with a start and goal location, number of iterations, configuration space, and step size
start = (10, 10)
goal = (90, 90)
num_iterations = 200
step_size = 5.0
goal_region = plt.Circle(goal, step_size, color='g', alpha=0.5)

# view the configuration space with the start and goal locations
fig = plt.figure("RRT Path Planning")
plt.imshow(config_space, cmap='binary', origin='lower')
plt.plot(start[0], start[1], 'ro', label='Start')
plt.plot(goal[0], goal[1], 'go', label='Goal')
ax = plt.gca()
ax.add_patch(goal_region)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
plt.legend()
plt.show()

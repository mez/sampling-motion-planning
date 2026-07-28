import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# We need a configuration space, a 2D space where 0 is free space and 1 is occupied space. We will create a simple configuration space with a few obstacles.
config_space = np.zeros((100, 100))
# Add some obstacles to the configuration space
config_space[20:40, 20:40] = 1  # Square obstacle
config_space[60:80, 60:80] = 1  # Another square

# Function to visualize the configuration space
def visualize_config_space(config_space):
    plt.imshow(config_space, cmap='gray', origin='lower')
    plt.title('Configuration Space')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.show()  

visualize_config_space(config_space)


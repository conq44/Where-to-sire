import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches


class OccupancyGrid:
    def __init__(self, bottom_left, top_right, resolution, obstacles):
        self.lo = bottom_left
        self.hi = top_right
        self.resolution = resolution
        self.obstacles = obstacles
        self.height = top_right[1] - bottom_left[1]
        self.width = top_right[0] - bottom_left[0]

    def get_neighbors(self, curr_pos):
        dir_count = {(0, 0): 0, (0, 1): 1, (-1, 1): 2, (-1, 0): 3,
                     (-1, -1): 4, (0, -1): 5, (1, -1): 6, (1, 0): 7, (1, 1): 8}  # might need a fix
        neighbors = []
        neighbor_directions = []
        x, y = curr_pos
        x_max, y_max = self.hi
        x_min, y_min = self.lo
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                x_new = x + i * self.resolution
                y_new = y + j * self.resolution
                new_pos = (x_new, y_new)
                if x_max >= x_new >= x_min and y_max >= y_new >= y_min:
                    if new_pos not in self.obstacles:
                        neighbors.append(new_pos)
                        neighbor_directions.append(dir_count[(i, j)])
        return neighbors, neighbor_directions

    def set_obstacles(self, num_obstacles):
        # randomly choose the obstacle coordinates
        pass

    def plot_grid(self, start, end, path):
        # plot the grid
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')

        for obs in self.obstacles:
            ax.add_patch(patches.Rectangle(obs, 1, 1, color='grey'))
        for pos in path:
            ax.add_patch(patches.Rectangle(pos, 1, 1, color='red'))
        ax.add_patch(patches.Rectangle(start, 1, 1, color='blue'))
        ax.add_patch(patches.Rectangle(end, 1, 1, color='green'))
        ticks = np.arange(0, self.height + 1, self.resolution)
        ax.set_xticks(ticks)
        ax.set_yticks(ticks)
        ax.set(xlim=(0, self.width), ylim=(0, self.height))
        plt.grid(linestyle="-", color='black')
        plt.show()

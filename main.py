import numpy as np
import heapq
import math
import time
from Occupancy_grid import *
from algorithms import *


bottom_left = (0, 0)
top_right = (100, 100)
resolution = 1
# obstacles = [(6, 10), (7, 10), (8, 10), (9, 10), (10, 10), (11, 10), (12, 10), (12, 11), (9, 11), (9, 12)]
# obstacles = []
obstacles = [[(0, 10), (40, 10)], [(40, 10), (40, 20)], [(40, 20), (0, 20)],
            [(100, 10), (50, 10)], [(50, 10), (50, 20)], [(50, 20), (100, 20)],
             [(0, 50), (70, 50)], [(70, 50), (70, 60)], [(70, 60), (0, 60)],
             [(40, 100), (40, 80)], [(40, 80), (50, 80)], [(50, 80), (50, 100)]]
grid = OccupancyGridRRT(bottom_left, top_right, resolution, obstacles)
start_pos = Node((0, 0))
start_pos.dist = 0
start_pos.direction = 90
end_pos = Node((10, 80))
# start_time_2 = time.time()
# path, discovered = Astar_search(start_pos, end_pos, grid)
# print("--- %s seconds for a star ---" % (time.time() - start_time_2))
# grid.plot_grid(start_pos.data, end_pos.data, path, discovered)
# start_time = time.time()
# path2, discovered2 = dijkstra(start_pos, end_pos, grid)
# print("--- %s seconds for dijkstra ---" % (time.time() - start_time))
# grid.plot_grid(start_pos.data, end_pos.data, path2, discovered2)
# # print(get_h_cost(Node((2,2)),Node((4,4))))
discovered, last_node, path = RRT(start_pos, end_pos, grid)
print(path)
shortcut_path = get_shortcut(path, obstacles)
print(shortcut_path)
grid.plot_grid(start_pos.data, end_pos.data, discovered, path, shortcut_path)
# grid.plot_grid(start_pos.data, end_pos.data, discovered, shortcut_path)